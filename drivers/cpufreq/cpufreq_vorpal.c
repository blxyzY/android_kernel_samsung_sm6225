// SPDX-License-Identifier: GPL-2.0
/*
 * Vorpal CPUFreq Governor v2.2 - Stability / Portability / Battery
 * Standalone governor - no scheduler hooks, no fair.c patching.
 *
 * Ported to kernel 4.19.
 *
 * v2.2 changes vs v2.1:
 *   - Frame-risk boost deleted (fought the thermal latch)
 *   - Touch/input handler deleted (input event is not load evidence)
 *   - Per-cluster gaming caps deleted, replaced with floors only
 *   - Warmup: 80% floor for 400ms, anchored to gaming_mode write
 *   - Slew descent 0.5%/ms (was 1%/ms)
 *   - Floor gate latched deadband 25/35
 *   - Thermal cool-down latched 80/85, steady relief floor 52
 *   - Daily: Big/Prime lift 70/68 -> 80 (arm 80/release 68)
 *            Little cap 65 (arm 72/release 55)
 *   - Util getter inlines schedutil_cpu_util minus RT-runnable shortcut
 *   - All updates under update_lock, fast-switch call moved inside
 *   - rfx_work uses __cpufreq_driver_target
 *   - irq_work/work_lock init before fast-switch return
 *
 * Author: Templar Dev (Steambot12)
 * 4.19 port
 */

#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/irq_work.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/sched/cpufreq.h>
#include <linux/sched/topology.h>
#include <linux/slab.h>
#include <linux/tick.h>
#include <uapi/linux/sched/types.h>
#include <asm/topology.h>
#include <linux/arch_topology.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

extern int vm_swappiness;
extern unsigned int sysctl_sched_latency;

/* === 4.19 COMPAT SHIM === */
#define RFX_OLD_ARCH_CAP_API 1

static inline unsigned long rfx_arch_cap(int cpu)
{
#if RFX_OLD_ARCH_CAP_API
	return arch_scale_cpu_capacity(NULL, cpu);
#else
	return arch_scale_cpu_capacity(cpu);
#endif
}

#ifndef CPUFREQ_NEED_UPDATE_LIMITS
#define CPUFREQ_NEED_UPDATE_LIMITS 0
#endif

#define CPUFREQ_VORPAL_PROGNAME     "Vorpal CPUFreq Governor"
#define CPUFREQ_VORPAL_AUTHOR       "Templar Dev"
#define CPUFREQ_VORPAL_VERSION      "2.2"

/* === 4.19: synchronize_rcu is synchronize_sched === */
#ifndef synchronize_rcu
#define synchronize_rcu() synchronize_sched()
#endif

/* === RATE LIMITS === */
#define RFX_UI_IDLE_PROTECTION_NS   (25 * NSEC_PER_MSEC)
#define RFX_GAME_LAUNCH_BOOST_NS    (15000 * NSEC_PER_MSEC)

#define CPUFREQ_VORPAL_BIG_UP_RATE_LIMIT_US      0
#define CPUFREQ_VORPAL_BIG_DOWN_RATE_LIMIT_US    8000
#define CPUFREQ_VORPAL_DEFAULT_RATE_LIMIT_US        10
#define CPUFREQ_VORPAL_UP_RATE_LIMIT_US             0
#define CPUFREQ_VORPAL_DOWN_RATE_LIMIT_US    8000
#define CPUFREQ_VORPAL_LITTLE_UP_RATE_LIMIT_US      0
#define CPUFREQ_VORPAL_LITTLE_DOWN_RATE_LIMIT_US  4000

#define RFX_LITTLE_DOWN_HEAVY_US    20000
#define RFX_LITTLE_DOWN_MEDIUM_US    4000
#define RFX_LITTLE_DOWN_LIGHT_US       50

#define CPUFREQ_VORPAL_PRIME_UP_RATE_LIMIT_US       0
#define CPUFREQ_VORPAL_PRIME_DOWN_RATE_LIMIT_US   8000
#define CPUFREQ_VORPAL_PRIME_RATE_LIMIT_US          1

/* === HISPEED / BLEND === */
#define CPUFREQ_VORPAL_DEFAULT_HISPEED_WINDOW_US    30
#define CPUFREQ_VORPAL_DEFAULT_HISPEED_FILTER_SHIFT 0

#define CPUFREQ_VORPAL_DEFAULT_HISPEED_BOOST_PCT   72
#define RFX_HISPEED_GAMING_PCT                     85
#define RFX_HISPEED_DAILY_PCT                      55
#define RFX_HISPEED_VIDEO_PCT                      72

#define IOWAIT_BOOST_MIN    (SCHED_CAPACITY_SCALE / 8)

#define HISPEED_HALFLIFE_NS    (6 * NSEC_PER_MSEC)
#define HISPEED_HALFLIFE_MAX                       8

/* === v2.2 IOWAIT CEILINGS === */
#define RFX_IOWAIT_CEIL_GAMING_PCT   60
#define RFX_IOWAIT_CEIL_DAILY_PCT    35

/* === v2.2 EMA === */
#define RFX_EMA_DECAY_SHIFT_GAMING   3
#define RFX_EMA_PERIOD_NS           (250 * NSEC_PER_USEC)

/* === v2.2 BOUNDED SLEW: 0.5% per ms === */
#define RFX_SLEW_MAX_PCT_X2_PER_MS    1   /* 0.5% = 1/2, pakai fixed-point */

/* === v2.2 GAMING FLOORS === */
#define RFX_PRIME_GAMING_FLOOR_PCT   64
#define RFX_BIG_GAMING_FLOOR_PCT     58
#define RFX_LITTLE_GAMING_FLOOR_PCT  38

/* === v2.2 HEADROOM RAMP === */
#define RFX_HEADROOM_GAMING_GATE     75
#define RFX_HEADROOM_GAMING_MAX_PCT   8
#define RFX_SAT_TO_MAX_PCT          100   /* disabled */

/* === v2.2 WARMUP === */
#define RFX_WARMUP_FLOOR_PCT         80
#define RFX_WARMUP_DURATION_NS      (400 * NSEC_PER_MSEC)

/* === v2.2 FLOOR GATE DEADBAND === */
#define RFX_FLOOR_GATE_ARM_PCT       35
#define RFX_FLOOR_GATE_RELEASE_PCT   25

/* === v2.2 THERMAL === */
#define RFX_THERMAL_COOL_ENTER_C     85
#define RFX_THERMAL_COOL_EXIT_C      80
#define RFX_THERMAL_COOL_RELIEF_PCT  52
#define RFX_THERMAL_WARM_C           70
#define RFX_THERMAL_POLL_ACTIVE_MS   2000
#define RFX_THERMAL_POLL_IDLE_MS     5000

/* === v2.2 DAILY PROFILE === */
#define RFX_DAILY_BIG_BASE_PCT       70
#define RFX_DAILY_PRIME_BASE_PCT     68
#define RFX_DAILY_BIG_LIFT_PCT       80
#define RFX_DAILY_BIG_ARM_PCT        80
#define RFX_DAILY_BIG_RELEASE_PCT    68
#define RFX_DAILY_LITTLE_CAP_PCT     65
#define RFX_DAILY_LITTLE_ARM_PCT     72
#define RFX_DAILY_LITTLE_RELEASE_PCT 55

#define RFX_DAILY_BIG_REST_US        3000
#define RFX_DAILY_LITTLE_REST_US     3000
#define RFX_DAILY_BIG_DOWN_US        2500
#define RFX_DAILY_LITTLE_DOWN_US     3000

/* === BURST GUARD === */
#define RFX_BURST_GUARD_NS    (250 * NSEC_PER_MSEC)
#define RFX_BURST_DROP_THRESHOLD                  12

/* === HEAVY SUSTAIN === */
#define RFX_SUSTAIN_HEAVY_ENTER_PCT   35
#define RFX_SUSTAIN_HEAVY_EXIT_PCT    20
#define RFX_SUSTAIN_HEAVY_BUSY_PCT     8
#define RFX_SUSTAIN_HEAVY_TICKS        1
#define RFX_SUSTAIN_EXIT_TICKS         8

#define RFX_GAMING_LOCK_DURATION_NS   (12000 * NSEC_PER_MSEC)

/* Adaptive Gaming */
#define RFX_GAMING_MAX_PCT              90
#define RFX_BIG_GAMING_MAX_PCT          88
#define RFX_GAME_LAUNCH_FLOOR_PCT       65
#define RFX_BIG_INTERACTIVE_FLOOR_PCT   15
#define RFX_LITTLE_GAMING_CAP_PCT       85

/* === LIGHT MODE === */
#define RFX_LIGHT_ENTER_PCT     3
#define RFX_LIGHT_ENTER_TICKS   3
#define RFX_LIGHT_EXIT_PCT     12

/* === IDLE === */
#define RFX_IDLE_STALE_NS      (30 * NSEC_PER_MSEC)
#define RFX_IDLE_HYSTERESIS_NS (25 * NSEC_PER_MSEC)

/* === FREQ FLOORS & CAPS === */
#define RFX_LITTLE_MAX_NON_GAMING_KHZ  960000
#define RFX_FG_LIGHT_BIG_CAP_KHZ      500000

#define RFX_INTERACTIVE_UTIL_PCT      1
#define RFX_INTERACTIVE_FLOOR_KHZ    300000
#define RFX_BIG_INTERACTIVE_FLOOR_KHZ  600000

#define RFX_INTERACTIVE_DURATION_NS  (3000 * NSEC_PER_MSEC)
#define RFX_VIDEO_DETECT_THRESHOLD_NS (200 * NSEC_PER_MSEC)
#define RFX_IDLE_DEEP_CAP_KHZ_FALLBACK  300000

/* === CLUSTER THRESHOLDS === */
#define RFX_LITTLE_CAP_THRESHOLD    614
#define RFX_PRIME_CAP_THRESHOLD     1000
#define RFX_BIG_DROP_PCT            13

/* === ACTIVITY STATE === */
#define RFX_ACT_UP_TICKS    1
#define RFX_ACT_DOWN_TICKS  1

#define RFX_ACT_IDLE_TO_LIGHT_PCT  4
#define RFX_ACT_LIGHT_TO_MED_PCT  20
#define RFX_ACT_MED_TO_HEAVY_PCT  40
#define RFX_ACT_HEAVY_TO_MED_PCT  22
#define RFX_ACT_MED_TO_LIGHT_PCT   6
#define RFX_ACT_LIGHT_TO_IDLE_PCT  3

#define RFX_LITTLE_LIGHT_MAX_FREQ_KHZ   400000
#define RFX_LITTLE_MED_MAX_FREQ_KHZ     800000

/* === BUILD-TIME ASSERTIONS (preprocessor, scope file) === */
#if RFX_FLOOR_GATE_ARM_PCT <= RFX_FLOOR_GATE_RELEASE_PCT
#error "RFX_FLOOR_GATE_ARM_PCT must be > RFX_FLOOR_GATE_RELEASE_PCT"
#endif
#if RFX_DAILY_BIG_ARM_PCT <= RFX_DAILY_BIG_RELEASE_PCT
#error "RFX_DAILY_BIG_ARM_PCT must be > RFX_DAILY_BIG_RELEASE_PCT"
#endif
#if RFX_DAILY_LITTLE_ARM_PCT <= RFX_DAILY_LITTLE_RELEASE_PCT
#error "RFX_DAILY_LITTLE_ARM_PCT must be > RFX_DAILY_LITTLE_RELEASE_PCT"
#endif
#if RFX_THERMAL_COOL_ENTER_C <= RFX_THERMAL_COOL_EXIT_C
#error "RFX_THERMAL_COOL_ENTER_C must be > RFX_THERMAL_COOL_EXIT_C"
#endif
#if RFX_DAILY_LITTLE_CAP_PCT > 100
#error "RFX_DAILY_LITTLE_CAP_PCT must be <= 100"
#endif
#if RFX_HEADROOM_GAMING_GATE < 0 || RFX_HEADROOM_GAMING_GATE > 100
#error "RFX_HEADROOM_GAMING_GATE must be in 0..100"
#endif

/* === ADAPTIVE HELPERS === */
static inline unsigned int rfx_adaptive_max(struct cpufreq_policy *policy,
                                             unsigned int pct)
{
	if (!policy || !policy->cpuinfo.max_freq)
		return 0;
	return (unsigned int)((u64)policy->cpuinfo.max_freq * pct / 100);
}

static inline unsigned int rfx_adaptive_floor(struct cpufreq_policy *policy,
                                               unsigned int pct)
{
	unsigned int floor;
	if (!policy || !policy->cpuinfo.max_freq)
		return 0;
	floor = (unsigned int)((u64)policy->cpuinfo.max_freq * pct / 100);
	return max(floor, policy->cpuinfo.min_freq);
}

/* === DATA STRUCTURES === */
enum rfx_cluster_type {
	RFX_CLUSTER_LITTLE = 0,
	RFX_CLUSTER_BIG    = 1,
	RFX_CLUSTER_PRIME  = 2,
	RFX_CLUSTER_NONE   = 3,
};

enum rfx_activity_state {
	RFX_ACT_IDLE   = 0,
	RFX_ACT_LIGHT  = 1,
	RFX_ACT_MEDIUM = 2,
	RFX_ACT_HEAVY  = 3,
};

enum rfx_mode {
	RFX_MODE_NORMAL = 0,
	RFX_MODE_GAMING = 1,
	RFX_MODE_VIDEO  = 2,
};

static inline bool rfx_is_little(unsigned int cpu)
{
	return rfx_arch_cap(cpu) <= (unsigned long)RFX_LITTLE_CAP_THRESHOLD;
}

static inline bool rfx_is_prime(unsigned int cpu)
{
	return rfx_arch_cap(cpu) >= (unsigned long)RFX_PRIME_CAP_THRESHOLD;
}

/* === Util getter — didefinisikan di kernel/sched/cpufreq_schedutil.c === */
extern void rfx_get_util_k419(int cpu, unsigned long boost,
				unsigned long *out_util, unsigned long *out_bw_min);
extern bool rfx_dl_bw_exceeded_k419(int cpu, unsigned long bwmin);

static inline bool rfx_driver_test_flags(unsigned int flags) { return false; }
static inline bool rfx_scx_switched_all(void) { return false; }
static inline bool rfx_cpu_uclamp_capped(unsigned int cpu) { return false; }

static inline unsigned int rfx_get_ref_freq(struct cpufreq_policy *policy)
{
	if (!policy)
		return 0;
	return policy->cpuinfo.max_freq;
}

static inline struct gov_attr_set *rfx_to_gov_attr_set(struct kobject *kobj)
{
	return container_of(kobj, struct gov_attr_set, kobj);
}

struct rfx_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		rate_limit_us;
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;
	unsigned int		hispeed_window_us;
	unsigned int		hispeed_filter_shift;
	unsigned int		hispeed_boost_pct;
	enum rfx_cluster_type	cluster_type;
	unsigned int		gaming_mode;
};

struct rfx_policy;

static void rfx_deferred_update(struct rfx_policy *rfx_pol);
static void rfx_work(struct kthread_work *work);

struct rfx_policy {
	struct cpufreq_policy	*policy;
	struct rfx_tunables	*tunables;
	struct list_head	tunables_hook;
	raw_spinlock_t		update_lock;
	u64			last_upfreq_time;
	u64			last_downfreq_time;
	s64			freq_update_delay_ns;
	s64			up_rate_delay_ns;
	s64			down_rate_delay_ns;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;
	struct irq_work		irq_work;
	struct kthread_work	work;
	struct mutex		work_lock;
	struct kthread_worker	worker;
	struct task_struct	*thread;
	bool			work_in_progress;
	bool			limits_changed;
	bool			need_freq_update;
	u8			sustain_heavy_ticks;
	u8			sustain_exit_ticks;
	u8			light_enter_ticks;
	bool			in_heavy_mode;
	bool			in_light_mode;
	bool			prev_heavy_mode;
	u64			guard_end_ns;
	unsigned int		guard_freq_khz;
	u64			interactive_end_ns;
	bool			prime_gaming_floor_active;
	u64			prime_gaming_floor_end_ns;
	bool			force_idle;
	u64			force_idle_start_ns;
	u64			last_real_update_ns;
	enum rfx_mode		current_mode;
	u64			mode_switch_time_ns;
	u64			gaming_lock_end_ns;
	u64			idle_entry_time_ns;
	bool			in_deep_idle;
	u64			game_launch_end_ns;
	bool			game_launching;

	/* v2.2: warmup */
	u64			warmup_end_ns;

	/* v2.2: latched floor gate */
	bool			floor_gate_latched;

	/* v2.2: latched thermal cool */
	bool			thermal_cool_latched;

	/* v2.2: latched daily sustained */
	bool			daily_sustain_latched;

	/* v2.2: bounded slew */
	unsigned int		slew_last_freq;
	u64			slew_last_ns;

	/* Thermal zone */
	struct thermal_zone_device *tz;
	struct delayed_work thermal_work;
	int			thermal_temp;
	bool			thermal_work_active;

	/* Auto ROM detection */
	u8			rom_tweak_detected;
	bool			rom_override_active;
};

struct rfx_cpu {
	struct update_util_data	update_util;
	struct rfx_policy	*rfx_policy;
	unsigned int		cpu;
	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;
	unsigned long		util;
	unsigned long		bwmin;
	u64			prev_idle_time;
	u64			prev_wall_time;
	unsigned int		busy_pct;
	unsigned int		filtered_busy_pct;
	bool			hispeed_active;
	u64			hispeed_start_ns;
	unsigned int		hispeed_idle_windows;
	enum rfx_activity_state	act_state;
	unsigned int		act_up_ticks;
	unsigned int		act_down_ticks;
	unsigned int		prev_util_pct;
	unsigned int		util_history[8];
	u8			util_history_idx;
	unsigned int		ema_util;
	u64			ema_last_ns;
#ifdef CONFIG_NO_HZ_COMMON
	unsigned long		saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct rfx_cpu, rfx_cpu);

static inline struct rfx_tunables *to_rfx_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct rfx_tunables, attr_set);
}

/* === v2.2 DIRECTIONAL EMA (keep sub-period remainders, signed deltas) === */
static unsigned long rfx_ema_filter(struct rfx_cpu *rfx_c,
				    unsigned long util,
				    unsigned long max_cap,
				    bool gaming, u64 time)
{
	u64 dt;
	s64 dt_signed;

	if (!rfx_c->ema_last_ns) {
		rfx_c->ema_last_ns = time;
		rfx_c->ema_util = util;
		return util;
	}

	if (!gaming) {
		rfx_c->ema_util = util;
		rfx_c->ema_last_ns = time;
		return util;
	}

	if (util >= rfx_c->ema_util) {
		rfx_c->ema_util = util;
		rfx_c->ema_last_ns = time;
		return util;
	}

	dt_signed = (s64)(time - rfx_c->ema_last_ns);
	if (dt_signed < 0)
		dt_signed = 0;
	dt = (u64)dt_signed;

	if (dt >= RFX_EMA_PERIOD_NS) {
		u64 periods = dt / RFX_EMA_PERIOD_NS;
		u64 remainder = dt % RFX_EMA_PERIOD_NS;
		unsigned int ema = rfx_c->ema_util;
		u64 i;
		if (periods > 8)
			periods = 8;
		for (i = 0; i < periods; i++)
			ema -= (ema - util) >> RFX_EMA_DECAY_SHIFT_GAMING;
		rfx_c->ema_util = ema;
		/* keep remainder */
		rfx_c->ema_last_ns = time - remainder;
	}
	return rfx_c->ema_util;
}

/* === MODE DETECTION === */
static void rfx_detect_mode(struct rfx_policy *rfx_pol, struct rfx_cpu *rfx_c,
			     unsigned long util, unsigned long max_cap, u64 time)
{
	unsigned int util_pct = max_cap ? util * 100 / max_cap : 0;
	bool heavy_load = util_pct >= 45;
	bool medium_load = util_pct >= 18 && util_pct < 55;
	bool periodic_pattern = false;
	u64 time_in_mode;
	unsigned int variance = 0, avg = 0;
	int i;

	rfx_c->util_history[rfx_c->util_history_idx] = util_pct;
	rfx_c->util_history_idx = (rfx_c->util_history_idx + 1) & 0x7;

	if (medium_load) {
		for (i = 0; i < 8; i++)
			avg += rfx_c->util_history[i];
		avg /= 8;
		for (i = 0; i < 8; i++) {
			int diff = (int)rfx_c->util_history[i] - avg;
			variance += diff < 0 ? -diff : diff;
		}
		if (variance < 20 && avg >= 15 && avg <= 55)
			periodic_pattern = true;
	}

	if (rfx_pol->tunables->gaming_mode) {
		rfx_pol->current_mode       = RFX_MODE_GAMING;
		rfx_pol->in_light_mode      = false;
		rfx_pol->force_idle         = false;
		rfx_pol->sustain_exit_ticks = 0;

		if (util_pct >= 5) {
			rfx_pol->in_heavy_mode      = true;
			rfx_pol->gaming_lock_end_ns = time + RFX_GAMING_LOCK_DURATION_NS;
		} else if (rfx_pol->gaming_lock_end_ns &&
			   time < rfx_pol->gaming_lock_end_ns) {
			rfx_pol->in_heavy_mode      = true;
			rfx_pol->sustain_exit_ticks = 0;
		} else {
			if (!rfx_pol->gaming_lock_end_ns ||
			    (time - rfx_pol->gaming_lock_end_ns) > (6000 * NSEC_PER_MSEC))
				rfx_pol->in_heavy_mode = false;
		}
		return;
	}

	if (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns) {
		rfx_pol->current_mode = RFX_MODE_GAMING;
		return;
	}

	time_in_mode = time - rfx_pol->mode_switch_time_ns;

	if (heavy_load && rfx_c->act_state == RFX_ACT_HEAVY) {
		if (rfx_pol->current_mode != RFX_MODE_GAMING) {
			if (time_in_mode > 24 * NSEC_PER_MSEC) {
				rfx_pol->current_mode = RFX_MODE_GAMING;
				rfx_pol->mode_switch_time_ns = time;
				rfx_pol->gaming_lock_end_ns = time + RFX_GAMING_LOCK_DURATION_NS;
			}
		}
	} else if (periodic_pattern && !heavy_load) {
		if (rfx_pol->current_mode != RFX_MODE_GAMING &&
		    !(rfx_pol->gaming_lock_end_ns &&
		      time < rfx_pol->gaming_lock_end_ns) &&
		    rfx_pol->current_mode != RFX_MODE_VIDEO) {
			if (time_in_mode > RFX_VIDEO_DETECT_THRESHOLD_NS) {
				rfx_pol->current_mode = RFX_MODE_VIDEO;
				rfx_pol->mode_switch_time_ns = time;
			}
		}
	} else if (util_pct < 6 && rfx_pol->in_light_mode) {
		if (rfx_pol->current_mode != RFX_MODE_NORMAL) {
			if (time_in_mode > 32 * NSEC_PER_MSEC) {
				rfx_pol->current_mode = RFX_MODE_NORMAL;
				rfx_pol->mode_switch_time_ns = time;
				rfx_pol->gaming_lock_end_ns = 0;
			}
		}
	}
}

static inline unsigned int rfx_get_hispeed_pct(struct rfx_policy *rfx_pol)
{
	switch (rfx_pol->current_mode) {
	case RFX_MODE_GAMING: return RFX_HISPEED_GAMING_PCT;
	case RFX_MODE_VIDEO:  return RFX_HISPEED_VIDEO_PCT;
	default:              return RFX_HISPEED_DAILY_PCT;
	}
}

/* === v2.2 HEADROOM: linear ramp to +8% above gate === */
static unsigned long rfx_apply_headroom_gaming(unsigned long util,
						unsigned long max_cap)
{
	unsigned int util_pct;
	unsigned long extra;

	if (!max_cap)
		return util;
	util_pct = (unsigned int)(util * 100 / max_cap);
	if (util_pct <= RFX_HEADROOM_GAMING_GATE)
		return util;
	/* ramp: gate..100 maps to 0..8% */
	extra = (unsigned long)util * (util_pct - RFX_HEADROOM_GAMING_GATE) *
		RFX_HEADROOM_GAMING_MAX_PCT / ((100 - RFX_HEADROOM_GAMING_GATE) * 100);
	return min(util + extra, max_cap);
}

static unsigned long rfx_apply_headroom(unsigned long util,
					unsigned long max_cap,
					bool is_heavy,
					enum rfx_mode mode)
{
	unsigned int util_pct;

	if (!max_cap)
		return util;
	util_pct = (unsigned int)(util * 100 / max_cap);

	if (util_pct >= RFX_SAT_TO_MAX_PCT)
		return max_cap;

	if (mode == RFX_MODE_GAMING)
		return rfx_apply_headroom_gaming(util, max_cap);

	if (mode == RFX_MODE_VIDEO) {
		if (util_pct >= 50)
			return min(util + util * 15 / 100, max_cap);
		return min(util + util * 8 / 100, max_cap);
	}

	/* daily: conservative */
	if (util_pct >= 70)
		return min(util + util * 6 / 100, max_cap);
	if (util_pct >= 45)
		return min(util + util * 3 / 100, max_cap);
	return util;
}

/* === ACTIVITY STATE === */
static bool rfx_act_update(struct rfx_cpu *rfx_c, unsigned long effective_util,
			   unsigned long max_cap, u64 time,
			   unsigned int *freq_cap_khz)
{
	unsigned long idle_th, light_up_th, med_up_th;
	unsigned long heavy_dn_th, med_dn_th, light_dn_th;
	bool force_down = false;
	s64 stale_delta;
	struct rfx_policy *rfx_pol = rfx_c->rfx_policy;
	s64 time_since_last_update;
	bool is_little = (max_cap <= RFX_LITTLE_CAP_THRESHOLD);

	if (rfx_c->last_update) {
		time_since_last_update = (s64)(time - rfx_c->last_update);
		if (time_since_last_update >= (s64)RFX_IDLE_STALE_NS) {
			rfx_pol->force_idle = true;
			rfx_pol->force_idle_start_ns = time;
			rfx_pol->last_real_update_ns = time;
			if (!rfx_pol->in_deep_idle) {
				rfx_pol->idle_entry_time_ns = time;
				rfx_pol->in_deep_idle = true;
			}
		} else if (rfx_pol->force_idle) {
			if (effective_util > 0 || rfx_c->filtered_busy_pct > 0) {
				rfx_pol->force_idle = false;
				rfx_pol->in_deep_idle = false;
			}
		}
	}

	if (rfx_pol->in_heavy_mode ||
	    (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns)) {
		rfx_c->act_state      = RFX_ACT_HEAVY;
		rfx_c->act_up_ticks   = 0;
		rfx_c->act_down_ticks = 0;
		rfx_pol->force_idle = false;
		rfx_pol->in_deep_idle = false;
		rfx_pol->last_real_update_ns = time;
		*freq_cap_khz = 0;
		return false;
	}

	if (rfx_pol->in_light_mode || rfx_pol->force_idle) {
		if (is_little) {
			if (rfx_pol->force_idle) {
				*freq_cap_khz = rfx_pol->policy->cpuinfo.min_freq;
				return true;
			}
			*freq_cap_khz = RFX_LITTLE_LIGHT_MAX_FREQ_KHZ;
			return true;
		} else {
			*freq_cap_khz = RFX_FG_LIGHT_BIG_CAP_KHZ;
			return false;
		}
	}

	if (!is_little) {
		*freq_cap_khz = 0;
		return false;
	}

	if (rfx_c->last_update && !rfx_pol->force_idle) {
		stale_delta = (s64)(time - rfx_c->last_update);
		if (stale_delta >= (s64)RFX_IDLE_STALE_NS) {
			rfx_c->act_state            = RFX_ACT_IDLE;
			rfx_c->act_up_ticks         = 0;
			rfx_c->act_down_ticks       = 0;
			rfx_c->filtered_busy_pct    = 0;
			rfx_c->hispeed_start_ns     = 0;
			rfx_c->hispeed_idle_windows = 0;
			rfx_pol->force_idle         = true;
			rfx_pol->in_deep_idle       = true;
			rfx_pol->idle_entry_time_ns = time;
			*freq_cap_khz = rfx_pol->policy->cpuinfo.min_freq;
			return true;
		}
	}

	if (effective_util == 0) {
		switch (rfx_c->act_state) {
		case RFX_ACT_IDLE:
		case RFX_ACT_LIGHT:
			*freq_cap_khz = rfx_pol->policy->cpuinfo.min_freq;
			return true;
		case RFX_ACT_MEDIUM:
			*freq_cap_khz = RFX_LITTLE_MED_MAX_FREQ_KHZ;
			return true;
		default:
			*freq_cap_khz = 0;
			return false;
		}
	}

	rfx_pol->last_real_update_ns = time;
	rfx_pol->in_deep_idle = false;

	idle_th      = max_cap * RFX_ACT_IDLE_TO_LIGHT_PCT / 100;
	light_up_th  = max_cap * RFX_ACT_LIGHT_TO_MED_PCT  / 100;
	med_up_th    = max_cap * RFX_ACT_MED_TO_HEAVY_PCT  / 100;
	heavy_dn_th  = max_cap * RFX_ACT_HEAVY_TO_MED_PCT  / 100;
	med_dn_th    = max_cap * RFX_ACT_MED_TO_LIGHT_PCT  / 100;
	light_dn_th  = max_cap * RFX_ACT_LIGHT_TO_IDLE_PCT / 100;

	if (effective_util > med_up_th && rfx_c->act_state != RFX_ACT_HEAVY) {
		rfx_c->act_up_ticks++;
		if (rfx_c->act_up_ticks >= RFX_ACT_UP_TICKS) {
			rfx_c->act_state      = RFX_ACT_HEAVY;
			rfx_c->act_up_ticks   = 0;
			rfx_c->act_down_ticks = 0;
		}
		*freq_cap_khz = 0;
		return false;
	}

	switch (rfx_c->act_state) {
	case RFX_ACT_IDLE:
		if (effective_util > idle_th) {
			rfx_c->act_up_ticks++;
			if (rfx_c->act_up_ticks >= RFX_ACT_UP_TICKS) {
				rfx_c->act_state    = RFX_ACT_LIGHT;
				rfx_c->act_up_ticks = 0;
			}
		}
		*freq_cap_khz = rfx_pol->policy->cpuinfo.min_freq;
		break;
	case RFX_ACT_LIGHT:
		if (effective_util > light_up_th) {
			rfx_c->act_up_ticks++;
			if (rfx_c->act_up_ticks >= RFX_ACT_UP_TICKS) {
				rfx_c->act_state    = RFX_ACT_MEDIUM;
				rfx_c->act_up_ticks = 0;
			}
		} else if (effective_util < light_dn_th) {
			rfx_c->act_down_ticks++;
			if (rfx_c->act_down_ticks >= RFX_ACT_DOWN_TICKS) {
				rfx_c->act_state      = RFX_ACT_IDLE;
				rfx_c->act_down_ticks = 0;
				force_down = true;
			}
		}
		*freq_cap_khz = RFX_LITTLE_LIGHT_MAX_FREQ_KHZ;
		break;
	case RFX_ACT_MEDIUM:
		if (effective_util > med_up_th) {
			rfx_c->act_up_ticks++;
			if (rfx_c->act_up_ticks >= RFX_ACT_UP_TICKS) {
				rfx_c->act_state    = RFX_ACT_HEAVY;
				rfx_c->act_up_ticks = 0;
			}
		} else if (effective_util < med_dn_th) {
			rfx_c->act_down_ticks++;
			if (rfx_c->act_down_ticks >= RFX_ACT_DOWN_TICKS) {
				rfx_c->act_state      = RFX_ACT_LIGHT;
				rfx_c->act_down_ticks = 0;
			}
		}
		*freq_cap_khz = RFX_LITTLE_MED_MAX_FREQ_KHZ;
		break;
	case RFX_ACT_HEAVY:
		if (effective_util < heavy_dn_th) {
			rfx_c->act_down_ticks++;
			if (rfx_c->act_down_ticks >= RFX_ACT_DOWN_TICKS) {
				rfx_c->act_state      = RFX_ACT_MEDIUM;
				rfx_c->act_down_ticks = 0;
			}
		}
		force_down    = false;
		*freq_cap_khz = RFX_LITTLE_MAX_NON_GAMING_KHZ;
		break;
	}

	return force_down;
}

/* === v2.2 BOUNDED SLEW 0.5%/ms === */
static unsigned int rfx_bounded_slew(struct rfx_policy *rfx_pol,
				     unsigned int target_freq,
				     u64 time)
{
	unsigned int ceiling;
	u64 dt_ms;
	unsigned int max_drop_x2;
	unsigned int drop_x2;

	if (!rfx_pol->policy)
		return target_freq;

	if (rfx_pol->slew_last_ns == 0) {
		rfx_pol->slew_last_freq = target_freq;
		rfx_pol->slew_last_ns   = time;
		return target_freq;
	}

	if (target_freq >= rfx_pol->slew_last_freq) {
		rfx_pol->slew_last_freq = target_freq;
		rfx_pol->slew_last_ns   = time;
		return target_freq;
	}

	ceiling = rfx_pol->policy->cpuinfo.max_freq;
	dt_ms = (time - rfx_pol->slew_last_ns) / NSEC_PER_MSEC;
	if (dt_ms == 0)
		dt_ms = 1;

	/* 0.5% = ceiling / 200 per ms. max_drop_x2 = ceiling * 1% ... */
	max_drop_x2 = (unsigned int)((u64)ceiling * RFX_SLEW_MAX_PCT_X2_PER_MS / 100);
	drop_x2 = max_drop_x2 * (unsigned int)dt_ms;
	drop_x2 /= 2;   /* half of the 1% scale = 0.5% */

	if (rfx_pol->slew_last_freq > drop_x2 &&
	    (rfx_pol->slew_last_freq - target_freq) > drop_x2)
		target_freq = rfx_pol->slew_last_freq - drop_x2;

	if (target_freq < rfx_pol->policy->cpuinfo.min_freq)
		target_freq = rfx_pol->policy->cpuinfo.min_freq;

	rfx_pol->slew_last_freq = target_freq;
	rfx_pol->slew_last_ns   = time;
	return target_freq;
}

/* === THERMAL POLLER (v2.2) === */
static void rfx_thermal_poll(struct work_struct *work)
{
	struct rfx_policy *rfx_pol = container_of(to_delayed_work(work),
						  struct rfx_policy, thermal_work);
	int temp = 0, ret;

	if (!rfx_pol->tz) {
		/* no sensor: stop re-arming */
		rfx_pol->thermal_work_active = false;
		return;
	}

	ret = thermal_zone_get_temp(rfx_pol->tz, &temp);
	if (ret) {
		rfx_pol->thermal_work_active = false;
		return;
	}
	rfx_pol->thermal_temp = temp;

	if (!rfx_pol->thermal_cool_latched) {
		if (temp >= RFX_THERMAL_COOL_ENTER_C * 1000)
			rfx_pol->thermal_cool_latched = true;
	} else {
		if (temp <= RFX_THERMAL_COOL_EXIT_C * 1000)
			rfx_pol->thermal_cool_latched = false;
	}

	schedule_delayed_work(&rfx_pol->thermal_work,
			      msecs_to_jiffies(RFX_THERMAL_POLL_IDLE_MS));
}

/* === SHOULD UPDATE FREQ === */
static bool rfx_should_update_freq(struct rfx_policy *rfx_pol, u64 time)
{
	s64 delta_ns;
	s64 effective_delay;
	bool going_up;

	if (!rfx_pol || !rfx_pol->policy)
		return false;
	if (!cpufreq_this_cpu_can_update(rfx_pol->policy))
		return false;

	if (unlikely(READ_ONCE(rfx_pol->limits_changed))) {
		WRITE_ONCE(rfx_pol->limits_changed, false);
		rfx_pol->need_freq_update = true;
		smp_mb();
		return true;
	} else if (rfx_pol->need_freq_update) {
		return true;
	}

	going_up = (rfx_pol->next_freq > rfx_pol->policy->cur);

	if (rfx_pol->force_idle) {
		effective_delay = 3 * NSEC_PER_USEC;
	} else if (rfx_pol->current_mode == RFX_MODE_GAMING) {
		effective_delay = going_up ? 0 : 4000 * NSEC_PER_USEC;
	} else if (rfx_pol->current_mode == RFX_MODE_VIDEO) {
		effective_delay = 25 * NSEC_PER_USEC;
	} else {
		effective_delay = rfx_pol->freq_update_delay_ns;
	}

	if (going_up)
		delta_ns = time - rfx_pol->last_upfreq_time;
	else
		delta_ns = time - rfx_pol->last_downfreq_time;

	return delta_ns >= effective_delay;
}

static bool rfx_update_next_freq(struct rfx_policy *rfx_pol, u64 time,
				 unsigned int next_freq, bool force_down)
{
	if (!rfx_pol)
		return false;

	if (rfx_pol->need_freq_update) {
		rfx_pol->need_freq_update = false;
		if (rfx_pol->next_freq == next_freq &&
		    !rfx_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS))
			return false;
	} else if (rfx_pol->next_freq == next_freq &&
		   rfx_pol->last_upfreq_time == time) {
		return false;
	}

	if (next_freq < rfx_pol->next_freq) {
		if (!force_down) {
			s64 down_delta = time - rfx_pol->last_downfreq_time;
			s64 effective_down_delay = rfx_pol->down_rate_delay_ns;
			if (effective_down_delay > 0 &&
			    down_delta < effective_down_delay)
				return false;
			rfx_pol->last_downfreq_time = time;
		}
	} else {
		s64 up_delta = time - rfx_pol->last_upfreq_time;
		if (rfx_pol->up_rate_delay_ns > 0 &&
		    up_delta < rfx_pol->up_rate_delay_ns)
			return false;
		rfx_pol->last_upfreq_time = time;
	}

	rfx_pol->next_freq = next_freq;
	return true;
}

/* === v2.2 GAMING FLOOR / WARMUP === */
static void rfx_gaming_floors(struct rfx_policy *rfx_pol,
			       unsigned long util, unsigned long max,
			       unsigned int *freq, u64 time)
{
	unsigned int util_pct = max ? (unsigned int)(util * 100 / max) : 0;
	bool is_prime = (max >= (unsigned long)RFX_PRIME_CAP_THRESHOLD);
	bool is_little = (max <= (unsigned long)RFX_LITTLE_CAP_THRESHOLD);

	/* Floor gate deadband 25/35 */
	if (!rfx_pol->floor_gate_latched) {
		if (util_pct >= RFX_FLOOR_GATE_ARM_PCT)
			rfx_pol->floor_gate_latched = true;
	} else {
		if (util_pct <= RFX_FLOOR_GATE_RELEASE_PCT)
			rfx_pol->floor_gate_latched = false;
	}

	/* Warmup floor for 400ms after gaming_mode write */
	if (rfx_pol->warmup_end_ns && time < rfx_pol->warmup_end_ns) {
		unsigned int warm = rfx_adaptive_floor(rfx_pol->policy,
						       RFX_WARMUP_FLOOR_PCT);
		if (*freq < warm)
			*freq = warm;
	}

	if (!rfx_pol->floor_gate_latched)
		return;

	if (is_prime) {
		unsigned int fl = rfx_adaptive_floor(rfx_pol->policy,
						      RFX_PRIME_GAMING_FLOOR_PCT);
		if (*freq < fl) *freq = fl;
	} else if (!is_little) {
		unsigned int fl = rfx_adaptive_floor(rfx_pol->policy,
						      RFX_BIG_GAMING_FLOOR_PCT);
		if (*freq < fl) *freq = fl;
	} else {
		unsigned int fl = rfx_adaptive_floor(rfx_pol->policy,
						      RFX_LITTLE_GAMING_FLOOR_PCT);
		if (*freq < fl) *freq = fl;
	}
}

/* === v2.2 DAILY CAPS === */
static void rfx_daily_caps(struct rfx_policy *rfx_pol,
			    unsigned long util, unsigned long max,
			    unsigned int *freq)
{
	unsigned int util_pct = max ? (unsigned int)(util * 100 / max) : 0;
	bool is_prime = (max >= (unsigned long)RFX_PRIME_CAP_THRESHOLD);
	bool is_little = (max <= (unsigned long)RFX_LITTLE_CAP_THRESHOLD);
	unsigned int cap;

	if (is_prime || (!is_little)) {
		/* latched sustained-demand lift */
		if (!rfx_pol->daily_sustain_latched) {
			if (util_pct >= RFX_DAILY_BIG_ARM_PCT)
				rfx_pol->daily_sustain_latched = true;
		} else {
			if (util_pct <= RFX_DAILY_BIG_RELEASE_PCT)
				rfx_pol->daily_sustain_latched = false;
		}
		cap = rfx_pol->daily_sustain_latched
			? rfx_adaptive_max(rfx_pol->policy, RFX_DAILY_BIG_LIFT_PCT)
			: rfx_adaptive_max(rfx_pol->policy,
					   is_prime ? RFX_DAILY_PRIME_BASE_PCT
						    : RFX_DAILY_BIG_BASE_PCT);
		if (*freq > cap) *freq = cap;
	} else {
		/* little */
		if (!rfx_pol->daily_sustain_latched) {
			if (util_pct >= RFX_DAILY_LITTLE_ARM_PCT)
				rfx_pol->daily_sustain_latched = true;
		} else {
			if (util_pct <= RFX_DAILY_LITTLE_RELEASE_PCT)
				rfx_pol->daily_sustain_latched = false;
		}
		cap = rfx_pol->daily_sustain_latched
			? rfx_adaptive_max(rfx_pol->policy, RFX_DAILY_LITTLE_ARM_PCT)
			: rfx_adaptive_max(rfx_pol->policy, RFX_DAILY_LITTLE_CAP_PCT);
		if (*freq > cap) *freq = cap;
	}
}

/* === NEXT FREQ === */
static unsigned int rfx_get_next_freq(struct rfx_policy *rfx_pol,
				      unsigned long util, unsigned long max,
				      unsigned int freq_cap_khz, bool is_heavy,
				      u64 time)
{
	struct cpufreq_policy *policy = rfx_pol->policy;
	unsigned int freq;
	bool is_prime  = (max >= (unsigned long)RFX_PRIME_CAP_THRESHOLD);

	if (!policy)
		return 0;

	util = rfx_apply_headroom(util, max, is_heavy, rfx_pol->current_mode);
	freq = rfx_get_ref_freq(policy);
	freq = (unsigned int)((u64)freq * util / max);
	freq = clamp_t(unsigned int, freq,
		       policy->cpuinfo.min_freq, policy->cpuinfo.max_freq);

	if (rfx_pol->current_mode == RFX_MODE_GAMING) {
		rfx_gaming_floors(rfx_pol, util, max, &freq, time);
	} else {
		rfx_daily_caps(rfx_pol, util, max, &freq);
	}

	/* thermal cool relief: proportional to depth */
	if (rfx_pol->thermal_cool_latched) {
		unsigned int relief = rfx_adaptive_floor(policy,
							 RFX_THERMAL_COOL_RELIEF_PCT);
		unsigned int cap = rfx_adaptive_max(policy, 100);
		if (freq > relief)
			freq = relief + (freq - relief) / 2;
		if (freq > cap)
			freq = cap;
	}

	/* ROM override */
	if (rfx_pol->rom_override_active && is_prime &&
	    !rfx_pol->tunables->gaming_mode) {
		if (rfx_pol->rom_tweak_detected == 2) {
			unsigned int rom_cap = rfx_adaptive_max(policy, 88);
			if (!rfx_pol->thermal_cool_latched && freq > rom_cap)
				freq = rom_cap;
		}
	}

	if (rfx_pol->in_heavy_mode &&
	    rfx_pol->current_mode != RFX_MODE_GAMING && !is_prime) {
		unsigned int exit_util_pct = max ?
			(unsigned int)(util * 100 / max) : 0;
		if (exit_util_pct < 20) {
			unsigned int exit_soft_cap = rfx_adaptive_floor(policy,
				RFX_BIG_INTERACTIVE_FLOOR_PCT);
			if (freq > exit_soft_cap)
				freq = exit_soft_cap;
		}
	}

	if (rfx_pol->prime_gaming_floor_active &&
	    rfx_pol->prime_gaming_floor_end_ns &&
	    time < rfx_pol->prime_gaming_floor_end_ns) {
		unsigned int fl = rfx_adaptive_floor(policy,
						     RFX_PRIME_GAMING_FLOOR_PCT);
		if (freq < fl) freq = fl;
	}

	if (!rfx_pol->in_heavy_mode &&
	    rfx_pol->interactive_end_ns && time < rfx_pol->interactive_end_ns) {
		struct rfx_cpu *lc = per_cpu_ptr(&rfx_cpu,
				cpumask_first(rfx_pol->policy->cpus));
		if (max <= RFX_LITTLE_CAP_THRESHOLD) {
			if (lc->hispeed_start_ns && freq < RFX_INTERACTIVE_FLOOR_KHZ)
				freq = RFX_INTERACTIVE_FLOOR_KHZ;
		} else {
			if (lc->hispeed_start_ns &&
			    freq < rfx_adaptive_floor(policy, RFX_BIG_INTERACTIVE_FLOOR_PCT))
				freq = rfx_adaptive_floor(policy, RFX_BIG_INTERACTIVE_FLOOR_PCT);
		}
	}

	if (rfx_pol->force_idle && !is_heavy)
		freq = policy->cpuinfo.min_freq;

	if (freq_cap_khz > 0 && freq > freq_cap_khz)
		freq = freq_cap_khz;

	if (freq == rfx_pol->cached_raw_freq && !rfx_pol->need_freq_update)
		return rfx_pol->next_freq;

	rfx_pol->cached_raw_freq = freq;
	freq = rfx_bounded_slew(rfx_pol, freq, time);

	return cpufreq_driver_resolve_freq(policy, freq);
}

static void rfx_get_util(struct rfx_cpu *rfx_c, unsigned long boost)
{
	rfx_get_util_k419(rfx_c->cpu, boost, &rfx_c->util, &rfx_c->bwmin);
}

static bool rfx_big_drop_force_down(struct rfx_policy *rfx_pol,
				    unsigned int next_freq)
{
	unsigned int threshold;
	if (rfx_pol->next_freq == 0)
		return false;
	threshold = rfx_pol->next_freq * RFX_BIG_DROP_PCT / 100;
	return next_freq < threshold;
}

/* === ADAPTIVE MODE === */
static void rfx_update_adaptive_mode(struct rfx_policy *rfx_pol,
				     struct rfx_cpu *rfx_c,
				     unsigned long effective_util,
				     unsigned long max_cap,
				     bool is_big, u64 time)
{
	unsigned int util_pct;
	bool heavy_cond, light_cond;
	bool interactive_cond;
	bool is_prime = (max_cap >= (unsigned long)RFX_PRIME_CAP_THRESHOLD);
	s64 idle_time;

	util_pct = (max_cap > 0)
		 ? (unsigned int)(effective_util * 100 / max_cap) : 0;

	rfx_detect_mode(rfx_pol, rfx_c, effective_util, max_cap, time);
	idle_time = (s64)(time - rfx_pol->last_real_update_ns);

	if (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns) {
		rfx_pol->in_heavy_mode = true;
		rfx_pol->in_light_mode = false;
		rfx_pol->force_idle = false;
		rfx_pol->last_real_update_ns = time;
		return;
	}

	if (is_big) {
		heavy_cond = (util_pct >= RFX_SUSTAIN_HEAVY_ENTER_PCT)
			  && (rfx_c->filtered_busy_pct >= RFX_SUSTAIN_HEAVY_BUSY_PCT
			      || rfx_c->busy_pct >= 18);

		if (!rfx_pol->in_heavy_mode) {
			if (heavy_cond) {
				rfx_pol->sustain_heavy_ticks++;
				if (rfx_pol->sustain_heavy_ticks >= RFX_SUSTAIN_HEAVY_TICKS) {
					rfx_pol->in_heavy_mode       = true;
					rfx_pol->in_light_mode       = false;
					rfx_pol->force_idle          = false;
					rfx_pol->interactive_end_ns  = 0;
					rfx_pol->sustain_heavy_ticks = 0;
					rfx_pol->sustain_exit_ticks  = 0;
					rfx_pol->light_enter_ticks   = 0;
					if (is_prime) {
						rfx_pol->prime_gaming_floor_active  = true;
						rfx_pol->prime_gaming_floor_end_ns  = 0;
					}
					rfx_pol->gaming_lock_end_ns = time + RFX_GAMING_LOCK_DURATION_NS;
				}
			} else {
				rfx_pol->sustain_heavy_ticks = 0;
			}
		} else {
			if (rfx_pol->tunables->gaming_mode) {
				rfx_pol->sustain_exit_ticks = 0;
			} else if (util_pct < RFX_SUSTAIN_HEAVY_EXIT_PCT) {
				rfx_pol->sustain_exit_ticks++;
				if (rfx_pol->sustain_exit_ticks >= RFX_SUSTAIN_EXIT_TICKS) {
					rfx_pol->in_heavy_mode       = false;
					rfx_pol->sustain_exit_ticks  = 0;
					rfx_pol->sustain_heavy_ticks = 0;
					if (is_prime && rfx_pol->prime_gaming_floor_active) {
						rfx_pol->prime_gaming_floor_end_ns =
							time + (300 * NSEC_PER_MSEC);
					}
				}
			} else {
				rfx_pol->sustain_exit_ticks = 0;
			}
		}
	}

	if (rfx_pol->in_heavy_mode) {
		rfx_pol->light_enter_ticks = 0;
		rfx_pol->force_idle = false;
		rfx_pol->last_real_update_ns = time;
		return;
	}

	interactive_cond = (util_pct >= RFX_INTERACTIVE_UTIL_PCT);
	if (interactive_cond) {
		u64 interactive_dur = is_big
			? RFX_INTERACTIVE_DURATION_NS
			: (500 * NSEC_PER_MSEC);
		rfx_pol->interactive_end_ns = time + interactive_dur;
		if (rfx_pol->in_light_mode) {
			rfx_pol->in_light_mode     = false;
			rfx_pol->light_enter_ticks = 0;
			rfx_pol->force_idle = false;
		}
		rfx_pol->last_real_update_ns = time;
	}

	if (rfx_pol->interactive_end_ns && time < rfx_pol->interactive_end_ns) {
		rfx_pol->light_enter_ticks = 0;
		rfx_pol->last_real_update_ns = time;
		return;
	}

	if (is_prime && rfx_pol->prime_gaming_floor_active) {
		if (rfx_pol->prime_gaming_floor_end_ns &&
		    time >= rfx_pol->prime_gaming_floor_end_ns) {
			rfx_pol->prime_gaming_floor_active = false;
			rfx_pol->prime_gaming_floor_end_ns = 0;
		}
	}

	if (idle_time > 40 * NSEC_PER_MSEC &&
	    util_pct == 0 &&
	    rfx_c->filtered_busy_pct == 0 &&
	    !rfx_pol->in_light_mode &&
	    (!rfx_pol->interactive_end_ns || time >= rfx_pol->interactive_end_ns)) {
		rfx_pol->force_idle = true;
		rfx_pol->force_idle_start_ns = time;
	}

	light_cond = (util_pct <= RFX_LIGHT_ENTER_PCT)
		  && (rfx_c->filtered_busy_pct < 2)
		  && (rfx_c->act_state <= RFX_ACT_LIGHT)
		  && (rfx_c->hispeed_start_ns == 0)
		  && !rfx_pol->force_idle
		  && (!rfx_pol->interactive_end_ns || time >= rfx_pol->interactive_end_ns);

	if (!rfx_pol->in_light_mode) {
		if (light_cond) {
			rfx_pol->light_enter_ticks++;
			if (rfx_pol->light_enter_ticks >= RFX_LIGHT_ENTER_TICKS) {
				rfx_pol->in_light_mode      = true;
				rfx_pol->light_enter_ticks  = 0;
			}
		} else {
			rfx_pol->light_enter_ticks = 0;
		}
	} else {
		if (util_pct > RFX_LIGHT_EXIT_PCT || rfx_c->hispeed_start_ns != 0
		    || rfx_c->filtered_busy_pct >= 2
		    || rfx_c->act_state >= RFX_ACT_MEDIUM
		    || rfx_pol->force_idle
		    || (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns)) {
			rfx_pol->in_light_mode     = false;
			rfx_pol->light_enter_ticks = 0;
		}
	}
}

/* === BUSY % === */
static unsigned int rfx_get_adaptive_shift(unsigned long util,
					   unsigned long max_cap,
					   unsigned int base_shift)
{
	unsigned int util_pct;
	if (!max_cap)
		return base_shift > 0 ? min(base_shift + 3, 12U) : 0;
	util_pct = (unsigned int)(util * 100 / max_cap);
	if (util_pct > 90) return 0;
	if (util_pct < 5)   return base_shift > 0 ? min(base_shift + 3, 12U) : 0;
	return min(base_shift, 12U);
}

static void rfx_update_busy_pct(struct rfx_cpu *rfx_c, unsigned int window_us,
				unsigned int base_shift, unsigned long max_cap,
				u64 time)
{
	u64 cur_idle, cur_wall;
	u64 wall_delta, idle_delta;
	unsigned int filter_shift;
	bool is_prime = (max_cap >= (unsigned long)RFX_PRIME_CAP_THRESHOLD);
	bool interactive_active;
	unsigned int idle_win_threshold;
	unsigned int step;

	filter_shift = rfx_get_adaptive_shift(rfx_c->util, max_cap, base_shift);

	cur_idle   = get_cpu_idle_time(rfx_c->cpu, &cur_wall, 1);
	wall_delta = cur_wall - rfx_c->prev_wall_time;
	if (wall_delta < (u64)window_us)
		return;

	idle_delta = (cur_idle > rfx_c->prev_idle_time)
		     ? (cur_idle - rfx_c->prev_idle_time) : 0;
	rfx_c->busy_pct = (wall_delta > idle_delta)
		? (unsigned int)(100 * (wall_delta - idle_delta) / wall_delta)
		: 0;

	rfx_c->prev_idle_time = cur_idle;
	rfx_c->prev_wall_time = cur_wall;
	rfx_c->hispeed_active = true;

	if (rfx_c->busy_pct == 0) {
		rfx_c->filtered_busy_pct = 0;
	} else if (!filter_shift) {
		rfx_c->filtered_busy_pct = rfx_c->busy_pct;
	} else if (rfx_c->busy_pct > rfx_c->filtered_busy_pct + 4) {
		rfx_c->filtered_busy_pct = rfx_c->busy_pct;
	} else {
		step = (rfx_c->filtered_busy_pct > rfx_c->busy_pct)
			? (rfx_c->filtered_busy_pct - rfx_c->busy_pct) >> filter_shift
			: 0;
		if (step < rfx_c->filtered_busy_pct)
			rfx_c->filtered_busy_pct -= step;
		else
			rfx_c->filtered_busy_pct = rfx_c->busy_pct;
	}

	if (rfx_c->filtered_busy_pct > 0) {
		rfx_c->hispeed_idle_windows = 0;
		if (!rfx_c->hispeed_start_ns) {
			rfx_c->hispeed_start_ns = time;
			rfx_c->rfx_policy->need_freq_update = true;
		}
	} else {
		interactive_active = rfx_c->rfx_policy->interactive_end_ns &&
				  time < rfx_c->rfx_policy->interactive_end_ns;
		if (!rfx_c->rfx_policy->in_heavy_mode)
			rfx_c->hispeed_idle_windows++;
		idle_win_threshold = is_prime ? 5 : 3;
		if (!interactive_active &&
		    rfx_c->hispeed_idle_windows > idle_win_threshold) {
			rfx_c->hispeed_start_ns   = 0;
			rfx_c->filtered_busy_pct = 0;
		}
	}
}

static unsigned long rfx_blend_util(struct rfx_cpu *rfx_c,
				    unsigned long pelt_util,
				    unsigned long max_cap, u64 time,
				    unsigned int hispeed_boost_pct)
{
	unsigned long hispeed_util;
	unsigned int half_lives;
	unsigned int effective_pct;
	struct rfx_policy *pol = rfx_c->rfx_policy;
	unsigned long min_blend;

	if (!rfx_c->filtered_busy_pct || !rfx_c->hispeed_start_ns) {
		if (pol->current_mode == RFX_MODE_VIDEO &&
		    pol->interactive_end_ns &&
		    time < pol->interactive_end_ns &&
		    (max_cap <= RFX_LITTLE_CAP_THRESHOLD)) {
			min_blend = max_cap * 20 / 100;
			return (pelt_util > min_blend) ? pelt_util : min_blend;
		}
		return pelt_util;
	}

	effective_pct = min(rfx_c->filtered_busy_pct, hispeed_boost_pct);
	hispeed_util  = max_cap * effective_pct / 100;
	if (hispeed_util <= pelt_util)
		return pelt_util;

	half_lives = (unsigned int)((time - rfx_c->hispeed_start_ns) / HISPEED_HALFLIFE_NS);
	if (half_lives >= HISPEED_HALFLIFE_MAX) {
		rfx_c->hispeed_start_ns = time;
		return pelt_util;
	}
	return min(pelt_util + ((hispeed_util - pelt_util) >> half_lives), max_cap);
}

/* === IOWAIT === */
static bool rfx_iowait_reset(struct rfx_cpu *rfx_c, u64 time,
			     bool set_iowait_boost)
{
	s64 delta_ns = time - rfx_c->last_update;
	if (delta_ns <= TICK_NSEC)
		return false;
	rfx_c->iowait_boost         = set_iowait_boost ? IOWAIT_BOOST_MIN : 0;
	rfx_c->iowait_boost_pending = set_iowait_boost;
	return true;
}

static void rfx_iowait_boost(struct rfx_cpu *rfx_c, u64 time,
			     unsigned int flags)
{
	bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;
	unsigned long max_cap;
	unsigned int iowait_cap;
	struct rfx_policy *rfx_pol = rfx_c->rfx_policy;
	unsigned int ceil_pct;

	if (rfx_c->iowait_boost) {
		if (!rfx_iowait_reset(rfx_c, time, set_iowait_boost))
			rfx_c->iowait_boost_pending = set_iowait_boost;
		return;
	}
	if (!set_iowait_boost) return;
	if (rfx_c->iowait_boost_pending) return;

	rfx_c->iowait_boost_pending = true;

	max_cap = rfx_arch_cap(rfx_c->cpu);
	ceil_pct = (rfx_pol && rfx_pol->current_mode == RFX_MODE_GAMING)
		? RFX_IOWAIT_CEIL_GAMING_PCT
		: RFX_IOWAIT_CEIL_DAILY_PCT;

	iowait_cap = (unsigned int)((u64)SCHED_CAPACITY_SCALE * ceil_pct / 100);
	if (rfx_c->iowait_boost >= max_cap) {
		rfx_c->iowait_boost = min_t(unsigned int,
					    rfx_c->iowait_boost << 1,
					    iowait_cap);
		return;
	}
	rfx_c->iowait_boost = IOWAIT_BOOST_MIN;
}

static unsigned long rfx_iowait_apply(struct rfx_cpu *rfx_c, u64 time,
				      unsigned long max_cap)
{
	if (!rfx_c->iowait_boost)
		return 0;
	if (rfx_iowait_reset(rfx_c, time, false))
		return 0;
	if (!rfx_c->iowait_boost_pending) {
		rfx_c->iowait_boost >>= 1;
		if (rfx_c->iowait_boost < IOWAIT_BOOST_MIN) {
			rfx_c->iowait_boost = 0;
			return 0;
		}
	}
	rfx_c->iowait_boost_pending = false;
	return rfx_c->iowait_boost * max_cap >> SCHED_CAPACITY_SHIFT;
}

/* === NOHZ === */
#ifdef CONFIG_NO_HZ_COMMON
static bool rfx_check_freq_hold_or_drop(struct rfx_cpu *rfx_c,
					unsigned long max_cap,
					bool *out_force_drop)
{
	unsigned long idle_calls;
	bool idle_calls_increased;
	struct rfx_policy *pol = rfx_c->rfx_policy;
	u64 idle_duration;
	u64 now;

	if (rfx_scx_switched_all()) return false;
	if (rfx_cpu_uclamp_capped(rfx_c->cpu)) return false;

	if (pol->force_idle) {
		if (out_force_drop) *out_force_drop = true;
		return false;
	}

	if (pol->in_deep_idle && pol->idle_entry_time_ns) {
		now = ktime_get_ns();
		idle_duration = now - pol->idle_entry_time_ns;
		if (idle_duration < RFX_IDLE_HYSTERESIS_NS) {
			if (out_force_drop) *out_force_drop = false;
			return true;
		}
		pol->in_deep_idle = false;
		pol->idle_entry_time_ns = 0;
		if (out_force_drop) *out_force_drop = false;
		return false;
	}

	idle_calls = tick_nohz_get_idle_calls_cpu(rfx_c->cpu);
	idle_calls_increased = idle_calls != rfx_c->saved_idle_calls;
	rfx_c->saved_idle_calls = idle_calls;

	if (max_cap <= RFX_LITTLE_CAP_THRESHOLD) {
		if (out_force_drop) *out_force_drop = idle_calls_increased;
		return false;
	}
	if (out_force_drop) *out_force_drop = false;
	return !idle_calls_increased;
}
#else
static inline bool rfx_check_freq_hold_or_drop(struct rfx_cpu *rfx_c,
						unsigned long max_cap,
						bool *out_force_drop)
{
	if (out_force_drop) *out_force_drop = false;
	return false;
}
#endif

static inline void rfx_ignore_dl_rate_limit(struct rfx_cpu *rfx_c)
{
	if (rfx_dl_bw_exceeded_k419(rfx_c->cpu, rfx_c->bwmin))
		rfx_c->rfx_policy->need_freq_update = true;
}

static void rfx_irq_work(struct irq_work *irq_work);

static void rfx_deferred_update(struct rfx_policy *rfx_pol)
{
	if (!rfx_pol->work_in_progress) {
		rfx_pol->work_in_progress = true;
		irq_work_queue(&rfx_pol->irq_work);
	}
}

/* === MAIN UPDATE (single) === */
static void rfx_update_single_freq(struct update_util_data *hook, u64 time,
				   unsigned int flags)
{
	struct rfx_cpu      *rfx_c   = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy   *rfx_pol = rfx_c->rfx_policy;
	struct rfx_tunables *tunables = rfx_pol->tunables;
	unsigned int         cached_freq = rfx_pol->cached_raw_freq;
	unsigned long        max_cap, boost, effective_util;
	unsigned int         next_f, freq_cap_khz = 0;
	bool                 force_down, act_force = false, nohz_drop = false;
	bool                 hold, is_heavy;
	unsigned int         cur_pct;
	unsigned int         gf;
	unsigned int         idle_cap;
	unsigned int         hispeed_pct;
	bool                 is_gaming;
	unsigned long        flags_irq;

	max_cap = rfx_arch_cap(rfx_c->cpu);

	raw_spin_lock_irqsave(&rfx_pol->update_lock, flags_irq);

	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);

	if (!rfx_should_update_freq(rfx_pol, time))
		goto out_unlock;

	boost          = rfx_iowait_apply(rfx_c, time, max_cap);
	rfx_get_util(rfx_c, boost);
	effective_util = max(rfx_c->util, boost);

	rfx_update_busy_pct(rfx_c, tunables->hispeed_window_us,
			    tunables->hispeed_filter_shift, max_cap, time);

	hispeed_pct = rfx_get_hispeed_pct(rfx_pol);
	effective_util = rfx_blend_util(rfx_c, effective_util, max_cap, time,
					hispeed_pct);

	is_gaming = (rfx_pol->current_mode == RFX_MODE_GAMING);
	effective_util = rfx_ema_filter(rfx_c, effective_util, max_cap,
					is_gaming, time);

	{
		bool is_big_cluster = (max_cap > RFX_LITTLE_CAP_THRESHOLD);
		rfx_update_adaptive_mode(rfx_pol, rfx_c, effective_util, max_cap,
					 is_big_cluster, time);
	}

	if (!rfx_pol->tunables->gaming_mode) {
		unsigned int cur_util_pct = max_cap ?
			(unsigned int)(effective_util * 100 / max_cap) : 0;
		if (rfx_c->act_state >= RFX_ACT_MEDIUM &&
		    rfx_c->prev_util_pct < 10 && cur_util_pct > 20)
			rfx_pol->interactive_end_ns = time + RFX_INTERACTIVE_DURATION_NS;
	}

	is_heavy = (rfx_c->act_state == RFX_ACT_HEAVY) || rfx_pol->in_heavy_mode ||
		   (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns);

	if (rfx_pol->interactive_end_ns && time < rfx_pol->interactive_end_ns)
		act_force = false;

	next_f = rfx_get_next_freq(rfx_pol, effective_util, max_cap,
				   freq_cap_khz, is_heavy, time);

	cur_pct = (max_cap > 0)
		? (unsigned int)(effective_util * 100 / max_cap) : 0;

	if (rfx_c->prev_util_pct > RFX_BURST_DROP_THRESHOLD &&
	    rfx_c->prev_util_pct > cur_pct &&
	    (rfx_c->prev_util_pct - cur_pct) >= RFX_BURST_DROP_THRESHOLD &&
	    (rfx_c->act_state == RFX_ACT_MEDIUM ||
	     rfx_c->act_state == RFX_ACT_HEAVY ||
	     (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns))) {
		if (!rfx_pol->guard_end_ns) {
			gf = rfx_pol->next_freq ? rfx_pol->next_freq : next_f;
			rfx_pol->guard_end_ns   = time + RFX_BURST_GUARD_NS;
			rfx_pol->guard_freq_khz = gf;
		}
	}
	rfx_c->prev_util_pct = cur_pct;

	if (rfx_pol->guard_end_ns && !rfx_pol->in_light_mode && !rfx_pol->force_idle) {
		if (time < rfx_pol->guard_end_ns) {
			if (next_f < rfx_pol->guard_freq_khz)
				next_f = rfx_pol->guard_freq_khz;
			act_force = false;
		} else {
			rfx_pol->guard_end_ns   = 0;
			rfx_pol->guard_freq_khz = 0;
		}
	}

	if (rfx_pol->interactive_end_ns && time >= rfx_pol->interactive_end_ns)
		rfx_pol->interactive_end_ns = 0;

	if ((rfx_pol->in_light_mode || rfx_pol->force_idle) && !rfx_pol->in_heavy_mode &&
	    !(rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns)) {
		idle_cap = rfx_pol->policy->cpuinfo.min_freq
			? rfx_pol->policy->cpuinfo.min_freq
			: RFX_IDLE_DEEP_CAP_KHZ_FALLBACK;
		rfx_pol->interactive_end_ns = 0;
		rfx_pol->guard_end_ns = 0;
		rfx_pol->guard_freq_khz = 0;
		if (next_f > idle_cap) next_f = idle_cap;
		act_force = true;
	}

	force_down = act_force || (rfx_pol->guard_end_ns == 0 &&
				   rfx_big_drop_force_down(rfx_pol, next_f));

	hold = rfx_check_freq_hold_or_drop(rfx_c, max_cap, &nohz_drop);
	force_down = force_down || nohz_drop;

	if (hold && next_f == rfx_pol->next_freq &&
	    !rfx_pol->need_freq_update && !force_down) {
		next_f = rfx_pol->next_freq;
		rfx_pol->cached_raw_freq = cached_freq;
	}

	if (max_cap <= (unsigned long)RFX_LITTLE_CAP_THRESHOLD) {
		if (rfx_pol->force_idle) {
			rfx_pol->down_rate_delay_ns = (s64)RFX_LITTLE_DOWN_LIGHT_US * NSEC_PER_USEC;
		} else {
			switch (rfx_c->act_state) {
			case RFX_ACT_HEAVY:
				rfx_pol->down_rate_delay_ns = (s64)RFX_LITTLE_DOWN_HEAVY_US * NSEC_PER_USEC;
				break;
			case RFX_ACT_MEDIUM:
				rfx_pol->down_rate_delay_ns = (s64)RFX_LITTLE_DOWN_MEDIUM_US * NSEC_PER_USEC;
				break;
			default:
				rfx_pol->down_rate_delay_ns = (s64)RFX_LITTLE_DOWN_LIGHT_US * NSEC_PER_USEC;
				break;
			}
		}
		if (rfx_pol->in_heavy_mode ||
		    (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns))
			rfx_pol->down_rate_delay_ns = (s64)RFX_LITTLE_DOWN_HEAVY_US * NSEC_PER_USEC;
	}

	if (max_cap <= (unsigned long)RFX_LITTLE_CAP_THRESHOLD) {
		if (rfx_pol->prev_heavy_mode && !rfx_pol->in_heavy_mode &&
		    !(rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns)) {
			if (rfx_c->act_state == RFX_ACT_HEAVY) {
				rfx_c->act_state      = RFX_ACT_MEDIUM;
				rfx_c->act_down_ticks = 0;
				rfx_c->act_up_ticks   = 0;
			}
			rfx_pol->guard_end_ns   = 0;
			rfx_pol->guard_freq_khz = 0;
			force_down = true;
		}
		rfx_pol->prev_heavy_mode = rfx_pol->in_heavy_mode;
	}

	if (!rfx_pol->prev_heavy_mode && rfx_pol->in_heavy_mode &&
	    !rfx_pol->game_launching &&
	    rfx_pol->current_mode == RFX_MODE_GAMING) {
		rfx_pol->game_launching     = true;
		rfx_pol->game_launch_end_ns = time + RFX_GAME_LAUNCH_BOOST_NS;
	}
	if (rfx_pol->game_launching && rfx_pol->game_launch_end_ns &&
	    time >= rfx_pol->game_launch_end_ns) {
		rfx_pol->game_launching     = false;
		rfx_pol->game_launch_end_ns = 0;
	}

	rfx_update_next_freq(rfx_pol, time, next_f, force_down);

	/* v2.2: fast-switch call inside the lock */
	if (rfx_pol->policy->fast_switch_enabled)
		cpufreq_driver_fast_switch(rfx_pol->policy, rfx_pol->next_freq);
	else
		rfx_deferred_update(rfx_pol);

out_unlock:
	raw_spin_unlock_irqrestore(&rfx_pol->update_lock, flags_irq);
}

/* === SHARED === */
static unsigned int rfx_next_freq_shared(struct rfx_cpu *rfx_c, u64 time,
					bool *force_down_out)
{
	struct rfx_policy   *rfx_pol  = rfx_c->rfx_policy;
	struct rfx_tunables *tunables = rfx_pol->tunables;
	struct cpufreq_policy *policy = rfx_pol->policy;
	unsigned long util = 0, max_cap;
	bool any_force_down = true;
	bool any_heavy = false;
	unsigned int j, next_f;
	unsigned int freq_cap_khz = 0, j_cap;
	unsigned int j_util_pct;
	struct rfx_cpu *lead;
	unsigned int sgf;
	unsigned int idle_cap;
	unsigned int hispeed_pct;

	max_cap = rfx_arch_cap(rfx_c->cpu);

	for_each_cpu(j, policy->cpus) {
		struct rfx_cpu *j_rfxc = per_cpu_ptr(&rfx_cpu, j);
		unsigned long j_boost, j_util;
		bool nohz_drop = false, j_force;

		j_boost = rfx_iowait_apply(j_rfxc, time, max_cap);
		rfx_get_util(j_rfxc, j_boost);
		j_util  = max(j_rfxc->util, j_boost);

		rfx_update_busy_pct(j_rfxc, tunables->hispeed_window_us,
				    tunables->hispeed_filter_shift, max_cap, time);

		hispeed_pct = rfx_get_hispeed_pct(rfx_pol);
		j_util  = rfx_blend_util(j_rfxc, j_util, max_cap, time,
					 hispeed_pct);

		{
			bool j_gaming = (rfx_pol->current_mode == RFX_MODE_GAMING);
			j_util = rfx_ema_filter(j_rfxc, j_util, max_cap, j_gaming, time);
		}

		{
			bool j_is_big = (max_cap > RFX_LITTLE_CAP_THRESHOLD);
			rfx_update_adaptive_mode(rfx_pol, j_rfxc, j_util, max_cap,
						 j_is_big, time);
		}

		j_force = rfx_act_update(j_rfxc, j_util, max_cap, time, &j_cap);
		rfx_check_freq_hold_or_drop(j_rfxc, max_cap, &nohz_drop);

		if (!j_force && !nohz_drop)
			any_force_down = false;

		if (j_rfxc->act_state == RFX_ACT_HEAVY || rfx_pol->in_heavy_mode ||
		    (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns))
			any_heavy = true;

		if (j_cap == 0)
			freq_cap_khz = 0;
		else if (freq_cap_khz == 0 || j_cap < freq_cap_khz)
			freq_cap_khz = j_cap;

		util = max(j_util, util);
	}

	next_f = rfx_get_next_freq(rfx_pol, util, max_cap, freq_cap_khz,
				   any_heavy, time);

	j_util_pct = (max_cap > 0)
		? (unsigned int)(util * 100 / max_cap) : 0;
	lead = per_cpu_ptr(&rfx_cpu, cpumask_first(policy->cpus));

	if (lead->prev_util_pct > RFX_BURST_DROP_THRESHOLD &&
	    lead->prev_util_pct > j_util_pct &&
	    (lead->prev_util_pct - j_util_pct) >= RFX_BURST_DROP_THRESHOLD &&
	    (any_heavy || rfx_pol->in_heavy_mode ||
	     (rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns))) {
		if (!rfx_pol->guard_end_ns) {
			sgf = rfx_pol->next_freq ? rfx_pol->next_freq : next_f;
			rfx_pol->guard_end_ns   = time + RFX_BURST_GUARD_NS;
			rfx_pol->guard_freq_khz = sgf;
		}
	}
	lead->prev_util_pct = j_util_pct;

	if (rfx_pol->guard_end_ns) {
		if (time < rfx_pol->guard_end_ns) {
			if (next_f < rfx_pol->guard_freq_khz)
				next_f = rfx_pol->guard_freq_khz;
			any_force_down = false;
		} else {
			rfx_pol->guard_end_ns   = 0;
			rfx_pol->guard_freq_khz = 0;
		}
	}

	if (rfx_pol->interactive_end_ns) {
		if (time < rfx_pol->interactive_end_ns)
			any_force_down = false;
		else
			rfx_pol->interactive_end_ns = 0;
	}

	if ((rfx_pol->in_light_mode || rfx_pol->force_idle) && !rfx_pol->in_heavy_mode &&
	    !(rfx_pol->gaming_lock_end_ns && time < rfx_pol->gaming_lock_end_ns)) {
		idle_cap = rfx_pol->policy->cpuinfo.min_freq
			? rfx_pol->policy->cpuinfo.min_freq
			: RFX_IDLE_DEEP_CAP_KHZ_FALLBACK;
		rfx_pol->interactive_end_ns = 0;
		rfx_pol->guard_end_ns = 0;
		rfx_pol->guard_freq_khz = 0;
		if (next_f > idle_cap) next_f = idle_cap;
		any_force_down = true;
	}

	if (force_down_out)
		*force_down_out = any_force_down ||
			(rfx_pol->guard_end_ns == 0 &&
			 rfx_big_drop_force_down(rfx_pol, next_f));

	return next_f;
}

static void rfx_update_shared(struct update_util_data *hook, u64 time,
			      unsigned int flags)
{
	struct rfx_cpu    *rfx_c  = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *rfx_pol = rfx_c->rfx_policy;
	unsigned int       next_f;
	bool               force_down = false;

	raw_spin_lock(&rfx_pol->update_lock);

	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);

	if (rfx_should_update_freq(rfx_pol, time)) {
		next_f = rfx_next_freq_shared(rfx_c, time, &force_down);
		rfx_update_next_freq(rfx_pol, time, next_f, force_down);

		if (rfx_pol->policy->fast_switch_enabled)
			cpufreq_driver_fast_switch(rfx_pol->policy, rfx_pol->next_freq);
		else
			rfx_deferred_update(rfx_pol);
	}

	raw_spin_unlock(&rfx_pol->update_lock);
}

/* === WORKER === */
static void rfx_work(struct kthread_work *work)
{
	struct rfx_policy *rfx_pol = container_of(work, struct rfx_policy, work);
	unsigned int  freq;
	unsigned long flags;

	raw_spin_lock_irqsave(&rfx_pol->update_lock, flags);
	freq = rfx_pol->next_freq;
	rfx_pol->work_in_progress = false;
	raw_spin_unlock_irqrestore(&rfx_pol->update_lock, flags);

	mutex_lock(&rfx_pol->work_lock);
	__cpufreq_driver_target(rfx_pol->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&rfx_pol->work_lock);
}

static void rfx_irq_work(struct irq_work *irq_work)
{
	struct rfx_policy *rfx_pol = container_of(irq_work, struct rfx_policy, irq_work);
	kthread_queue_work(&rfx_pol->worker, &rfx_pol->work);
}

/* === SYSFS === */
static struct rfx_tunables *rfx_global_tunables;
static DEFINE_MUTEX(rfx_global_tunables_lock);

#define RFX_TUNABLE_UINT(name) \
	static ssize_t name##_show(struct gov_attr_set *attr_set, char *buf) \
	{ return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->name); } \
	static ssize_t name##_store(struct gov_attr_set *attr_set, \
				    const char *buf, size_t count) \
	{ struct rfx_tunables *t = to_rfx_tunables(attr_set); \
	  unsigned int val; if (kstrtouint(buf, 10, &val)) return -EINVAL; \
	  t->name = val; return count; } \
	static struct governor_attr name = __ATTR_RW(name)

static ssize_t rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{ return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->rate_limit_us); }
static ssize_t rate_limit_us_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct rfx_tunables *tunables = to_rfx_tunables(attr_set);
	struct rfx_policy *rfx_pol;
	unsigned int val;
	if (kstrtouint(buf, 10, &val)) return -EINVAL;
	tunables->rate_limit_us = val;
	list_for_each_entry(rfx_pol, &attr_set->policy_list, tunables_hook)
		rfx_pol->freq_update_delay_ns = val * NSEC_PER_USEC;
	return count;
}
static struct governor_attr rate_limit_us =
	__ATTR(rate_limit_us, 0644, rate_limit_us_show, rate_limit_us_store);

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{ return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->up_rate_limit_us); }
static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct rfx_tunables *tunables = to_rfx_tunables(attr_set);
	struct rfx_policy *rfx_pol;
	unsigned int val;
	if (kstrtouint(buf, 10, &val)) return -EINVAL;
	tunables->up_rate_limit_us = val;
	list_for_each_entry(rfx_pol, &attr_set->policy_list, tunables_hook)
		rfx_pol->up_rate_delay_ns = (s64)val * NSEC_PER_USEC;
	return count;
}
static struct governor_attr up_rate_limit_us =
	__ATTR(up_rate_limit_us, 0644, up_rate_limit_us_show, up_rate_limit_us_store);

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{ return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->down_rate_limit_us); }
static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct rfx_tunables *tunables = to_rfx_tunables(attr_set);
	struct rfx_policy *rfx_pol;
	unsigned int val;
	if (kstrtouint(buf, 10, &val)) return -EINVAL;
	tunables->down_rate_limit_us = val;
	list_for_each_entry(rfx_pol, &attr_set->policy_list, tunables_hook)
		rfx_pol->down_rate_delay_ns = (s64)val * NSEC_PER_USEC;
	return count;
}
static struct governor_attr down_rate_limit_us =
	__ATTR(down_rate_limit_us, 0644, down_rate_limit_us_show, down_rate_limit_us_store);

static ssize_t hispeed_boost_pct_show(struct gov_attr_set *attr_set, char *buf)
{ return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->hispeed_boost_pct); }
static ssize_t hispeed_boost_pct_store(struct gov_attr_set *attr_set,
				       const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	unsigned int val;
	if (kstrtouint(buf, 10, &val)) return -EINVAL;
	if (val < 1 || val > 100) return -EINVAL;
	t->hispeed_boost_pct = val;
	return count;
}
static struct governor_attr hispeed_boost_pct =
	__ATTR(hispeed_boost_pct, 0644, hispeed_boost_pct_show, hispeed_boost_pct_store);

static ssize_t gaming_mode_show(struct gov_attr_set *attr_set, char *buf)
{ return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->gaming_mode); }
static ssize_t gaming_mode_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	struct rfx_policy *rfx_pol;
	unsigned int val;
	u64 now = ktime_get_ns();

	if (kstrtouint(buf, 10, &val)) return -EINVAL;
	if (val > 1) return -EINVAL;

	t->gaming_mode = val;

	list_for_each_entry(rfx_pol, &attr_set->policy_list, tunables_hook) {
		/* v2.2: one helper resets every latch on mode switch */
		rfx_pol->gaming_lock_end_ns      = 0;
		rfx_pol->current_mode            = val ? RFX_MODE_GAMING : RFX_MODE_NORMAL;
		rfx_pol->in_heavy_mode           = false;
		rfx_pol->in_light_mode           = false;
		rfx_pol->force_idle              = false;
		rfx_pol->need_freq_update        = true;
		rfx_pol->mode_switch_time_ns     = now;
		rfx_pol->game_launching          = false;
		rfx_pol->game_launch_end_ns      = 0;
		rfx_pol->prime_gaming_floor_active = false;
		rfx_pol->prime_gaming_floor_end_ns = 0;
		rfx_pol->floor_gate_latched      = false;
		rfx_pol->daily_sustain_latched   = false;
		rfx_pol->thermal_cool_latched    = false;
		/* v2.2 warmup anchored to this write */
		rfx_pol->warmup_end_ns = val ? (now + RFX_WARMUP_DURATION_NS) : 0;
	}
	return count;
}
static struct governor_attr gaming_mode =
	__ATTR(gaming_mode, 0644, gaming_mode_show, gaming_mode_store);

RFX_TUNABLE_UINT(hispeed_window_us);
RFX_TUNABLE_UINT(hispeed_filter_shift);

static struct attribute *rfx_little_attrs[] = {
	&hispeed_boost_pct.attr,
	&rate_limit_us.attr,
	NULL
};

static struct attribute *rfx_big_attrs[] = {
	&down_rate_limit_us.attr,
	&hispeed_boost_pct.attr,
	&hispeed_filter_shift.attr,
	&rate_limit_us.attr,
	&gaming_mode.attr,
	NULL
};

static struct attribute *rfx_prime_attrs[] = {
	&hispeed_boost_pct.attr,
	&hispeed_filter_shift.attr,
	&hispeed_window_us.attr,
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&gaming_mode.attr,
	NULL
};

static void rfx_tunables_free(struct kobject *kobj)
{
	kfree(to_rfx_tunables(rfx_to_gov_attr_set(kobj)));
}

/* 4.19: .default_attrs, not .default_groups */
static struct kobj_type rfx_little_ktype = {
	.default_attrs  = rfx_little_attrs,
	.sysfs_ops      = &governor_sysfs_ops,
	.release        = rfx_tunables_free,
};

static struct kobj_type rfx_big_ktype = {
	.default_attrs  = rfx_big_attrs,
	.sysfs_ops      = &governor_sysfs_ops,
	.release        = rfx_tunables_free,
};

static struct kobj_type rfx_prime_ktype = {
	.default_attrs  = rfx_prime_attrs,
	.sysfs_ops      = &governor_sysfs_ops,
	.release        = rfx_tunables_free,
};

static struct cpufreq_governor vorpal_gov;

/* === POLICY ALLOC === */
static struct rfx_policy *rfx_policy_alloc(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol;
	rfx_pol = kzalloc(sizeof(*rfx_pol), GFP_KERNEL);
	if (!rfx_pol) return NULL;
	rfx_pol->policy = policy;
	raw_spin_lock_init(&rfx_pol->update_lock);
	return rfx_pol;
}

static void rfx_policy_free(struct rfx_policy *rfx_pol)
{
	kfree(rfx_pol);
}

/* === KTHREAD === */
static int rfx_kthread_create(struct rfx_policy *rfx_pol)
{
	struct task_struct *thread;
	struct cpufreq_policy *policy = rfx_pol->policy;
	int ret;

	/* v2.2: init irq_work & work_lock BEFORE the fast-switch return */
	kthread_init_work(&rfx_pol->work, rfx_work);
	kthread_init_worker(&rfx_pol->worker);
	init_irq_work(&rfx_pol->irq_work, rfx_irq_work);
	mutex_init(&rfx_pol->work_lock);

	if (policy->fast_switch_enabled)
		return 0;

	thread = kthread_create(kthread_worker_fn, &rfx_pol->worker,
				"rfx_gov/%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("vorpal: kthread create failed %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = rfx_setattr_sugov_k419(thread);   /* <-- pakai helper */
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_DEADLINE (sugov)\n", __func__);
		return ret;
	}

	rfx_pol->thread = thread;

	if (policy->dvfs_possible_from_any_cpu)
		set_cpus_allowed_ptr(thread, policy->related_cpus);
	else
		kthread_bind_mask(thread, policy->related_cpus);

	wake_up_process(thread);
	return 0;
}

static void rfx_kthread_stop(struct rfx_policy *rfx_pol)
{
	if (rfx_pol->policy->fast_switch_enabled)
		return;
	kthread_flush_worker(&rfx_pol->worker);
	kthread_stop(rfx_pol->thread);
	mutex_destroy(&rfx_pol->work_lock);
}

/* === TUNABLES === */
static struct rfx_tunables *rfx_tunables_alloc(struct rfx_policy *rfx_pol)
{
	struct rfx_tunables *tunables;
	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &rfx_pol->tunables_hook);
		if (!have_governor_per_policy())
			rfx_global_tunables = tunables;
	}
	return tunables;
}

static void rfx_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		rfx_global_tunables = NULL;
}

/* === ROM DETECT === */
static u8 rfx_detect_rom_tweak(void)
{
	int score = 0;
	if (vm_swappiness <= 5) score += 3;
	else if (vm_swappiness <= 15) score += 2;
	else if (vm_swappiness <= 30) score += 1;

	if (sysctl_sched_latency <= 1000000UL) score += 3;
	else if (sysctl_sched_latency <= 3000000UL) score += 2;
	else if (sysctl_sched_latency <= 5000000UL) score += 1;

	if (score >= 5) return 2;
	else if (score >= 2) return 1;
	return 0;
}

/* === INIT === */
static int rfx_init(struct cpufreq_policy *policy)
{
	struct rfx_policy   *rfx_pol;
	struct rfx_tunables *tunables;
	unsigned long        max_cap;
	int ret = 0;

	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	rfx_pol = rfx_policy_alloc(policy);
	if (!rfx_pol) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = rfx_kthread_create(rfx_pol);
	if (ret)
		goto free_rfx_pol;

	mutex_lock(&rfx_global_tunables_lock);

	if (rfx_global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = rfx_pol;
		rfx_pol->tunables = rfx_global_tunables;
		gov_attr_set_get(&rfx_global_tunables->attr_set, &rfx_pol->tunables_hook);

		rfx_pol->freq_update_delay_ns = (s64)rfx_global_tunables->rate_limit_us * NSEC_PER_USEC;
		rfx_pol->up_rate_delay_ns     = (s64)rfx_global_tunables->up_rate_limit_us * NSEC_PER_USEC;
		rfx_pol->down_rate_delay_ns   = (s64)rfx_global_tunables->down_rate_limit_us * NSEC_PER_USEC;
		goto out;
	}

	tunables = rfx_tunables_alloc(rfx_pol);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	tunables->hispeed_window_us    = CPUFREQ_VORPAL_DEFAULT_HISPEED_WINDOW_US;
	tunables->hispeed_filter_shift = CPUFREQ_VORPAL_DEFAULT_HISPEED_FILTER_SHIFT;
	tunables->hispeed_boost_pct    = CPUFREQ_VORPAL_DEFAULT_HISPEED_BOOST_PCT;
	tunables->gaming_mode          = 0;

	rfx_pol->rom_tweak_detected  = rfx_detect_rom_tweak();
	rfx_pol->rom_override_active = (rfx_pol->rom_tweak_detected > 0);
	if (rfx_pol->rom_override_active)
		pr_info("vorpal: ROM tweak detected (level %u)\n",
			rfx_pol->rom_tweak_detected);

	max_cap = rfx_arch_cap(cpumask_first(policy->cpus));

	if (max_cap <= (unsigned long)RFX_LITTLE_CAP_THRESHOLD) {
		tunables->cluster_type       = RFX_CLUSTER_LITTLE;
		tunables->rate_limit_us      = CPUFREQ_VORPAL_DEFAULT_RATE_LIMIT_US;
		tunables->up_rate_limit_us   = CPUFREQ_VORPAL_LITTLE_UP_RATE_LIMIT_US;
		tunables->down_rate_limit_us = CPUFREQ_VORPAL_LITTLE_DOWN_RATE_LIMIT_US;
	} else if (max_cap >= (unsigned long)RFX_PRIME_CAP_THRESHOLD) {
		tunables->cluster_type       = RFX_CLUSTER_PRIME;
		tunables->rate_limit_us      = CPUFREQ_VORPAL_PRIME_RATE_LIMIT_US;
		tunables->up_rate_limit_us   = CPUFREQ_VORPAL_PRIME_UP_RATE_LIMIT_US;
		tunables->down_rate_limit_us = CPUFREQ_VORPAL_PRIME_DOWN_RATE_LIMIT_US;
	} else {
		tunables->cluster_type       = RFX_CLUSTER_BIG;
		tunables->rate_limit_us      = CPUFREQ_VORPAL_DEFAULT_RATE_LIMIT_US;
		tunables->up_rate_limit_us   = CPUFREQ_VORPAL_BIG_UP_RATE_LIMIT_US;
		tunables->down_rate_limit_us = CPUFREQ_VORPAL_BIG_DOWN_RATE_LIMIT_US;
	}

	policy->governor_data = rfx_pol;
	rfx_pol->tunables     = tunables;

	rfx_pol->freq_update_delay_ns = (s64)tunables->rate_limit_us * NSEC_PER_USEC;
	rfx_pol->up_rate_delay_ns     = (s64)tunables->up_rate_limit_us * NSEC_PER_USEC;
	rfx_pol->down_rate_delay_ns   = (s64)tunables->down_rate_limit_us * NSEC_PER_USEC;

	/* Thermal zone (optional) */
	rfx_pol->tz = thermal_zone_get_zone_by_name("cpu");
	if (!rfx_pol->tz)
		rfx_pol->tz = thermal_zone_get_zone_by_name("soc");
	if (rfx_pol->tz) {
		INIT_DELAYED_WORK(&rfx_pol->thermal_work, rfx_thermal_poll);
		schedule_delayed_work(&rfx_pol->thermal_work,
				      msecs_to_jiffies(RFX_THERMAL_POLL_ACTIVE_MS));
		rfx_pol->thermal_work_active = true;
	} else {
		pr_info("vorpal: no thermal zone, poller disabled\n");
	}

	{
		struct kobj_type *ktype;
		switch (tunables->cluster_type) {
		case RFX_CLUSTER_LITTLE: ktype = &rfx_little_ktype; break;
		case RFX_CLUSTER_PRIME:  ktype = &rfx_prime_ktype;  break;
		default:                 ktype = &rfx_big_ktype;    break;
		}
		ret = kobject_init_and_add(&tunables->attr_set.kobj, ktype,
					   get_governor_parent_kobj(policy),
					   "%s", vorpal_gov.name);
	}
	if (ret)
		goto fail;

out:
	mutex_unlock(&rfx_global_tunables_lock);
	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;
	rfx_clear_global_tunables();
stop_kthread:
	rfx_kthread_stop(rfx_pol);
	mutex_unlock(&rfx_global_tunables_lock);
free_rfx_pol:
	rfx_policy_free(rfx_pol);
disable_fast_switch:
	cpufreq_disable_fast_switch(policy);
	pr_err("vorpal: init failed error %d\n", ret);
	return ret;
}

/* === EXIT === */
static void rfx_exit(struct cpufreq_policy *policy)
{
	struct rfx_policy   *rfx_pol  = policy->governor_data;
	struct rfx_tunables *tunables = rfx_pol->tunables;
	unsigned int         count;

	if (rfx_pol->thermal_work_active) {
		cancel_delayed_work_sync(&rfx_pol->thermal_work);
		rfx_pol->thermal_work_active = false;
	}

	mutex_lock(&rfx_global_tunables_lock);
	count = gov_attr_set_put(&tunables->attr_set, &rfx_pol->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		rfx_clear_global_tunables();
	mutex_unlock(&rfx_global_tunables_lock);

	rfx_kthread_stop(rfx_pol);
	rfx_policy_free(rfx_pol);
	cpufreq_disable_fast_switch(policy);
}

/* === START === */
static int rfx_start(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = policy->governor_data;
	void (*uu)(struct update_util_data *data, u64 time, unsigned int flags);
	unsigned int cpu;
	u64 now;
	int i;

	rfx_pol->freq_update_delay_ns = (s64)rfx_pol->tunables->rate_limit_us * NSEC_PER_USEC;
	rfx_pol->up_rate_delay_ns     = (s64)rfx_pol->tunables->up_rate_limit_us * NSEC_PER_USEC;
	rfx_pol->down_rate_delay_ns   = (s64)rfx_pol->tunables->down_rate_limit_us * NSEC_PER_USEC;

	now = ktime_get_ns();
	rfx_pol->last_upfreq_time    = now;
	rfx_pol->last_downfreq_time  = now;
	rfx_pol->next_freq           = policy->cur > 0 ? policy->cur : policy->cpuinfo.min_freq;
	rfx_pol->work_in_progress    = false;
	rfx_pol->limits_changed      = false;
	rfx_pol->cached_raw_freq     = 0;
	rfx_pol->need_freq_update    = false;
	rfx_pol->sustain_heavy_ticks = 0;
	rfx_pol->sustain_exit_ticks  = 0;
	rfx_pol->light_enter_ticks   = 0;
	rfx_pol->in_heavy_mode       = false;
	rfx_pol->in_light_mode       = false;
	rfx_pol->prev_heavy_mode     = false;
	rfx_pol->guard_end_ns        = 0;
	rfx_pol->guard_freq_khz      = 0;
	rfx_pol->interactive_end_ns  = 0;
	rfx_pol->prime_gaming_floor_active = false;
	rfx_pol->prime_gaming_floor_end_ns = 0;
	rfx_pol->force_idle          = false;
	rfx_pol->force_idle_start_ns = 0;
	rfx_pol->last_real_update_ns = now;
	rfx_pol->current_mode        = RFX_MODE_NORMAL;
	rfx_pol->mode_switch_time_ns = now;
	rfx_pol->gaming_lock_end_ns  = 0;
	rfx_pol->idle_entry_time_ns  = 0;
	rfx_pol->in_deep_idle        = false;
	rfx_pol->game_launch_end_ns  = 0;
	rfx_pol->game_launching      = false;
	/* v2.2 latches */
	rfx_pol->warmup_end_ns         = 0;
	rfx_pol->floor_gate_latched    = false;
	rfx_pol->thermal_cool_latched  = false;
	rfx_pol->daily_sustain_latched = false;
	rfx_pol->slew_last_freq        = 0;
	rfx_pol->slew_last_ns          = 0;

	uu = policy_is_shared(policy) ? rfx_update_shared : rfx_update_single_freq;

	for_each_cpu(cpu, policy->cpus) {
		struct rfx_cpu *rfx_c = per_cpu_ptr(&rfx_cpu, cpu);
		memset(rfx_c, 0, sizeof(*rfx_c));
		rfx_c->cpu        = cpu;
		rfx_c->rfx_policy = rfx_pol;
		rfx_c->act_state  = RFX_ACT_IDLE;
		rfx_c->prev_idle_time = get_cpu_idle_time(cpu, &rfx_c->prev_wall_time, 1);
		rfx_c->util_history_idx = 0;
		for (i = 0; i < 8; i++)
			rfx_c->util_history[i] = 0;
		cpufreq_add_update_util_hook(cpu, &rfx_c->update_util, uu);
	}
	return 0;
}

/* === STOP === */
static void rfx_stop(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&rfx_pol->irq_work);
		kthread_cancel_work_sync(&rfx_pol->work);
	}
}

/* === LIMITS === */
static void rfx_limits(struct cpufreq_policy *policy)
{
	struct rfx_policy *rfx_pol = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&rfx_pol->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&rfx_pol->work_lock);
	}

	smp_wmb();
	WRITE_ONCE(rfx_pol->limits_changed, true);
}

/* === GOVERNOR STRUCT === */
static struct cpufreq_governor vorpal_gov = {
	.name              = "vorpal",
	.owner             = THIS_MODULE,
	.dynamic_switching = true,   /* 4.19: bool */
	.init              = rfx_init,
	.exit              = rfx_exit,
	.start             = rfx_start,
	.stop              = rfx_stop,
	.limits            = rfx_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_VORPAL
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &vorpal_gov;
}
#endif

/* === MODULE === */
static int __init vorpal_gov_init(void)
{
	pr_info("%s %s by %s\n", CPUFREQ_VORPAL_PROGNAME,
		CPUFREQ_VORPAL_VERSION, CPUFREQ_VORPAL_AUTHOR);
	pr_info("v2.2: floors-only gaming | latched deadband | warmup | slow slew\n");
	return cpufreq_register_governor(&vorpal_gov);
}

static void __exit vorpal_gov_exit(void)
{
	cpufreq_unregister_governor(&vorpal_gov);
}

module_init(vorpal_gov_init);
module_exit(vorpal_gov_exit);

MODULE_AUTHOR(CPUFREQ_VORPAL_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(CPUFREQ_VORPAL_PROGNAME);
