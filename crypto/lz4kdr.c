// SPDX-License-Identifier: Dual BSD/GPL
/*
 * Crypto API wrapper for LZ4KDR (see include/linux/lz4kdr.h for what
 * it is and how it relates to Huawei's original LZ4KD).
 *
 * This is the integration point for zram trees that predate the
 * zcomp_ops "backend" abstraction (drivers/block/zram/backend_*.c) --
 * i.e. zram versions where the compressor is selected purely by name
 * through crypto_alloc_comp()/crypto_alloc_scomp() and there is no
 * per-algorithm backend file to add zram-side. Registering "lz4kdr"
 * here is the whole integration; nothing under drivers/block/zram/
 * needs to change. Select it exactly like any other zram compressor:
 *
 *   echo lz4kdr > /sys/block/zram0/comp_algorithm
 *
 * or make it the boot-time default by editing zram_drv.c's
 * default_compressor string, if your tree has one.
 *
 * Both the old CRYPTO_ALG_TYPE_COMPRESS ("struct crypto_alg" /
 * crypto_comp) interface and the newer SCOMP interface are registered
 * below, mirroring how in-tree crypto/lz4.c covers this same kernel
 * version range -- whichever one your zram's crypto_alloc_comp() ends
 * up resolving to, this module backs it. If your tree is new enough
 * to have dropped the old COMPRESS type entirely, just delete the
 * "old-style" section below and the crypto_register_alg()/
 * crypto_unregister_alg() calls in the init/exit functions; the SCOMP
 * half is self-contained.
 *
 * No dict/level support -- LZ4KDR doesn't have either concept.
 *
 * The per-tfm/per-ctx `ht` buffer is the hash table lz4kdr_encode()
 * searches with. It's kzalloc()'d once (not vmalloc()'d -- at
 * lz4kdr_encode_state_bytes_min()==2048 bytes it's well within
 * kmalloc's normal range) and, critically, the kzalloc zeroing IS the
 * one-time cold-start zero lz4kdr_encode()'s API contract requires --
 * lz4kdr does NOT re-zero this buffer on every compress call the way
 * upstream LZ4KD does, see lz4kdr_encode.c's probe-loop comment for
 * why that's still correct. Both allocation paths below (cra_init and
 * SCOMP's alloc_ctx) run once per tfm, not once per request, so this
 * holds either way.
 */

#include <linux/crypto.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/lz4kdr.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <crypto/internal/scompress.h>

/* ---- shared compress/decompress helpers ------------------------- */

/*
 * lz4kdr_encode() returning 0 means INCOMPRESSIBLE, not an error (see
 * include/linux/lz4kdr.h) -- the page won't beat *dlen. The crypto
 * compress API has no "store raw / try the next compressor" verdict
 * of its own, only success-with-a-length or failure, so report the
 * input back at its own length instead of failing outright.
 *
 * This mirrors zram's own huge/raw-store convention: zram_drv.c
 * treats a returned length >= huge_class_size as "store this page
 * raw, from the original uncompressed buffer, not from the
 * compressor's output" (see its zcomp_compress()/comp_len handling),
 * so the dst buffer's contents in this case are never actually read.
 * *dlen == slen (i.e. == PAGE_SIZE for a zram page) is exactly the
 * length that trips that path.
 *
 * Returning -EINVAL here instead (as a naive `ret <= 0` check would)
 * makes crypto_comp_compress() fail the whole write: zram_drv.c then
 * aborts the write with "Compression failed!" instead of storing the
 * page, so reclaim can't evict it. Under memory pressure that stalls
 * reclaim and can trigger the OOM killer the instant swapout hits its
 * first incompressible page -- worth avoiding even though it costs us
 * the ability to distinguish "genuinely incompressible" from "real
 * error" in this wrapper's return value.
 */
static int __lz4kdr_compress(const u8 *src, unsigned int slen,
                 u8 *dst, unsigned int *dlen, void *ht)
{
   int ret = lz4kdr_encode(ht, src, dst, slen, *dlen, 0);

   if (ret < 0)
       return -EINVAL;
   if (ret == 0) {
       *dlen = slen;
       return 0;
   }
   *dlen = ret;
   return 0;
}

static int __lz4kdr_decompress(const u8 *src, unsigned int slen,
               u8 *dst, unsigned int *dlen)
{
   int ret = lz4kdr_decode(src, dst, slen, *dlen);

   if (ret <= 0)
       return -EINVAL;
   *dlen = ret;
   return 0;
}

/* ---- old-style struct crypto_alg (CRYPTO_ALG_TYPE_COMPRESS) ----- */

struct lz4kdr_ctx {
   void *ht;
};

static int lz4kdr_init(struct crypto_tfm *tfm)
{
   struct lz4kdr_ctx *ctx = crypto_tfm_ctx(tfm);

   /* see the top-of-file comment: this kzalloc's zeroing is the
    * required one-time cold-start zero, not re-done per call. */
   ctx->ht = kzalloc(lz4kdr_encode_state_bytes_min(), GFP_KERNEL);
   if (!ctx->ht)
       return -ENOMEM;
   return 0;
}

static void lz4kdr_exit(struct crypto_tfm *tfm)
{
   struct lz4kdr_ctx *ctx = crypto_tfm_ctx(tfm);

   kfree(ctx->ht);
}

static int lz4kdr_compress_crypto(struct crypto_tfm *tfm, const u8 *src,
                  unsigned int slen, u8 *dst,
                  unsigned int *dlen)
{
   struct lz4kdr_ctx *ctx = crypto_tfm_ctx(tfm);

   return __lz4kdr_compress(src, slen, dst, dlen, ctx->ht);
}

static int lz4kdr_decompress_crypto(struct crypto_tfm *tfm, const u8 *src,
                    unsigned int slen, u8 *dst,
                    unsigned int *dlen)
{
   return __lz4kdr_decompress(src, slen, dst, dlen);
}

static struct crypto_alg alg = {
	.cra_name		= "lz4kdr",
	.cra_flags		= CRYPTO_ALG_TYPE_COMPRESS,
	.cra_ctxsize		= sizeof(struct lz4kdr_ctx),
	.cra_module		= THIS_MODULE,
	.cra_list		= LIST_HEAD_INIT(alg.cra_list),
	.cra_init		= lz4kdr_init,
	.cra_exit		= lz4kdr_exit,
	.cra_u			= { .compress = {
	.coa_compress		= lz4kdr_compress_crypto,
	.coa_decompress		= lz4kdr_decompress_crypto } }
};

/* ---- SCOMP interface ---- */

static void *lz4kdr_alloc_ctx(struct crypto_scomp *tfm)
{
   void *ht = kzalloc(lz4kdr_encode_state_bytes_min(), GFP_KERNEL);

   if (!ht)
       return ERR_PTR(-ENOMEM);
   return ht;
}

static void lz4kdr_free_ctx(struct crypto_scomp *tfm, void *ctx)
{
   kfree(ctx);
}

static int lz4kdr_scompress(struct crypto_scomp *tfm, const u8 *src,
                unsigned int slen, u8 *dst, unsigned int *dlen,
                void *ctx)
{
   return __lz4kdr_compress(src, slen, dst, dlen, ctx);
}

static int lz4kdr_sdecompress(struct crypto_scomp *tfm, const u8 *src,
                  unsigned int slen, u8 *dst, unsigned int *dlen,
                  void *ctx)
{
   return __lz4kdr_decompress(src, slen, dst, dlen);
}

static struct scomp_alg lz4kdr_scomp = {
   .alloc_ctx      = lz4kdr_alloc_ctx,
   .free_ctx       = lz4kdr_free_ctx,
   .compress       = lz4kdr_scompress,
   .decompress     = lz4kdr_sdecompress,
   .base           = {
       .cra_name       = "lz4kdr",
       .cra_driver_name    = "lz4kdr-scomp",
       .cra_module     = THIS_MODULE,
       .cra_priority       = 300,
   }
};

/* ---- module init/exit ---- */

static int __init lz4kdr_mod_init(void)
{
   int ret;

   ret = crypto_register_alg(&alg);
   if (ret)
       return ret;

   ret = crypto_register_scomp(&lz4kdr_scomp);
   if (ret) {
       crypto_unregister_alg(&alg);
       return ret;
   }

   return ret;
}

static void __exit lz4kdr_mod_fini(void)
{
   crypto_unregister_alg(&alg);
   crypto_unregister_scomp(&lz4kdr_scomp);
}

module_init(lz4kdr_mod_init);
module_exit(lz4kdr_mod_fini);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("LZ4KDR Compression Algorithm (speed-tuned LZ4KD derivative)");
MODULE_ALIAS_CRYPTO("lz4kdr");