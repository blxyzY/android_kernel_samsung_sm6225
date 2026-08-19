#include <linux/notifier.h>
#include <linux/mm.h>

static BLOCKING_NOTIFIER_HEAD(show_mem_extra_chain);

int show_mem_extra_notifier_register(struct notifier_block *nb)
{
    return blocking_notifier_chain_register(&show_mem_extra_chain, nb);
}
EXPORT_SYMBOL_GPL(show_mem_extra_notifier_register);

int show_mem_extra_notifier_unregister(struct notifier_block *nb)
{
    return blocking_notifier_chain_unregister(&show_mem_extra_chain, nb);
}
EXPORT_SYMBOL_GPL(show_mem_extra_notifier_unregister);

void show_mem_extra_call_notifiers(void)
{
    blocking_notifier_call_chain(&show_mem_extra_chain, 0, NULL);
}
EXPORT_SYMBOL_GPL(show_mem_extra_call_notifiers);

int show_mem_notifier_register(struct notifier_block *nb)
{
    return blocking_notifier_chain_register(&show_mem_extra_chain, nb);
}
EXPORT_SYMBOL_GPL(show_mem_notifier_register);

void show_mem_call_notifiers(void)
{
    blocking_notifier_call_chain(&show_mem_extra_chain, 0, NULL);
}
EXPORT_SYMBOL_GPL(show_mem_call_notifiers);

void mm_debug_dump_tasks(void)
{
    /* Dummy implementation */
    pr_info("mm_debug_dump_tasks: called\n");
}
EXPORT_SYMBOL_GPL(mm_debug_dump_tasks);
