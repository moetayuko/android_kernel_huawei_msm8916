/* arch/arm/mach-msm/memory.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/bootmem.h>
#include <linux/module.h>
#include <linux/memblock.h>
#include <asm/memblock.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/mach/map.h>
#include <asm/cacheflush.h>
#include <asm/setup.h>
#include <mach/msm_memtypes.h>
#include <mach/memory.h>
#include <linux/hardirq.h>
#include <mach/msm_iomap.h>
#include <soc/qcom/socinfo.h>
#include <linux/sched.h>
#include <linux/of_fdt.h>

#ifdef CONFIG_HUAWEI_KERNEL
extern unsigned int huawei_reserve_memory_size;
extern unsigned int huawei_reserve_memory_start;
#endif
char *memtype_name[] = {
	"EBI0",
	"EBI1"
};

static int __init check_for_compat(unsigned long node)
{
	char **start = __compat_exports_start;

	for ( ; start < __compat_exports_end; start++)
		if (of_flat_dt_is_compatible(node, *start))
			return 1;

	return 0;
}

int __init dt_scan_for_memory_reserve(unsigned long node, const char *uname,
		int depth, void *data)
{
	unsigned int *memory_remove_prop;
	unsigned long memory_remove_prop_length;
	unsigned int *memory_reserve_prop;
	unsigned long memory_reserve_prop_length;
	unsigned int memory_size;
	unsigned int memory_start;
	unsigned int num_holes = 0;
	int i;
	int ret;
#ifdef CONFIG_HUAWEI_KERNEL
	unsigned int *memory_reserve_huawei_prop;
	unsigned long memory_reserve_huawei_prop_length;
#endif

	memory_remove_prop = of_get_flat_dt_prop(node,
						"qcom,memblock-remove",
						&memory_remove_prop_length);

	memory_reserve_prop = of_get_flat_dt_prop(node,
						"qcom,memblock-reserve",
						&memory_reserve_prop_length);

#ifdef CONFIG_HUAWEI_KERNEL
	memory_reserve_huawei_prop = of_get_flat_dt_prop(node,
						 "huawei,memblock-remove-huawei",
						 &memory_reserve_huawei_prop_length);
#endif

#ifndef CONFIG_HUAWEI_KERNEL
	if (memory_remove_prop || memory_reserve_prop) {
#else
	if (memory_remove_prop || memory_reserve_prop ||memory_reserve_huawei_prop) {
#endif
		if (!check_for_compat(node))
			goto out;
	} else {
		goto out;
	}

	if (memory_remove_prop) {
		if (!memory_remove_prop_length || (memory_remove_prop_length %
				(2 * sizeof(unsigned int)) != 0)) {
			WARN(1, "Memory remove malformed\n");
			goto mem_reserve;
		}

		num_holes = memory_remove_prop_length /
					(2 * sizeof(unsigned int));

		for (i = 0; i < (num_holes * 2); i += 2) {
			memory_start = be32_to_cpu(memory_remove_prop[i]);
			memory_size = be32_to_cpu(memory_remove_prop[i+1]);

			ret = memblock_remove(memory_start, memory_size);
			if (ret)
				WARN(1, "Failed to remove memory %x-%x\n",
				memory_start, memory_start+memory_size);
			else
				pr_info("Node %s removed memory %x-%x\n", uname,
				memory_start, memory_start+memory_size);
		}
	}

mem_reserve:

	if (memory_reserve_prop) {
		if (memory_reserve_prop_length != (2*sizeof(unsigned int))) {
			WARN(1, "Memory reserve malformed\n");
#ifndef CONFIG_HUAWEI_KERNEL
			goto out;
#else
			goto huawei_remove_out;
#endif
		}

		memory_start = be32_to_cpu(memory_reserve_prop[0]);
		memory_size = be32_to_cpu(memory_reserve_prop[1]);

		ret = memblock_reserve(memory_start, memory_size);
		if (ret)
			WARN(1, "Failed to reserve memory %x-%x\n",
				memory_start, memory_start+memory_size);
		else
			pr_info("Node %s memblock_reserve memory %x-%x\n",
				uname, memory_start, memory_start+memory_size);
	}
#ifdef CONFIG_HUAWEI_KERNEL
huawei_remove_out:

	if (memory_reserve_huawei_prop) {
		if (memory_reserve_huawei_prop_length != (2*sizeof(unsigned int))) {
			WARN(1, "Huawei memory remove malformed\n");
			goto out;
		}

		memory_start = be32_to_cpu(memory_reserve_huawei_prop[0]);
		memory_size = be32_to_cpu(memory_reserve_huawei_prop[1]);

		pr_info("start = %08x, size = %08x\n", memory_start, memory_size);

		ret = memblock_reserve(memory_start, memory_size);
		if (ret)
			WARN(1, "Failed to reserve huawei memory %x-%x\n",
				memory_start, memory_start+memory_size);
		else {
			pr_info("Node %s reserve huawei memory %x-%x\n", uname,
				memory_start, memory_start+memory_size);

			huawei_reserve_memory_start = memory_start;
			huawei_reserve_memory_size = memory_size;
		}
	}
#endif

out:
	return 0;
}

/* Function to remove any meminfo blocks which are of size zero */
static void merge_meminfo(void)
{
	int i = 0;

	while (i < meminfo.nr_banks) {
		struct membank *bank = &meminfo.bank[i];

		if (bank->size == 0) {
			memmove(bank, bank + 1,
			(meminfo.nr_banks - i) * sizeof(*bank));
			meminfo.nr_banks--;
			continue;
		}
		i++;
	}
}

/*
 * Function to scan the device tree and adjust the meminfo table to
 * reflect the memory holes.
 */
int __init dt_scan_for_memory_hole(unsigned long node, const char *uname,
		int depth, void *data)
{
	unsigned int *memory_remove_prop;
	unsigned long memory_remove_prop_length;
	unsigned long hole_start;
	unsigned long hole_size;
	unsigned int num_holes = 0;
	int i = 0;
#ifdef CONFIG_HUAWEI_KERNEL
	unsigned int *memory_remove_huawei_prop;
	unsigned long memory_remove_huawei_prop_length;
#endif

	memory_remove_prop = of_get_flat_dt_prop(node,
						"qcom,memblock-remove",
						&memory_remove_prop_length);

#ifdef CONFIG_HUAWEI_KERNEL
	memory_remove_huawei_prop = of_get_flat_dt_prop(node,
						"huawei,memblock-remove-huawei",
						&memory_remove_huawei_prop_length);
#endif

#ifdef CONFIG_HUAWEI_KERNEL
	if (memory_remove_huawei_prop) {
		if (!check_for_compat(node))
			goto huawei_out;
	} else {
		goto huawei_out;
	}

	if (memory_remove_huawei_prop) {
		if (memory_remove_huawei_prop_length != (2*sizeof(unsigned int))) {
			WARN(1, "Huawei memory remove malformed\n");
			goto huawei_out;
		}

		hole_start = be32_to_cpu(memory_remove_huawei_prop[0]);
		hole_size = be32_to_cpu(memory_remove_huawei_prop[1]);

		adjust_meminfo(hole_start, hole_size);
	}

huawei_out:
#endif
	if (memory_remove_prop) {
		if (!check_for_compat(node))
			goto out;
	} else {
		goto out;
	}

	if (memory_remove_prop) {
		if (!memory_remove_prop_length || (memory_remove_prop_length %
			(2 * sizeof(unsigned int)) != 0)) {
			WARN(1, "Memory remove malformed\n");
			goto out;
		}

		num_holes = memory_remove_prop_length /
					(2 * sizeof(unsigned int));

		for (i = 0; i < (num_holes * 2); i += 2) {
			hole_start = be32_to_cpu(memory_remove_prop[i]);
			hole_size = be32_to_cpu(memory_remove_prop[i+1]);

			adjust_meminfo(hole_start, hole_size);
		}
	}

out:
	return 0;
}

/*
 * Split the memory bank to reflect the hole, if present,
 * using the start and end of the memory hole.
 */
void adjust_meminfo(unsigned long start, unsigned long size)
{
	int i;

	for (i = 0; i < meminfo.nr_banks; i++) {
		struct membank *bank = &meminfo.bank[i];

		if (((start + size) <= (bank->start + bank->size)) &&
			(start >= bank->start)) {
			memmove(bank + 1, bank,
				(meminfo.nr_banks - i) * sizeof(*bank));
			meminfo.nr_banks++;
			i++;

			bank->size = start - bank->start;
			bank[1].start = (start + size);
			bank[1].size -= (bank->size + size);
			bank[1].highmem = 0;
			merge_meminfo();
		}
	}
}

/* Provide a string that anonymous device tree allocations (those not
 * directly associated with any driver) can use for their "compatible"
 * field */
EXPORT_COMPAT("qcom,msm-contig-mem");
