/* fs/proc/app_info.c
 *
 * Copyright (c) 2016 dianlujitao. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/fs.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

const void* dt_get_property_for_fac(char* key, int* pvalue_len)
{
    struct device_node *dp = NULL;

    if (key == NULL)
    {
        printk(KERN_ERR "param is NULL!\n");
        return NULL;
    }

    dp = of_find_node_by_path("/huawei_fac_info");
    if(!of_device_is_available(dp))
    {
        printk(KERN_ERR "device is not available!\n");
        return NULL;
    }

   return of_get_property(dp, key, pvalue_len);
}

static int app_info_proc_show(struct seq_file *m, void *v)
{
    int product_name_len = 0;
    const char* product_name = NULL;

    product_name = dt_get_property_for_fac("fac,product_name",&product_name_len);

    if(product_name == NULL)
    {
        printk(KERN_ERR "get product name fail!\n");
        return 1;
    }

    seq_printf(m, "%s\n", product_name);

    return 0;
}

static int app_info_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, app_info_proc_show, NULL);
}

static const struct file_operations app_info_proc_fops =
{
    .open		= app_info_proc_open,
    .read		= seq_read,
    .llseek		= seq_lseek,
    .release		= single_release,
};

static int __init proc_app_info_init(void)
{
    proc_create("app_info", 0, NULL, &app_info_proc_fops);

    return 0;
}

module_init(proc_app_info_init);
