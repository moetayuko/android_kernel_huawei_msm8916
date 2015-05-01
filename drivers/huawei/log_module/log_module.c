/*
 * log_module.c
 *
 * provide interface to other module to enable/disable log system.
 *
 * Copyright (C) 2012-2014  Huawei Corporation
 */


/*
 *  this module should provide such function
 *  for user space:
 *  1, interface to control the log and log level
 *  2, interface to control the module
 *  3,
 *
 *  for native module:
 *  1, method to add/remove dedicated module into log control
 *  2,
 */


/*
 *  TODOs:
 * 1, need to optimize the parm debug macro
 * 2, optimize the including file list.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/log_module.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

//#define SELF_DEBUG_SELF_ON

#ifdef SELF_DEBUG_SELF_ON
#undef DBG_P
#define DBG_P printk
#else
#undef DBG_P
#define DBG_P pr_debug
#endif

/* this is only for UT */
#define SELF_DEBUG

static long globle_device_number = 0;

static LIST_HEAD(hw_log_debug_mask_list);
static DEFINE_MUTEX(log_module_lock);

static int log_module_set_device_log(struct hw_device_debug * device_in);
static int log_module_get_devices(struct hw_device_debug * device_in, struct hw_device_debug * device_out);
static int log_module_count_devices(void);

#ifdef SELF_DEBUG
static struct hw_device_debug module1_example ={
    .device_name = "debug_mask_example",
    .category = HW_DEVICE_MODULE_LCD, /* LCD is one example */
    .id_num = 0,
};

static struct hw_device_debug module2_example ={
    .device_name = "dynamic_debug_example",
    .category = HW_DEVICE_MODULE_CAMERA, /* Camera is one example */
    .id_num = 1,
};

static ssize_t set_device_log_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
    int j = 0;
    int rc = 0;

    DBG_P("[%s] buf = [%s] , count = [%d]\n", __func__, buf, count);

    /* base is 10 */
    rc = kstrtoint(buf, 10, &j);

    if(1 == j)
    {
        log_module_set_device_log(NULL);
    }
    else if (2 == j)
    {
        log_module_set_device_log(&module1_example);
    }
    else
    {
        log_module_set_device_log(&module2_example);
    }

    return count;
}

static ssize_t get_num_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int rc_num = 0;
    rc_num = log_module_count_devices();
    DBG_P("get_num_store = [%d]\n", rc_num);
    return snprintf( buf, PAGE_SIZE, "%d\n", rc_num);
}

/* this test is reverse as the example modules registered. this can test the three cases
 * 1, first time, pass NULL in;
 * 2, get the next device;
 * 3, touch the list end
 */
static ssize_t  get_devices_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct hw_device_debug pdevice_in;
    struct hw_device_debug pdevice_out;

    DBG_P("[%s],[%d]\n", __func__, __LINE__);

    log_module_get_devices(NULL, &pdevice_in);

    DBG_P("get NULL device, and get [%s]\n", pdevice_in.device_name);

    log_module_get_devices(&pdevice_in, &pdevice_out);

    DBG_P("get next device, and get [%s]\n", pdevice_out.device_name);

    log_module_get_devices(&pdevice_out, &pdevice_in);

    DBG_P("get final device, and get [%s]\n", pdevice_in.device_name);

    return 1;
}

DEVICE_ATTR(set_device_log, S_IWUSR | S_IRUSR, NULL,set_device_log_store);
DEVICE_ATTR(get_num, S_IWUSR | S_IRUSR, get_num_show, NULL);
DEVICE_ATTR(get_devices, S_IWUSR | S_IRUSR, get_devices_show, NULL);

static struct attribute * lg_module_dev_attrs[] = {
    &dev_attr_set_device_log.attr,
    &dev_attr_get_num.attr,
    &dev_attr_get_devices.attr,
    NULL,
};


static struct attribute_group log_module_dev_attr_grp = {
            .attrs = lg_module_dev_attrs,
};

#endif
/* UT is END*/


/* use queue to add module */

int register_log_module(struct hw_device_debug * new_module)
{
    struct list_head * pos = NULL;
    struct hw_device_debug * device = NULL;


    /* check module validity , at least we need name and category*/
    if (NULL == new_module
        || new_module->category <= HW_DEVICE_MODULE_INVALID
        /* most time, we don't also regist all module */
        || new_module->category >= HW_DEVICE_MODULE_ALL)
    {
        pr_err("[%s] invalid parm\n", __func__);
        return -EINVAL;
    }

    DBG_P("[%s], [%d] = [%s]\n", __func__, __LINE__, new_module->device_name);

    /* to make atom ops */
    mutex_lock(&log_module_lock);

    /* check if conflict */
    list_for_each(pos, &hw_log_debug_mask_list)
    {
        device = list_entry(pos, struct hw_device_debug, list);
        /* if there is registered device_name */
        if(NULL == device)
        {
            /* ok, we didn't find the confict one, add it in  */
            DBG_P(" ok, we didn't find the confict one, add [%s] in the list !\n", new_module->device_name);
            break;
        }

        if(strncmp(device->device_name, new_module->device_name, MAX_MODULE_NAME_LEN))
        {
            DBG_P("[%s] didn't match [%s], continue to find other \n", new_module->device_name, device->device_name);
            continue;
        }
        else
        {
            DBG_P("Same name module! reject!\n");
            mutex_unlock(&log_module_lock);
            return -1;
        }

    }

    /* find the empty module position */
    new_module->id_num = globle_device_number;
    globle_device_number++;
    list_add_tail(&new_module->list, &hw_log_debug_mask_list);
    mutex_unlock(&log_module_lock);
    DBG_P("[%s], [%d] , register [%s], globle_device_number = [%lu]\n", __func__, __LINE__, new_module->device_name , globle_device_number);

    return 0;
}

EXPORT_SYMBOL(register_log_module);

void unregister_log_module(struct hw_device_debug *new_module)
{
    mutex_lock(&log_module_lock);
    list_del(&new_module->list);
    globle_device_number--;
    mutex_unlock(&log_module_lock);
}

EXPORT_SYMBOL(unregister_log_module);


/* we suppose that where will less that sizeof(int) module need us to */
static int log_module_count_devices(void)
{
    return globle_device_number;
}


/* this func will provide device array by
 *  repeatly called from user space
 *
 */
static int log_module_get_devices(struct hw_device_debug * device_in, struct hw_device_debug * device_out)
{
    struct hw_device_debug * device = NULL;
    int rc =0;

    if(NULL == device_in)
    {
        /* if cannot find , user want to get back the first device in the list */
        device = list_first_entry(&hw_log_debug_mask_list, struct hw_device_debug, list);
    }
    else
    {
        printk("we are looking for list = [%s]\n", device_in->device_name);
        /* if find we need to get the next device */
        device = list_first_entry(&device_in->list, struct hw_device_debug, list);
    }


    if(NULL != device)
    {
        printk("we get the device = [%s]\n", device->device_name);
        memcpy(device_out, device , sizeof(struct hw_device_debug));
        return rc;
    }
    else
    {
        printk("ok, we get no device\n");
        /* if list is NULL, just return NULL to user and notice no device yet */
        device_out = NULL;
        return -ENODEV;
    }

}

static struct hw_device_debug * log_module_find_device(struct hw_device_debug * device_in)
{
    struct hw_device_debug * device = NULL;
    struct list_head * pos;

    /* parm checking */
    if(NULL == device_in)
    {
        DBG_P("[%s] returned  at [%d] \n", __func__, __LINE__);
        return NULL;
    }

    DBG_P("we are looking for [%s], id = [%lu] \n", device_in->device_name, device_in->id_num);

    mutex_lock(&log_module_lock);

    /* poll each node ,  there is no optimize algorithm here so far. */
    list_for_each(pos, &hw_log_debug_mask_list)
    {
        device = list_entry(pos, struct hw_device_debug, list);

        if(NULL == device)
        {
            DBG_P("we reach the list end , and will break out \n");
            break;
        }
        DBG_P("we found [%s], id = [%lu]\n", device->device_name, device->id_num);

        /* we will use id_num to avoid dynamic module confict, name is not enough yet */
        if(!strncmp(device->device_name, device_in->device_name, MAX_MODULE_NAME_LEN)
            && (device->id_num == device_in->id_num))
        {
            DBG_P("ok, we finaly found [%s] == [%s], id = [%lu]\n", device->device_name, device_in->device_name, device_in->id_num);
            break;
        }
    }
    mutex_unlock(&log_module_lock);

    return device;
}


/*  Read func provide user space to get the parm. Just use for debug. */
static ssize_t log_module_read(struct file *fp, char __user *buf, size_t count, loff_t *pos)
{
    return -EOPNOTSUPP;
}

/* I don't want to provide write func, Zzz... */
static ssize_t log_module_write(struct file *fp, const char __user *buf, size_t count, loff_t *pos)
{
    return -EOPNOTSUPP;
}

/* Just set up some initialize. */
static int log_module_open(struct inode *ip, struct file *fp)
{
    /* check hw_log_debug_mask_list is empy or not */
    if(list_empty(&hw_log_debug_mask_list))
    {
        printk("list is empty\n");
        return -ENODEV;
    }
    return 0;
}

/* Could do nothing.. maybe some deinit */
static int log_module_release(struct inode *ip, struct file *fp)
{
    return -EOPNOTSUPP;
}

/* go through the queue to find the module and enable it */

static int log_module_set_device_log(struct hw_device_debug * device_in)
{
    struct hw_device_debug * device_local = NULL;
    int rc = 0;

    if(NULL == device_in)
    {
        DBG_P("[%s] returned [%d] \n", __func__, __LINE__);
        return -EINVAL;
    }

    /* find the device thru the name */
    device_local = log_module_find_device(device_in);

    if(NULL == device_local)
    {
        DBG_P("[%s] returned [%d] \n", __func__, __LINE__);
        return -EINVAL;
    }

    /*coverity: This less-than-zero comparison of an unsigned value is never true. "name < 0U".*/
    /* level checking  */
    if(device_local->log_level > HW_DEVICE_DEBUG_LEVEL_ALL)
    {
        printk("invalue level value, here we do nothing \n");
        return -EPERM;
    }

    /* we need to pass the log level of input here to alter the local one */
    if(device_local->hw_dynamic_debug_config)
      rc = device_local->hw_dynamic_debug_config(&device_in->log_level);
    /* we just allow one log methed, so we use else if here */
    else if(device_local->hw_debug_mask_config)
        rc = device_local->hw_debug_mask_config(&device_in->log_level);

    return rc;

}

/* main interface for user space.
 * func should provide is :
 * 1, set device(first of all): a)  open/close log; b) set level
 * 2, get module name list
 * 3, get module number.  we use queue, so we can support looooooot of modules.
 */
static long log_module_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    /* convert from user addr to the pointer */
    void __user * argp = (void __user *)arg;
    int rc = 0;
    struct hw_device_debug device_in;
    struct hw_device_debug  device_out;

    memset(&device_in, 0x0, sizeof(struct hw_device_debug));
    memset(&device_out, 0x0, sizeof(struct hw_device_debug));

    switch (cmd) {
        case LOG_M_SET_DEVICE_LOG:
            {
                /* get the parm */
                if(copy_from_user(&device_in, argp, sizeof(struct hw_device_debug)))
                {
                    return -EIO;
                }

                rc = log_module_set_device_log(&device_in);
            }
            break;

        case LOG_M_GET_MODULE_NUM:
            {
                /* this should be done before use any other ioctl  */
                rc = log_module_count_devices();

                if (copy_to_user(argp, &rc, sizeof(int))) {
                    return -EFAULT;
                }
            }
            break;

        case LOG_M_GET_MODULE:
        /* this should be done before enable/disable
          *  application level should know how much
          */
        {
            if(copy_from_user(&device_in, argp, sizeof(struct hw_device_debug)))
            {
                return -EIO;
            }

            rc = log_module_get_devices(&device_in, &device_out);

            if (copy_to_user(argp, &device_out, sizeof(struct hw_device_debug))) {
                return -EFAULT;
            }
        }
        break;
        case LOG_M_GET_MODULE_HEAD:
        /* this should be done before enable/disable
          *  application level should know how much
          */
        {
            rc = log_module_get_devices(NULL, &device_out);

            if (copy_to_user(argp, &device_out, sizeof(struct hw_device_debug))) {
                return -EFAULT;
            }
        }
        break;
        default:
            break;
    };

    return rc;

}

static const struct file_operations lg_module_fops = {
	.owner = THIS_MODULE,
	.read = log_module_read,
	.write = log_module_write,
	.open = log_module_open,
	.release = log_module_release,
	.unlocked_ioctl = log_module_ioctl,
};

static struct miscdevice lg_module_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lg_module_interface",
	.fops = &lg_module_fops,
};


static int __init hw_debug_log_init_module(void)
{
    int ret = -1;

    ret = misc_register(&lg_module_dev);

    if (ret < 0) {
        printk("%s: log_module_dev register failed %d\n", __func__, ret);
        return ret;
    }

#ifdef SELF_DEBUG
    ret = sysfs_create_group(&lg_module_dev.this_device->kobj, &log_module_dev_attr_grp);
#endif


    return 0;
}

static void __exit hw_debug_log_exit_module(void)
{
    int ret = -1;

    ret = misc_deregister(&lg_module_dev);
}

rootfs_initcall(hw_debug_log_init_module);
module_exit(hw_debug_log_exit_module);
MODULE_DESCRIPTION("Huawei log control module");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("huawei driver");
MODULE_VERSION("1.0");
