#include <linux/module.h>
#include <linux/gfp.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <linux/iopoll.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>


struct msm_sim
{
    struct platform_device *pdev;
    struct timer_list  timer_detect;
    unsigned int irq;
    unsigned int status;
    unsigned int gpio_pin;

    struct pinctrl *pinctrl;
    struct pinctrl_state *pin_default;
    struct pinctrl_state *pin_sleep;
};
static struct wake_lock sim_wake_lock;
struct msm_sim *sim_host = NULL;
struct work_struct sim_work; 
struct workqueue_struct *sim_wq = NULL;



unsigned int get_sim_status(unsigned int sim_detection_gpio)
{
    int electrical_lvl;
	
    electrical_lvl = gpio_get_value(sim_detection_gpio);
    pr_info(" electrical_lvl = %d  \n", electrical_lvl);
    return electrical_lvl;
}

#define SIM_DETECT_COUNT    2

static void sim_detect_work(struct work_struct *work)
{
    static int detect_count = 0;
    unsigned int new_status;
    char *sim_on_duty[2]    = { "SIM_STATE=ON DUTY", NULL };
    char *sim_off_duty[2]    = { "SIM_STATE=OFF DUTY", NULL };
    
    new_status = get_sim_status(sim_host->gpio_pin);
    pr_info(KERN_INFO "sim card status change from %s to %s, detect_count = %d.\n",
           sim_host->status == 1 ? "on duty":"off duty",
           new_status== 1 ? "on duty":"off duty", detect_count);
    
    if((new_status != sim_host->status) && (detect_count < SIM_DETECT_COUNT - 1))
    {
        detect_count++;
        mod_timer(&sim_host->timer_detect, jiffies+HZ/4);
    }
    else if((new_status != sim_host->status) && (detect_count >= SIM_DETECT_COUNT - 1))
    {
        detect_count = 0;
        sim_host->status = new_status; 
        kobject_uevent_env(&sim_host->pdev->dev.kobj, KOBJ_CHANGE,( sim_host->status ? sim_on_duty : sim_off_duty));  
        pr_info("send sim state change uevent........................end\n");
        
        wake_unlock(&sim_wake_lock);
    }
    else
    {
        detect_count = 0;
        wake_unlock(&sim_wake_lock);
    }
}

static void timer_detect_func(unsigned long arg)
{
    pr_info("sim -%s[%d]: %s\n", __FILE__, __LINE__, __FUNCTION__);  
    queue_work(sim_wq, &sim_work);
}

irqreturn_t sim_gpio_irq_handler(int irq, void *dev_id)
{
    struct msm_sim *sim_host = (struct msm_sim *)dev_id;

    if (irq != sim_host->irq)
        return IRQ_NONE;
    
    wake_lock(&sim_wake_lock);
    
    mod_timer(&sim_host->timer_detect, jiffies+HZ/4);

    return IRQ_HANDLED;
}

 /* Move the code requesting irq to function probe */
static int msm_sim_gpio_init(void)
{
    int irq;
    int err = 0;
    enum   of_gpio_flags flags = OF_GPIO_ACTIVE_LOW;
 
    sim_host->gpio_pin = of_get_named_gpio_flags(sim_host->pdev->dev.of_node, "qcom,sim-detect", 0, &flags);
    if (!gpio_is_valid(sim_host->gpio_pin))
    {
        dev_err(&sim_host->pdev->dev, "%s: Failed to parser dt cd-gpio\n", __func__);
        goto get_gpio_err;
    }
   
    err = gpio_request_one(sim_host->gpio_pin, GPIOF_DIR_IN, "sim gpio");
    if (err)
        goto request_gpio_err;  
    
    irq = gpio_to_irq(sim_host->gpio_pin);
    if (irq < 0)
    {
        pr_err("msm_sim: probe gpio_to_irq  error!\n");
        goto  get_irq_err;
    }
    sim_host->irq = irq;

    init_timer(&sim_host->timer_detect);
    sim_host->timer_detect.function = &timer_detect_func;

    return 0;    
 
get_irq_err:
    gpio_free(sim_host->gpio_pin);  
request_gpio_err:
get_gpio_err:
    return err;
}

static ssize_t online_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{	
    return sprintf(buf, "%d\n", get_sim_status(sim_host->gpio_pin));
}

static DEVICE_ATTR(simonline, S_IRUGO  , online_show, NULL);

static struct attribute *dev_sim_attributes[] = {
	&dev_attr_simonline.attr,
		NULL
};

static const struct attribute_group dev_sim_attr_group= {
	.attrs = dev_sim_attributes,
};

static int sim_pinctrl_init(struct msm_sim *host)
{
    struct platform_device *pdev = host->pdev;

    host->pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR_OR_NULL(host->pinctrl)) {
        dev_err(&pdev->dev, "get pinctrl failed\n");
        return PTR_ERR(host->pinctrl);
    }

    host->pin_default = pinctrl_lookup_state(host->pinctrl,
            "sim_default");
    if (IS_ERR_OR_NULL(host->pin_default)) {
            dev_err(&pdev->dev,"lookup default state failed\n");
            return PTR_ERR(host->pin_default);
    }

    host->pin_sleep = pinctrl_lookup_state(host->pinctrl,
            "sim_sleep");
    if (IS_ERR_OR_NULL(host->pin_sleep)) {
            dev_err(&pdev->dev, "lookup sleep state failed\n");
            return PTR_ERR(host->pin_sleep);
    }

     return 0;
}

static int sim_msm_probe(struct platform_device  *pdev)
{
    int ret = 0;
    pr_info("sim_msm_probe START\n");  
    sim_host = devm_kzalloc(&pdev->dev, sizeof(struct msm_sim), GFP_KERNEL);
    if (!sim_host)
        return -ENOMEM;

    memset(sim_host, 0, sizeof(struct msm_sim));
    
    sim_host->pdev = pdev;
    
    ret = msm_sim_gpio_init();
    if(ret)
        goto err_sim_init;          

    ret = sim_pinctrl_init(sim_host);
    if(ret)
        goto err_sim_init;

    ret = pinctrl_select_state(sim_host->pinctrl, sim_host->pin_default);
    if(ret) {
        dev_err(&pdev->dev, "Can't select pinctrl default state\n");
        goto err_sim_init;
    }

    sim_host->status = get_sim_status(sim_host->gpio_pin);
    
    sim_wq = create_workqueue("sim_workqueue");
    if(!sim_wq)
        goto err_create_workqueue;
    
    INIT_WORK(&sim_work, sim_detect_work);
    wake_lock_init(&sim_wake_lock, WAKE_LOCK_SUSPEND, "sim detect"); 

	ret = sysfs_create_group(&pdev->dev.kobj, &dev_sim_attr_group);
    if(ret)
        goto err_create_file;

    ret = request_irq(sim_host->irq, sim_gpio_irq_handler,
                IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "sim cd", sim_host);
    if (ret < 0)
    {
        pr_err("request_irq  error!\n");
        goto request_irq_err;
    }
    
    return 0;

request_irq_err:
err_create_file:
    destroy_workqueue(sim_wq);
err_create_workqueue:
err_sim_init:
	wake_lock_destroy(&sim_wake_lock);
    kfree(sim_host);
    return ret;
		
}

static int sim_msm_remove(struct platform_device *pdev)
{
    if (gpio_is_valid(sim_host->gpio_pin))
        gpio_free(sim_host->gpio_pin);
    free_irq(sim_host->irq, NULL);
    destroy_workqueue(sim_wq);
    wake_lock_destroy(&sim_wake_lock);
    kfree(sim_host);

    return 0;
}

static const struct of_device_id sim_msm_dt_match[] = {
    { .compatible = "huawei,hw-sim-detect", },
    { },
};

static struct platform_driver sim_msm_driver = {
    .probe     = sim_msm_probe,
    .remove    = sim_msm_remove,
    .driver    = {
        .name  = "qcom,msm-sim",
	 .owner = THIS_MODULE,
	 .of_match_table = sim_msm_dt_match,
     },
};

static int __init msm_sim_init(void)
{
	
	return platform_driver_register(&sim_msm_driver);	
	
}

static void __exit msm_sim_exit(void)
{
	platform_driver_unregister(&sim_msm_driver);
}


module_init(msm_sim_init);
module_exit(msm_sim_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dev_sim");
