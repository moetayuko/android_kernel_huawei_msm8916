/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>

#include <linux/regulator/consumer.h>
#include <linux/miscdevice.h>
#include <mach/gpiomux.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/spinlock_types.h>
#include <linux/kthread.h>

#include <linux/qpnp/pwm.h>
#include "pn547.h"
#include <linux/wakelock.h>
#define MAX_BUFFER_SIZE	512

struct pn547_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct device		*dev;
	struct i2c_client	*client;
	struct miscdevice	pn547_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	unsigned int		irq_gpio;
	unsigned int		clk_req_gpio;
	bool				irq_enabled;
	spinlock_t			irq_enabled_lock;
	bool			do_reading;
	struct wake_lock   wl;
	bool cancel_read;
};

static void pn547_disable_irq(struct pn547_dev *pn547_dev)
{
	unsigned long flags;
//	pr_info("%s ++ \n", __func__);
	spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
	if (pn547_dev->irq_enabled) {
		disable_irq_nosync(pn547_dev->client->irq);
		pn547_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static void pn547_enable_irq(struct pn547_dev *pn547_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
	if (!pn547_dev->irq_enabled) {
		pn547_dev->irq_enabled = true;
		enable_irq(pn547_dev->client->irq);
	}
	spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn547_dev_irq_handler(int irq, void *dev_id)
{
	struct pn547_dev *pn547_dev = dev_id;
//	pr_info("%s ++ \n", __func__);

	if(gpio_get_value(pn547_dev->irq_gpio) != 1)
		return IRQ_HANDLED;

	pn547_disable_irq(pn547_dev);
	wake_lock_timeout(&pn547_dev->wl, 1 * HZ);

	pn547_dev->do_reading = 1;

	/* Wake up waiting readers */
	wake_up(&pn547_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn547_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev *pn547_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

//	pr_info("%s ++ \n", __func__);

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

//	pr_info("%s : read request for %zu bytes.\n", __func__, count);


    mutex_lock(&pn547_dev->read_mutex);

	if (!gpio_get_value(pn547_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}
		//pr_info("Waiting for PN547 IRQ.\n");

		pn547_dev->do_reading = 0;
		pn547_enable_irq(pn547_dev);
		
		ret = wait_event_interruptible(pn547_dev->read_wq,
				pn547_dev->do_reading);

		pn547_disable_irq(pn547_dev);
		//pr_info("PN547 IRQ high.\n");

		if (pn547_dev->cancel_read) {
			pn547_dev->cancel_read = false;
			ret = -1;
			goto fail;
		}

		if (ret)
			goto fail;

	}


	/* Read data */
	ret = i2c_master_recv(pn547_dev->client, tmp, count);
    mutex_unlock(&pn547_dev->read_mutex);

	//pr_info("%s : i2c read %zu bytes. status : %d\n", __func__, count, ret);

	if (ret < 0) {
		pr_err("%s: PN547 i2c_master_recv returned %d\n", __func__, ret);

		return ret;
	}

	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);

		return -EIO;
	}

	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);

		return -EFAULT;
	}

	return ret;

fail:
	mutex_unlock(&pn547_dev->read_mutex);
       pr_err("%s : goto fail, and ret : %d \n", __func__,ret);

	return ret;
}

static ssize_t pn547_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev  *pn547_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

//	pr_info("%s ++ \n", __func__);

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	//pr_info("%s : writing %zu bytes.\n", __func__, count);

	/* Write data */
	ret = i2c_master_send(pn547_dev->client, tmp, count);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}

	return ret;
}

static int pn547_dev_open(struct inode *inode, struct file *filp)
{
	struct pn547_dev *pn547_dev = container_of(filp->private_data,
						struct pn547_dev,
						pn547_device);

	filp->private_data = pn547_dev;
	pn547_enable_irq(pn547_dev);
	pr_err("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static long pn547_dev_ioctl(struct file *filp,
			    unsigned int cmd, unsigned long arg)
{
	struct pn547_dev *pn547_dev = filp->private_data;

	pr_info("%s ++    cmd = 0x%x \n", __func__,cmd);

	switch (cmd) {
	case PN544_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			pr_err("%s power on with firmware\n", __func__);
			gpio_set_value(pn547_dev->ven_gpio,0);
			gpio_set_value(pn547_dev->firm_gpio, 1);
			msleep(60);
			gpio_set_value(pn547_dev->ven_gpio, 0);
			msleep(60);
			gpio_set_value(pn547_dev->ven_gpio, 1);
			msleep(60);
		} else if (arg == 1) {
			/* power on */
			pr_err("%s power on\n", __func__);
			gpio_set_value(pn547_dev->firm_gpio, 0);
			gpio_set_value(pn547_dev->ven_gpio, 1);// 1
			irq_set_irq_wake(pn547_dev->client->irq,1);
			msleep(20);
		} else  if (arg == 0) {
			/* power off */
			pr_err("%s power off\n", __func__);
			gpio_set_value(pn547_dev->firm_gpio, 0);
			gpio_set_value(pn547_dev->ven_gpio, 0); //0
			irq_set_irq_wake(pn547_dev->client->irq,0);
			msleep(60);
		} else if (arg == 3) {
			pr_info("%s Read Cancel\n", __func__);
			pn547_dev->cancel_read = true;
			pn547_dev->do_reading = 1;
			wake_up(&pn547_dev->read_wq);
		} else {
			pr_err("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		pr_err("%s bad ioctl 0x%x\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn547_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn547_dev_read,
	.write	= pn547_dev_write,
	.open	= pn547_dev_open,
	.unlocked_ioctl	= pn547_dev_ioctl,
};

static int pn547_parse_dt(struct device *dev,
			 struct pn547_i2c_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
 	int ret = 0;

	pdata->irq_gpio =  of_get_named_gpio_flags(np, "nxp,nfc_int", 0,NULL);
	if (pdata->irq_gpio < 0) {
		pr_err( "failed to get \"huawei,nfc_int\"\n");
		goto err;
	}

	pdata->fwdl_en_gpio = of_get_named_gpio_flags(np, "nxp,nfc_firm", 0,NULL);
	if (pdata->fwdl_en_gpio< 0) {
		pr_err( "failed to get \"huawei,nfc_firm\"\n");
		goto err;
	}
	
	pdata->ven_gpio = of_get_named_gpio_flags(np, "nxp,nfc_ven", 0,NULL);
	if (pdata->ven_gpio < 0) {
		pr_err( "failed to get \"huawei,nfc_ven\"\n");
		goto err;
	}
	 
	pdata->clk_req_gpio = of_get_named_gpio(np, "nxp,nfc_clk", 0);
	if (pdata->clk_req_gpio < 0) {
		pr_err( "failed to get \"huawei,nfc_clk\"\n");
		goto err;
	}
	//printk("huawei,clk-req-gpio=%d\n",pdata->clk_req_gpio);
	pr_info("%s : huawei,clk-req-gpio=%d\n",__func__,pdata->clk_req_gpio);
err:
	return ret;
}

static struct gpiomux_setting nfc_irq_act = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting nfc_irq_sus = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting nfc_fwdl_act = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting nfc_fwdl_sus = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static int pn547_gpio_request(struct device *dev,
				struct pn547_i2c_platform_data *pdata)
{
	int ret;
//	int gpio_config=0;

	//printk("pn547_gpio_request enter\n");
	pr_info("%s : pn547_gpio_request enter\n", __func__);

	//NFC_INT
	ret = gpio_request(pdata->irq_gpio, "nfc_int");
	if(ret)
		goto err_irq;	
	msm_gpiomux_write(pdata->irq_gpio,
				GPIOMUX_ACTIVE, &nfc_irq_act, NULL);
	msm_gpiomux_write(pdata->irq_gpio,
				GPIOMUX_SUSPENDED, &nfc_irq_sus, NULL);
//	gpio_config = GPIO_CFG(pdata->irq_gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
//	gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
	ret = gpio_direction_input(pdata->irq_gpio);
	if(ret)
		goto err_fwdl_en;

	//NFC_FWDL
	ret = gpio_request(pdata->fwdl_en_gpio, "nfc_wake");
	if(ret)
		goto err_fwdl_en;
	msm_gpiomux_write(pdata->fwdl_en_gpio,
				GPIOMUX_ACTIVE, &nfc_fwdl_act, NULL);
	msm_gpiomux_write(pdata->fwdl_en_gpio,
				GPIOMUX_SUSPENDED, &nfc_fwdl_sus, NULL);
//	gpio_config = GPIO_CFG(pdata->fwdl_en_gpio, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
//	gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
	ret = gpio_direction_output(pdata->fwdl_en_gpio,0);
	if(ret)
		goto err_ven;

	//NFC_VEN
	ret=gpio_request(pdata->ven_gpio,"nfc_ven");
	if(ret)
		goto err_ven;
	ret = gpio_direction_output(pdata->ven_gpio, 0);
	if(ret)
		goto err_clk_req;

	//NFC_CLKReq
	ret=gpio_request(pdata->clk_req_gpio,"nfc_clk_req");
	if(ret)
		goto err_clk_req;
	ret = gpio_direction_input(pdata->clk_req_gpio);
	if(ret)
		goto err_clk_input;

	return 0;

err_clk_input:
	gpio_free(pdata->clk_req_gpio);
err_clk_req:
	gpio_free(pdata->ven_gpio);
err_ven:
	gpio_free(pdata->fwdl_en_gpio);
err_fwdl_en:
	gpio_free(pdata->irq_gpio);
err_irq:

	pr_err( "%s: gpio request err %d\n", __func__, ret);
	return ret;
}

static void pn547_gpio_release(struct pn547_i2c_platform_data *pdata)
{
	gpio_free(pdata->ven_gpio);
	gpio_free(pdata->irq_gpio);
	gpio_free(pdata->fwdl_en_gpio);
	gpio_free(pdata->clk_req_gpio);
}

static int pn547_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct clk *nfc_clk = NULL;
	struct pn547_i2c_platform_data *platform_data;
	struct pn547_dev *pn547_dev;

	dev_dbg(&client->dev, "%s begin:\n", __func__);
		
	platform_data = kzalloc(sizeof(struct pn547_i2c_platform_data),
				GFP_KERNEL);

	if (platform_data == NULL) {
		dev_err(&client->dev, "failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_platform_data;
	}

	ret = pn547_parse_dt(&client->dev, platform_data);
	if (ret < 0) {
		dev_err(&client->dev, "failed to parse device tree: %d\n", ret);
		goto err_parse_dt;
	}

	nfc_clk  = clk_get(&client->dev, "pn547_clk");
	if (nfc_clk == NULL) {
		dev_err(&client->dev, "failed to get clk: %d\n", ret);
		goto err_parse_dt;
	}
	clk_set_rate(nfc_clk,19200000);
	
	ret = clk_prepare_enable(nfc_clk);
	if (ret) {
		dev_err(&client->dev, "failed to enable clk: %d\n", ret);
		goto err_parse_dt;
	}


	ret = pn547_gpio_request(&client->dev, platform_data);
	if (ret) {
		dev_err(&client->dev, "failed to request gpio\n");
		goto err_gpio_request;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c check failed\n", __func__);
		ret = -ENODEV;
		goto err_i2c;
	}

	pn547_dev = kzalloc(sizeof(*pn547_dev), GFP_KERNEL);
	if (pn547_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}
       client->irq = gpio_to_irq(platform_data->irq_gpio);

	pn547_dev->irq_gpio = platform_data->irq_gpio;
	pn547_dev->ven_gpio  = platform_data->ven_gpio;
	pn547_dev->firm_gpio  = platform_data->fwdl_en_gpio;
	pn547_dev->clk_req_gpio = platform_data->clk_req_gpio;
	pn547_dev->client   = client;
	pn547_dev->dev = &client->dev;
	pn547_dev->do_reading = 0;

	/* Initialise mutex and work queue */
	init_waitqueue_head(&pn547_dev->read_wq);
	mutex_init(&pn547_dev->read_mutex);
	spin_lock_init(&pn547_dev->irq_enabled_lock);
       wake_lock_init(&pn547_dev->wl,WAKE_LOCK_SUSPEND,"nfc_locker");
	pn547_dev->pn547_device.minor = MISC_DYNAMIC_MINOR;
	pn547_dev->pn547_device.name = "pn544";
	pn547_dev->pn547_device.fops = &pn547_dev_fops;

	ret = misc_register(&pn547_dev->pn547_device);
	if (ret) {
		dev_err(&client->dev, "%s: misc_register err %d\n",
			__func__, ret);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	dev_info(&client->dev, "%s : requesting IRQ %d\n",
		__func__, client->irq);
	pn547_dev->irq_enabled = true;
	ret = request_irq(client->irq, pn547_dev_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND | IRQF_ONESHOT,
			client->name, pn547_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	pn547_disable_irq(pn547_dev);
	i2c_set_clientdata(client, pn547_dev);

	dev_dbg(&client->dev, "%s success.\n", __func__);
	return 0;

err_request_irq_failed:
	misc_deregister(&pn547_dev->pn547_device);

err_misc_register:
	mutex_destroy(&pn547_dev->read_mutex);
	kfree(pn547_dev);
err_exit:
err_i2c:
	pn547_gpio_release(platform_data);
err_gpio_request:
err_parse_dt:
	kfree(platform_data);
err_platform_data:
	dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	return ret;
}

static int pn547_remove(struct i2c_client *client)
{
	struct pn547_dev *pn547_dev;

	dev_info(&client->dev, "%s ++\n", __func__);
	pn547_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn547_dev);
	misc_deregister(&pn547_dev->pn547_device);
	mutex_destroy(&pn547_dev->read_mutex);
    wake_lock_destroy(&pn547_dev->wl);
	gpio_free(pn547_dev->irq_gpio);
	gpio_free(pn547_dev->ven_gpio);
	gpio_free(pn547_dev->firm_gpio);
	gpio_free(pn547_dev->clk_req_gpio);
	kfree(pn547_dev);

	return 0;
}

static const struct i2c_device_id pn547_id[] = {
	{ "pn547", 0 },
	{ }
};

static struct of_device_id pn547_match_table[] = {
	{ .compatible = "nxp,pn547", },
	{ },
};

static struct i2c_driver pn547_driver = {
	.id_table	= pn547_id,
	.probe		= pn547_probe,
	.remove		= pn547_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pn547",
		.of_match_table	= pn547_match_table,
	},
};

/*
 * module load/unload record keeping
 */

static int __init pn547_dev_init(void)
{
       //printk("### %s begin! \n",__func__);
       pr_info("### %s begin! \n",__func__);
	return i2c_add_driver(&pn547_driver);
}
module_init(pn547_dev_init);

static void __exit pn547_dev_exit(void)
{
	i2c_del_driver(&pn547_driver);
}
module_exit(pn547_dev_exit);

MODULE_AUTHOR("SERI");
MODULE_DESCRIPTION("NFC pn547 driver");
MODULE_LICENSE("GPL");
