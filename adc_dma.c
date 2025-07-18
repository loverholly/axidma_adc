#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/mm.h>
#include <linux/exportfs.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/of_dma.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/dma/xilinx_dma.h>

#define ADC_DMAINIT_S 1
#define MB_SIZE (1024 * 1024)
#define KB_SIZE (1024)
#define MAX_BUF_SLOT 32

#define ADCDATA_SIZE (66 * KB_SIZE)

/*
 *buf_con:缓存数,不大于32
 *adc_phys:dma物理内存地址
 *adc_virt:dma物理内存对应的映射地址
 *mem_switch:当前dma使用的内存
 *mem_flag:缓存标识，被dma写之后对应位置1，读取之后对应位置0
 *read_mem:当前读取的内存块
 *
 */
struct adcdma_para {
	struct dma_chan *dma;
	int buf_con;
	volatile int mem_switch;
	spinlock_t lock;
	int mem_slot_size;
	volatile int mem_flag;
	volatile int dma_issue;
	volatile int read_mem;
	volatile int write_mem;
	phys_addr_t adc_phys[MAX_BUF_SLOT];
	void __iomem *adc_virt[MAX_BUF_SLOT];
#if !ADC_DMAINIT_S
	struct scatterlist sglis[0x80];
#endif
};

struct adcdma_fun {
	struct class *adcdma_class;
	struct device *adcdma_dev;
	wait_queue_head_t read_queue;
	int irq_reprot;
	int trans_en;
	dev_t t_dev;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;
	struct adcdma_para *dmad;
	struct platform_device *pdev;
	int used;
};

struct adcdma_fun *adcdma_data;

static int adcdma_setup(struct adcdma_para *dmap, int nfb);

#define ADC_IOCTL_MAGIC 'M'

#define CMD_GET_DMADATA _IO(ADC_IOCTL_MAGIC, 0)
#define CMD_SET_START _IO(ADC_IOCTL_MAGIC, 1)
#define CMD_GET_DMA_WRPOINTER _IO(ADC_IOCTL_MAGIC, 2)
#define CMD_SET_BUF_SIZE _IO(ADC_IOCTL_MAGIC, 3)
#define CMD_GET_DATA_READY _IO(ADC_IOCTL_MAGIC, 4)

static void adc_dma_free(struct platform_device *pdev, struct adcdma_para *dmap)
{
	int size;
	int i = 0;

	size = dmap->mem_slot_size;

#if ADC_DMAINIT_S
	for (i = 0; i < dmap->buf_con; i++) {
		dma_free_coherent(&pdev->dev, PAGE_ALIGN(size),
		                  dmap->adc_virt[i], dmap->adc_phys[i]);
	}
#endif
}

static int adc_dma_alloc(struct platform_device *pdev, struct adcdma_para *dmap)
{
	int i = 0;
	int cnt = dmap->buf_con;
	int hsize = dmap->mem_slot_size;

	for (i = 0; i < cnt; i++) {
		dmap->adc_virt[i] = dma_alloc_coherent(&pdev->dev, PAGE_ALIGN(hsize),
		                                       &dmap->adc_phys[i], GFP_KERNEL);
		if (dmap->adc_virt[i] == NULL) {
			printk("can't alloc mem\n");
			/* here need be continue */
			break;
		}
	}

	return i;
}

static void irq_change_mem(struct adcdma_para *dmap, int num)
{
	struct adcdma_para *dmad_tmp = dmap;
	int next;
	int mem_switch = dmad_tmp->mem_switch;

	spin_lock_irq(&dmap->lock);
	dmad_tmp->mem_flag |= (1 << mem_switch);
	next = (mem_switch + 1) % dmad_tmp->buf_con;
	if ((dmad_tmp->mem_flag & (1 << next)) == 0) {
		dmad_tmp->write_mem = (dmad_tmp->write_mem + 1) % dmad_tmp->buf_con;
		dmad_tmp->mem_switch = next;
		dmad_tmp->dma_issue++;
		adcdma_setup(dmap, next);
		dma_async_issue_pending(dmap->dma);
	}

	/* to avoid dma issue twice, context switch between tasklet and task */
	dmad_tmp->dma_issue--;
	spin_unlock_irq(&dmap->lock);
}

static void adcdma_irq_handler(void *data)
{
	int num = (int)data;

	adcdma_data->irq_reprot = 1;

	if (adcdma_data->trans_en) {
		wake_up_interruptible(&adcdma_data->read_queue);
		irq_change_mem(adcdma_data->dmad, num);
	}
}

static int adcdma_setup(struct adcdma_para *dmap, int nfb)
{
	struct dma_async_tx_descriptor *desc;
	enum dma_ctrl_flags flags;
	u32 size = dmap->mem_slot_size;
	int ret = 0;

	flags = DMA_PREP_INTERRUPT;

#if ADC_DMAINIT_S
	desc = dmaengine_prep_slave_single(dmap->dma, dmap->adc_phys[nfb],
	                                   size, DMA_DEV_TO_MEM, flags);
	if (!desc) {
		pr_err("Failed to prepare DMA descriptor\n");
		return -ENOMEM;
	} else {
		ret = dmaengine_submit(desc);
		if (ret < 0)
			printk("submit err\n");
		desc->callback = adcdma_irq_handler;
		desc->callback_param = (void *)nfb;
	}
#else
	struct scatterlist *rx_sg = &dmap->sglis[nfb];
	struct dma_device *rx_dev = dmap->dma->device;

	dmap->adc_virt[nfb] = kmalloc(ADCDATA_SIZE, GFP_KERNEL);
	dma_addr_t adc_dma_rx;
	dma_cookie_t rx_cookie;

	memset(dmap->adc_virt[nfb], 0, ADCDATA_SIZE);

	adc_dma_rx = dma_map_single(rx_dev->dev, dmap->adc_virt[nfb],
	                            ADCDATA_SIZE, DMA_MEM_TO_DEV);

	dma_unmap_single(rx_dev->dev, adc_dma_rx,
	                 ADCDATA_SIZE,
	                 DMA_MEM_TO_DEV);

	adc_dma_rx = dma_map_single(rx_dev->dev, dmap->adc_virt[nfb],
	                            ADCDATA_SIZE, DMA_DEV_TO_MEM);

	sg_init_table(rx_sg, 1);
	sg_dma_address(rx_sg) = adc_dma_rx;
	sg_dma_len(rx_sg) = ADCDATA_SIZE;

	desc = rx_dev->device_prep_slave_sg(dmap->dma, rx_sg, 1,
	                                    DMA_DEV_TO_MEM, flags, NULL);

	rx_cookie = desc->tx_submit(desc);
	desc->callback = adcdma_irq_handler;
	desc->callback_param = (void *)nfb;
#endif

	return 0;
}

static int data_to_user(struct adcdma_fun *dma_data, void __user *user_buf)
{
	int size;
	int ret = 0;
	struct adcdma_para *dmap = dma_data->dmad;

	if (READ_ONCE(dmap->mem_flag) == 0) {
		ret = -1;
		goto done;
	}

	size = dmap->mem_slot_size;
	ret = copy_to_user(user_buf, dmap->adc_virt[dmap->read_mem], size) ? -EFAULT : 0;
	if (ret < 0) {
		ret = -1;
		goto done;
	}

	spin_lock_irq(&dmap->lock);
	dmap->mem_flag &= ~(1 << dmap->read_mem);
	dmap->read_mem = (dmap->read_mem + 1) % dmap->buf_con;

	/* dma may stop by buf full */
	if (!READ_ONCE(dmap->mem_flag) && READ_ONCE(dmap->dma_issue) == 0 && dma_data->trans_en) {
		dmap->mem_switch = (dmap->mem_switch + 1) % dmap->buf_con;
		dmap->write_mem = (dmap->write_mem + 1) % dmap->buf_con;
		dmap->dma_issue++;
		adcdma_setup(dma_data->dmad, dmap->mem_switch);
		dma_async_issue_pending(dma_data->dmad->dma);
	}
	spin_unlock_irq(&dmap->lock);

done:
	return ret;
}

static int param_to_user(struct adcdma_fun *dma_data, void __user *user_buf)
{
	struct adcdma_para *dmap = dma_data->dmad;
	u32 pointer = (dmap->read_mem & 0xffff) | ((dmap->write_mem & 0xffff) << 16);
	if (dma_data->trans_en == 0)
		pointer = 0;

	return copy_to_user(user_buf, &pointer, sizeof(pointer));
}

static int get_data_ready_to_user(struct adcdma_fun *dma_data, void __user *user_buf)
{
	struct adcdma_para *dmap = dma_data->dmad;
	u32 pointer = READ_ONCE(dmap->mem_flag) ? 1 : 0;

	spin_lock_irq(&dmap->lock);
	if (!pointer && READ_ONCE(dmap->dma_issue) == 0 && dma_data->trans_en) {
		dmap->mem_switch = (dmap->mem_switch + 1) % dmap->buf_con;
		dmap->write_mem = (dmap->write_mem + 1) % dmap->buf_con;
		dmap->dma_issue++;
		adcdma_setup(dma_data->dmad, dmap->mem_switch);
		dma_async_issue_pending(dma_data->dmad->dma);
	}
	spin_unlock_irq(&dmap->lock);

	return copy_to_user(user_buf, &pointer, sizeof(pointer));
}

static long adcdma_func_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	void __user *user_arg = (void __user *)arg;
	int flag;
	u32 set_param;

	switch (cmd) {
	case CMD_SET_BUF_SIZE:
		ret = copy_from_user(&set_param, user_arg, sizeof(set_param));
		if (!ret) {
			int num;
			u32 buf_size = set_param & 0xffffff;
			u32 buf_slot = (set_param & 0xff000000) >> 24;
			/* printk("size/slot %x/%x\n", buf_size, buf_slot); */
			if (buf_size * buf_slot > 0x2000000 || buf_slot > MAX_BUF_SLOT) {
				ret = -EFAULT;
				break;
			} else {
				/* force stop dma engine */
				adcdma_data->trans_en = 0;
				dmaengine_terminate_all(adcdma_data->dmad->dma);

				/* realloc adc dma buf */
				adcdma_data->dmad->mem_switch = 0;
				adcdma_data->dmad->mem_flag = 0;
				adcdma_data->dmad->read_mem = 0;
				adcdma_data->dmad->write_mem = 0;
				adc_dma_free(adcdma_data->pdev, adcdma_data->dmad);

				if (buf_slot > MAX_BUF_SLOT)
					buf_slot = MAX_BUF_SLOT;

				adcdma_data->dmad->buf_con = buf_slot;
				adcdma_data->dmad->mem_slot_size = buf_size;
				num = adc_dma_alloc(adcdma_data->pdev, adcdma_data->dmad);
				adcdma_data->dmad->buf_con = num;
			}
		} else {
			ret = -1;
		}
		break;
	case CMD_GET_DMA_WRPOINTER:
		ret = param_to_user(adcdma_data, user_arg);
		break;

	case CMD_GET_DATA_READY:
		ret = get_data_ready_to_user(adcdma_data, user_arg);
		break;

	case CMD_GET_DMADATA:
		ret = data_to_user(adcdma_data, user_arg);
		break;

	case CMD_SET_START:
		copy_from_user(&flag, user_arg, sizeof(flag));

		adcdma_data->dmad->mem_switch = 0;
		adcdma_data->dmad->mem_flag = 0;
		adcdma_data->dmad->read_mem = 0;
		adcdma_data->dmad->write_mem = 0;
		adcdma_data->dmad->dma_issue = 0;

		if (flag == 0) {
			adcdma_data->trans_en = 0;
			dmaengine_terminate_all(adcdma_data->dmad->dma);
		} else {
			adcdma_data->trans_en = 1;
			adcdma_data->dmad->dma_issue++;
			ret = adcdma_setup(adcdma_data->dmad, 0);
			dma_async_issue_pending(adcdma_data->dmad->dma);
		}

#if 0				/* config should be in app */
		gpiod_set_value_cansleep(adcdma_data->enable_gpio, flag);
#endif

		break;

	default:
		ret = -EFAULT;
		break;
	}

	return ret;
}

static unsigned int adcdma_func_poll(struct file *file, struct poll_table_struct *wait)
{
	int mask = 0;

	poll_wait(file, &adcdma_data->read_queue, wait);
	if (adcdma_data->irq_reprot == 1) {
		adcdma_data->irq_reprot = 0;
		mask |= (POLLIN | POLLRDNORM);
	}

	return mask;
}

__attribute__((unused)) void adcdma_reset(void)
{
	gpiod_set_value_cansleep(adcdma_data->reset_gpio, 0);
	gpiod_set_value_cansleep(adcdma_data->enable_gpio, 0);
	gpiod_set_value_cansleep(adcdma_data->reset_gpio, 1);
}

int adcdma_func_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	if (adcdma_data->used >= 1) {
		printk("%s driver busy\n", file->f_path.dentry->d_iname);
		return -EBUSY;
	}

	adcdma_data->used = 1;
	adcdma_data->trans_en = 1;

	// dmaengine_terminate_all(adcdma_data->dmad[i]->dma);
	adcdma_data->dmad->mem_switch = 0;
	adcdma_data->dmad->mem_flag = 0;
	adcdma_data->dmad->read_mem = 0;
	adcdma_data->dmad->write_mem = 0;
	adcdma_data->dmad->dma_issue = 0;

	// adcdma_reset();

	return ret;
}

int adcdma_func_release(struct inode *inode, struct file *file)
{
	dmaengine_terminate_all(adcdma_data->dmad->dma);
	adcdma_data->trans_en = 0;
	adcdma_data->used = 0;

#if !ADC_DMAINIT_S
	int j = 0;
	for (j = 0; j < adcdma_data->dmad->buf_con; j++)
		kfree(adcdma_data->dmad->adc_virt[j]);
#endif
	return 0;
}

const struct file_operations adcdma_fops = {
	.owner = THIS_MODULE,
	.open = adcdma_func_open,
	.release = adcdma_func_release,
	.unlocked_ioctl = adcdma_func_ioctl,
	.poll = adcdma_func_poll,
};

int adcdma_cdev_init(struct adcdma_fun *pdata)
{
	int rc;
	struct cdev *c_dev;

	rc = alloc_chrdev_region(&pdata->t_dev, 0, 1, "adcdma_fun");

	if (rc)
		goto out_err;

	c_dev = cdev_alloc();
	if (!c_dev)
		goto out_err;

	cdev_init(c_dev, &adcdma_fops);
	rc = cdev_add(c_dev, pdata->t_dev, 1);
	if (rc)
		goto out_unreg;

	pdata->adcdma_class = class_create(THIS_MODULE, "adcdma");
	if (IS_ERR(pdata->adcdma_class)) {
		printk("[err]class_create error\n");
		rc = -1;
		goto out_devdel;
	}

	pdata->adcdma_dev = device_create(pdata->adcdma_class, NULL,
	                                  pdata->t_dev, NULL, "adcdma_fun");
	if (!pdata->adcdma_dev)	{
		rc = -1;
		goto class_err;
	}

	return 0;

class_err:
	class_destroy(pdata->adcdma_class);

out_devdel:
	cdev_del(c_dev);

out_unreg:
	unregister_chrdev_region(pdata->t_dev, 1);

out_err:

	return rc;
}

int of_adcdma_data(struct adcdma_fun *pdata, struct platform_device *pdev)
{
	int cnt = 0;
	int ret = 0;
	int hsize = 0;

#if 0				/* should be remove in real project */
	pdata->reset_gpio = devm_gpiod_get_optional(&pdev->dev, "reset",
	                    GPIOD_OUT_LOW);
	if (IS_ERR(pdata->reset_gpio)) {

		printk("[zgq]get reset gpio err\n");
		return PTR_ERR(pdata->reset_gpio);
	}

	pdata->enable_gpio = devm_gpiod_get_optional(&pdev->dev, "enable",
	                     GPIOD_OUT_LOW);
	if (IS_ERR(pdata->enable_gpio))	{
		printk("[zgq]get enable gpio err\n");
		return PTR_ERR(pdata->enable_gpio);
	}

	adcdma_reset();
#endif

	pdata->dmad = devm_kmalloc(&pdev->dev, sizeof(struct adcdma_para),
	                           GFP_KERNEL);
	if (pdata->dmad == NULL) {
		printk("kmalloc err\n");
		return -1;
	}

	memset(pdata->dmad, 0, sizeof(struct adcdma_para));
	pdata->dmad->dma = dma_request_chan(&pdev->dev, "adc");
	if (IS_ERR_OR_NULL(pdata->dmad->dma)) {
		printk("get dma0 err\n");
		return PTR_ERR(pdata->dmad->dma);
	}

	ret = of_property_read_u32(pdev->dev.of_node,
	                           "num-buf", &cnt);
	if (ret < 0) {
		pr_err("adcdmatest: missing num-frm property\n");
		return ret;
	}

	cnt = cnt > MAX_BUF_SLOT ? MAX_BUF_SLOT : cnt;

	pdata->dmad->buf_con = cnt;
	pdata->dmad->mem_slot_size = hsize = ADCDATA_SIZE;
	spin_lock_init(&pdata->dmad->lock);

#if ADC_DMAINIT_S
	adc_dma_alloc(pdev, pdata->dmad);
#endif
	init_waitqueue_head(&pdata->read_queue);
	return 0;
}

static int adcdma_fun_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adcdma_fun *pdata = dev_get_platdata(dev);
	int ret = 0;

	if (!pdata) {
		pdata = devm_kzalloc(dev, sizeof(struct adcdma_fun), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		platform_set_drvdata(pdev, pdata);
	}

	printk("probe the adcdma!\n");
	adcdma_data = pdata;
	adcdma_data->pdev = pdev;
	ret = of_adcdma_data(pdata, pdev);
	if (ret < 0)
		goto out;

	printk("adcdma data init done!\n");
	ret = adcdma_cdev_init(pdata);
	if (ret < 0)
		goto out;
	printk("adcdma cdev init!\n");

out:
	return ret;
}

static int adcdma_fun_remove(struct platform_device *pdev)
{
	dma_release_channel(adcdma_data->dmad->dma);
	adc_dma_free(pdev, adcdma_data->dmad);

	device_unregister(adcdma_data->adcdma_dev);
	unregister_chrdev_region(adcdma_data->t_dev, 1);
	class_destroy(adcdma_data->adcdma_class);

	return 0;
}

static struct of_device_id adcdma_fun_of_match[] = {
	{
		.compatible = "adcdma_demo",
	},
	{},
};

static struct platform_driver adcdma_fun_device_driver = {
	.probe = adcdma_fun_probe,
	.remove = adcdma_fun_remove,
	.driver = {
		.name = "adcdma_demo",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(adcdma_fun_of_match),
	}
};

static int __init adcdma_fun_init(void)
{
	return platform_driver_register(&adcdma_fun_device_driver);
}

static void __exit adcdma_fun_exit(void)
{
	platform_driver_unregister(&adcdma_fun_device_driver);
}

module_init(adcdma_fun_init);
module_exit(adcdma_fun_exit);

// module_platform_driver(adcdma_fun_device_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("uisrc");
