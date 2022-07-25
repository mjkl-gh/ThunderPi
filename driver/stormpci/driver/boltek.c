/*
 *  Copyright (C) 2008 Boltek
 *  Author Patrick McManus <mcmanus@ducksong.com>
 *  Based on port of windows driver
 *
 *  Boltek StormTracker PCI Lightning Detector
 *  http://www.boltek.com/stormtracker.html
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Requires >= 2.6.11
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/cdev.h>

#define BOLTEK_VERSION		"v1:1.0.0"
static u16 boltek_abi_version  = 0x0001;

#define BOLTEK_DATA_NORTH_OFFSET      (detector.mem + 0x00) /* 16 bits read */
#define BOLTEK_DATA_WEST_OFFSET	      (detector.mem + 0x02) /* 16 bits read */

#define BOLTEK_SQUELCH_OFFSET	      (detector.mem + 0x00) /* 16 bits write */
#define BOLTEK_FORCE_TRIGGER_OFFSET   (detector.mem + 0x02) /* 16 bits write */
#define BOLTEK_RESET_TIMESTAMP_OFFSET (detector.mem + 0x04) /* 16 bits write */

#define BOLTEK_PLX_CONTROL	      (detector.ctl + 0x50) /* 32 bits read */

#define BOLTEK_STRIKE_READY_BIT (1 << 2)
#define BOLTEK_CLK_ENABLE_BIT	(1 << 8)

#define BOLTEK_IOCTL_RESTART	   _IO(0xEA, 0xA0)
#define BOLTEK_IOCTL_FORCE_TRIGGER _IO(0xEA, 0xA1)
#define BOLTEK_IOCTL_STRIKE_READY  _IOR(0xEA, 0xA2, u8)
#define BOLTEK_IOCTL_GET_DATA	   _IOR(0xEA, 0xA3, struct stormpci_packed_data)
#define BOLTEK_IOCTL_SET_SQUELCH   _IOW(0xEA, 0xA4, u8)
#define BOLTEK_IOCTL_GET_VERSION   _IOR(0xEA, 0xA5, u16)

#define BOLTEK_BUFFERSIZE 512
struct stormpci_packed_data {
	u16 usNorth[BOLTEK_BUFFERSIZE];
	u16 usWest [BOLTEK_BUFFERSIZE];
};

static struct boltek_device {
	struct cdev cdev;
	struct class *class;
	int opened;
	int registered;
	int major;
	int active;
	void __iomem *ctl;  /* bar0 (also bar1 as io port) */
	void __iomem *mem;  /* bar2 */
	u16 squelch;
}  detector;

static DEFINE_SPINLOCK(boltek_lock);

static struct pci_device_id boltek_pci_tbl[] __devinitdata = {
	{
		.vendor = PCI_VENDOR_ID_PLX,
		.device = PCI_DEVICE_ID_PLX_9050,
		.subvendor = PCI_VENDOR_ID_PLX,
		.subdevice = PCI_DEVICE_ID_PLX_9050,
		.class = 0,
		.class_mask = 0,
		.driver_data = 0x00
	},
	{ 0, }
};

/* Some Prototypes */
static int __devinit boltek_probe(struct pci_dev *pdev,
				  const struct pci_device_id *pci_id);
static void __devexit boltek_remove(struct pci_dev *pdev);
static int boltek_open(struct inode *inode, struct file *file);
static long boltek_unlocked_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg);
static int boltek_release(struct inode *inode, struct file *file);

static struct pci_driver boltek_pci_driver = {
	.name = "boltek",
	.id_table = boltek_pci_tbl,
	.probe = boltek_probe,
	.remove = boltek_remove,
};

static struct file_operations boltek_file_ops = {
	.open =	   boltek_open,
	.release = boltek_release,
	.unlocked_ioctl = boltek_unlocked_ioctl,
	/* .compat_ioctl is not needed, because there are no variable sized
	   types are part of the api */
};


/* call with lock held */
static void boltek_restartboard_l(void)
{
	/* Stop adc Clock so FIFO will reset */
	u32  data;

	/* clear clock enable */
	data = ioread32(BOLTEK_PLX_CONTROL);
	data &= ~BOLTEK_CLK_ENABLE_BIT;
	iowrite32(data, BOLTEK_PLX_CONTROL);

	/* update squelch */
	iowrite16(detector.squelch, BOLTEK_SQUELCH_OFFSET);

	/* set clock enable */
	data = ioread32(BOLTEK_PLX_CONTROL);
	data |= BOLTEK_CLK_ENABLE_BIT;
	iowrite32(data, BOLTEK_PLX_CONTROL);

	return;
}

static int boltek_release(struct inode *inode, struct file *file)
{
	spin_lock(&boltek_lock);
	detector.opened--;
	WARN_ON(detector.opened);
	spin_unlock(&boltek_lock);
	return 0;
}

static int boltek_open(struct inode *inode, struct file *file)
{
	int rv = 0;

	spin_lock(&boltek_lock);

	if (detector.active == 0)
		rv = -EINVAL;
	else if (detector.opened)
		rv = -EACCES;
	else {
		detector.opened++;
		iowrite16(0, BOLTEK_RESET_TIMESTAMP_OFFSET);
		boltek_restartboard_l();
	}
	spin_unlock(&boltek_lock);

	return rv;
}

static long boltek_unlocked_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int rv = 0;

	spin_lock(&boltek_lock);

	switch (cmd) {
	case BOLTEK_IOCTL_RESTART:
		boltek_restartboard_l();
		break;

	case BOLTEK_IOCTL_FORCE_TRIGGER:
		iowrite16(0, BOLTEK_FORCE_TRIGGER_OFFSET);
		break;

	case BOLTEK_IOCTL_STRIKE_READY: {
		u32 data;
		u8 nv;

		/* when the control bit 2 is _off_ there is strike data ready */

		data = ioread32(BOLTEK_PLX_CONTROL);
		nv = (!(data & BOLTEK_STRIKE_READY_BIT)) ? 1 : 0;
		if (copy_to_user(argp, &nv, sizeof(nv))) {
			rv = -EFAULT;
			break;
		}
	}
		break;

	case BOLTEK_IOCTL_GET_DATA:  {
		struct stormpci_packed_data nv;
		u16 cnt;

		/*
		 * store data if there is room in the capture buffer
		 * Read N FIFO, E-Field on D8
		 */
		for (cnt = 0; cnt < BOLTEK_BUFFERSIZE; cnt++)
			nv.usNorth[cnt] = ioread16(BOLTEK_DATA_NORTH_OFFSET);

		/* Read W FIFO, GPS on D8-15 */
		for (cnt = 0; cnt < BOLTEK_BUFFERSIZE; cnt++)
			nv.usWest[cnt] = ioread16(BOLTEK_DATA_WEST_OFFSET);

		if (copy_to_user(argp, &nv, sizeof(nv))) {
			rv = -EFAULT;
			break;
		}
	}
		break;

	case BOLTEK_IOCTL_SET_SQUELCH: {
		u8 nv;

		if (copy_from_user(&nv, argp, sizeof(nv))) {
			rv = -EFAULT;
			break;
		}

		if (nv <= 15)
			detector.squelch = nv;
		else {
			printk(KERN_INFO
			       "IOCTL for boltek set squelch out of range\n");
			rv = -EINVAL;
			break;
		}
	}
		break;

	case BOLTEK_IOCTL_GET_VERSION: {
		if (copy_to_user(argp, &boltek_abi_version, sizeof(u16))) {
			rv = -EFAULT;
			break;
		}
	}
		break;

	default:
		rv = -EINVAL;
		break;
	}
	spin_unlock(&boltek_lock);
	return rv;
}

static int __devinit boltek_probe(struct pci_dev *pdev,
				  const struct pci_device_id *pci_id)
{
	int rv = 0;

	spin_lock(&boltek_lock);
	rv = pci_enable_device(pdev);
	if (rv)	 {
		dev_err(&pdev->dev, "PCI Enable Failed\n");
		goto boltek_probe_done;
	}

	rv = pci_request_regions(pdev, "boltek");
	if (rv)	 {
		dev_err(&pdev->dev, "PCI Request Regions Failed\n");
		goto boltek_probe_done;
	}

	/* There is a little problem with another PCI device that is
	   also known as 10b5:9050 - it is a HDLC WAN interface
	   driver implemented via pci200syn.c . We differentiate between
	   them using the device BARs. the pci200 has 3 bars of 128,
	   1024, and >= 16384.. while the boltek lightning detector has
	   3 of 128, 128, and 16.
	*/


	if (((int)pci_resource_len(pdev, 0) != 128) ||
	    ((int)pci_resource_len(pdev, 1) != 128) ||
	    ((int)pci_resource_len(pdev, 2) != 16)) {
		dev_err(&pdev->dev,
			"Device BARs unexpected sizes - "
			"maybe this is not a boltek device?\n");
		pci_release_regions(pdev);
		pci_disable_device(pdev);
		rv = -EFAULT;
		goto boltek_probe_done;
	}

	detector.ctl = pci_iomap(pdev, 0, 128);
	detector.mem = pci_iomap(pdev, 2, 16);
	if ((detector.ctl == NULL) ||
	    (detector.mem == NULL)) {
		dev_err(&pdev->dev, "Device BAR Mapping problem\n");
		pci_release_regions(pdev);
		pci_disable_device(pdev);
		rv = -EFAULT;
		goto boltek_probe_done;
	}

	if (IS_ERR(device_create(detector.class, &pdev->dev,
				 MKDEV(detector.major, 0),
				 "lightning-%d", 0)))
		dev_err(&pdev->dev,
			"can't create sysfs entry for /dev/lightning-0\n");
	else
		printk(KERN_INFO
		       "Boltek Lightning Detector started at"
		       " /dev/lightning-0 major %d minor 0\n",
		       detector.major);

	detector.active = 1;
boltek_probe_done:
	spin_unlock(&boltek_lock);
	return rv;
}

static void __devexit boltek_remove(struct pci_dev *pdev)
{
	device_destroy(detector.class, MKDEV(detector.major, 0));

	if (detector.mem != NULL) {
		pci_iounmap(pdev, detector.mem);
		detector.mem = NULL;
	}

	if (detector.ctl != NULL) {
		pci_iounmap(pdev, detector.ctl);
		detector.ctl = NULL;
	}

	if (detector.active) {
		detector.active = 0;
		pci_release_regions(pdev);
		pci_disable_device(pdev);
	}
	return;
}

static int __init boltek_init(void)
{
	int rv;
	dev_t dev;

	detector.registered = 0;
	detector.opened = 0;
	detector.squelch = 0;
	detector.major	= -1;
	detector.active = 0;			  /* probe will enable */
	detector.mem = NULL;
	detector.ctl = NULL;
	detector.class = NULL;

	cdev_init(&detector.cdev, &boltek_file_ops);
	detector.cdev.owner = THIS_MODULE;

	printk(KERN_INFO "Boltek Lightning Detector %s\n", BOLTEK_VERSION);

	detector.class = class_create(THIS_MODULE, "boltek");
	if (IS_ERR(detector.class)) {
		printk(KERN_INFO
		       "boltek class creation failed\n");
		return -EFAULT;
	}

	rv = alloc_chrdev_region(&dev, 0, 1, "boltek");
	detector.major = rv ? -1 : MAJOR(dev);

	if (detector.major < 0) {
		printk(KERN_INFO
		       "boltek alloc chrdev failed\n");
		return -EFAULT;
	}

	cdev_add(&detector.cdev, MKDEV(detector.major, 0), 1);

	rv = pci_register_driver(&boltek_pci_driver);
	if (rv) {
		printk(KERN_INFO
		       "boltek register failed\n");
		return rv;
	}

	detector.registered = 1;
	return 0;
}

static void __exit boltek_exit(void)
{
	printk(KERN_INFO
	       "Boltek Lightning Detector Module Removal %s\n", BOLTEK_VERSION);

	if (detector.registered)
		pci_unregister_driver(&boltek_pci_driver);

	if (detector.major >= 0) {
		cdev_del(&detector.cdev);
		unregister_chrdev_region(MKDEV(detector.major, 0), 1);
	}

	if (detector.class)
		class_destroy(detector.class);

	detector.registered = 0;
	detector.major = -1;

	WARN_ON(detector.opened);
	WARN_ON(detector.active);
	WARN_ON(detector.mem != NULL);
	WARN_ON(detector.ctl != NULL);

	return;
}

module_init(boltek_init);
module_exit(boltek_exit);

MODULE_DEVICE_TABLE(pci, boltek_pci_tbl);
MODULE_VERSION(BOLTEK_VERSION);
MODULE_AUTHOR("Patrick McManus <mcmanus@ducksong.com>");
MODULE_DESCRIPTION("Driver for Boltek Lightning Detector");
MODULE_LICENSE("GPL v2");

