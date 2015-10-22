/*
 * AD9854 SPI DDS driver
 *
 * Copyright 2015 SYSU SIST JinWei Hwang.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/notifier.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/mm.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include "linux/../../drivers/base/base.h"

//=================AD5235相关=================================
#define AD5235_NUM 4
// /*声明show函数*/
static ssize_t ad5235_0_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf);
static ssize_t ad5235_1_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf);
static ssize_t ad5235_2_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf);
static ssize_t ad5235_3_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf);

static struct kobj_attribute  ad5235_name_attr[] = {
	[0] = __ATTR(name, 0444, ad5235_0_name_show, NULL),
	[1] = __ATTR(name, 0444, ad5235_1_name_show, NULL),
	[2] = __ATTR(name, 0444, ad5235_2_name_show, NULL),
	[3] = __ATTR(name, 0444, ad5235_3_name_show, NULL),
};

static char *spi_name[] = {"spi1.3", "spi1.4", "spi1.5", "spi1.6"};
static char *spi_iio_link_name[] = {"ad5235_0", "ad5235_1", "ad5235_2", "ad5235_3"};
static int spi_sysfs_create_success[AD5235_NUM] = {0};
static struct kobject *spi_kobj[AD5235_NUM];
static struct device *spi_dev[AD5235_NUM];

/*当读文件时执行的操作*/
ssize_t ad5235_0_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf)
{
	const char name[] = "ad5235_0\n";
	sprintf(buf, "%s", name);
	return strlen(name) + 1;
}

ssize_t ad5235_1_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf)
{
	const char name[] = "ad5235_1\n";
	sprintf(buf, "%s", name);
	return strlen(name) + 1;
}

ssize_t ad5235_2_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf)
{
	const char name[] = "ad5235_2\n";
	sprintf(buf, "%s", name);
	return strlen(name) + 1;
}

ssize_t ad5235_3_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf)
{
	const char name[] = "ad5235_3\n";
	sprintf(buf, "%s", name);
	return strlen(name) + 1;
}
//==========================================================

//===================AD5242相关==============================
#define AD5242_NUM 4
// /*声明show函数*/
static ssize_t ad5242_0_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf);
static ssize_t ad5242_1_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf);
static ssize_t ad5242_2_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf);
static ssize_t ad5242_3_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf);

static struct kobj_attribute  ad5242_name_attr[] = {
	[0] = __ATTR(name, 0444, ad5242_0_name_show, NULL),
	[1] = __ATTR(name, 0444, ad5242_1_name_show, NULL),
	[2] = __ATTR(name, 0444, ad5242_2_name_show, NULL),
	[3] = __ATTR(name, 0444, ad5242_3_name_show, NULL),
};
static char *i2c_name[] = {"0-002c", "0-002d", "0-002e", "0-002f"};
static char *i2c_iio_link_name[] = {"ad5242_0", "ad5242_1", "ad5242_2", "ad5242_3"};
static int i2c_sysfs_create_success[AD5242_NUM] = {0};
static struct kobject *i2c_kobj[AD5242_NUM];
static struct device *i2c_dev[AD5242_NUM];

/*当读文件时执行的操作*/
ssize_t ad5242_0_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf)
{
	const char name[] = "ad5242_0\n";
	sprintf(buf, "%s", name);
	return strlen(name) + 1;
}

ssize_t ad5242_1_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf)
{
	const char name[] = "ad5242_1\n";
	sprintf(buf, "%s", name);
	return strlen(name) + 1;
}

ssize_t ad5242_2_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf)
{
	const char name[] = "ad5242_2\n";
	sprintf(buf, "%s", name);
	return strlen(name) + 1;
}

ssize_t ad5242_3_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf)
{
	const char name[] = "ad5242_3\n";
	sprintf(buf, "%s", name);
	return strlen(name) + 1;
}
//==========================================================

//=================PWM相关====================================
#define PWM0_EXPORT_FILE "/sys/class/pwm/pwmchip0/export"
#define PWM0_UNEXPORT_FILE "/sys/class/pwm/pwmchip0/unexport"
static ssize_t pwm0_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf);

struct device * pwm_dev;
struct kobject *pwm_kobj;
struct kobject *pwm_list_kobj;
static const char pwm_name[5] = "pwm0";
static const char pwm_iio_link_name[5] = "pwm0";
static struct kobj_attribute  pwm0_name_attr = __ATTR(name, 0444, pwm0_name_show, NULL);

ssize_t pwm0_name_show(struct kobject *kobject, struct kobj_attribute *attr, char *buf)
{
	const char name[5] = "pwm0";
	sprintf(buf, "%s", name);
	return strlen(name) + 1;
}
//==========================================================
static void filewrite(char* filename, char* data)
{
	struct file *filp;
	mm_segment_t fs;
	filp = filp_open(filename, O_WRONLY, 0);
	if (IS_ERR(filp))
	{
		printk("open file %s error...\n", filename);
		return;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	filp->f_op->write(filp, data, strlen(data), &filp->f_pos);
	set_fs(fs);
	filp_close(filp, NULL);
}

static int iio_link_init(void)
{
	int err;
	int i;
	// find ad5235 spi dir kobj
	for (i = 0; i < AD5235_NUM; i++)
	{
		spi_dev[i] = bus_find_device_by_name(&spi_bus_type, NULL, spi_name[i]);
		if (spi_dev[i])
		{
			spi_kobj[i] = &spi_dev[i]->kobj;
			if (spi_kobj[i])
			{
				printk("find %s name: %s\n", spi_name[i], spi_kobj[i]->name);
				err = sysfs_create_file(spi_kobj[i], &ad5235_name_attr[i].attr);
				if (err)
				{
					printk("Create %s name file error!\n", spi_name[i]);
					continue;
				}
				err = sysfs_create_link(&iio_bus_type.p->devices_kset->kobj, spi_kobj[i], spi_iio_link_name[i]);
				if (err)
				{
					printk("Create %s iio-link(%s) error!\n", spi_name[i], spi_iio_link_name[i]);
					sysfs_remove_file(spi_kobj[i], &ad5235_name_attr[i].attr);
					continue;
				}
				spi_sysfs_create_success[i] = 1;
			}
		}
		else
		{
			printk("can't find %s\n", spi_name[i]);
			continue;
		}
	}

	//find ad5242 i2c dir kobj
	for (i = 0; i < AD5242_NUM; i++) {
		i2c_dev[i] = bus_find_device_by_name(&i2c_bus_type, NULL, i2c_name[i]);
		if (i2c_dev[i])
		{
			i2c_kobj[i] = &i2c_dev[i]->kobj;
			if (i2c_kobj[i])
			{
				printk("find %s name: %s\n", i2c_name[i], i2c_kobj[i]->name);
				sysfs_remove_file(i2c_kobj[i], &ad5242_name_attr[i].attr); //remove the original file
				err = sysfs_create_file(i2c_kobj[i], &ad5242_name_attr[i].attr); //create new name file,and this file will noe be removed when this module remove
				if (err)
				{
					printk("Create %s name file error!\n", i2c_name[i]);
					continue;
				}
				err = sysfs_create_link(&iio_bus_type.p->devices_kset->kobj, i2c_kobj[i], i2c_iio_link_name[i]);
				if (err)
				{
					printk("Create %s iio-link(%s) error!\n", i2c_name[i], i2c_iio_link_name[i]);
					continue;
				}
				i2c_sysfs_create_success[i] = 1;
			}
		}
		else
		{
			printk("can't find %s\n", i2c_name[i]);
			continue;
		}
	}

	// export pwm0
	filewrite(PWM0_EXPORT_FILE, "0");
	// create pwm name file and iio symlink
	//printk("platform_bus_type.p->subsys.kobj.name : %s\n", platform_bus_type.p->subsys.kobj.name);
	pwm_dev = bus_find_device_by_name(&platform_bus_type, NULL, "20050030.pwm");
	if (pwm_dev)
	{
		pwm_kobj = &pwm_dev -> kobj;
		if (pwm_kobj)
		{
			printk("pwm_kobj.name : %s\n", pwm_kobj->name);
			list_for_each_entry(pwm_list_kobj, &pwm_kobj->kset->list, entry) {
				if (kobject_name(pwm_list_kobj) && !strcmp(kobject_name(pwm_list_kobj), pwm_name)) {
					printk("find pwm name : %s.\n", pwm_list_kobj->name);
					break;
				}
			}
			if (pwm_list_kobj)
			{
				err = sysfs_create_file(pwm_list_kobj, &pwm0_name_attr.attr);
				if (err)
				{
					printk("Create %s name file error!\n", pwm_name);
					goto error_pwm0;
				}
				err = sysfs_create_link(&iio_bus_type.p->devices_kset->kobj, pwm_list_kobj, pwm_iio_link_name);
				if (err)
				{
					printk("Create %s iio-link(%s) error!\n", pwm_name, pwm_iio_link_name);
					sysfs_remove_file(pwm_list_kobj, &pwm0_name_attr.attr);
					goto error_pwm0;
				}
			}
		}
	}
	// list_for_each(list, &iio_bus_type.p->subsys.list) {
	// 	kobj_num++;
	// 	iio_kobj = list_entry(list, struct kobject, entry);
	// 	printk("kboj %d name: %s.\n", kobj_num, iio_kobj->name);
	// }
	// printk("kobj_num: %d\n", kobj_num);
error_pwm0:
	return 0;
}

static void iio_link_exit(void)
{
	int i;
	// delete ad5235 iio name file and symlink
	for (i = 0; i < AD5235_NUM; i++)
	{
		if (spi_sysfs_create_success[i])
		{
			sysfs_remove_link(&iio_bus_type.p->devices_kset->kobj, spi_iio_link_name[i]);
			sysfs_remove_file(spi_kobj[i], &ad5235_name_attr[i].attr);
		}
	}

	// delete ad5242 iio name file and symlink
	for (i = 0; i < AD5242_NUM; i++)
	{
		if (i2c_sysfs_create_success[i])
		{
			sysfs_remove_link(&iio_bus_type.p->devices_kset->kobj, i2c_iio_link_name[i]);
			//sysfs_remove_file(i2c_kobj[i], &ad5242_name_attr[i].attr);
		}
	}

	if (pwm_list_kobj)
	{
		sysfs_remove_link(&iio_bus_type.p->devices_kset->kobj, pwm_iio_link_name);
		sysfs_remove_file(pwm_list_kobj, &pwm0_name_attr.attr);
	}
	// unexport pwm0
	filewrite(PWM0_UNEXPORT_FILE, "0");
}

module_init(iio_link_init);
module_exit(iio_link_exit);

MODULE_AUTHOR("JinWei Hwang <zsjinwei@live.com>");
MODULE_DESCRIPTION("Analog Devices AD9854 Driver");
MODULE_LICENSE("GPL v2");
