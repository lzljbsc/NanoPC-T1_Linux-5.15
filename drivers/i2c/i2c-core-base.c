// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Linux I2C core
 *
 * Copyright (C) 1995-99 Simon G. Vogl
 *   With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi>
 *   Mux support by Rodolfo Giometti <giometti@enneenne.com> and
 *   Michael Lawnick <michael.lawnick.ext@nsn.com>
 *
 * Copyright (C) 2013-2017 Wolfram Sang <wsa@kernel.org>
 */

#define pr_fmt(fmt) "i2c-core: " fmt

#include <dt-bindings/i2c/i2c.h>
#include <linux/acpi.h>
#include <linux/clk/clk-conf.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqflags.h>
#include <linux/jump_label.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeirq.h>
#include <linux/property.h>
#include <linux/rwsem.h>
#include <linux/slab.h>

#include "i2c-core.h"

#define CREATE_TRACE_POINTS
#include <trace/events/i2c.h>

/* 用于 new_device sysfs 属性节点的标志
 * 用户空间可在从机地址中设置一下两个标志
 * 10位地址模式, 做为从设备模式 */
#define I2C_ADDR_OFFSET_TEN_BIT	0xa000
#define I2C_ADDR_OFFSET_SLAVE	0x1000

/* I2C_ADDR_7BITS_MAX : 7bit 从机地址的最大地址
 * 0x78及以后的是保留地址, 参考 i2c_check_7bit_addr_validity_strict */
/* I2C_ADDR_7BITS_COUNT : 7bit 从机地址的数量
 * 用于在创建中断域时，提供创建的数量 */
#define I2C_ADDR_7BITS_MAX	0x77
#define I2C_ADDR_7BITS_COUNT	(I2C_ADDR_7BITS_MAX + 1)

/* 在 i2c 规范中， 0x7c - 0x7f 为保留地址
 * 0x7c 这个地址可以用于探测设备的 ID，在规范中，也对厂商ID，器件ID等信息做了规
 * 范 */
#define I2C_ADDR_DEVICE_ID	0x7c

/*
 * core_lock protects i2c_adapter_idr, and guarantees that device detection,
 * deletion of detected devices are serialized
 */
/* i2c_adapter_idr 及 保护锁
 * bus 号 及 i2c_adapter 指针使用 idr 管理 */
static DEFINE_MUTEX(core_lock);
static DEFINE_IDR(i2c_adapter_idr);

/* 新注册 适配器 或 驱动时，驱动程序探测设备处理函数 */
static int i2c_detect(struct i2c_adapter *adapter, struct i2c_driver *driver);

/* 调试用 */
static DEFINE_STATIC_KEY_FALSE(i2c_trace_msg_key);

/* i2c 子系统已初始化标记
 * 用于避免在 子系统未初始化时, 有其它模块注册设备 */
static bool is_registered;

/* debugfs 中的 i2c 根目录 */
static struct dentry *i2c_debugfs_root;

/* 调试用 */
int i2c_transfer_trace_reg(void)
{
	static_branch_inc(&i2c_trace_msg_key);
	return 0;
}

/* 调试用 */
void i2c_transfer_trace_unreg(void)
{
	static_branch_dec(&i2c_trace_msg_key);
}

/* 根据总线速率返回总线模式
 * 这是根据总线模式的最高频率换算的, 并非根据实际速率 */
const char *i2c_freq_mode_string(u32 bus_freq_hz)
{
	switch (bus_freq_hz) {
	case I2C_MAX_STANDARD_MODE_FREQ:
		return "Standard Mode (100 kHz)";
	case I2C_MAX_FAST_MODE_FREQ:
		return "Fast Mode (400 kHz)";
	case I2C_MAX_FAST_MODE_PLUS_FREQ:
		return "Fast Mode Plus (1.0 MHz)";
	case I2C_MAX_TURBO_MODE_FREQ:
		return "Turbo Mode (1.4 MHz)";
	case I2C_MAX_HIGH_SPEED_MODE_FREQ:
		return "High Speed Mode (3.4 MHz)";
	case I2C_MAX_ULTRA_FAST_MODE_FREQ:
		return "Ultra Fast Mode (5.0 MHz)";
	default:
		return "Unknown Mode";
	}
}
EXPORT_SYMBOL_GPL(i2c_freq_mode_string);

/* i2c 设备 match id 表匹配
 * 该表格与 of_match_table 作用类似 */
const struct i2c_device_id *i2c_match_id(const struct i2c_device_id *id,
						const struct i2c_client *client)
{
	if (!(id && client))
		return NULL;

    /* 逐个匹配 i2c_device_id 表中的 name , 字符串匹配方式 */
	while (id->name[0]) {
		if (strcmp(client->name, id->name) == 0)
			return id;
		id++;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(i2c_match_id);

/* 查找指定 i2c 从设备匹配的驱动数据 */
const void *i2c_get_match_data(const struct i2c_client *client)
{
	struct i2c_driver *driver = to_i2c_driver(client->dev.driver);
	const struct i2c_device_id *match;
	const void *data;

    /* 通过设备节点，查找在驱动中匹配的数据
     * 查找驱动中的 of_match_table 的匹配项 */
	data = device_get_match_data(&client->dev);
	if (!data) {
        /* 如果匹配失败，则查找驱动中的 id_table 数据 */
		match = i2c_match_id(driver->id_table, client);
		if (!match)
			return NULL;

        /* 匹配成功了, 则返回对应的驱动数据 */
		data = (const void *)match->driver_data;
	}

	return data;
}
EXPORT_SYMBOL(i2c_get_match_data);

/* i2c bus  match 回调
 * 用于匹配 i2c bus 上的设备和驱动
 * 传入的 dev 和 drv 是 i2c 类型的设备和驱动 */
static int i2c_device_match(struct device *dev, struct device_driver *drv)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_driver	*driver;


	/* Attempt an OF style match */
    /* 尝试 of 设备树类型匹配, 匹配成功返回非NULL */
	if (i2c_of_match_device(drv->of_match_table, client))
		return 1;

	/* Then ACPI style match */
    /* ACPI 风格，arm不支持 */
	if (acpi_driver_match_device(dev, drv))
		return 1;

    /* i2c_device_id 表在 i2c_driver 结构中
     * 这里通过设备模型的 struct device_driver 结构找到包含它的driver 结构 */
	driver = to_i2c_driver(drv);

	/* Finally an I2C match */
    /* match id 方式匹配 */
	if (i2c_match_id(driver->id_table, client))
		return 1;

	return 0;
}

/* 注册 i2c 从设备时的 uevent 回调
 * 本函数中将添加 MODALIAS 环境变量  */
static int i2c_device_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc;

    /* 与 of_device_modalias 一样，只是放到 env 中
     * 参考 modalias_show 方法 */
	rc = of_device_uevent_modalias(dev, env);
	if (rc != -ENODEV)
		return rc;

    /* acpi arm 不支持 */
	rc = acpi_device_uevent_modalias(dev, env);
	if (rc != -ENODEV)
		return rc;

    /* 以上都不支持, 则使用固定的前缀及设备名 */
	return add_uevent_var(env, "MODALIAS=%s%s", I2C_MODULE_PREFIX, client->name);
}

/* i2c bus recovery routines */
static int get_scl_gpio_value(struct i2c_adapter *adap)
{
	return gpiod_get_value_cansleep(adap->bus_recovery_info->scl_gpiod);
}

static void set_scl_gpio_value(struct i2c_adapter *adap, int val)
{
	gpiod_set_value_cansleep(adap->bus_recovery_info->scl_gpiod, val);
}

static int get_sda_gpio_value(struct i2c_adapter *adap)
{
	return gpiod_get_value_cansleep(adap->bus_recovery_info->sda_gpiod);
}

static void set_sda_gpio_value(struct i2c_adapter *adap, int val)
{
	gpiod_set_value_cansleep(adap->bus_recovery_info->sda_gpiod, val);
}

static int i2c_generic_bus_free(struct i2c_adapter *adap)
{
	struct i2c_bus_recovery_info *bri = adap->bus_recovery_info;
	int ret = -EOPNOTSUPP;

	if (bri->get_bus_free)
		ret = bri->get_bus_free(adap);
	else if (bri->get_sda)
		ret = bri->get_sda(adap);

	if (ret < 0)
		return ret;

	return ret ? 0 : -EBUSY;
}

/*
 * We are generating clock pulses. ndelay() determines durating of clk pulses.
 * We will generate clock with rate 100 KHz and so duration of both clock levels
 * is: delay in ns = (10^6 / 100) / 2
 */
#define RECOVERY_NDELAY		5000
#define RECOVERY_CLK_CNT	9

/* i2c recovery 操作 */
int i2c_generic_scl_recovery(struct i2c_adapter *adap)
{
	struct i2c_bus_recovery_info *bri = adap->bus_recovery_info;
	int i = 0, scl = 1, ret = 0;

	if (bri->prepare_recovery)
		bri->prepare_recovery(adap);
	if (bri->pinctrl)
		pinctrl_select_state(bri->pinctrl, bri->pins_gpio);

	/*
	 * If we can set SDA, we will always create a STOP to ensure additional
	 * pulses will do no harm. This is achieved by letting SDA follow SCL
	 * half a cycle later. Check the 'incomplete_write_byte' fault injector
	 * for details. Note that we must honour tsu:sto, 4us, but lets use 5us
	 * here for simplicity.
	 */
	bri->set_scl(adap, scl);
	ndelay(RECOVERY_NDELAY);
	if (bri->set_sda)
		bri->set_sda(adap, scl);
	ndelay(RECOVERY_NDELAY / 2);

	/*
	 * By this time SCL is high, as we need to give 9 falling-rising edges
	 */
	while (i++ < RECOVERY_CLK_CNT * 2) {
		if (scl) {
			/* SCL shouldn't be low here */
			if (!bri->get_scl(adap)) {
				dev_err(&adap->dev,
					"SCL is stuck low, exit recovery\n");
				ret = -EBUSY;
				break;
			}
		}

		scl = !scl;
		bri->set_scl(adap, scl);
		/* Creating STOP again, see above */
		if (scl)  {
			/* Honour minimum tsu:sto */
			ndelay(RECOVERY_NDELAY);
		} else {
			/* Honour minimum tf and thd:dat */
			ndelay(RECOVERY_NDELAY / 2);
		}
		if (bri->set_sda)
			bri->set_sda(adap, scl);
		ndelay(RECOVERY_NDELAY / 2);

		if (scl) {
			ret = i2c_generic_bus_free(adap);
			if (ret == 0)
				break;
		}
	}

	/* If we can't check bus status, assume recovery worked */
	if (ret == -EOPNOTSUPP)
		ret = 0;

	if (bri->unprepare_recovery)
		bri->unprepare_recovery(adap);
	if (bri->pinctrl)
		pinctrl_select_state(bri->pinctrl, bri->pins_default);

	return ret;
}
EXPORT_SYMBOL_GPL(i2c_generic_scl_recovery);

int i2c_recover_bus(struct i2c_adapter *adap)
{
	if (!adap->bus_recovery_info)
		return -EBUSY;

	dev_dbg(&adap->dev, "Trying i2c bus recovery\n");
	return adap->bus_recovery_info->recover_bus(adap);
}
EXPORT_SYMBOL_GPL(i2c_recover_bus);

/* i2c recovery gpio pinctrl 初始化 */
static void i2c_gpio_init_pinctrl_recovery(struct i2c_adapter *adap)
{
	struct i2c_bus_recovery_info *bri = adap->bus_recovery_info;
	struct device *dev = &adap->dev;
	struct pinctrl *p = bri->pinctrl;

	/*
	 * we can't change states without pinctrl, so remove the states if
	 * populated
	 */
	if (!p) {
		bri->pins_default = NULL;
		bri->pins_gpio = NULL;
		return;
	}

	if (!bri->pins_default) {
		bri->pins_default = pinctrl_lookup_state(p,
							 PINCTRL_STATE_DEFAULT);
		if (IS_ERR(bri->pins_default)) {
			dev_dbg(dev, PINCTRL_STATE_DEFAULT " state not found for GPIO recovery\n");
			bri->pins_default = NULL;
		}
	}
	if (!bri->pins_gpio) {
		bri->pins_gpio = pinctrl_lookup_state(p, "gpio");
		if (IS_ERR(bri->pins_gpio))
			bri->pins_gpio = pinctrl_lookup_state(p, "recovery");

		if (IS_ERR(bri->pins_gpio)) {
			dev_dbg(dev, "no gpio or recovery state found for GPIO recovery\n");
			bri->pins_gpio = NULL;
		}
	}

	/* for pinctrl state changes, we need all the information */
	if (bri->pins_default && bri->pins_gpio) {
		dev_info(dev, "using pinctrl states for GPIO recovery");
	} else {
		bri->pinctrl = NULL;
		bri->pins_default = NULL;
		bri->pins_gpio = NULL;
	}
}

/* i2c recovery 相关 gpio 初始化 */
static int i2c_gpio_init_generic_recovery(struct i2c_adapter *adap)
{
	struct i2c_bus_recovery_info *bri = adap->bus_recovery_info;
	struct device *dev = &adap->dev;
	struct gpio_desc *gpiod;
	int ret = 0;

	/*
	 * don't touch the recovery information if the driver is not using
	 * generic SCL recovery
	 */
	if (bri->recover_bus && bri->recover_bus != i2c_generic_scl_recovery)
		return 0;

	/*
	 * pins might be taken as GPIO, so we should inform pinctrl about
	 * this and move the state to GPIO
	 */
	if (bri->pinctrl)
		pinctrl_select_state(bri->pinctrl, bri->pins_gpio);

	/*
	 * if there is incomplete or no recovery information, see if generic
	 * GPIO recovery is available
	 */
	if (!bri->scl_gpiod) {
		gpiod = devm_gpiod_get(dev, "scl", GPIOD_OUT_HIGH_OPEN_DRAIN);
		if (PTR_ERR(gpiod) == -EPROBE_DEFER) {
			ret  = -EPROBE_DEFER;
			goto cleanup_pinctrl_state;
		}
		if (!IS_ERR(gpiod)) {
			bri->scl_gpiod = gpiod;
			bri->recover_bus = i2c_generic_scl_recovery;
			dev_info(dev, "using generic GPIOs for recovery\n");
		}
	}

	/* SDA GPIOD line is optional, so we care about DEFER only */
	if (!bri->sda_gpiod) {
		/*
		 * We have SCL. Pull SCL low and wait a bit so that SDA glitches
		 * have no effect.
		 */
		gpiod_direction_output(bri->scl_gpiod, 0);
		udelay(10);
		gpiod = devm_gpiod_get(dev, "sda", GPIOD_IN);

		/* Wait a bit in case of a SDA glitch, and then release SCL. */
		udelay(10);
		gpiod_direction_output(bri->scl_gpiod, 1);

		if (PTR_ERR(gpiod) == -EPROBE_DEFER) {
			ret = -EPROBE_DEFER;
			goto cleanup_pinctrl_state;
		}
		if (!IS_ERR(gpiod))
			bri->sda_gpiod = gpiod;
	}

cleanup_pinctrl_state:
	/* change the state of the pins back to their default state */
	if (bri->pinctrl)
		pinctrl_select_state(bri->pinctrl, bri->pins_default);

	return ret;
}

/* i2c recovery 相关gpio 初始化 */
static int i2c_gpio_init_recovery(struct i2c_adapter *adap)
{
	i2c_gpio_init_pinctrl_recovery(adap);
	return i2c_gpio_init_generic_recovery(adap);
}

/* 初始化 i2c recovery 相关回调 */
static int i2c_init_recovery(struct i2c_adapter *adap)
{
	struct i2c_bus_recovery_info *bri = adap->bus_recovery_info;
	bool is_error_level = true;
	char *err_str;

    /* 主机 adapter 未提供相应结构，则直接退出 */
	if (!bri)
		return 0;

    /* 初始化 gpio, 可能因gpio未初始化完成而需要延迟加载 */
	if (i2c_gpio_init_recovery(adap) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (!bri->recover_bus) {
		err_str = "no suitable method provided";
		is_error_level = false;
		goto err;
	}

	if (bri->scl_gpiod && bri->recover_bus == i2c_generic_scl_recovery) {
		bri->get_scl = get_scl_gpio_value;
		bri->set_scl = set_scl_gpio_value;
		if (bri->sda_gpiod) {
			bri->get_sda = get_sda_gpio_value;
			/* FIXME: add proper flag instead of '0' once available */
			if (gpiod_get_direction(bri->sda_gpiod) == 0)
				bri->set_sda = set_sda_gpio_value;
		}
	} else if (bri->recover_bus == i2c_generic_scl_recovery) {
		/* Generic SCL recovery */
		if (!bri->set_scl || !bri->get_scl) {
			err_str = "no {get|set}_scl() found";
			goto err;
		}
		if (!bri->set_sda && !bri->get_sda) {
			err_str = "either get_sda() or set_sda() needed";
			goto err;
		}
	}

	return 0;
 err:
	if (is_error_level)
		dev_err(&adap->dev, "Not using recovery: %s\n", err_str);
	else
		dev_dbg(&adap->dev, "Not using recovery: %s\n", err_str);
	adap->bus_recovery_info = NULL;

	return -EINVAL;
}

/* smbus 通知中断机制
 * 通过从机 addr 转换为 主机中断域中的中断号 */
static int i2c_smbus_host_notify_to_irq(const struct i2c_client *client)
{
	struct i2c_adapter *adap = client->adapter;
	unsigned int irq;

	if (!adap->host_notify_domain)
		return -ENXIO;

	if (client->flags & I2C_CLIENT_TEN)
		return -EINVAL;

	irq = irq_create_mapping(adap->host_notify_domain, client->addr);

	return irq > 0 ? irq : -ENXIO;
}

/* i2c bus probe 回调函数
 * 当 i2c 设备和驱动匹配成功后，将调用本函数 */
static int i2c_device_probe(struct device *dev)
{
    /* 检查设备是否为 i2c client 设备, 其它设备处理不了 */
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_driver	*driver;
	int status;

	if (!client)
		return 0;

    /* init_irq 是在 i2c_board_info 中由 irq 提供
     * 或者通过 resources 中解析得到的,
     * irq 是真正使用的, 所以这里把 init_irq 赋值给 irq */
	client->irq = client->init_irq;

    /* 如果并没有设置过 irq, 也就是 i2c_board_info 中并未提供 init_irq
     * 这里就进一步处理 irq */
	if (!client->irq) {
		int irq = -ENOENT;

		if (client->flags & I2C_CLIENT_HOST_NOTIFY) {
            /* 这个好像是 smbus 的 alert 机制
             * 但看内核中用的非常少 
             * 大概分析下，就是安装了一个中断，中断域是 i2c 主机 adapter */
			dev_dbg(dev, "Using Host Notify IRQ\n");
			/* Keep adapter active when Host Notify is required */
			pm_runtime_get_sync(&client->adapter->dev);
			irq = i2c_smbus_host_notify_to_irq(client);
		} else if (dev->of_node) {
            /* 设备具有设备树节点，则从设备树中查找中断号
             * 这里默认查找的中断名为 "irq" 当然,可能查不到。。。 */
			irq = of_irq_get_byname(dev->of_node, "irq");
            /* 如果没有查找到指定名的中断, 则默认获取第0个中断
             * 当然, 这里也有可能获取不到.... */
			if (irq == -EINVAL || irq == -ENODATA)
				irq = of_irq_get(dev->of_node, 0);
		} else if (ACPI_COMPANION(dev)) {
            /* ACPI 方式, arm 中不支持 */
			irq = i2c_acpi_get_irq(client);
		}
        /* 如果处理irq 过程中返回错误为 EPROBE_DEFER
         * 则表示需要延迟 probe 驱动, 则退出本次 probe */
		if (irq == -EPROBE_DEFER) {
			status = irq;
			goto put_sync_adapter;
		}

        /* irq < 0 为错误状态, 将其置为无效中断 0 */
		if (irq < 0)
			irq = 0;

        /* client->irq 使用最终确认的 irq */
		client->irq = irq;
	}

    /* 在调用 probe 之前, 设备与驱动已经匹配了
     * dev->driver 就是匹配的驱动, 这里找到对应的 i2c_driver */
	driver = to_i2c_driver(dev->driver);

	/*
	 * An I2C ID table is not mandatory, if and only if, a suitable OF
	 * or ACPI ID table is supplied for the probing device.
	 */
    /* 当且仅当为探测设备提供了合适的of或acpi id表时， i2c id 表不是强制的
     * 这里就是判断这个设备是否真的与驱动匹配成功了
     * 如果这三种匹配方式都没有成功，则说明根本没有匹配成功，返回错误 */
	if (!driver->id_table &&
	    !acpi_driver_match_device(dev, dev->driver) &&
	    !i2c_of_match_device(dev->driver->of_match_table, client)) {
		status = -ENODEV;
		goto put_sync_adapter;
	}

    /* 从机支持 wakeup 
     * 则解析 wakeup 对应的 irq */
	if (client->flags & I2C_CLIENT_WAKE) {
		int wakeirq;

        /* 获取 wakeup irq
         * EPROBE_DEFER 表示需延时 probe */
		wakeirq = of_irq_get_byname(dev->of_node, "wakeup");
		if (wakeirq == -EPROBE_DEFER) {
			status = wakeirq;
			goto put_sync_adapter;
		}

		device_init_wakeup(&client->dev, true);

        /* 获取到单独的 wakeirq, 则设置 wakeirq
         * 若没有单独的 wakeirq, 则与 client->irq 共享
         * 没有任何 irq, 则不设置任何 irq */
		if (wakeirq > 0 && wakeirq != client->irq)
			status = dev_pm_set_dedicated_wake_irq(dev, wakeirq);
		else if (client->irq > 0)
			status = dev_pm_set_wake_irq(dev, client->irq);
		else
			status = 0;

		if (status)
			dev_warn(&client->dev, "failed to set up wakeup irq\n");
	}

	dev_dbg(dev, "probe\n");

    /* 设置设备的时钟配置 */
	status = of_clk_set_defaults(dev->of_node, false);
	if (status < 0)
		goto err_clear_wakeup_irq;

    /* 将设备附加到它的电源域上 */
	status = dev_pm_domain_attach(&client->dev, true);
	if (status)
		goto err_clear_wakeup_irq;

    /* 创建一个设备资源组,
     * 这可以方便的释放所有设备相关的资源 
     * 这是一个开放的资源组，所有的资源都在这个组里
     * 所以可以使用 devres_release_group 一次释放所有资源 */
	client->devres_group_id = devres_open_group(&client->dev, NULL,
						    GFP_KERNEL);
	if (!client->devres_group_id) {
		status = -ENOMEM;
		goto err_detach_pm_domain;
	}

	/*
	 * When there are no more users of probe(),
	 * rename probe_new to probe.
	 */
    /* probe_new 是新的驱动模型接口, 只有一个参数
     * 优先调用新的接口 probe_new
     * 老的接口 probe 两个参数，第二个参数为匹配到的 id_table 元素
     * 在使用设备树时，第二个参数不应使用,应该使用设备接口获取匹配到的 of_match_table */
	if (driver->probe_new)
		status = driver->probe_new(client);
	else if (driver->probe)
		status = driver->probe(client,
				       i2c_match_id(driver->id_table, client));
	else
		status = -EINVAL;

	/*
	 * Note that we are not closing the devres group opened above so
	 * even resources that were attached to the device after probe is
	 * run are released when i2c_device_remove() is executed. This is
	 * needed as some drivers would allocate additional resources,
	 * for example when updating firmware.
	 */

	if (status)
		goto err_release_driver_resources;

	return 0;

err_release_driver_resources:
	devres_release_group(&client->dev, client->devres_group_id);
err_detach_pm_domain:
	dev_pm_domain_detach(&client->dev, true);
err_clear_wakeup_irq:
	dev_pm_clear_wake_irq(&client->dev);
	device_init_wakeup(&client->dev, false);
put_sync_adapter:
	if (client->flags & I2C_CLIENT_HOST_NOTIFY)
		pm_runtime_put_sync(&client->adapter->dev);

	return status;
}

/* i2c 从设备移除回调 */
static void i2c_device_remove(struct device *dev)
{
	struct i2c_client	*client = to_i2c_client(dev);
	struct i2c_driver	*driver;

    /* 这里不需要检查 i2c_client 了
     * 能调用到 remove 函数能确定是 i2c 从设备了 */
	driver = to_i2c_driver(dev->driver);
	if (driver->remove) {
		int status;

		dev_dbg(dev, "remove\n");

        /* 先调用从设备驱动提供的 remove 回调，驱动做一些处理 */
		status = driver->remove(client);
		if (status)
			dev_warn(dev, "remove failed (%pe), will be ignored\n", ERR_PTR(status));
	}

    /* 释放从设备的 devres groups
     * 这将释放设备相关的所有资源(devres 方式获取的) */
	devres_release_group(&client->dev, client->devres_group_id);

    /* 电源管理相关的处理 */
	dev_pm_domain_detach(&client->dev, true);

	dev_pm_clear_wake_irq(&client->dev);
	device_init_wakeup(&client->dev, false);

	client->irq = 0;
	if (client->flags & I2C_CLIENT_HOST_NOTIFY)
		pm_runtime_put(&client->adapter->dev);
}

/* i2c 从设备的 shutdown 回调
 * 处理比较简单，直接调用 shutdown 或者 关闭从设备的中断响应 */
static void i2c_device_shutdown(struct device *dev)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct i2c_driver *driver;

	if (!client || !dev->driver)
		return;
	driver = to_i2c_driver(dev->driver);
	if (driver->shutdown)
		driver->shutdown(client);
	else if (client->irq > 0)
		disable_irq(client->irq);
}

/* i2c client 从设备的 设备模型 release 回调 */
static void i2c_client_dev_release(struct device *dev)
{
    /* 释放在注册 i2c_client 时分配的设备结构内存 */
	kfree(to_i2c_client(dev));
}

/* sysfs name 属性, 用于显示设备名
 * 会检查设备为 从机 还是 适配器 */
static ssize_t
name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    /* 通过设备的 type 区分
     * 具有 i2c_client_type 或 i2c_adapter_type */
	return sprintf(buf, "%s\n", dev->type == &i2c_client_type ?
		       to_i2c_client(dev)->name : to_i2c_adapter(dev)->name);
}
static DEVICE_ATTR_RO(name);

/* sysfs modalias 属性
 * 该属性只有 i2c 从设备有 */
static ssize_t
modalias_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int len;

    /* 设备树提供的设备
     * 提供设备树相关的设备信息
     * 打印信息形式如下: of:Npca-gpio0T(null)Cnxp,pca9555 
     * N后面是设备树的节点名
     * T后面是设备的类型
     * C后面是设备的 compatible */
	len = of_device_modalias(dev, buf, PAGE_SIZE);
	if (len != -ENODEV)
		return len;

    /* acpi arm 不支持 */
	len = acpi_device_modalias(dev, buf, PAGE_SIZE - 1);
	if (len != -ENODEV)
		return len;

	return sprintf(buf, "%s%s\n", I2C_MODULE_PREFIX, client->name);
}
static DEVICE_ATTR_RO(modalias);

/* i2c 从设备的属性组, 所有从设备都具有的默认属性 */
static struct attribute *i2c_dev_attrs[] = {
	&dev_attr_name.attr,
	/* modalias helps coldplug:  modprobe $(cat .../modalias) */
	&dev_attr_modalias.attr,
	NULL
};
ATTRIBUTE_GROUPS(i2c_dev);

/* i2c bus 
 * 所有 i2c 设备和驱动都挂到该bus上
 * 由该bus完成匹配及probe调用 */
struct bus_type i2c_bus_type = {
	.name		= "i2c",
	.match		= i2c_device_match,
	.probe		= i2c_device_probe,
	.remove		= i2c_device_remove,
	.shutdown	= i2c_device_shutdown,
};
EXPORT_SYMBOL_GPL(i2c_bus_type);

/* i2c client 
 * 所有的 i2c 设备, 都是这种类型的
 * 可用于判断一个 device 是否为 i2c_client 设备 */
struct device_type i2c_client_type = {
	.groups		= i2c_dev_groups,
	.uevent		= i2c_device_uevent,
	.release	= i2c_client_dev_release,
};
EXPORT_SYMBOL_GPL(i2c_client_type);


/**
 * i2c_verify_client - return parameter as i2c_client, or NULL
 * @dev: device, probably from some driver model iterator
 *
 * When traversing the driver model tree, perhaps using driver model
 * iterators like @device_for_each_child(), you can't assume very much
 * about the nodes you find.  Use this function to avoid oopses caused
 * by wrongly treating some non-I2C device as an i2c_client.
 */
/* 检查设备是否为 i2c 设备, 如果是, 则返回 i2c_client 指针 */
struct i2c_client *i2c_verify_client(struct device *dev)
{
	return (dev->type == &i2c_client_type)
			? to_i2c_client(dev)
			: NULL;
}
EXPORT_SYMBOL(i2c_verify_client);


/* Return a unique address which takes the flags of the client into account */
/* 返回一个唯一的地址，该地址会考虑到客户端的标志
 * 也就是返回的地址是具有 I2C_CLIENT_* 标志的 */
static unsigned short i2c_encode_flags_to_addr(struct i2c_client *client)
{
	unsigned short addr = client->addr;

	/* For some client flags, add an arbitrary offset to avoid collisions */
	if (client->flags & I2C_CLIENT_TEN)
		addr |= I2C_ADDR_OFFSET_TEN_BIT;

	if (client->flags & I2C_CLIENT_SLAVE)
		addr |= I2C_ADDR_OFFSET_SLAVE;

	return addr;
}

/* This is a permissive address validity check, I2C address map constraints
 * are purposely not enforced, except for the general call address. */
/* 地址有效性检查
 * 检查 10位地址 与 7位地址, 7位地址屏蔽 0x00 与 0x7f */
static int i2c_check_addr_validity(unsigned int addr, unsigned short flags)
{
	if (flags & I2C_CLIENT_TEN) {
		/* 10-bit address, all values are valid */
		if (addr > 0x3ff)
			return -EINVAL;
	} else {
		/* 7-bit address, reject the general call address */
		if (addr == 0x00 || addr > 0x7f)
			return -EINVAL;
	}
	return 0;
}

/* And this is a strict address validity check, used when probing. If a
 * device uses a reserved address, then it shouldn't be probed. 7-bit
 * addressing is assumed, 10-bit address devices are rare and should be
 * explicitly enumerated. */
/* 这是一个严格的地址有效性检查，在探测时使用。如果设备使用了保留的地址，则不应
 * 该对其进行探测。假定是7位寻址，10位地址设备很少，应该明确列举。
 * 在应用层进行探测时，会有部分地址默认不进行探测，就是这些保留地址
 * 当然，可以设置对所有地址的探测 */
int i2c_check_7bit_addr_validity_strict(unsigned short addr)
{
	/*
	 * Reserved addresses per I2C specification:
	 *  0x00       General call address / START byte
	 *  0x01       CBUS address
	 *  0x02       Reserved for different bus format
	 *  0x03       Reserved for future purposes
	 *  0x04-0x07  Hs-mode master code
	 *  0x78-0x7b  10-bit slave addressing
	 *  0x7c-0x7f  Reserved for future purposes
	 */
	if (addr < 0x08 || addr > 0x77)
		return -EINVAL;
	return 0;
}

/* 检查指定的从设备地址是否与指定的设备的地址 相同 */
static int __i2c_check_addr_busy(struct device *dev, void *addrp)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	int			addr = *(int *)addrp;

    /* 待检测的从设备地址需要加入 10位地址 和 slave 标志位
     * 若与指定的从机地址相同, 则认为已被使用 */
	if (client && i2c_encode_flags_to_addr(client) == addr)
		return -EBUSY;
	return 0;
}

/* walk up mux tree */
/* 向上遍历 mux 树
 * 即检查指定的从设备地址, 在整个 i2c mux 链中是否已使用 */
static int i2c_check_mux_parents(struct i2c_adapter *adapter, int addr)
{
    /* 这里又检查了父设备是否为 i2c adapter,
     * 如果是 i2c mux 级联的, 需要递归检查 */
	struct i2c_adapter *parent = i2c_parent_is_i2c_adapter(adapter);
	int result;

    /* 这里先检查本机 i2c adapter 下的所有从设备地址
     * 遍历所有的子设备, 并使用 __i2c_check_addr_busy 检查 */
	result = device_for_each_child(&adapter->dev, &addr,
					__i2c_check_addr_busy);

    /* 如果指定地址在本机 adapter 中并未使用,
     * 并且父设备还是 i2c adapter， 则递归检查 */
	if (!result && parent)
		result = i2c_check_mux_parents(parent, addr);

	return result;
}

/* recurse down mux tree */
/* 向下递归 mux 树 */
/* 需要在某个 i2c adapter 中注册一个从机设备
 * 如果还有下挂的 i2c mux , 那该设备不能与下挂的所有 mux 的设备地址冲突
 * 因为访问任何 i2c mux 的某个设备, 该从机设备都会收到数据的
 * 所以需要遍历所有下挂的 i2c mux, 确保没有任何重复的 */
static int i2c_check_mux_children(struct device *dev, void *addrp)
{
	int result;

    /* 如果待检测的设备还是 i2c adapter
     * 那需要遍历其下的所有设备, 当然有可能出现递归
     * 如果是 i2c client 设备了, 就直接检测地址是否冲突 */
	if (dev->type == &i2c_adapter_type)
		result = device_for_each_child(dev, addrp,
						i2c_check_mux_children);
	else
		result = __i2c_check_addr_busy(dev, addrp);

	return result;
}

/* 检查地址是否忙
 * 原理是检查指定的地址是否已有设备占用，有占用那就是忙了  */
/* 注意, 如果这个 i2c adapter 是通过 i2c mux 扩展的
 * 那它所属的父设备下挂的从设备 与 i2c mux 下的从设备不能冲突 */
static int i2c_check_addr_busy(struct i2c_adapter *adapter, int addr)
{
    /* 因为有 i2c mux 设备的存在, 需要检查其父设备是否也是 i2c adapter */
	struct i2c_adapter *parent = i2c_parent_is_i2c_adapter(adapter);
	int result = 0;

    /* 如果父设备是 i2c adapter, 需要检查父设备中的从机地址是否忙 */
	if (parent)
		result = i2c_check_mux_parents(parent, addr);

    /* 若向上遍历的 mux 中没有相同的从机地址设备, 则需要向下检查
     * 即需要向下检查 mux , 也需要检查本 adapter 下的所有从机
     * 如果从机是 i2c adapter, 还需要继续向下检查 */
	if (!result)
		result = device_for_each_child(&adapter->dev, &addr,
						i2c_check_mux_children);

	return result;
}

/**
 * i2c_adapter_lock_bus - Get exclusive access to an I2C bus segment
 * @adapter: Target I2C bus segment
 * @flags: I2C_LOCK_ROOT_ADAPTER locks the root i2c adapter, I2C_LOCK_SEGMENT
 *	locks only this branch in the adapter tree
 */
/* i2c_adapter_lock_bus - 获得对 I2C 总线的独占访问权
 * @adapter:    目标 I2C 总线 
 * @flags:      I2C_LOCK_ROOT_ADAPTER 锁定根i2c 适配器, I2C_LOCK_SEGMENT 只锁定
 * 适配器树中的这个分支
 * */
static void i2c_adapter_lock_bus(struct i2c_adapter *adapter,
				 unsigned int flags)
{
	rt_mutex_lock_nested(&adapter->bus_lock, i2c_adapter_depth(adapter));
}

/**
 * i2c_adapter_trylock_bus - Try to get exclusive access to an I2C bus segment
 * @adapter: Target I2C bus segment
 * @flags: I2C_LOCK_ROOT_ADAPTER trylocks the root i2c adapter, I2C_LOCK_SEGMENT
 *	trylocks only this branch in the adapter tree
 */
/* i2c_adapter_trylock_bus - 尝试获得对 I2C 总线的独占访问权
 * 与 i2c_adapter_lock_bus 作用相同, 但不会阻塞 */
static int i2c_adapter_trylock_bus(struct i2c_adapter *adapter,
				   unsigned int flags)
{
	return rt_mutex_trylock(&adapter->bus_lock);
}

/**
 * i2c_adapter_unlock_bus - Release exclusive access to an I2C bus segment
 * @adapter: Target I2C bus segment
 * @flags: I2C_LOCK_ROOT_ADAPTER unlocks the root i2c adapter, I2C_LOCK_SEGMENT
 *	unlocks only this branch in the adapter tree
 */
/* i2c_adapter_unlock_bus - 释放对 I2C 总线的独占访问权
 * i2c_adapter_lock_bus 的逆向操作 */
static void i2c_adapter_unlock_bus(struct i2c_adapter *adapter,
				   unsigned int flags)
{
	rt_mutex_unlock(&adapter->bus_lock);
}

/* 设置 i2c 设备名 */
static void i2c_dev_set_name(struct i2c_adapter *adap,
			     struct i2c_client *client,
			     struct i2c_board_info const *info)
{
	struct acpi_device *adev = ACPI_COMPANION(&client->dev);

    /* 若设备提供了 dev_name, 则以 i2c-{dev_name} 命名
     * 从设备树解析的从设备，并没有 dev_name */
	if (info && info->dev_name) {
		dev_set_name(&client->dev, "i2c-%s", info->dev_name);
		return;
	}

    /* acpi arm 不支持 */
	if (adev) {
		dev_set_name(&client->dev, "i2c-%s", acpi_dev_name(adev));
		return;
	}

    /* 从设备并未指定特定的名字, 则以所属总线及从设备地址命名
     * 以设备树方式提供的从设备, 默认都是这种
     * 在 sys/bus/i2c/devices 中显示 */
	dev_set_name(&client->dev, "%d-%04x", i2c_adapter_id(adap),
		     i2c_encode_flags_to_addr(client));
}

/* 从 i2c 设备的 resource 中获取中断资源(中断号起始号) */
int i2c_dev_irq_from_resources(const struct resource *resources,
			       unsigned int num_resources)
{
	struct irq_data *irqd;
	int i;

    /* 遍历设备的所有资源, 寻找 IORESOURCE_IRQ 类型的资源 */
	for (i = 0; i < num_resources; i++) {
		const struct resource *r = &resources[i];

		if (resource_type(r) != IORESOURCE_IRQ)
			continue;

        /* 设备资源可以向需要设置的 irq 传递触发器标志。
         * IORESOURCE_BITS 的触发器标志 与 IRQF_TRIGGER 的标志 一一对应
         * 该流程并不影响获取的中断号, 只是设置额外的中断标志 */
		if (r->flags & IORESOURCE_BITS) {
			irqd = irq_get_irq_data(r->start);
			if (!irqd)
				break;

			irqd_set_trigger_type(irqd, r->flags & IORESOURCE_BITS);
		}

		return r->start;
	}

	return 0;
}

/*
 * Serialize device instantiation in case it can be instantiated explicitly
 * and by auto-detection
 */
/* 序列化设备实例化
 * 这是在注册新的从机设备时，向 i2c 子系统中添加设备时，先锁定某个从机地址
 * 保证添加设备时，该从机地址的使用是串行的，避免冲突 */
static int i2c_lock_addr(struct i2c_adapter *adap, unsigned short addr,
			 unsigned short flags)
{
    /* 只处理7位地址，10位地址直接返回
     * 类似互斥锁，通过设置 位图 中的某个bit(addr映射的)，标记该地址已被使用
     * 设置之前先检查是否已被设置, 若已被设置，则返回该地址 忙 */
	if (!(flags & I2C_CLIENT_TEN) &&
	    test_and_set_bit(addr, adap->addrs_in_instantiation))
		return -EBUSY;

	return 0;
}

/* 清除指定从机地址 正在使用标记 */
static void i2c_unlock_addr(struct i2c_adapter *adap, unsigned short addr,
			    unsigned short flags)
{
	if (!(flags & I2C_CLIENT_TEN))
		clear_bit(addr, adap->addrs_in_instantiation);
}

/**
 * i2c_new_client_device - instantiate an i2c device
 * @adap: the adapter managing the device
 * @info: describes one I2C device; bus_num is ignored
 * Context: can sleep
 *
 * Create an i2c device. Binding is handled through driver model
 * probe()/remove() methods.  A driver may be bound to this device when we
 * return from this function, or any later moment (e.g. maybe hotplugging will
 * load the driver module).  This call is not appropriate for use by mainboard
 * initialization logic, which usually runs during an arch_initcall() long
 * before any i2c_adapter could exist.
 *
 * This returns the new i2c client, which may be saved for later use with
 * i2c_unregister_device(); or an ERR_PTR to describe the error.
 */
/* i2c_new_client_device - 实例化i2c设备
 * @adap:   管理设备的适配器
 * @info:   描述了一种I2C器件；Bus_num被忽略
 *
 * 创建i2c设备。绑定是通过驱动程序模型probe()/remove（）方法处理的。当我们从这个
 * 函数返回时，驱动程序可能被绑定到这个设备上，或者任何稍后的时刻（例如，可能热
 * 插拔将加载驱动程序模块）。该调用不适合由主板初始化逻辑使用，通常在任何
 * i2c_adapter可能存在之前很久在arch_initcall（）期间运行。
 *
 * 返回值为指向新客户端的 i2c_client, 做为 i2c_unregister_device() 注销使用；
 * 如果发生错误，则使用 ERR_PTR 来描述错误
 * */
struct i2c_client *
i2c_new_client_device(struct i2c_adapter *adap, struct i2c_board_info const *info)
{
	struct i2c_client	*client;
	int			status;

    /* 分配一个 i2c client , 用于新注册的从机设备 */
	client = kzalloc(sizeof *client, GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

    /* 记录从机设备所属的适配器 adapter */
	client->adapter = adap;

    /* 平台相关数据, 这是由板级代码中提供 i2c_board_info 时提供的
     * 使用设备树时,该指针为空 */
	client->dev.platform_data = info->platform_data;
    /* 从设备的标志位, 参考 I2C_CLIENT_*  */
	client->flags = info->flags;
    /* 从设备的从机地址 */
	client->addr = info->addr;

    /* 记录设备信息中的中断号
     * 如果没有中断号, 则从 板级信息中的 resources 获取中断号 */
	client->init_irq = info->irq;
	if (!client->init_irq)
		client->init_irq = i2c_dev_irq_from_resources(info->resources,
							 info->num_resources);

    /* 使用设备信息中的 type 做为从设备 name
     * 在设备树提供的设备中, 也是将 compatible 中的型号名放到 type 中 */
	strlcpy(client->name, info->type, sizeof(client->name));

    /* 检查地址是否有效, 是否符合 7位地址 或 10位地址 有效范围  */
	status = i2c_check_addr_validity(client->addr, client->flags);
	if (status) {
		dev_err(&adap->dev, "Invalid %d-bit I2C address 0x%02hx\n",
			client->flags & I2C_CLIENT_TEN ? 10 : 7, client->addr);
		goto out_err_silent;
	}

    /* 锁定从设备地址，保证下面的注册流程对于该从机地址是原子操作 */
	status = i2c_lock_addr(adap, client->addr, client->flags);
	if (status)
		goto out_err_silent;

	/* Check for address business */
    /* 检查待注册设备的从设备地址是否已经被注册过了
     * 在一个单一总线中，从机地址必须唯一 */
	status = i2c_check_addr_busy(adap, i2c_encode_flags_to_addr(client));
	if (status)
		goto out_err;

    /* 初始化从设备的 设备模型 结构 */
    /* 设置从机设备的父设备为 所属适配器对应的设备，形成树状结构 */
	client->dev.parent = &client->adapter->dev;
    /* 设置设备所属的bus，将在后续匹配驱动时使用 */
	client->dev.bus = &i2c_bus_type;
    /* 设置设备的 type，这将影响默认属性 及 不同类型设备的处理 */
	client->dev.type = &i2c_client_type;
    /* 设置设备的设备树节点，不是设备树配置的设备此处为空 */
	client->dev.of_node = of_node_get(info->of_node);
	client->dev.fwnode = info->fwnode;

    /* 设置从设备名, 这将显示在 sysfs 中 */
	i2c_dev_set_name(adap, client, info);

    /* i2c_board_info 是否提供了 swnode 节点
     * 这是一种可以提供类似设备树中的属性一样的结构, 
     * 通过 swnode 节点，提供特定的设备配置
     * 属性结构为 property_entry */
	if (info->swnode) {
        /* 将 swnode 设置到设备中
         * 设备如果已经有了 fwnode ， 就会设置到 secondary 中 */
		status = device_add_software_node(&client->dev, info->swnode);
		if (status) {
			dev_err(&adap->dev,
				"Failed to add software node to client %s: %d\n",
				client->name, status);
			goto out_err_put_of_node;
		}
	}

    /* 从上面的流程中可知
     * of_node fwnode swnode 三个可能都不存在
     * 如果一个设备是以 i2c_board_info 方式注册, 并且并未提供 swnode 
     * 那就是这个设备没有任何的 node 了，也就没有任何属性
     * 当然了，还有一个可以提供一些信息，就是 platform_data 
     * 可以在定义 i2c_board_info 时提供 platform_data 数据，由特定的驱动解析 */

    /* 向设备驱动模型中注册设备
     * 这个过程可能发生 设备与驱动 的匹配，返回时可能已匹配驱动了 */
	status = device_register(&client->dev);
	if (status)
		goto out_remove_swnode;

	dev_dbg(&adap->dev, "client [%s] registered with bus id %s\n",
		client->name, dev_name(&client->dev));

    /* 释放从地址的引用
     * 这里已经将从设备注册成功了，就算有其它相同从机地址的设备注册
     * 也会被上面的 i2c_check_addr_busy 拦截 */
	i2c_unlock_addr(adap, client->addr, client->flags);

	return client;

out_remove_swnode:
	device_remove_software_node(&client->dev);
out_err_put_of_node:
	of_node_put(info->of_node);
out_err:
	dev_err(&adap->dev,
		"Failed to register i2c client %s at 0x%02x (%d)\n",
		client->name, client->addr, status);
	i2c_unlock_addr(adap, client->addr, client->flags);
out_err_silent:
	kfree(client);
	return ERR_PTR(status);
}
EXPORT_SYMBOL_GPL(i2c_new_client_device);

/**
 * i2c_unregister_device - reverse effect of i2c_new_*_device()
 * @client: value returned from i2c_new_*_device()
 * Context: can sleep
 */
/* i2c_unregister_device - i2c_new_*_device() 的逆向操作
 * @client: i2c_new_*_device() 返回的值
 *
 * 注销 i2c 从设备
 * */
void i2c_unregister_device(struct i2c_client *client)
{
	if (IS_ERR_OR_NULL(client))
		return;

    /* 如果设备具有设备树节点，则解引用释放节点 */
	if (client->dev.of_node) {
		of_node_clear_flag(client->dev.of_node, OF_POPULATED);
		of_node_put(client->dev.of_node);
	}

    /* acpi arm 不支持 */
	if (ACPI_COMPANION(&client->dev))
		acpi_device_clear_enumerated(ACPI_COMPANION(&client->dev));
    /* 移除 swnode，这是通过 device_add_software_node 单独添加的 */
	device_remove_software_node(&client->dev);
    /* 从设备驱动模型中注销设备，会发生设备与驱动的解绑操作 */
	device_unregister(&client->dev);
}
EXPORT_SYMBOL_GPL(i2c_unregister_device);

/**
 * i2c_find_device_by_fwnode() - find an i2c_client for the fwnode
 * @fwnode: &struct fwnode_handle corresponding to the &struct i2c_client
 *
 * Look up and return the &struct i2c_client corresponding to the @fwnode.
 * If no client can be found, or @fwnode is NULL, this returns NULL.
 *
 * The user must call put_device(&client->dev) once done with the i2c client.
 */
/* i2c_find_device_by_fwnode() - 使用 fwnode 查找所属的 i2c_client
 * @fwnode: 对应到属于 &struct i2c_client 的 &struct fwnode_handle
 *
 * 查找并返回 @fwnode 对应的 &struct i2c_client.
 * 如果找不到，或者 @fwnode为NULL，则返回 NULL
 *
 * 在使用完 i2c client 时，用户必须调用 put_device(&client->dev) 释放 
 * */
struct i2c_client *i2c_find_device_by_fwnode(struct fwnode_handle *fwnode)
{
	struct i2c_client *client;
	struct device *dev;

    /* 提供了 fwnode == NULL，则直接返回 NULL */
	if (!fwnode)
		return NULL;

    /* 遍历 i2c bus 上的所有设备，匹配设备的 fwnode */
	dev = bus_find_device_by_fwnode(&i2c_bus_type, fwnode);
	if (!dev)
		return NULL;

    /* 检查匹配到的设备是否为 i2c_client
     * 不是则释放查找到的设备 */
	client = i2c_verify_client(dev);
	if (!client)
		put_device(dev);

	return client;
}
EXPORT_SYMBOL(i2c_find_device_by_fwnode);


/* 虚拟 i2c 设备 匹配列表 */
/* 这种设备是为 单个器件 但有多个从机地址 的 这种设备设计的
 * 通过一个主地址匹配到实际的驱动，然后申请附加从地址的虚拟设备
 * 实现可以通过多个设备节点的访问
 * 典型的使用为 EEPROMS, 参考 i2c_new_dummy_device 注释 */
static const struct i2c_device_id dummy_id[] = {
	{ "dummy", 0 },
	{ "smbus_host_notify", 0 },
	{ },
};

/* 虚拟 i2c 设备驱动回调，无需做任何操作，均由实际驱动完成 */
static int dummy_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	return 0;
}

/* 虚拟 i2c 设备驱动回调，无需做任何操作，均由实际驱动完成 */
static int dummy_remove(struct i2c_client *client)
{
	return 0;
}

/* 定义一个 虚拟 i2c 设备的驱动，用于匹配虚拟 i2c 设备 */
static struct i2c_driver dummy_driver = {
	.driver.name	= "dummy",
	.probe		= dummy_probe,
	.remove		= dummy_remove,
	.id_table	= dummy_id,
};

/**
 * i2c_new_dummy_device - return a new i2c device bound to a dummy driver
 * @adapter: the adapter managing the device
 * @address: seven bit address to be used
 * Context: can sleep
 *
 * This returns an I2C client bound to the "dummy" driver, intended for use
 * with devices that consume multiple addresses.  Examples of such chips
 * include various EEPROMS (like 24c04 and 24c08 models).
 *
 * These dummy devices have two main uses.  First, most I2C and SMBus calls
 * except i2c_transfer() need a client handle; the dummy will be that handle.
 * And second, this prevents the specified address from being bound to a
 * different driver.
 *
 * This returns the new i2c client, which should be saved for later use with
 * i2c_unregister_device(); or an ERR_PTR to describe the error.
 */
/* i2c_new_dummy_device - 返回一个绑定到虚拟驱动程序的新i2c虚拟设备
 * @adapter:    所属的 adapter适配器
 * @address:    7bit 从机地址 
 *
 * 这将返回一个绑定到"虚拟"驱动程序的i2c从设备，用于消耗多个地址的设备。
 * 这种芯片的例子包括各种eeprom(如24c04 和 24c08).
 *
 * 这种虚拟设备有两个主要用途。首先，除 i2c_transfer() 外，大多数i2c和smbus调用
 * 都需要一个 clinet 句柄，虚拟设备就是这个句柄。其次，这可以防止指定的地址绑定
 * 到不同的驱动程序。有个占用地址的作用。
 *
 * 成功返回虚拟的虚拟设备 i2c_client, 使用 i2c_unregister_device 释放；
 * 失败返回 ERR_PTR 描述的错误
 * */
struct i2c_client *i2c_new_dummy_device(struct i2c_adapter *adapter, u16 address)
{
    /* 虚拟设备的板级信息 */
	struct i2c_board_info info = {
		I2C_BOARD_INFO("dummy", address),
	};

    /* 使用 i2c_new_client_device 创建设备
     * 虚拟设备与普通设备 在注册中并无差异，只是没有真实的驱动操作 */
	return i2c_new_client_device(adapter, &info);
}
EXPORT_SYMBOL_GPL(i2c_new_dummy_device);

/* 注销指定的虚拟设备 */
static void devm_i2c_release_dummy(void *client)
{
	i2c_unregister_device(client);
}

/**
 * devm_i2c_new_dummy_device - return a new i2c device bound to a dummy driver
 * @dev: device the managed resource is bound to
 * @adapter: the adapter managing the device
 * @address: seven bit address to be used
 * Context: can sleep
 *
 * This is the device-managed version of @i2c_new_dummy_device. It returns the
 * new i2c client or an ERR_PTR in case of an error.
 */
/* devm_i2c_new_dummy_device - 返回一个绑定到虚拟驱动程序的新 i2c 虚拟设备
 * @dev:    被管理的资源绑定到的设备
 * @adapter:所属的 adapter适配器
 * @address:7bit从机地址
 *
 * 是 i2c_new_dummy_device 的设备资源管理版本，与其工作一致
 * */
struct i2c_client *devm_i2c_new_dummy_device(struct device *dev,
					     struct i2c_adapter *adapter,
					     u16 address)
{
	struct i2c_client *client;
	int ret;

    /* 通过调用 i2c_new_dummy_device 实现虚拟设备创建 */
	client = i2c_new_dummy_device(adapter, address);
	if (IS_ERR(client))
		return client;

    /* 将新创建的虚拟设备加入到设备资源管理中
     * 资源释放函数为 devm_i2c_release_dummy */
	ret = devm_add_action_or_reset(dev, devm_i2c_release_dummy, client);
	if (ret)
		return ERR_PTR(ret);

	return client;
}
EXPORT_SYMBOL_GPL(devm_i2c_new_dummy_device);

/**
 * i2c_new_ancillary_device - Helper to get the instantiated secondary address
 * and create the associated device
 * @client: Handle to the primary client
 * @name: Handle to specify which secondary address to get
 * @default_addr: Used as a fallback if no secondary address was specified
 * Context: can sleep
 *
 * I2C clients can be composed of multiple I2C slaves bound together in a single
 * component. The I2C client driver then binds to the master I2C slave and needs
 * to create I2C dummy clients to communicate with all the other slaves.
 *
 * This function creates and returns an I2C dummy client whose I2C address is
 * retrieved from the platform firmware based on the given slave name. If no
 * address is specified by the firmware default_addr is used.
 *
 * On DT-based platforms the address is retrieved from the "reg" property entry
 * cell whose "reg-names" value matches the slave name.
 *
 * This returns the new i2c client, which should be saved for later use with
 * i2c_unregister_device(); or an ERR_PTR to describe the error.
 */
/* i2c_new_ancillary_device - 用于获取实例化的从机地址并创建关联的设备
 * @client: 主的从设备句柄
 * @name:   用于获取辅助地址的特殊句柄, 属性字符串
 * @default_addr:   没有指定的辅助地址时，使用的默认地址
 *
 * i2c clients 可以由多个绑定在一起的 i2c slave 组成。然后，i2c clinets 驱动程序
 * 绑定到主i2c clinet，并需要创建i2c虚拟客户端与所有其它从设备通信。
 *
 * 该函数创建并返回一个i2c虚拟从设备，根据给定的从设备地址名称从平台固件中检索其
 * i2c地址。如果固件没有指定地址，则使用 default_addr.
 *
 * 在基于dt的平台上，从 "reg" 属性条目单元格中检索地址，该单元格的 "reg-names"
 * 值与从属名称匹配。
 * 
 * 本函数用于由设备树等设备信息指定了从设备的第二或第三地址，并且由 "reg-names"
 * 列表中的名字对应了，这样从机就可以使用设备信息中的辅助地址。
 * 相比于 i2c_new_dummy_device 的处理，只是增加了从设备信息中获取地址的操作
 * */
struct i2c_client *i2c_new_ancillary_device(struct i2c_client *client,
						const char *name,
						u16 default_addr)
{
	struct device_node *np = client->dev.of_node;
	u32 addr = default_addr;
	int i;

    /* 若存在设备树，则从设备树中获取指定 名字的从机地址
     * 当然，可能没有设备树 或者 并没有指定名字的 从机地址
     * 没有获取到有效的从机地址，就是用 default_addr 了 */
	if (np) {
		i = of_property_match_string(np, "reg-names", name);
		if (i >= 0)
			of_property_read_u32_index(np, "reg", i, &addr);
	}

    /* 完全使用 i2c_new_dummy_device 进行虚拟设备注册 */
	dev_dbg(&client->adapter->dev, "Address for %s : 0x%x\n", name, addr);
	return i2c_new_dummy_device(client->adapter, addr);
}
EXPORT_SYMBOL_GPL(i2c_new_ancillary_device);

/* ------------------------------------------------------------------------- */

/* I2C bus adapters -- one roots each I2C or SMBUS segment */

/* i2c adapter 设备模型释放回调
 * 用于在注销 i2c adapter 时, 通知已确定被移除
 * 参考 i2c_del_adapter 中的处理 */
static void i2c_adapter_dev_release(struct device *dev)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	complete(&adap->dev_released);
}

/* 获取 i2c adapter 的深度
 * 适用于具有 i2c mux 的情况, 适配器下的设备可能还是适配器 */
unsigned int i2c_adapter_depth(struct i2c_adapter *adapter)
{
	unsigned int depth = 0;

    /* 向上遍历父设备, 直到不再是 i2c adapter 适配器 */
	while ((adapter = i2c_parent_is_i2c_adapter(adapter)))
		depth++;

	WARN_ONCE(depth >= MAX_LOCKDEP_SUBCLASSES,
		  "adapter depth exceeds lockdep subclass limit\n");

	return depth;
}
EXPORT_SYMBOL_GPL(i2c_adapter_depth);

/*
 * Let users instantiate I2C devices through sysfs. This can be used when
 * platform initialization code doesn't contain the proper data for
 * whatever reason. Also useful for drivers that do device detection and
 * detection fails, either because the device uses an unexpected address,
 * or this is a compatible device with different ID register values.
 *
 * Parameter checking may look overzealous, but we really don't want
 * the user to provide incorrect parameters.
 */
/* 通过用户空间实例化一个 i2c 从设备
 * 比如 echo i2c_test 0x48 > /sys/bus/i2c/devices/i2c-6/new_device
 * 这样就创建了一个名为 i2c_test 从地址为 0x48 的从设备
 *
 * 让用户通过sysfs实例化i2c设备。当平台初始化代码由于某种原因没有包含适当的数据
 * 时，可以使用该方法。对于进行设备检测的驱动也很有用，但检测失败了，可能因为设
 * 备使用了意外的地址，或者这是一个兼容的设备，具有不同的ID寄存器值。
 *
 * 参数检测可能看起来有些过分，但真的不希望用户提供不正确的参数
 *
 * 传入的参数只能是设备名和从机地址两个参数，并且参数后面还不能有空格等字符。。。
 * 否则一律报错
 * */
static ssize_t
new_device_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
    /* 用户空间实例化设备时，是在某个 适配器 下的 */
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct i2c_board_info info;
	struct i2c_client *client;
	char *blank, end;
	int res;

    /* 用于新创建设备的 i2c_board_info */
	memset(&info, 0, sizeof(struct i2c_board_info));

    /* 参数中至少包含两个内容，一个是设备名，另一个是从机地址
     * 所以这里先找一下参数分隔符 ' ' */
	blank = strchr(buf, ' ');
	if (!blank) {
		dev_err(dev, "%s: Missing parameters\n", "new_device");
		return -EINVAL;
	}
    /* 拷贝设备名到 type 中, 后续匹配使用 */
	if (blank - buf > I2C_NAME_SIZE - 1) {
		dev_err(dev, "%s: Invalid device name\n", "new_device");
		return -EINVAL;
	}
	memcpy(info.type, buf, blank - buf);

	/* Parse remaining parameters, reject extra parameters */
    /* 解析剩余参数，拒绝额外参数
     * 这里解析从设备地址信息
     * 注意, end 是地址后面的字符，在下面要判断这个字符 */
	res = sscanf(++blank, "%hi%c", &info.addr, &end);
	if (res < 1) {
		dev_err(dev, "%s: Can't parse I2C address\n", "new_device");
		return -EINVAL;
	}
    /* 检测从地址后面的字符，如果不是 '\n' 就报错。。。
     * 这也太。。。 */
	if (res > 1  && end != '\n') {
		dev_err(dev, "%s: Extra parameters\n", "new_device");
		return -EINVAL;
	}

    /* 传入的从机地址中可以标记地址类型(7位 或 10位地址) */
	if ((info.addr & I2C_ADDR_OFFSET_TEN_BIT) == I2C_ADDR_OFFSET_TEN_BIT) {
		info.addr &= ~I2C_ADDR_OFFSET_TEN_BIT;
		info.flags |= I2C_CLIENT_TEN;
	}

    /* 从机地址还可以包含是否做为从机设备使用  */
	if (info.addr & I2C_ADDR_OFFSET_SLAVE) {
		info.addr &= ~I2C_ADDR_OFFSET_SLAVE;
		info.flags |= I2C_CLIENT_SLAVE;
	}

    /* 注册新的 i2c 从设备, client 指向新设备结构 */
	client = i2c_new_client_device(adap, &info);
	if (IS_ERR(client))
		return PTR_ERR(client);

	/* Keep track of the added device */
    /* 将用户空间新注册的从设备结构挂到所属适配器的 userspace_clients 链表上
     * 这个可以用于以后注销适配器时统一释放从设备 */
	mutex_lock(&adap->userspace_clients_lock);
	list_add_tail(&client->detected, &adap->userspace_clients);
	mutex_unlock(&adap->userspace_clients_lock);
	dev_info(dev, "%s: Instantiated device %s at 0x%02hx\n", "new_device",
		 info.type, info.addr);

	return count;
}
static DEVICE_ATTR_WO(new_device);

/*
 * And of course let the users delete the devices they instantiated, if
 * they got it wrong. This interface can only be used to delete devices
 * instantiated by i2c_sysfs_new_device above. This guarantees that we
 * don't delete devices to which some kernel code still has references.
 *
 * Parameter checking may look overzealous, but we really don't want
 * the user to delete the wrong device.
 */
/* 当然, 如果用户做错了，可以让他们删除实例化的设备。该接口只能用于删除上面
 * i2c_sysfs_new_device 实例化的设备。这保证了我们不会删除某些内核代码任然对齐引
 * 用的设备。
 *
 * 这是 i2c_sysfs_new_device 的逆向操作, 从用户空间移除从机设备 
 * 需要注意，提供的地址必须与 i2c_sysfs_new_device 时一致, 包含标志位
 * */
static ssize_t
delete_device_store(struct device *dev, struct device_attribute *attr,
		    const char *buf, size_t count)
{
    /* 用户空间实例化设备时，是在某个 适配器 下的 */
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct i2c_client *client, *next;
	unsigned short addr;
	char end;
	int res;

	/* Parse parameters, reject extra parameters */
    /* 同 i2c_sysfs_new_device 一样, 参数检查很严格
     * 提供的只能是从机地址，并且只能有从机地址 */
	res = sscanf(buf, "%hi%c", &addr, &end);
	if (res < 1) {
		dev_err(dev, "%s: Can't parse I2C address\n", "delete_device");
		return -EINVAL;
	}
	if (res > 1  && end != '\n') {
		dev_err(dev, "%s: Extra parameters\n", "delete_device");
		return -EINVAL;
	}

	/* Make sure the device was added through sysfs */
    /* 需要确保指定移除的设备 是从用户空间添加的
     * 这里检查 userspace_clients 链表中的从设备, 确认地址是否相同 */
	res = -ENOENT;
	mutex_lock_nested(&adap->userspace_clients_lock,
			  i2c_adapter_depth(adap));
	list_for_each_entry_safe(client, next, &adap->userspace_clients,
				 detected) {
        /* 检查从机地址是否完全一致, 完全一致才认为是指定移除的设备 */
		if (i2c_encode_flags_to_addr(client) == addr) {
			dev_info(dev, "%s: Deleting device %s at 0x%02hx\n",
				 "delete_device", client->name, client->addr);

            /* 从链表及 i2c 子系统中移除从设备 */
			list_del(&client->detected);
			i2c_unregister_device(client);
			res = count;
			break;
		}
	}
	mutex_unlock(&adap->userspace_clients_lock);

	if (res < 0)
		dev_err(dev, "%s: Can't find device in list\n",
			"delete_device");
	return res;
}
static DEVICE_ATTR_IGNORE_LOCKDEP(delete_device, S_IWUSR, NULL,
				  delete_device_store);

/* i2c adapter 默认属性
 * 用于显示设备名, 注册新设备, 移除新注册的设备 */
static struct attribute *i2c_adapter_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_new_device.attr,
	&dev_attr_delete_device.attr,
	NULL
};
ATTRIBUTE_GROUPS(i2c_adapter);

/* i2c adapter 设备类型 
 * 所有的 适配器 设备模型 device.type 都指向本结构 */
struct device_type i2c_adapter_type = {
	.groups		= i2c_adapter_groups,
	.release	= i2c_adapter_dev_release,
};
EXPORT_SYMBOL_GPL(i2c_adapter_type);

/**
 * i2c_verify_adapter - return parameter as i2c_adapter or NULL
 * @dev: device, probably from some driver model iterator
 *
 * When traversing the driver model tree, perhaps using driver model
 * iterators like @device_for_each_child(), you can't assume very much
 * about the nodes you find.  Use this function to avoid oopses caused
 * by wrongly treating some non-I2C device as an i2c_adapter.
 */
/* i2c_verify_adapter - 返回 所属的 i2c_adapter 或者 NULL
 * @dev:    设备结构，属于某个 i2c_adapter 
 *
 * 当遍历驱动程序模型树时，可能使用 @device_for_each_child() 这样的驱动程序模型
 * 迭代器，不能对找到的节点做太多假设。使用该函数可以避免错误的将某些非i2c设备视
 * 为i2c_adapter而导致的错误。
 *
 * 本函数用于在遍历设备时，通过检查设备的 type，确保设备确实为 i2c_adapter
 * 而不是认为在某个bus上遍历到的设备都是某种类型
 * */
struct i2c_adapter *i2c_verify_adapter(struct device *dev)
{
    /* 所有的 i2c_adapter 设备都时 i2c_adapter_type 类型的 */
	return (dev->type == &i2c_adapter_type)
			? to_i2c_adapter(dev)
			: NULL;
}
EXPORT_SYMBOL(i2c_verify_adapter);

#ifdef CONFIG_I2C_COMPAT
/* i2c 兼容class，创建一个 i2c-adapter class 
 * 用于兼容老的应用程序，在以后的版本中已经被删除了 */
static struct class_compat *i2c_adapter_compat_class;
#endif

/* 遍历使用 i2c_register_board_info 注册的从设备板级结构
 * 找到与新注册的适配bus号相同的从设备, 并注册从设备 */
static void i2c_scan_static_board_info(struct i2c_adapter *adapter)
{
	struct i2c_devinfo	*devinfo;

    /* 所有静态注册的从设备都挂在 __i2c_board_list 链表上
     * 逐个遍历，若与适配器 bus 号一致，则进行从设备注册 */
	down_read(&__i2c_board_lock);
	list_for_each_entry(devinfo, &__i2c_board_list, list) {
		if (devinfo->busnum == adapter->nr &&
		    IS_ERR(i2c_new_client_device(adapter, &devinfo->board_info)))
			dev_err(&adapter->dev,
				"Can't create device at 0x%02x\n",
				devinfo->board_info.addr);
	}
	up_read(&__i2c_board_lock);
}

/* 处理 适配器 的从设备探测
 * 在新注册适配器或者驱动时，将调用该函数进行设备探测
 * 当然，驱动程序必须支持探测功能 */
static int i2c_do_add_adapter(struct i2c_driver *driver,
			      struct i2c_adapter *adap)
{
	/* Detect supported devices on that bus, and instantiate them */
    /* 检测驱动支持的设备，并实例化 */
	i2c_detect(adap, driver);

	return 0;
}

/* 这是在注册 adapter 时，遍历 i2c bus 上的所有 driver 时的回调
 * 会对每一个 driver 调用一次本函数
 * 该方法用于由支持探测功能的驱动程序 探测新注册 适配器下的从设备
 * 若探测成功，则实例化从设备
 * 第一个参数 d 为遍历到的 driver
 * 第二个参数 data 为 adapter */
static int __process_new_adapter(struct device_driver *d, void *data)
{
    /* 转换为 i2c_driver , 调用 i2c_do_add_adapter 完成操作 */
	return i2c_do_add_adapter(to_i2c_driver(d), data);
}

/* i2c 适配器的总线锁定处理
 * 用于某个器件对某条总线或整个适配器的独占访问 */
static const struct i2c_lock_operations i2c_adapter_lock_ops = {
	.lock_bus =    i2c_adapter_lock_bus,
	.trylock_bus = i2c_adapter_trylock_bus,
	.unlock_bus =  i2c_adapter_unlock_bus,
};

/* 注销主机通知的软irq */
static void i2c_host_notify_irq_teardown(struct i2c_adapter *adap)
{
	struct irq_domain *domain = adap->host_notify_domain;
	irq_hw_number_t hwirq;

	if (!domain)
		return;

    /* 解除 irq 映射，释放 irq */
	for (hwirq = 0 ; hwirq < I2C_ADDR_7BITS_COUNT ; hwirq++)
		irq_dispose_mapping(irq_find_mapping(domain, hwirq));

    /* 移除 irq domain, i2c 适配 通知irq domain 清空 */
	irq_domain_remove(domain);
	adap->host_notify_domain = NULL;
}

/* i2c 适配器的 通知irq domain 映射回调 */
static int i2c_host_notify_irq_map(struct irq_domain *h,
					  unsigned int virq,
					  irq_hw_number_t hw_irq_num)
{
	irq_set_chip_and_handler(virq, &dummy_irq_chip, handle_simple_irq);

	return 0;
}

static const struct irq_domain_ops i2c_host_notify_irq_ops = {
	.map = i2c_host_notify_irq_map,
};

/* 注册用于主机通知的软irq */
static int i2c_setup_host_notify_irq_domain(struct i2c_adapter *adap)
{
	struct irq_domain *domain;

    /* 主机 adapter 需要具备 I2C_FUNC_SMBUS_HOST_NOTIFY 能力
     * 不具备该能力的, 不注册 irq , 返回成功 */
	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_HOST_NOTIFY))
		return 0;

    /* 注册 irq domain, 失败则返回错误 */
	domain = irq_domain_create_linear(adap->dev.parent->fwnode,
					  I2C_ADDR_7BITS_COUNT,
					  &i2c_host_notify_irq_ops, adap);
	if (!domain)
		return -ENOMEM;

    /* 记录主机的 irq domain */
	adap->host_notify_domain = domain;

	return 0;
}

/**
 * i2c_handle_smbus_host_notify - Forward a Host Notify event to the correct
 * I2C client.
 * @adap: the adapter
 * @addr: the I2C address of the notifying device
 * Context: can't sleep
 *
 * Helper function to be called from an I2C bus driver's interrupt
 * handler. It will schedule the Host Notify IRQ.
 */
/* i2c_handle_smbus_host_notify - 将主机通知事件转发给正确的用户
 * @adap:   主机 adapter
 * @addr:   通知设备的I2C地址
 *
 * 从I2C总线驱动程序的中断处理程序调用的辅助函数。它将调度主机通知IRQ。
 * */
int i2c_handle_smbus_host_notify(struct i2c_adapter *adap, unsigned short addr)
{
	int irq;

	if (!adap)
		return -EINVAL;

    /* 使用从机addr 查找中断号 */
	irq = irq_find_mapping(adap->host_notify_domain, addr);
	if (irq <= 0)
		return -ENXIO;

    /* 使用查找到的中断号产生中断请求 */
	generic_handle_irq(irq);

	return 0;
}
EXPORT_SYMBOL_GPL(i2c_handle_smbus_host_notify);

/* 注册 i2c adapter
 * adap->nr 已初始化的 */
static int i2c_register_adapter(struct i2c_adapter *adap)
{
	int res = -EINVAL;

	/* Can't register until after driver model init */
    /* i2c 子系统未初始化完成时，不允许注册，返回失败 */
	if (WARN_ON(!is_registered)) {
		res = -EAGAIN;
		goto out_list;
	}

	/* Sanity checks */
    /* i2c adapter 要有个名字，没有初始化名字返回失败 */
	if (WARN(!adap->name[0], "i2c adapter has no name"))
		goto out_list;

    /* i2c adapter algo 不可为空
     * algo 是总线的操作方法，为空没有意义了 */
	if (!adap->algo) {
		pr_err("adapter '%s': no algo supplied!\n", adap->name);
		goto out_list;
	}

    /* i2c adapter 的总线锁操作
     * 用于某个设备独占总线使用
     * 若 adapter 未提供，则使用默认的锁处理操作 */
	if (!adap->lock_ops)
		adap->lock_ops = &i2c_adapter_lock_ops;

    /* 一些内部使用的锁初始化 */
	adap->locked_flags = 0;
	rt_mutex_init(&adap->bus_lock);
	rt_mutex_init(&adap->mux_lock);
	mutex_init(&adap->userspace_clients_lock);
	INIT_LIST_HEAD(&adap->userspace_clients);

	/* Set default timeout to 1 second if not already set */
    /* 如果超时时间未设置，则默认设置为 1s */
	if (adap->timeout == 0)
		adap->timeout = HZ;

	/* register soft irqs for Host Notify */
    /* 注册用于主机通知的软irq
     * 注意，这里并不一定真的注册 irq domain 了
     * 如果主机 adapter 不支持, 也不会注册的, 但返回成功 */
	res = i2c_setup_host_notify_irq_domain(adap);
	if (res) {
		pr_err("adapter '%s': can't create Host Notify IRQs (%d)\n",
		       adap->name, res);
		goto out_list;
	}

    /* i2c adapter 基本信息处理完成，开始向设备模型注册设备
     * 所有的 adapter 命名均为 i2c-x 形式
     * 所属bus为 i2c_bus_type 
     * 设备类型为 i2c_adapter_type 
     * */
	dev_set_name(&adap->dev, "i2c-%d", adap->nr);
	adap->dev.bus = &i2c_bus_type;
	adap->dev.type = &i2c_adapter_type;
	res = device_register(&adap->dev);
	if (res) {
		pr_err("adapter '%s': can't register device (%d)\n", adap->name, res);
		goto out_list;
	}

    /* 创建一个 debugfs 目录，以 设备名 命名， 在 i2c 目录下 */
	adap->debugfs = debugfs_create_dir(dev_name(&adap->dev), i2c_debugfs_root);

    /* smbus 的 alert 机制，单纯的 i2c bus 不用关注 */
	res = i2c_setup_smbus_alert(adap);
	if (res)
		goto out_reg;

    /* 电源管理相关，不是主要流程，不关注 */
	pm_runtime_no_callbacks(&adap->dev);
	pm_suspend_ignore_children(&adap->dev, true);
	pm_runtime_enable(&adap->dev);

    /* i2c recovery 恢复机制初始化 */
	res = i2c_init_recovery(adap);
	if (res == -EPROBE_DEFER)
		goto out_reg;

    /* 完成了上面的操作，就已经完成了 i2c adapter 的注册
     * 核心还是将 device 注册到设备模型中 */
	dev_dbg(&adap->dev, "adapter [%s] registered\n", adap->name);

    /* 创建新注册设备与其父设备的链接文件，兼容使用 */
#ifdef CONFIG_I2C_COMPAT
	res = class_compat_create_link(i2c_adapter_compat_class, &adap->dev,
				       adap->dev.parent);
	if (res)
		dev_warn(&adap->dev,
			 "Failed to create compatibility class link\n");
#endif

	/* create pre-declared device nodes */
    /* 解析设备树中的子节点, 注册为 i2c devices
     * 注意，注册过程中会进行设备/驱动匹配
     * 所以这里的设备有可能已经匹配了驱动了 */
	of_i2c_register_devices(adap);
    /* acpi 相关, arm不支持 */
	i2c_acpi_install_space_handler(adap);
	i2c_acpi_register_devices(adap);

    /* 使用静态分配的 bus 号的适配器
     * 可能具有静态注册的从设备, 需要遍历注册一下 */
	if (adap->nr < __i2c_first_dynamic_bus_num)
		i2c_scan_static_board_info(adap);

	/* Notify drivers */
	mutex_lock(&core_lock);
    /* 通知一下 i2c bus 上的所有 driver 
     * 有新的i2c适配器设备注册进来了，赶紧来匹配啦。。。*/
    /* 这是一种 i2c 从设备探测机制:
     * 当有新的适配器添加时，调用所有的 i2c 驱动程序，若支持探测功能，
     * 则调用驱动程序的探测回调，并自动将设备进行实例化，注册 i2c 从设备 */
	bus_for_each_drv(&i2c_bus_type, NULL, adap, __process_new_adapter);
	mutex_unlock(&core_lock);

	return 0;

out_reg:
	debugfs_remove_recursive(adap->debugfs);
	init_completion(&adap->dev_released);
	device_unregister(&adap->dev);
	wait_for_completion(&adap->dev_released);
out_list:
	mutex_lock(&core_lock);
	idr_remove(&i2c_adapter_idr, adap->nr);
	mutex_unlock(&core_lock);
	return res;
}

/**
 * __i2c_add_numbered_adapter - i2c_add_numbered_adapter where nr is never -1
 * @adap: the adapter to register (with adap->nr initialized)
 * Context: can sleep
 *
 * See i2c_add_numbered_adapter() for details.
 */
/* __i2c_add_numbered_adapter - 注意, nr 永不为 -1
 * @adap:   将要注册的 adapter (adap->nr 已被初始化的)
 *
 * 细节说明在 i2c_add_numbered_adapter() 中 */
static int __i2c_add_numbered_adapter(struct i2c_adapter *adap)
{
	int id;

    /* 使用指定的bus号 nr 从 idr 中分配
     * 目的是占用这个 bus号，也用于测试这个bus号是否已经被占用过了 */
	mutex_lock(&core_lock);
	id = idr_alloc(&i2c_adapter_idr, adap, adap->nr, adap->nr + 1, GFP_KERNEL);
	mutex_unlock(&core_lock);
	if (WARN(id < 0, "couldn't get idr"))
		return id == -ENOSPC ? -EBUSY : id;

    /* 调用 i2c_register_adapter 完成最终的注册 */
	return i2c_register_adapter(adap);
}

/**
 * i2c_add_adapter - declare i2c adapter, use dynamic bus number
 * @adapter: the adapter to add
 * Context: can sleep
 *
 * This routine is used to declare an I2C adapter when its bus number
 * doesn't matter or when its bus number is specified by an dt alias.
 * Examples of bases when the bus number doesn't matter: I2C adapters
 * dynamically added by USB links or PCI plugin cards.
 *
 * When this returns zero, a new bus number was allocated and stored
 * in adap->nr, and the specified adapter became available for clients.
 * Otherwise, a negative errno value is returned.
 */
/* i2c_add_adapter - 使用动态总线号，声明 i2c 适配器 
 * @adapter:    将要添加的 adapter
 *
 * 当总线编号不重要，或者总线编号由dt别名指定时，该例程用于声明I2C适配器。总线编
 * 号无关紧要时的base示例：由USB链接或PCI插件卡动态添加的I2C适配器。
 *
 * 当它返回0时，分配一个新的总线编号并存储在adap->nr中，并且指定的适配器对客户端
 * 可用。否则，返回负的errno值。
 * */
int i2c_add_adapter(struct i2c_adapter *adapter)
{
	struct device *dev = &adapter->dev;
	int id;

    /* 具有设备树节点的, 尝试从设备树 alias 中获取总线编号
     * 如 alias 中的 i2c0  i2c1 */
	if (dev->of_node) {
        /* 获取 alias 中的 id 号, i2c0 i2c1 */
		id = of_alias_get_id(dev->of_node, "i2c");
		if (id >= 0) {
            /* alias 中的 id 号有效, 则做为bus号,并记录在 nr 中
             * 同时使用 __i2c_add_numbered_adapter 进行注册 */
			adapter->nr = id;
			return __i2c_add_numbered_adapter(adapter);
		}
	}

    /* 走到这里，说明设备无设备树或者无法从设备树中获取有效的bus号
     * 那就需要动态分配了，动态分配使用了 idr 
     * 从 __i2c_first_dynamic_bus_num 开始动态分配一个 id
     * 如果未分配成功(id < 0) 则返回失败 */
	mutex_lock(&core_lock);
	id = idr_alloc(&i2c_adapter_idr, adapter,
		       __i2c_first_dynamic_bus_num, 0, GFP_KERNEL);
	mutex_unlock(&core_lock);
	if (WARN(id < 0, "couldn't get idr"))
		return id;

    /* 分配成功了，则将 id 做为 bus号, 并给 nr  */
	adapter->nr = id;

    /* 调用 i2c_register_adapter 完成最终的注册 */
	return i2c_register_adapter(adapter);
}
EXPORT_SYMBOL(i2c_add_adapter);

/**
 * i2c_add_numbered_adapter - declare i2c adapter, use static bus number
 * @adap: the adapter to register (with adap->nr initialized)
 * Context: can sleep
 *
 * This routine is used to declare an I2C adapter when its bus number
 * matters.  For example, use it for I2C adapters from system-on-chip CPUs,
 * or otherwise built in to the system's mainboard, and where i2c_board_info
 * is used to properly configure I2C devices.
 *
 * If the requested bus number is set to -1, then this function will behave
 * identically to i2c_add_adapter, and will dynamically assign a bus number.
 *
 * If no devices have pre-been declared for this bus, then be sure to
 * register the adapter before any dynamically allocated ones.  Otherwise
 * the required bus ID may not be available.
 *
 * When this returns zero, the specified adapter became available for
 * clients using the bus number provided in adap->nr.  Also, the table
 * of I2C devices pre-declared using i2c_register_board_info() is scanned,
 * and the appropriate driver model device nodes are created.  Otherwise, a
 * negative errno value is returned.
 */
/* 添加一个 i2c adapter, 使用固定分配的 bus 号
 * @adap:   将要注册的 adapter(adap->nr 已被初始化)
 *
 * 当 I2C 适配器的总线编号很重要的时候，就会用这个例程来声明它。打个比方啊，要是
 * I2C 适配器来自片上系统 CPU，或者是以其他方式集成到系统主板里的，而且还要用
 * i2c_board_info 来正确设置 I2C 设备，那就得用这个例程 。
 *
 * 如果请求的总线号被设置为 -1，那么此函数将与 i2c_add_adapter 行为完全相同，并
 * 将动态分配一个总线号。
 *
 * 如果此总线上尚未声明任何设备，则务必在注册任何动态分配的设备之前先注册适配器。
 * 否则，所需的总线 ID 可能不可用。
 *
 * 返回零时，指定的适配器便可供使用该总线编号（即在 adap->nr 中提供的值）
 * 的客户端访问。此外，会扫描使用 i2c_register_board_info() 预先声明的 I2C 设备
 * 表，并创建相应的驱动模型设备节点。否则，将返回一个负的 errno 值。
 * */
int i2c_add_numbered_adapter(struct i2c_adapter *adap)
{
    /* adap->nr == -1 表示将动态分配总线号
     * 本函数与 i2c_add_adapter() 完全一致 */
	if (adap->nr == -1) /* -1 means dynamically assign bus id */
		return i2c_add_adapter(adap);

    /* 使用指定的 nr 进行注册 */
	return __i2c_add_numbered_adapter(adap);
}
EXPORT_SYMBOL_GPL(i2c_add_numbered_adapter);

/* 移除一个 i2c adapter, 由 i2c 驱动处理使用探测创建的设备
 * 本函数中仅注销了使用驱动探测出的从设备 */
static void i2c_do_del_adapter(struct i2c_driver *driver,
			      struct i2c_adapter *adapter)
{
	struct i2c_client *client, *_n;

	/* Remove the devices we created ourselves as the result of hardware
	 * probing (using a driver's detect method) */
    /* 移除使用由探测的方式创建的设备
     * 这里只检查了驱动的 clients 链表，释放通过探测创建的设备 */
	list_for_each_entry_safe(client, _n, &driver->clients, detected) {
        /* 需要检查从设备所属的适配器是否就是需要注销的适配器 */
		if (client->adapter == adapter) {
			dev_dbg(&adapter->dev, "Removing %s at 0x%x\n",
				client->name, client->addr);
            /* 从驱动的探测设备链表中移除，并注销从设备 */
			list_del(&client->detected);
			i2c_unregister_device(client);
		}
	}
}

/* 注销从设备，如果是虚拟设备则不处理 */
static int __unregister_client(struct device *dev, void *dummy)
{
    /* 获取从机结构指针, 如果是 适配器，则是NULL */
    /* 如果是适配器，那也是将要注销的这个适配器的从机，
     * 这时将会出现套娃了，注销某个从机，这个从机中将注销适配器，
     * 那这个适配器也会注销所有的从机 */
	struct i2c_client *client = i2c_verify_client(dev);
    /* 如果从机是虚拟设备 "dummy" ，将跳过 */
	if (client && strcmp(client->name, "dummy"))
		i2c_unregister_device(client);
	return 0;
}

/* 注销 虚拟从设备
 * 本函数中其实不区分是否为虚拟从设备
 * 但因为调用本函数之前，已经将各种设备注销了，只剩下虚拟设备了 */
static int __unregister_dummy(struct device *dev, void *dummy)
{
	struct i2c_client *client = i2c_verify_client(dev);
	i2c_unregister_device(client);
	return 0;
}

/* 处理 i2c 适配器被移除的情况
 * 移除适配器时，对所有的驱动调用本函数，参数为驱动结构 及 适配器结构 */
static int __process_removed_adapter(struct device_driver *d, void *data)
{
    /* 转换为 i2c_driver, 完全由 i2c_do_del_adapter 处理 */
	i2c_do_del_adapter(to_i2c_driver(d), data);
	return 0;
}

/**
 * i2c_del_adapter - unregister I2C adapter
 * @adap: the adapter being unregistered
 * Context: can sleep
 *
 * This unregisters an I2C adapter which was previously registered
 * by @i2c_add_adapter or @i2c_add_numbered_adapter.
 */
/* i2c_del_adapter - 注销 i2c adapter
 * @adap:   将要被注销的 adapter 指针
 *
 * 这将注销之前由 @i2c_add_adapter 或 @i2c_add_numbered_adapter 注册的 i2c适配器 */
void i2c_del_adapter(struct i2c_adapter *adap)
{
	struct i2c_adapter *found;
	struct i2c_client *client, *next;

	/* First make sure that this adapter was ever added */
    /* 首先确保这个适配器已经被注册成功了 */
    /* 是通过检查适配的编号在 idr 中是否存在确认的
     * idr 中存放了编号对应的适配器指针，检查相同则认为是已经被注册成功的 */
	mutex_lock(&core_lock);
	found = idr_find(&i2c_adapter_idr, adap->nr);
	mutex_unlock(&core_lock);
	if (found != adap) {
		pr_debug("attempting to delete unregistered adapter [%s]\n", adap->name);
		return;
	}

    /* acpi arm 不支持 */
	i2c_acpi_remove_space_handler(adap);
	/* Tell drivers about this removal */
    /* 通知 drivers 这个 适配将要被移除了
     * 遍历 i2c bus 上的每一个驱动，调用 __process_removed_adapter */
    /* 这里是通知所有的 i2c 驱动 这个适配器将要移除了
     * 驱动需要将所有探测的设备注销掉
     * 因为探测出的设备是挂在 驱动的设备链表中的，并未在适配器链表中 */
	mutex_lock(&core_lock);
	bus_for_each_drv(&i2c_bus_type, NULL, adap,
			       __process_removed_adapter);
	mutex_unlock(&core_lock);

	/* Remove devices instantiated from sysfs */
    /* 移除通过 sysfs 创建的从设备 */
    /* 通过 sysfs 创建的设备，挂在适配器的 userspace_clients 链表中 */
	mutex_lock_nested(&adap->userspace_clients_lock,
			  i2c_adapter_depth(adap));
	list_for_each_entry_safe(client, next, &adap->userspace_clients,
				 detected) {
		dev_dbg(&adap->dev, "Removing %s at 0x%x\n", client->name,
			client->addr);
		list_del(&client->detected);
		i2c_unregister_device(client);
	}
	mutex_unlock(&adap->userspace_clients_lock);

	/* Detach any active clients. This can't fail, thus we do not
	 * check the returned value. This is a two-pass process, because
	 * we can't remove the dummy devices during the first pass: they
	 * could have been instantiated by real devices wishing to clean
	 * them up properly, so we give them a chance to do that first. */
    /* 分类所有活动从机。这不会失败，因此不检查返回值。
     * 这是一个两阶段的过程，因为我们不能在第一阶段移除虚拟设备：它们可能希望被
     * 真实的实例化设备清除，所以给它们一个机会先做这些事 */
    /* 这里是先遍历所有的真实设备，所有的虚拟设备都是由真实设备创建的，
     * 所以虚拟设备应该由真实设备注销，所以这里先注销真实设备，
     * 然后再遍历所有剩余的设备，(应该只有虚拟设备了)，全部注销掉 */
	device_for_each_child(&adap->dev, NULL, __unregister_client);
	device_for_each_child(&adap->dev, NULL, __unregister_dummy);

    /* 移除兼容使用的设备软链接 */
#ifdef CONFIG_I2C_COMPAT
	class_compat_remove_link(i2c_adapter_compat_class, &adap->dev,
				 adap->dev.parent);
#endif

	/* device name is gone after device_unregister */
    /* 先输出调试信息，因为设备名将在 device_unregister 之后消失(不可访问) */
	dev_dbg(&adap->dev, "adapter [%s] unregistered\n", adap->name);

    /* 电源管理相关 */
	pm_runtime_disable(&adap->dev);

    /* 注销主机通知的软irq */
	i2c_host_notify_irq_teardown(adap);

    /* 递归删除由 debugfs 创建的目录及其子目录和文件 */
	debugfs_remove_recursive(adap->debugfs);

	/* wait until all references to the device are gone
	 *
	 * FIXME: This is old code and should ideally be replaced by an
	 * alternative which results in decoupling the lifetime of the struct
	 * device from the i2c_adapter, like spi or netdev do. Any solution
	 * should be thoroughly tested with DEBUG_KOBJECT_RELEASE enabled!
	 */
    /* 等待，直到所有对该设备的引用都结束
     * 初始化完成量，将在所有引用都结束后，调用 i2c_adapter_dev_release 函数
     * 该函数中将发送完成量，这时就表明待移除的适配器已完全没有引用了 */
	init_completion(&adap->dev_released);
	device_unregister(&adap->dev);
	wait_for_completion(&adap->dev_released);

	/* free bus id */
    /* 从 idr 中释放 适配器编号
     * 可能还有其它适配器注册时使用 */
	mutex_lock(&core_lock);
	idr_remove(&i2c_adapter_idr, adap->nr);
	mutex_unlock(&core_lock);

	/* Clear the device structure in case this adapter is ever going to be
	   added again */
    /* 清除设备结构，以防再次添加此适配器 */
	memset(&adap->dev, 0, sizeof(adap->dev));
}
EXPORT_SYMBOL(i2c_del_adapter);

/* devm 版本 i2c 适配器注销
 * 完全调用 i2c_del_adapter 处理 */
static void devm_i2c_del_adapter(void *adapter)
{
	i2c_del_adapter(adapter);
}

/**
 * devm_i2c_add_adapter - device-managed variant of i2c_add_adapter()
 * @dev: managing device for adding this I2C adapter
 * @adapter: the adapter to add
 * Context: can sleep
 *
 * Add adapter with dynamic bus number, same with i2c_add_adapter()
 * but the adapter will be auto deleted on driver detach.
 */
/* devm_i2c_add_adapter - devm 版本的 i2c_add_adapter()
 * @dev:    管理添加此i2c适配器的设备
 * @adapter:将被添加的适配器
 *
 * 添加具有动态总线号的适配器，完全由 i2c_add_adapter 处理
 * */
int devm_i2c_add_adapter(struct device *dev, struct i2c_adapter *adapter)
{
	int ret;

    /* 直接调用 i2c_add_adapter() 处理即可 */
	ret = i2c_add_adapter(adapter);
	if (ret)
		return ret;

    /* 添加到 devm 管理中 */
	return devm_add_action_or_reset(dev, devm_i2c_del_adapter, adapter);
}
EXPORT_SYMBOL_GPL(devm_i2c_add_adapter);

/* 匹配 fwnode 与设备的 fwnode 
 * 如果一致，则返回 1 ，否则 返回 0*/
static int i2c_dev_or_parent_fwnode_match(struct device *dev, const void *data)
{
    /* 获取设备的 fwnode 并比较，相同则匹配成功 */
	if (dev_fwnode(dev) == data)
		return 1;

    /* 检查父设备的 fwnode，相同则匹配成功 */
	if (dev->parent && dev_fwnode(dev->parent) == data)
		return 1;

	return 0;
}

/**
 * i2c_find_adapter_by_fwnode() - find an i2c_adapter for the fwnode
 * @fwnode: &struct fwnode_handle corresponding to the &struct i2c_adapter
 *
 * Look up and return the &struct i2c_adapter corresponding to the @fwnode.
 * If no adapter can be found, or @fwnode is NULL, this returns NULL.
 *
 * The user must call put_device(&adapter->dev) once done with the i2c adapter.
 */
/* i2c_find_adapter_by_fwnode() - 使用 fwnode 查找所属的适配器
 * @fwnode: 属于 $struct i2c_adapter 的 &struct fwnode_handle
 *
 * 查找并返回与@fwnode 对应的 &struct i2c_adapter. 
 * 如果没有找到适配器，或者 @fwnode 为 NULL, 则返回 NULL
 *
 * 使用完适配器后，必须使用 put_device 释放
 * */
struct i2c_adapter *i2c_find_adapter_by_fwnode(struct fwnode_handle *fwnode)
{
	struct i2c_adapter *adapter;
	struct device *dev;

	if (!fwnode)
		return NULL;

    /* 遍历 i2c bus 的所有设备，调用 i2c_dev_or_parent_fwnode_match 进行匹配 */
	dev = bus_find_device(&i2c_bus_type, NULL, fwnode,
			      i2c_dev_or_parent_fwnode_match);
	if (!dev)
		return NULL;

    /* 检查匹配成功设备是否为 i2c 适配器
     * 不是适配器，则释放并返回 NULL */
	adapter = i2c_verify_adapter(dev);
	if (!adapter)
		put_device(dev);

	return adapter;
}
EXPORT_SYMBOL(i2c_find_adapter_by_fwnode);

/**
 * i2c_get_adapter_by_fwnode() - find an i2c_adapter for the fwnode
 * @fwnode: &struct fwnode_handle corresponding to the &struct i2c_adapter
 *
 * Look up and return the &struct i2c_adapter corresponding to the @fwnode,
 * and increment the adapter module's use count. If no adapter can be found,
 * or @fwnode is NULL, this returns NULL.
 *
 * The user must call i2c_put_adapter(adapter) once done with the i2c adapter.
 * Note that this is different from i2c_find_adapter_by_node().
 */
/* i2c_get_adapter_by_fwnode() - 使用 fwnode 查找所属的适配器
 * @fwnode: 属于 $struct i2c_adapter 的 &struct fwnode_handle
 * 
 * 处理与 i2c_find_adapter_by_fwnode 一致，但增加了 模块引用操作
 * 本函数获取的 i2c 适配器，必须使用 i2c_put_adapter 释放
 * */
struct i2c_adapter *i2c_get_adapter_by_fwnode(struct fwnode_handle *fwnode)
{
	struct i2c_adapter *adapter;

    /* 完全调用 i2c_find_adapter_by_fwnode 处理，失败则返回 NULL */
	adapter = i2c_find_adapter_by_fwnode(fwnode);
	if (!adapter)
		return NULL;

    /* 增加模块引用计数，失败则释放设备并返回NULL */
	if (!try_module_get(adapter->owner)) {
		put_device(&adapter->dev);
		adapter = NULL;
	}

	return adapter;
}
EXPORT_SYMBOL(i2c_get_adapter_by_fwnode);

/* 获取指定的属性值，并设置到时序参数中
 * 若没有该属性，并且启用了默认值，则设置为默认值 */
static void i2c_parse_timing(struct device *dev, char *prop_name, u32 *cur_val_p,
			    u32 def_val, bool use_def)
{
	int ret;

	ret = device_property_read_u32(dev, prop_name, cur_val_p);
	if (ret && use_def)
		*cur_val_p = def_val;

	dev_dbg(dev, "%s: %u\n", prop_name, *cur_val_p);
}

/**
 * i2c_parse_fw_timings - get I2C related timing parameters from firmware
 * @dev: The device to scan for I2C timing properties
 * @t: the i2c_timings struct to be filled with values
 * @use_defaults: bool to use sane defaults derived from the I2C specification
 *		  when properties are not found, otherwise don't update
 *
 * Scan the device for the generic I2C properties describing timing parameters
 * for the signal and fill the given struct with the results. If a property was
 * not found and use_defaults was true, then maximum timings are assumed which
 * are derived from the I2C specification. If use_defaults is not used, the
 * results will be as before, so drivers can apply their own defaults before
 * calling this helper. The latter is mainly intended for avoiding regressions
 * of existing drivers which want to switch to this function. New drivers
 * almost always should use the defaults.
 */
/* i2c_parse_fw_timings - 从固件中获取 i2c 相关的时序参数
 * @dev:    扫描i2c时序特性的设备
 * @t:      用于填充时序参数的结构
 * @use_defaults:   当属性没有找到时，指示是否使用i2c规范的默认值，否则不更新
 *
 * 扫描器件以查找描述信号时序参数的通用I2C属性，并将获取的结果填充至指定结构体中。
 * 若某属性未被找到且use_defaults为true，则采用符合I2C规范所规定的最大时间值作为
 * 默认值。若use_defaults为false，则不进行默认值填充，保持结果不变，以便驱动程序
 * 可在调用该辅助函数前自行设置所需默认值。此设计主要旨在避免现有驱动程序在迁移
 * 至该函数时出现行为回退问题。对于新开发的驱动程序，建议始终启用默认值机制。
 * */
void i2c_parse_fw_timings(struct device *dev, struct i2c_timings *t, bool use_defaults)
{
	bool u = use_defaults;
	u32 d;

	i2c_parse_timing(dev, "clock-frequency", &t->bus_freq_hz,
			 I2C_MAX_STANDARD_MODE_FREQ, u);

	d = t->bus_freq_hz <= I2C_MAX_STANDARD_MODE_FREQ ? 1000 :
	    t->bus_freq_hz <= I2C_MAX_FAST_MODE_FREQ ? 300 : 120;
	i2c_parse_timing(dev, "i2c-scl-rising-time-ns", &t->scl_rise_ns, d, u);

	d = t->bus_freq_hz <= I2C_MAX_FAST_MODE_FREQ ? 300 : 120;
	i2c_parse_timing(dev, "i2c-scl-falling-time-ns", &t->scl_fall_ns, d, u);

	i2c_parse_timing(dev, "i2c-scl-internal-delay-ns",
			 &t->scl_int_delay_ns, 0, u);
	i2c_parse_timing(dev, "i2c-sda-falling-time-ns", &t->sda_fall_ns,
			 t->scl_fall_ns, u);
	i2c_parse_timing(dev, "i2c-sda-hold-time-ns", &t->sda_hold_ns, 0, u);
	i2c_parse_timing(dev, "i2c-digital-filter-width-ns",
			 &t->digital_filter_width_ns, 0, u);
	i2c_parse_timing(dev, "i2c-analog-filter-cutoff-frequency",
			 &t->analog_filter_cutoff_freq_hz, 0, u);
}
EXPORT_SYMBOL_GPL(i2c_parse_fw_timings);

/* ------------------------------------------------------------------------- */

/* 遍历 i2c bus 总线的设备，使用 fn 处理 */
int i2c_for_each_dev(void *data, int (*fn)(struct device *dev, void *data))
{
	int res;

	mutex_lock(&core_lock);
	res = bus_for_each_dev(&i2c_bus_type, NULL, data, fn);
	mutex_unlock(&core_lock);

	return res;
}
EXPORT_SYMBOL_GPL(i2c_for_each_dev);

/* 新注册驱动程序，使用新驱动进行探测设备 */
static int __process_new_driver(struct device *dev, void *data)
{
    /* 只处理 i2c adapter 适配器 */
	if (dev->type != &i2c_adapter_type)
		return 0;
    /* 直接将驱动程序 和 适配器 交给 i2c_do_add_adapter 处理 */
	return i2c_do_add_adapter(data, to_i2c_adapter(dev));
}

/*
 * An i2c_driver is used with one or more i2c_client (device) nodes to access
 * i2c slave chips, on a bus instance associated with some i2c_adapter.
 */
/* 一个 i2c_driver 与一个或多个 i2c_client(从设备) 节点一起使用，在与某个
 * i2c_adapter关联的总线示例上访问i2c从芯片 */

/* 注册一个 i2c_driver
 * 在驱动中，使用较多的是 i2c.h 中的 i2c_add_driver */
int i2c_register_driver(struct module *owner, struct i2c_driver *driver)
{
	int res;

	/* Can't register until after driver model init */
    /* 检查 i2c 子系统是否初始化成功，返回 EAGAIN 直到注册成功 */
	if (WARN_ON(!is_registered))
		return -EAGAIN;

	/* add the driver to the list of i2c drivers in the driver core */
    /* 将该驱动程序添加到驱动程序核心中的 i2c 驱动程序列表中 */
    /* 指定驱动程序所在的bus -> i2c bus
     * 初始化探测链表头 clients */
	driver->driver.owner = owner;
	driver->driver.bus = &i2c_bus_type;
	INIT_LIST_HEAD(&driver->clients);

	/* When registration returns, the driver core
	 * will have called probe() for all matching-but-unbound devices.
	 */
    /* 当注册接口返回时，驱动核心将对所有匹配但未绑定的设备调用 probe() 
     * 这里会对所有静态注册、设备树等从设备进行匹配
     * 是使用驱动遍历设备的过程 */
	res = driver_register(&driver->driver);
	if (res)
		return res;

	pr_debug("driver [%s] registered\n", driver->driver.name);

	/* Walk the adapters that are already present */
    /* 遍历已经存在的适配器 */
    /* 使用新驱动程序进行设备探测的处理
     * 新注册了驱动程序，可能匹配新的设备，所以要遍历适配器进行探测 */
	i2c_for_each_dev(driver, __process_new_driver);

	return 0;
}
EXPORT_SYMBOL(i2c_register_driver);

/* 处理移除 i2c 驱动的操作
 * 为了处理适配器设备，因为适配下挂载很多设备
 * 如果适配的驱动移除了，那下挂的所有设备都需要移除 */
static int __process_removed_driver(struct device *dev, void *data)
{
    /* 只处理 i2c 适配器
     * 如果驱动绑定了适配器，需要移除适配器 */
	if (dev->type == &i2c_adapter_type)
		i2c_do_del_adapter(data, to_i2c_adapter(dev));
	return 0;
}

/**
 * i2c_del_driver - unregister I2C driver
 * @driver: the driver being unregistered
 * Context: can sleep
 */
/* i2c_del_driver - 注销 i2c 驱动
 * @driver: 将要被注册的 i2c 驱动
 * */
void i2c_del_driver(struct i2c_driver *driver)
{
    /* 遍历驱动绑定的所有设备
     * 主要是需要处理适配器驱动，如果设备为适配器，需要先移除适配器 */
	i2c_for_each_dev(driver, __process_removed_driver);

    /* 从设备驱动模型中移除驱动 */
	driver_unregister(&driver->driver);
	pr_debug("driver [%s] unregistered\n", driver->driver.name);
}
EXPORT_SYMBOL(i2c_del_driver);

/* ------------------------------------------------------------------------- */

/* i2c_clients_command 内部使用的命令结构 */
struct i2c_cmd_arg {
	unsigned	cmd;
	void		*arg;
};

/* 对某个从设备执行 command 回调 */
static int i2c_cmd(struct device *dev, void *_arg)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_cmd_arg	*arg = _arg;
	struct i2c_driver	*driver;

    /* 需要确认该设备为 i2c 从设备，并且已经绑定了驱动 */
	if (!client || !client->dev.driver)
		return 0;

    /* 转换为 i2c 驱动结构
     * 如果具有 command 回调，则使用给定的命令结构做为参数调用 */
	driver = to_i2c_driver(client->dev.driver);
	if (driver->command)
		driver->command(client, arg->cmd, arg->arg);
	return 0;
}

/* 使用给定的参数，调用所有连接的客户端的 i2c_client->command() 回调 */
void i2c_clients_command(struct i2c_adapter *adap, unsigned int cmd, void *arg)
{
	struct i2c_cmd_arg	cmd_arg;

    /* 组织命令结构
     * 这是因为遍历操作只能传入一个参数，所以组织成一个结构体 */
	cmd_arg.cmd = cmd;
	cmd_arg.arg = arg;
    /* 遍历所有的子设备，并调用 i2c_cmd 函数 */
	device_for_each_child(&adap->dev, &cmd_arg, i2c_cmd);
}
EXPORT_SYMBOL(i2c_clients_command);

/* i2c 子系统初始化入口
 * 是在 postcore_initcall 中调用，先于所有的设备驱动 */
static int __init i2c_init(void)
{
	int retval;

    /* 获得 alias 中 i2c 节点 最高的id
     * 不管设备有没有使能，找的是所有设备节点中 id最高的 */
	retval = of_alias_get_highest_id("i2c");

    /* 检查设备树中定义的 i2c 最大 id 是否大于动态分配起始值
     * 如果大于等于起始值，则起始值从最大的id后面开始 */
	down_write(&__i2c_board_lock);
	if (retval >= __i2c_first_dynamic_bus_num)
		__i2c_first_dynamic_bus_num = retval + 1;
	up_write(&__i2c_board_lock);

    /* 注册 i2c bus 
     * 这是所有 i2c 设备和驱动 使用的 bus，都挂在 i2c bus 上 */
	retval = bus_register(&i2c_bus_type);
	if (retval)
		return retval;

    /* 标记 i2c bus 已注册成功 */
	is_registered = true;

    /* debugfs 文件系统中创建 i2c 目录,是所有 i2c 调试信息的根目录 */
	i2c_debugfs_root = debugfs_create_dir("i2c", NULL);

    /* CONFIG_I2C_COMPAT 用于创建与原 sysfs 兼容的设备/驱动 链接 */
    /* 非关键流程，用来与老的 i2c 驱动兼容的 */
#ifdef CONFIG_I2C_COMPAT
	i2c_adapter_compat_class = class_compat_register("i2c-adapter");
	if (!i2c_adapter_compat_class) {
		retval = -ENOMEM;
		goto bus_err;
	}
#endif

    /* 添加 i2c 驱动, dummy_driver */
	retval = i2c_add_driver(&dummy_driver);
	if (retval)
		goto class_err;

    /* 如果开启了设备树动态修改功能, 则注册 i2c bus 的通知机制
     * 这将在设备树改动时调用通知函数 */
	if (IS_ENABLED(CONFIG_OF_DYNAMIC))
		WARN_ON(of_reconfig_notifier_register(&i2c_of_notifier));
    /* ACPI arm 中不支持 */
	if (IS_ENABLED(CONFIG_ACPI))
		WARN_ON(acpi_reconfig_notifier_register(&i2c_acpi_notifier));

    /* 全部OK了，返回成功啦。。。 */
	return 0;

class_err:
#ifdef CONFIG_I2C_COMPAT
	class_compat_unregister(i2c_adapter_compat_class);
bus_err:
#endif
	is_registered = false;
	bus_unregister(&i2c_bus_type);
	return retval;
}

/* i2c 子系统退出函数
 * 就是  i2c_init 的反初始化操作 */
static void __exit i2c_exit(void)
{
	if (IS_ENABLED(CONFIG_ACPI))
		WARN_ON(acpi_reconfig_notifier_unregister(&i2c_acpi_notifier));
	if (IS_ENABLED(CONFIG_OF_DYNAMIC))
		WARN_ON(of_reconfig_notifier_unregister(&i2c_of_notifier));
	i2c_del_driver(&dummy_driver);
#ifdef CONFIG_I2C_COMPAT
	class_compat_unregister(i2c_adapter_compat_class);
#endif
	debugfs_remove_recursive(i2c_debugfs_root);
	bus_unregister(&i2c_bus_type);
	tracepoint_synchronize_unregister();
}

/* We must initialize early, because some subsystems register i2c drivers
 * in subsys_initcall() code, but are linked (and initialized) before i2c.
 */
/* 我们必须尽早初始化，因为有些子系统在 subsys_initcall() 代码中注册了 i2c驱动程
 * 序，但在i2c之前链接(和初始化) */
/* 这种公共子系统的初始化要尽可能的早，而且这也不依赖任何硬件，只是一些数据结构
 * 的初始化，以避免其它的设备或驱动注册时还未初始化的情况(虽然可以返回 EAGAIN),
 * 但尽早的初始化还是可以避免一些无用的操作的 */
postcore_initcall(i2c_init);
module_exit(i2c_exit);

/* ----------------------------------------------------
 * the functional interface to the i2c busses.
 * ----------------------------------------------------
 */

/* Check if val is exceeding the quirk IFF quirk is non 0 */
/* 检查 val 是否超过了 quirk, quirk 为非0值 */
#define i2c_quirk_exceeded(val, quirk) ((quirk) && ((val) > (quirk)))

/* 打印 quirk 错误提示消息 */
static int i2c_quirk_error(struct i2c_adapter *adap, struct i2c_msg *msg, char *err_msg)
{
	dev_err_ratelimited(&adap->dev, "adapter quirk: %s (addr 0x%04x, size %u, %s)\n",
			    err_msg, msg->addr, msg->len,
			    msg->flags & I2C_M_RD ? "read" : "write");
	return -EOPNOTSUPP;
}

/* 检查提供的消息体 是否满足 适配器的限制
 * 主要是检查消息体数量、读写数据长度等
 * 参考 i2c_adapter_quirks 结构描述 */
static int i2c_check_for_quirks(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	const struct i2c_adapter_quirks *q = adap->quirks;
	int max_num = q->max_num_msgs, i;
	bool do_len_check = true;

	if (q->flags & I2C_AQ_COMB) {
		max_num = 2;

		/* special checks for combined messages */
		if (num == 2) {
			if (q->flags & I2C_AQ_COMB_WRITE_FIRST && msgs[0].flags & I2C_M_RD)
				return i2c_quirk_error(adap, &msgs[0], "1st comb msg must be write");

			if (q->flags & I2C_AQ_COMB_READ_SECOND && !(msgs[1].flags & I2C_M_RD))
				return i2c_quirk_error(adap, &msgs[1], "2nd comb msg must be read");

			if (q->flags & I2C_AQ_COMB_SAME_ADDR && msgs[0].addr != msgs[1].addr)
				return i2c_quirk_error(adap, &msgs[0], "comb msg only to same addr");

			if (i2c_quirk_exceeded(msgs[0].len, q->max_comb_1st_msg_len))
				return i2c_quirk_error(adap, &msgs[0], "msg too long");

			if (i2c_quirk_exceeded(msgs[1].len, q->max_comb_2nd_msg_len))
				return i2c_quirk_error(adap, &msgs[1], "msg too long");

			do_len_check = false;
		}
	}

	if (i2c_quirk_exceeded(num, max_num))
		return i2c_quirk_error(adap, &msgs[0], "too many messages");

	for (i = 0; i < num; i++) {
		u16 len = msgs[i].len;

		if (msgs[i].flags & I2C_M_RD) {
			if (do_len_check && i2c_quirk_exceeded(len, q->max_read_len))
				return i2c_quirk_error(adap, &msgs[i], "msg too long");

			if (q->flags & I2C_AQ_NO_ZERO_LEN_READ && len == 0)
				return i2c_quirk_error(adap, &msgs[i], "no zero length");
		} else {
			if (do_len_check && i2c_quirk_exceeded(len, q->max_write_len))
				return i2c_quirk_error(adap, &msgs[i], "msg too long");

			if (q->flags & I2C_AQ_NO_ZERO_LEN_WRITE && len == 0)
				return i2c_quirk_error(adap, &msgs[i], "no zero length");
		}
	}

	return 0;
}

/**
 * __i2c_transfer - unlocked flavor of i2c_transfer
 * @adap: Handle to I2C bus
 * @msgs: One or more messages to execute before STOP is issued to
 *	terminate the operation; each message begins with a START.
 * @num: Number of messages to be executed.
 *
 * Returns negative errno, else the number of messages executed.
 *
 * Adapter lock must be held when calling this function. No debug logging
 * takes place.
 */
/* __i2c_transfer - 非锁定总线的 i2c_transfer
 * @adap:   I2C总线句柄
 * @msgs:   在STOP信号发出终止操作之前要执行的一条或多条消息；每条消息
 *          之前都有一个START信号
 * @num:    需要被执行的消息数量
 *
 * 错误是返回负数errno，否则返回已被执行的消息数
 *
 * 在调用本函数时，必须保持适配器锁。没有调试日志记录。
 * 直接调用本函数时，函数内并不会进行适配器加锁操作，而是由外部保证的，
 * 需要确保已经进行了加锁操作 
 * 在 i2c mux 驱动中，向 i2c mux 器件发送消息时，因外层已经加锁了，所以使用这些
 * 不加锁的函数处理
 * */
int __i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	unsigned long orig_jiffies;
	int ret, try;

    /* 需要I2C适配器提供 master_xfer 回调 */
	if (!adap->algo->master_xfer) {
		dev_dbg(&adap->dev, "I2C level transfers not supported\n");
		return -EOPNOTSUPP;
	}

    /* 至少需要有一个消息结构 */
	if (WARN_ON(!msgs || num < 1))
		return -EINVAL;

    /* 检查 i2c 适配器是否为挂起状态 */
	ret = __i2c_check_suspended(adap);
	if (ret)
		return ret;

    /* 若适配器具有 quirks 限制结构，则进行消息体检查
     * 避免待传输的消息超出适配器的限制，导致传输失败 */
	if (adap->quirks && i2c_check_for_quirks(adap, msgs, num))
		return -EOPNOTSUPP;

	/*
	 * i2c_trace_msg_key gets enabled when tracepoint i2c_transfer gets
	 * enabled.  This is an efficient way of keeping the for-loop from
	 * being executed when not needed.
	 */
    /* 调试使用 */
	if (static_branch_unlikely(&i2c_trace_msg_key)) {
		int i;
		for (i = 0; i < num; i++)
			if (msgs[i].flags & I2C_M_RD)
				trace_i2c_read(adap, &msgs[i], i);
			else
				trace_i2c_write(adap, &msgs[i], i);
	}

	/* Retry automatically on arbitration loss */
    /* 传输失败，但返回值为 EAGAIN , 则进行重传 */
    /* 超时控制，若超时，也不再进行重传 */
	orig_jiffies = jiffies;
	for (ret = 0, try = 0; try <= adap->retries; try++) {
        /* 原子操作模式下，适配器应该提供 master_xfer_atomic
         * 否则将会直接调用 master_xfer 回调 */
        /* 在适配器的 master_xfer 回调中，也可能做了类似这里的处理，
         * 错误重传，超时等，适配器驱动保证传输及返回值即可 */
		if (i2c_in_atomic_xfer_mode() && adap->algo->master_xfer_atomic)
			ret = adap->algo->master_xfer_atomic(adap, msgs, num);
		else
			ret = adap->algo->master_xfer(adap, msgs, num);

        /* 非 EAGAIN 错误，则退出传输 */
		if (ret != -EAGAIN)
			break;
        /* 传输超时，则退出 */
		if (time_after(jiffies, orig_jiffies + adap->timeout))
			break;
	}

    /* 调试使用 */
	if (static_branch_unlikely(&i2c_trace_msg_key)) {
		int i;
		for (i = 0; i < ret; i++)
			if (msgs[i].flags & I2C_M_RD)
				trace_i2c_reply(adap, &msgs[i], i);
		trace_i2c_result(adap, num, ret);
	}

	return ret;
}
EXPORT_SYMBOL(__i2c_transfer);

/**
 * i2c_transfer - execute a single or combined I2C message
 * @adap: Handle to I2C bus
 * @msgs: One or more messages to execute before STOP is issued to
 *	terminate the operation; each message begins with a START.
 * @num: Number of messages to be executed.
 *
 * Returns negative errno, else the number of messages executed.
 *
 * Note that there is no requirement that each message be sent to
 * the same slave address, although that is the most common model.
 */
/* i2c_transfer - 执行一条或多条I2C消息
 * @adap:   I2C总线句柄
 * @msgs:   在STOP信号发出终止操作之前要执行的一条或多条消息；每条消息
 *          之前都有一个START信号
 * @num:    需要被执行的消息数量
 *
 * 错误是返回负数errno，否则返回已被执行的消息数
 *
 * 注意，并没有要求每个消息都发送到相同的从地址，虽然这时最常见的模式
 * */
int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	int ret;

	/* REVISIT the fault reporting model here is weak:
	 *
	 *  - When we get an error after receiving N bytes from a slave,
	 *    there is no way to report "N".
	 *
	 *  - When we get a NAK after transmitting N bytes to a slave,
	 *    there is no way to report "N" ... or to let the master
	 *    continue executing the rest of this combined message, if
	 *    that's the appropriate response.
	 *
	 *  - When for example "num" is two and we successfully complete
	 *    the first message but get an error part way through the
	 *    second, it's unclear whether that should be reported as
	 *    one (discarding status on the second message) or errno
	 *    (discarding status on the first one).
	 */
    /* 重新审视这里的错误报告模型：
     *  - 当从一个从机中接收到N个字节后，发生了错误，没有办法报告"N"
     *  - 当向从机发送N个字节后得到一个 NAK时，没有办法报告 "N"...
     *    或者让主机继续执行组合消息的剩余部分，即使这是正常的响应
     *  - 例如，当 "num" 为2时，成功完成了第一个消息，但在第二个消息中
     *    出现了错误，不清楚应该报告1（在第二个消息中丢弃状态)还是errno
     *    (在第一个消息中丢弃状态) */
    /* 因为函数的参数是简单的，并不会包含返回的状态（当然，可能会有返回的数据长
     * 度），并且函数的返回值也是简单的，而在通信中，存在多种错误状态，无法对所
     * 有的错误状态都正确的返回，所以这里就只返回了被执行的消息数量 
     * 这种返回也是很合理的，由调用者进一步判断应该如何处理返回值不是期望值的情
     * 况，这样可以由调用者更灵活的处理 */
	ret = __i2c_lock_bus_helper(adap);
	if (ret)
		return ret;

    /* 传输过程完全由 __i2c_transfer 控制 */
	ret = __i2c_transfer(adap, msgs, num);
    /* 释放总线的独占访问 */
	i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);

	return ret;
}
EXPORT_SYMBOL(i2c_transfer);

/**
 * i2c_transfer_buffer_flags - issue a single I2C message transferring data
 *			       to/from a buffer
 * @client: Handle to slave device
 * @buf: Where the data is stored
 * @count: How many bytes to transfer, must be less than 64k since msg.len is u16
 * @flags: The flags to be used for the message, e.g. I2C_M_RD for reads
 *
 * Returns negative errno, or else the number of bytes transferred.
 */
/* i2c_transfer_buffer_flags - 发出一条 I2C消息，传输数据给从设备 或 读从设备数
 * 据
 * @client: 从设备句柄
 * @buf:    数据存储区
 * @count:  待传输数据的长度，必须小于64K，因为 msg.len 是 u16的
 * @flags:  用于该消息的标志，例如用于读取的 I2C_M_RD, 或者是写入
 * 错误则返回负数错误值，否则返回被传输的数据长度
 * */
int i2c_transfer_buffer_flags(const struct i2c_client *client, char *buf,
			      int count, u16 flags)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = flags | (client->flags & I2C_M_TEN),
		.len = count,
		.buf = buf,
	};

    /* 直接调用 i2c_transfer 传输一个 msg 即可 */
	ret = i2c_transfer(client->adapter, &msg, 1);

	/*
	 * If everything went ok (i.e. 1 msg transferred), return #bytes
	 * transferred, else error code.
	 */
    /* i2c_transfer 传输成功时，返回值为需要传输的 msg 数量，这里就是 1
     * 所以 ret == 1 表示传输成功，否则返回错误码 */
	return (ret == 1) ? count : ret;
}
EXPORT_SYMBOL(i2c_transfer_buffer_flags);

/**
 * i2c_get_device_id - get manufacturer, part id and die revision of a device
 * @client: The device to query
 * @id: The queried information
 *
 * Returns negative errno on error, zero on success.
 */
/* i2c_get_device_id - 获取设备的制造商、器件id 和 修订版本
 * @client: 要查询的设备
 * @id:     查询到的信息
 * */
int i2c_get_device_id(const struct i2c_client *client,
		      struct i2c_device_identity *id)
{
	struct i2c_adapter *adap = client->adapter;
	union i2c_smbus_data raw_id;
	int ret;

    /* 检查 I2C 适配器是否支持块读指令 */
	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		return -EOPNOTSUPP;

    /* 使用块读指令读取 id 信息，在 raw_id 结构中 */
	raw_id.block[0] = 3;
	ret = i2c_smbus_xfer(adap, I2C_ADDR_DEVICE_ID, 0,
			     I2C_SMBUS_READ, client->addr << 1,
			     I2C_SMBUS_I2C_BLOCK_DATA, &raw_id);
	if (ret)
		return ret;

    /* 读回的数据拼装为 id 信息，参考 I2C 规范 */
	id->manufacturer_id = (raw_id.block[1] << 4) | (raw_id.block[2] >> 4);
	id->part_id = ((raw_id.block[2] & 0xf) << 5) | (raw_id.block[3] >> 3);
	id->die_revision = raw_id.block[3] & 0x7;
	return 0;
}
EXPORT_SYMBOL_GPL(i2c_get_device_id);

/* ----------------------------------------------------
 * the i2c address scanning function
 * Will not work for 10-bit addresses!
 * ----------------------------------------------------
 */

/*
 * Legacy default probe function, mostly relevant for SMBus. The default
 * probe method is a quick write, but it is known to corrupt the 24RF08
 * EEPROMs due to a state machine bug, and could also irreversibly
 * write-protect some EEPROMs, so for address ranges 0x30-0x37 and 0x50-0x5f,
 * we use a short byte read instead. Also, some bus drivers don't implement
 * quick write, so we fallback to a byte read in that case too.
 * On x86, there is another special case for FSC hardware monitoring chips,
 * which want regular byte reads (address 0x73.) Fortunately, these are the
 * only known chips using this I2C address on PC hardware.
 * Returns 1 if probe succeeded, 0 if not.
 */
/* 遗留的默认探测函数主要与SMBus相关。该函数默认采用快速写入方式进行设备探测，但
 * 已知由于状态机存在缺陷，此方法可能导致24RF08 EEPROM损坏，并可能对某些EEPROM触
 * 发不可逆的写保护机制。因此，针对地址范围0x30–0x37和0x50–0x5F，系统改用短字节
 * 读取方式进行探测。此外，部分总线驱动程序未实现快速写入功能，在此类情况下亦会
 * 回退至字节读取模式。在x86平台上，FSC硬件监控芯片存在另一特殊情形：其位于I2C地
 * 址0x73的设备必须通过标准字节读取方式进行访问。值得庆幸的是，该I2C地址在PC硬件
 * 中仅被此类芯片使用，不存在地址冲突风险。若探测成功，则返回1；否则返回0。
 * */
static int i2c_default_probe(struct i2c_adapter *adap, unsigned short addr)
{
	int err;
	union i2c_smbus_data dummy;

#ifdef CONFIG_X86
	if (addr == 0x73 && (adap->class & I2C_CLASS_HWMON)
	 && i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_BYTE_DATA))
		err = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ, 0,
				     I2C_SMBUS_BYTE_DATA, &dummy);
	else
#endif
	if (!((addr & ~0x07) == 0x30 || (addr & ~0x0f) == 0x50)
	 && i2c_check_functionality(adap, I2C_FUNC_SMBUS_QUICK))
		err = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_WRITE, 0,
				     I2C_SMBUS_QUICK, NULL);
	else if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_BYTE))
		err = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ, 0,
				     I2C_SMBUS_BYTE, &dummy);
	else {
		dev_warn(&adap->dev, "No suitable probing method supported for address 0x%02X\n",
			 addr);
		err = -EOPNOTSUPP;
	}

	return err >= 0;
}

/* 对指定的从机地址进行设备探测
 * 检查在指定的从机地址上是否有响应（即是否有器件）
 * 并调用驱动的 detect 进行详细的探测
 * 若探测成功，则 注册从设备 */
static int i2c_detect_address(struct i2c_client *temp_client,
			      struct i2c_driver *driver)
{
	struct i2c_board_info info;
	struct i2c_adapter *adapter = temp_client->adapter;
	int addr = temp_client->addr;
	int err;

	/* Make sure the address is valid */
    /* 地址有效性检测，驱动探测的地址不能为保留的地址 */
	err = i2c_check_7bit_addr_validity_strict(addr);
	if (err) {
		dev_warn(&adapter->dev, "Invalid probe address 0x%02x\n",
			 addr);
		return err;
	}

	/* Skip if already in use (7 bit, no need to encode flags) */
    /* 检测地址是否已被使用,可能设备已经被注册 */
	if (i2c_check_addr_busy(adapter, addr))
		return 0;

	/* Make sure there is something at this address */
    /* 确保待探测的从机地址上是有响应的，即确认是有这么一个硬件器件的 */
	if (!i2c_default_probe(adapter, addr))
		return 0;

	/* Finally call the custom detection function */
    /* 基本检查完成，调用驱动提供的 detect , 由驱动做详细的芯片型号等检查 */
    /* 构造一个 i2c_board_info, 像调用 probe 一样调用 detect  */
	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = addr;
	err = driver->detect(temp_client, &info);
	if (err) {
		/* -ENODEV is returned if the detection fails. We catch it
		   here as this isn't an error. */
		return err == -ENODEV ? 0 : err;
	}

	/* Consistency check */
    /* 一致性检验 */
    /* 在上面的 detect 探测中，最重要的处理就是要通过探测的芯片 ID，设置 info的type，
     * 通过检查返回的 info 中的 type 字段，确认是否探测成功
     * 若 info.type 不为空，则进行从机设备的注册 */
	if (info.type[0] == '\0') {
		dev_err(&adapter->dev,
			"%s detection function provided no name for 0x%x\n",
			driver->driver.name, addr);
	} else {
		struct i2c_client *client;

		/* Detection succeeded, instantiate the device */
        /* 探测成功，实例化从机设备 */
		if (adapter->class & I2C_CLASS_DEPRECATED)
			dev_warn(&adapter->dev,
				"This adapter will soon drop class based instantiation of devices. "
				"Please make sure client 0x%02x gets instantiated by other means. "
				"Check 'Documentation/i2c/instantiating-devices.rst' for details.\n",
				info.addr);

        /* 使用新探测到的设备信息(info.type) 及探测的从机地址
         * 创建一个 从机设备
         * 创建成功则通过 detected 加入到驱动的 clients 链表中 */
		dev_dbg(&adapter->dev, "Creating %s at 0x%02x\n",
			info.type, info.addr);
		client = i2c_new_client_device(adapter, &info);
		if (!IS_ERR(client))
			list_add_tail(&client->detected, &driver->clients);
		else
			dev_err(&adapter->dev, "Failed creating %s at 0x%02x\n",
				info.type, info.addr);
	}
	return 0;
}

/* 进行从设备的探测
 * 探测只会发生在 注册适配器 或 注册新的驱动 */
static int i2c_detect(struct i2c_adapter *adapter, struct i2c_driver *driver)
{
	const unsigned short *address_list;
	struct i2c_client *temp_client;
	int i, err = 0;

    /* 驱动必须支持探测功能
     * 有探测回调函数 detect 及 地址列表
     * 探测时就是使用地址列表中的每个地址访问设备 */
	address_list = driver->address_list;
	if (!driver->detect || !address_list)
		return 0;

	/* Warn that the adapter lost class based instantiation */
    /* 适配器不支持 class，警告适配器丢失了基于类的实例化 */
	if (adapter->class == I2C_CLASS_DEPRECATED) {
		dev_dbg(&adapter->dev,
			"This adapter dropped support for I2C classes and won't auto-detect %s devices anymore. "
			"If you need it, check 'Documentation/i2c/instantiating-devices.rst' for alternatives.\n",
			driver->driver.name);
		return 0;
	}

	/* Stop here if the classes do not match */
    /* 若适配器支持的 class 与 驱动支持的 class 不能匹配，则无法匹配 */
	if (!(adapter->class & driver->class))
		return 0;

	/* Set up a temporary client to help detect callback */
    /* 设置一个临时的 clinet 用于从设备探测回调 */
	temp_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!temp_client)
		return -ENOMEM;
	temp_client->adapter = adapter;

    /* 遍历驱动支持的 从地址列表，逐个处理, 核心是调用驱动的 detect 回调 */
	for (i = 0; address_list[i] != I2C_CLIENT_END; i += 1) {
		dev_dbg(&adapter->dev,
			"found normal entry for adapter %d, addr 0x%02x\n",
			i2c_adapter_id(adapter), address_list[i]);
        /* 临时从设备地址 */
		temp_client->addr = address_list[i];
        /* 对从机进行探测，若探测成功 则会注册从设备 */
		err = i2c_detect_address(temp_client, driver);
		if (unlikely(err))
			break;
	}

	kfree(temp_client);
	return err;
}

/* i2c 快速探测函数(读操作)
 * 使用 I2C_SMBUS_QUICK 命令类型，发送对从设备的读操作，进行从设备探测 */
int i2c_probe_func_quick_read(struct i2c_adapter *adap, unsigned short addr)
{
    /* 使用 I2C_SMBUS_QUICK 命令类型，进行从设备读操作
     * 实际处理中，是发送了 一个 i2c_msg 消息，该消息没有数据内容，
     * 从总线传输的消息看，只是传输了 start信号 和 从设备地址(读) 和 STOP
     * 如果消息传输成功，表示从设备回复了 ACK，那从设备就是存在的 */
	return i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ, 0,
			      I2C_SMBUS_QUICK, NULL) >= 0;
}
EXPORT_SYMBOL_GPL(i2c_probe_func_quick_read);

/* 探测从设备
 * @info:   提供用于探测成功后，创建从设备时使用的信息
 * @addr_list:  需要探测的地址列表，只能是 7bit地址
 * @probe:  探测处理函数，若未提供，则默认使用 i2c_default_probe 
 *
 * 本函数检查 addr_list 地址列表中的地址有效性，并检查地址是否已被占用(注册了对
 * 应的从设备), 并对每个地址调用 probe 回调，若某个地址探测成功，则返回 1
 * 当探测成功，probe返回 1时，将使用对应的从机地址注册新的从设备 
 * */
struct i2c_client *
i2c_new_scanned_device(struct i2c_adapter *adap,
		       struct i2c_board_info *info,
		       unsigned short const *addr_list,
		       int (*probe)(struct i2c_adapter *adap, unsigned short addr))
{
	int i;

    /* 未提供 probe 时，使用默认的 i2c_default_probe */
	if (!probe)
		probe = i2c_default_probe;

    /* 遍历地址列表的从地址，只能处理7bit地址
     * 使用 I2C_CLIENT_END 做为地址结束符 */
	for (i = 0; addr_list[i] != I2C_CLIENT_END; i++) {
		/* Check address validity */
        /* 检查地址有效性，屏蔽保留地址 */
		if (i2c_check_7bit_addr_validity_strict(addr_list[i]) < 0) {
			dev_warn(&adap->dev, "Invalid 7-bit address 0x%02x\n",
				 addr_list[i]);
			continue;
		}

		/* Check address availability (7 bit, no need to encode flags) */
        /* 检查地址是否已被占用 */
		if (i2c_check_addr_busy(adap, addr_list[i])) {
			dev_dbg(&adap->dev,
				"Address 0x%02x already in use, not probing\n",
				addr_list[i]);
			continue;
		}

		/* Test address responsiveness */
        /* 测试从地址是否有设备响应，若成功响应则应该返回 1，退出本循环 */
		if (probe(adap, addr_list[i]))
			break;
	}

    /* 探测了所有地址均未成功，则返回失败 */
	if (addr_list[i] == I2C_CLIENT_END) {
		dev_dbg(&adap->dev, "Probing failed, no device found\n");
		return ERR_PTR(-ENODEV);
	}

    /* 使用探测有响应的地址组织从设备板级信息
     * info 中应该有 调用者 提供的其它信息 */
    /* 使用新的设备板级信息进行设备注册，并返回从设备句柄 */
	info->addr = addr_list[i];
	return i2c_new_client_device(adap, info);
}
EXPORT_SYMBOL_GPL(i2c_new_scanned_device);

/* 增加 i2c adapter 的引用
 * 参数 nr 为 adapter 的 bus 号
 * bus 号是 i2c-dev 设备节点的 次设备号, 与 i2c-x 序号相同 */
struct i2c_adapter *i2c_get_adapter(int nr)
{
	struct i2c_adapter *adapter;

    /* 从 idr 中找到 nr bus 号的 i2c_adapter 结构
     * 找不到说明没有注册过这个 i2c_adapter, 返回 NULL */
	mutex_lock(&core_lock);
	adapter = idr_find(&i2c_adapter_idr, nr);
	if (!adapter)
		goto exit;

    /* 尝试增加对 i2c_adapter 的引用
     * 无法引用则返回 NULL */
	if (try_module_get(adapter->owner))
		get_device(&adapter->dev);
	else
		adapter = NULL;

 exit:
	mutex_unlock(&core_lock);
	return adapter;
}
EXPORT_SYMBOL(i2c_get_adapter);

/* 释放对 i2c_adapter 的引用
 * 需要与 i2c_get_adapter 成对使用 */
void i2c_put_adapter(struct i2c_adapter *adap)
{
	if (!adap)
		return;

    /* 需要在 put_device 之前, 保证 adap 不会被释放 */
	module_put(adap->owner);
	/* Should be last, otherwise we risk use-after-free with 'adap' */
    /* 需要最后才能释放对 dev 的引用
     * 因为在释放引用时, 可能会导致 adapter 结构被释放, 出现错误引用 */
	put_device(&adap->dev);
}
EXPORT_SYMBOL(i2c_put_adapter);

/**
 * i2c_get_dma_safe_msg_buf() - get a DMA safe buffer for the given i2c_msg
 * @msg: the message to be checked
 * @threshold: the minimum number of bytes for which using DMA makes sense.
 *	       Should at least be 1.
 *
 * Return: NULL if a DMA safe buffer was not obtained. Use msg->buf with PIO.
 *	   Or a valid pointer to be used with DMA. After use, release it by
 *	   calling i2c_put_dma_safe_msg_buf().
 *
 * This function must only be called from process context!
 */
/* i2c_get_dma_safe_msg_buf - 为给定的 i2c_msg 获取一个DMA安全的缓冲区
 * @msg:    需要被检查、分配空间的信息
 * @threshold:  可以使用DMA的最小字节数。至少应该是 1
 * 
 * 返回值: 如果没有获得DMA安全缓冲区，则返回NULL，需使用 PIO方式处理 msg->buf.
 *         或者是一个用于DMA的有效指针。
 *         使用完成后，调用 i2c_put_dma_safe_msg_buf() 进行释放
 *
 * 注意，该函数只能在进程上下文中调用。
 * */
u8 *i2c_get_dma_safe_msg_buf(struct i2c_msg *msg, unsigned int threshold)
{
	/* also skip 0-length msgs for bogus thresholds of 0 */
    /* 对于虚假阈值0，也跳过长度为 0 的消息 */
	if (!threshold)
		pr_debug("DMA buffer for addr=0x%02x with length 0 is bogus\n",
			 msg->addr);
	if (msg->len < threshold || msg->len == 0)
		return NULL;

    /* 如果 msg->buf 本身是 DMA安全的，则直接返回 */
	if (msg->flags & I2C_M_DMA_SAFE)
		return msg->buf;

	pr_debug("using bounce buffer for addr=0x%02x, len=%d\n",
		 msg->addr, msg->len);

    /* 需要分 读操作 和 写操作
     * 读操作，分配空闲的DMA内存即可
     * 写操作，分配DMA内存后，还需将 msg->buf 的内容拷贝过去 */
	if (msg->flags & I2C_M_RD)
		return kzalloc(msg->len, GFP_KERNEL);
	else
		return kmemdup(msg->buf, msg->len, GFP_KERNEL);
}
EXPORT_SYMBOL_GPL(i2c_get_dma_safe_msg_buf);

/**
 * i2c_put_dma_safe_msg_buf - release DMA safe buffer and sync with i2c_msg
 * @buf: the buffer obtained from i2c_get_dma_safe_msg_buf(). May be NULL.
 * @msg: the message which the buffer corresponds to
 * @xferred: bool saying if the message was transferred
 */
/* i2c_put_dma_safe_msg_buf - 释放DMA安全缓冲区并于i2c_msg同步
 * @buf:    从 i2c_get_dma_safe_msg_buf() 获得的缓冲区，可能为NULL
 * @msg:    缓冲区所对应的消息，即需要被填充的消息体
 * @xferred:bool值，表示消息是否已被传输(对于读操作)
 *
 * 分配DMA内存后，如果是写操作，那数据已经被发送了，直接释放即可
 * 但对于读操作，读回的数据在DMA内存中，需要拷贝到 msg 中
 * */
void i2c_put_dma_safe_msg_buf(u8 *buf, struct i2c_msg *msg, bool xferred)
{
    /* 缓冲区检查，无需缓冲区则返回 */
	if (!buf || buf == msg->buf)
		return;

    /* 若为读操作，并且数据已被传输
     * 则将DMA内存中的数据拷贝到 msg 中 */
	if (xferred && msg->flags & I2C_M_RD)
		memcpy(msg->buf, buf, msg->len);

    /* 释放DMA内存 */
	kfree(buf);
}
EXPORT_SYMBOL_GPL(i2c_put_dma_safe_msg_buf);

MODULE_AUTHOR("Simon G. Vogl <simon@tk.uni-linz.ac.at>");
MODULE_DESCRIPTION("I2C-Bus main module");
MODULE_LICENSE("GPL");
