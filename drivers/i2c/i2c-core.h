/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * i2c-core.h - interfaces internal to the I2C framework
 * I2C框架内部的接口
 */

#include <linux/kconfig.h>
#include <linux/rwsem.h>

/* 静态注册板级信息时使用
 * 用于记录 struct i2c_board_info 数据，并加入到链表中 */
/* 在使用 i2c_register_board_info 时，提供的 i2c_board_info
 * 由函数内部转换为 i2c_devinfo 结构，并挂到链表中 */
struct i2c_devinfo {
	struct list_head	list;
	int			busnum;
	struct i2c_board_info	board_info;
};

/* board_lock protects board_list and first_dynamic_bus_num.
 * only i2c core components are allowed to use these symbols.
 */
/* __i2c_board_lock 用于保护 __i2c_board_list 和 __i2c_first_dynamic_bus_num
 * 只能由 i2c 核心层使用 */
/* __i2c_board_list 用于做为由代码中注册的 i2c_board_info 的链表头 */
extern struct rw_semaphore	__i2c_board_lock;
extern struct list_head	__i2c_board_list;
extern int		__i2c_first_dynamic_bus_num;

/* 检查7bit地址是否为有效地址(去除预留地址) */
int i2c_check_7bit_addr_validity_strict(unsigned short addr);
/* 从 i2c 设备的 resource 中获取中断资源(中断号起始号) */
int i2c_dev_irq_from_resources(const struct resource *resources,
			       unsigned int num_resources);

/*
 * We only allow atomic transfers for very late communication, e.g. to access a
 * PMIC when powering down. Atomic transfers are a corner case and not for
 * generic use!
 */
/* 只允许在非常晚的通信中进行原子传输，例如在关闭电源时访问PMIC。
 * 原子传输是一种极限情况，不适合通用使用 */
/* 这里的理解是，某些极限情况下，传输时需要保证是原子操作，即不能有中断打断，也
 * 不会被其它任务打断等，系统正常运行状态下，不应该原子操作 */
static inline bool i2c_in_atomic_xfer_mode(void)
{
	return system_state > SYSTEM_RUNNING &&
	       (IS_ENABLED(CONFIG_PREEMPT_COUNT) ? !preemptible() : irqs_disabled());
}

/* 锁定i2c适配器 总线 */
static inline int __i2c_lock_bus_helper(struct i2c_adapter *adap)
{
	int ret = 0;

    /* 判断是否需要进行原子操作，若无需原子操作，则为调用 i2c_lock_bus */
	if (i2c_in_atomic_xfer_mode()) {
		WARN(!adap->algo->master_xfer_atomic && !adap->algo->smbus_xfer_atomic,
		     "No atomic I2C transfer handler for '%s'\n", dev_name(&adap->dev));
		ret = i2c_trylock_bus(adap, I2C_LOCK_SEGMENT) ? 0 : -EAGAIN;
	} else {
		i2c_lock_bus(adap, I2C_LOCK_SEGMENT);
	}

	return ret;
}

/* 检查 i2c 适配器是否为挂起状态
 * 若为挂起状态，返回 ESHUTDOWN, 否则返回 0 */
static inline int __i2c_check_suspended(struct i2c_adapter *adap)
{
    /* 挂起状态，由 i2c_mark_adapter_suspended 设置
     * 恢复状态，由 i2c_mark_adapter_resumed 设置 */
	if (test_bit(I2C_ALF_IS_SUSPENDED, &adap->locked_flags)) {
        /* 报告总线的挂起状态
         * 通过 I2C_ALF_SUSPEND_REPORTED 控制仅第一次调用时报告 */
		if (!test_and_set_bit(I2C_ALF_SUSPEND_REPORTED, &adap->locked_flags))
			dev_WARN(&adap->dev, "Transfer while suspended\n");
		return -ESHUTDOWN;
	}

	return 0;
}

/* acpi 结构，arm 不支持 */
#ifdef CONFIG_ACPI
void i2c_acpi_register_devices(struct i2c_adapter *adap);

int i2c_acpi_get_irq(struct i2c_client *client);
#else /* CONFIG_ACPI */
static inline void i2c_acpi_register_devices(struct i2c_adapter *adap) { }

static inline int i2c_acpi_get_irq(struct i2c_client *client)
{
	return 0;
}
#endif /* CONFIG_ACPI */
extern struct notifier_block i2c_acpi_notifier;

/* acpi 结构，arm 不支持 */
#ifdef CONFIG_ACPI_I2C_OPREGION
int i2c_acpi_install_space_handler(struct i2c_adapter *adapter);
void i2c_acpi_remove_space_handler(struct i2c_adapter *adapter);
#else /* CONFIG_ACPI_I2C_OPREGION */
static inline int i2c_acpi_install_space_handler(struct i2c_adapter *adapter) { return 0; }
static inline void i2c_acpi_remove_space_handler(struct i2c_adapter *adapter) { }
#endif /* CONFIG_ACPI_I2C_OPREGION */

/* 遍历适配器设备节点下的子节点，创建 i2c 从设备 */
#ifdef CONFIG_OF
void of_i2c_register_devices(struct i2c_adapter *adap);
#else
static inline void of_i2c_register_devices(struct i2c_adapter *adap) { }
#endif
/* 动态设备树 i2c核心层通知器 */
extern struct notifier_block i2c_of_notifier;
