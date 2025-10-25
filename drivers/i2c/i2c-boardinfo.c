// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * i2c-boardinfo.c - collect pre-declarations of I2C devices
 */

#include <linux/export.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/property.h>
#include <linux/rwsem.h>
#include <linux/slab.h>

#include "i2c-core.h"


/* These symbols are exported ONLY FOR the i2c core.
 * No other users will be supported.
 */
DECLARE_RWSEM(__i2c_board_lock);
EXPORT_SYMBOL_GPL(__i2c_board_lock);

LIST_HEAD(__i2c_board_list);
EXPORT_SYMBOL_GPL(__i2c_board_list);

int __i2c_first_dynamic_bus_num;
EXPORT_SYMBOL_GPL(__i2c_first_dynamic_bus_num);


/**
 * i2c_register_board_info - statically declare I2C devices
 * @busnum: identifies the bus to which these devices belong
 * @info: vector of i2c device descriptors
 * @len: how many descriptors in the vector; may be zero to reserve
 *	the specified bus number.
 *
 * Systems using the Linux I2C driver stack can declare tables of board info
 * while they initialize.  This should be done in board-specific init code
 * near arch_initcall() time, or equivalent, before any I2C adapter driver is
 * registered.  For example, mainboard init code could define several devices,
 * as could the init code for each daughtercard in a board stack.
 *
 * The I2C devices will be created later, after the adapter for the relevant
 * bus has been registered.  After that moment, standard driver model tools
 * are used to bind "new style" I2C drivers to the devices.  The bus number
 * for any device declared using this routine is not available for dynamic
 * allocation.
 *
 * The board info passed can safely be __initdata, but be careful of embedded
 * pointers (for platform_data, functions, etc) since that won't be copied.
 */
/* i2c_register_board_info - 静态声明I2C设备
 * @busnum: 标识这些设备所属的总线
 * @info:   i2c设备描述符向量
 * @len:    向量中有多少描述子；可以为零，以保留指定的 bus 编号。
 *
 * 使用 Linux i2c driver 的系统可以在初始化时声明板级信息表。
 * 这应该在注册任何 i2c 适配器驱动程序之前，在 arch_initcall() 阶段 或 等效的特
 * 定与板的 init 代码中完成。（总之就是要尽量的早一些）
 * 例如，主板的init代码可以定义多个设备，单板中每个子卡也可以定义多个设备。
 *
 * 在注册相关总线的适配器之后，将创建i2c设备。然后，使用标准驱动程序模型工具将
 * ”新型" i2c 驱动程序绑定到设备。使用该例程声明的任何设备的总线号都不能用于动态
 * 分配。
 *
 * 传递的板级信息可以安全的放在 __initdata 中，但要注意嵌入的指针(如
 * platform_data, 函数等), 因为它们不会被复制
 * */
int i2c_register_board_info(int busnum, struct i2c_board_info const *info, unsigned len)
{
	int status;

	down_write(&__i2c_board_lock);

	/* dynamic bus numbers will be assigned after the last static one */
    /* 动态分配的bus号必须在最后一个静态号之后
     * 所以可以使用本函数保留一些 bus号(注册一个 busnum 大一点的从设备) */
	if (busnum >= __i2c_first_dynamic_bus_num)
		__i2c_first_dynamic_bus_num = busnum + 1;

    /* 逐个遍历需要注册的 i2c_board_info
     * 任何一个出错则会退出 */
	for (status = 0; len; len--, info++) {
		struct i2c_devinfo	*devinfo;

        /* 分配新的 struct i2c_devinfo
         * 该结构是最终注册到 i2c 核心层的数据 */
		devinfo = kzalloc(sizeof(*devinfo), GFP_KERNEL);
		if (!devinfo) {
			pr_debug("i2c-core: can't register boardinfo!\n");
			status = -ENOMEM;
			break;
		}

        /* 记录从设备所在的适配器 bus号
         * 并且 拷贝 i2c_board_info, 注意，这里是拷贝的 */
		devinfo->busnum = busnum;
		devinfo->board_info = *info;

        /* 从设备的资源结构也是拷贝的 */
		if (info->resources) {
			devinfo->board_info.resources =
				kmemdup(info->resources,
					info->num_resources *
						sizeof(*info->resources),
					GFP_KERNEL);
			if (!devinfo->board_info.resources) {
				status = -ENOMEM;
				kfree(devinfo);
				break;
			}
		}

        /* 添加到 __i2c_board_list 链表 
         * 在注册适配器时，将遍历该链表进行设备注册 */
		list_add_tail(&devinfo->list, &__i2c_board_list);
	}

	up_write(&__i2c_board_lock);

	return status;
}
