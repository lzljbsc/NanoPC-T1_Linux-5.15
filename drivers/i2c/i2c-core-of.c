// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Linux I2C core OF support code
 *
 * Copyright (C) 2008 Jochen Friedrich <jochen@scram.de>
 * based on a previous patch from Jon Smirl <jonsmirl@gmail.com>
 *
 * Copyright (C) 2013, 2018 Wolfram Sang <wsa@kernel.org>
 */

#include <dt-bindings/i2c/i2c.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/sysfs.h>

#include "i2c-core.h"

/* 将 i2c 从设备节点转换为 i2c_board_info 结构中的信息 */
int of_i2c_get_board_info(struct device *dev, struct device_node *node,
			  struct i2c_board_info *info)
{
	u32 addr;
	int ret;

    /* i2c_board_info 清0, 所有信息都放到这个结构中 */
	memset(info, 0, sizeof(*info));

    /* 获取兼容类型
     * 使用 compatible 属性中，获取第一个 , 之后的字符
     * 比如 nxp,pca9555  那这里type将是 pca9555 */
	if (of_modalias_node(node, info->type, sizeof(info->type)) < 0) {
		dev_err(dev, "of_i2c: modalias failure on %pOF\n", node);
		return -EINVAL;
	}

    /* 获取 reg 属性的值
     * reg 属性在 i2c 设备中非常重要，被做为 i2c 从设备的器件地址
     * 该属性必须被定义，否则将注册失败 */
	ret = of_property_read_u32(node, "reg", &addr);
	if (ret) {
		dev_err(dev, "of_i2c: invalid reg on %pOF\n", node);
		return ret;
	}

    /* 设备树中的 10bit 从机地址标志
     * 若为 10bit地址，则清除地址标记，并在 flags 中设置标记 */
	if (addr & I2C_TEN_BIT_ADDRESS) {
		addr &= ~I2C_TEN_BIT_ADDRESS;
		info->flags |= I2C_CLIENT_TEN;
	}

    /* 从机地址标记，标记做为从机使用
     * 但内核代码中做为从机的非常少 */
	if (addr & I2C_OWN_SLAVE_ADDRESS) {
		addr &= ~I2C_OWN_SLAVE_ADDRESS;
		info->flags |= I2C_CLIENT_SLAVE;
	}

    /* 转换到 i2c_board_info 结构中
     * 后面都使用该结构了 */
	info->addr = addr;
    /* 记录设备树节点，以及转换为 fwnode 节点 */
	info->of_node = node;
	info->fwnode = of_fwnode_handle(node);

    /* host-notify 标志 */
	if (of_property_read_bool(node, "host-notify"))
		info->flags |= I2C_CLIENT_HOST_NOTIFY;

    /* wakeup-source 标志 */
	if (of_get_property(node, "wakeup-source", NULL))
		info->flags |= I2C_CLIENT_WAKE;

	return 0;
}
EXPORT_SYMBOL_GPL(of_i2c_get_board_info);

/* 使用设备树中的从设备节点，注册 i2c 从设备 */
static struct i2c_client *of_i2c_register_device(struct i2c_adapter *adap,
						 struct device_node *node)
{
	struct i2c_client *client;
	struct i2c_board_info info;
	int ret;

	dev_dbg(&adap->dev, "of_i2c: register %pOF\n", node);

    /* 将从设备节点转换为 i2c_board_info 信息
     * 只转换基本信息，设备特殊信息需要在各设备驱动中解析 */
	ret = of_i2c_get_board_info(&adap->dev, node, &info);
	if (ret)
		return ERR_PTR(ret);

    /* 注册 i2c client 设备
     * 所属的适配器为 adap ， 从设备信息在 info 中 */
	client = i2c_new_client_device(adap, &info);
	if (IS_ERR(client))
		dev_err(&adap->dev, "of_i2c: Failure registering %pOF\n", node);

	return client;
}

/* 遍历适配器设备节点下的子节点，创建 i2c 从设备 */
void of_i2c_register_devices(struct i2c_adapter *adap)
{
	struct device_node *bus, *node;
	struct i2c_client *client;

	/* Only register child devices if the adapter has a node pointer set */
    /* 只有适配器具有设备节点，才注册子设备 */
	if (!adap->dev.of_node)
		return;

	dev_dbg(&adap->dev, "of_i2c: walking child nodes\n");

    /* 先寻找设备节点下的 "i2c-bus" 命名的子节点
     * 有些设备中，适配器设备节点下，可能存在多个以 "i2c-bus" 命名的子节点
     * 这些子节点才是真正的 i2c bus */
    /* 大部分的设备都是没有 i2c-bus 子节点的，都是直接做为适配器使用
     * 直接扫描适配器下的子节点做为 i2c 从设备 */
	bus = of_get_child_by_name(adap->dev.of_node, "i2c-bus");
	if (!bus)
		bus = of_node_get(adap->dev.of_node);

    /* 遍历 节点下的所有子节点，每一个子节点就是一个从设备 */
	for_each_available_child_of_node(bus, node) {
        /* 检查设备是否已被创建了，若已被创建则跳过 */
		if (of_node_test_and_set_flag(node, OF_POPULATED))
			continue;

        /* 将从设备节点做为设备注册到 i2c 子系统中(实际是注册到设备模型中) */
		client = of_i2c_register_device(adap, node);
		if (IS_ERR(client)) {
			dev_err(&adap->dev,
				 "Failed to create I2C device for %pOF\n",
				 node);
			of_node_clear_flag(node, OF_POPULATED);
		}
	}

    /* 释放适配器设备节点 */
	of_node_put(bus);
}

/* i2c sysfs 接口添加的设备匹配 */
/* 这是通过 new_device sysfs 接口创建的设备
 * 在应用空间，提供了 new_device 文件，可以通过向其写入设备名及从地址
 * 创建一个从设备，如  echo "abc 12" > new_device 
 * 这个在 i2c-core-base.c 中 new_device_store 处理的 
 * */
static const struct of_device_id*
i2c_of_match_device_sysfs(const struct of_device_id *matches,
				  struct i2c_client *client)
{
	const char *name;

	for (; matches->compatible[0]; matches++) {
		/*
		 * Adding devices through the i2c sysfs interface provides us
		 * a string to match which may be compatible with the device
		 * tree compatible strings, however with no actual of_node the
		 * of_match_device() will not match
		 */
        /* 通过 i2c sysfs 接口添加设备时，提供了一个字符串来匹配，
         * 该字符串可能与设备树兼容的字符串相匹配，但是没有实际的 of_node,
         * 所以 of_match_device() 将不会匹配成功 */
        /* 先完全匹配兼容字符串 */
		if (sysfs_streq(client->name, matches->compatible))
			return matches;

        /* 再匹配兼容型号 */
		name = strchr(matches->compatible, ',');
		if (!name)
			name = matches->compatible;
		else
			name++;

		if (sysfs_streq(client->name, name))
			return matches;
	}

	return NULL;
}

/* i2c_client 尝试匹配 of_device_id 
 * of_device_id 由驱动程序提供，在注册驱动或设备时尝试匹配
 * 返回匹配到的 of_device_id 结构指针
 * 另外，在设备树匹配失败时，会尝试匹配通过 sysfs 接口添加的设备 */
const struct of_device_id
*i2c_of_match_device(const struct of_device_id *matches,
		     struct i2c_client *client)
{
	const struct of_device_id *match;

	if (!(client && matches))
		return NULL;

    /* 尝试匹配驱动的兼容列表
     * of_match_device 将返回最佳匹配项的指针 */
	match = of_match_device(matches, &client->dev);
	if (match)
		return match;

    /* 设备树中并未匹配的设备
     * 再检查一下 sysfs 接口添加的设备 */
	return i2c_of_match_device_sysfs(matches, client);
}
EXPORT_SYMBOL_GPL(i2c_of_match_device);

/* 启用设备树动态修改、加载时使用
 * 用于在设备树改动时通知 i2c 核心层, 做添加、移除设备 */
#if IS_ENABLED(CONFIG_OF_DYNAMIC)
static int of_i2c_notify(struct notifier_block *nb, unsigned long action,
			 void *arg)
{
    /* 这是设备树相关的数据, 其中 dn 指向设备树节点 */
	struct of_reconfig_data *rd = arg;
	struct i2c_adapter *adap;
	struct i2c_client *client;

    /* 检查设备树的操作是添加 还是 移除 */
	switch (of_reconfig_get_state_change(action, rd)) {
	case OF_RECONFIG_CHANGE_ADD:
        /* 根据新注册的设备的父设备节点，找到 i2c adapter
         * 如果找不到, 则正常退出
         * ?? 这里会出现找不到的情况吗, 为啥会找不到。。。 */
		adap = of_find_i2c_adapter_by_node(rd->dn->parent);
		if (adap == NULL)
			return NOTIFY_OK;	/* not for us */

        /* 检查是否已被创建过设备了 */
		if (of_node_test_and_set_flag(rd->dn, OF_POPULATED)) {
			put_device(&adap->dev);
			return NOTIFY_OK;
		}

        /* 使用新添加的设备节点，注册 i2c 从设备 */
		client = of_i2c_register_device(adap, rd->dn);
		if (IS_ERR(client)) {
			dev_err(&adap->dev, "failed to create client for '%pOF'\n",
				 rd->dn);
			put_device(&adap->dev);
			of_node_clear_flag(rd->dn, OF_POPULATED);
			return notifier_from_errno(PTR_ERR(client));
		}
		put_device(&adap->dev);
		break;
	case OF_RECONFIG_CHANGE_REMOVE:
        /* 移除操作
         * 设备节点并未注册为设备的，无需做任何操作 */
		/* already depopulated? */
		if (!of_node_check_flag(rd->dn, OF_POPULATED))
			return NOTIFY_OK;

		/* find our device by node */
        /* 通过设备树节点找到已经被注册的设备结构
         * 找不到那就是没有啦。。。 */
		client = of_find_i2c_device_by_node(rd->dn);
		if (client == NULL)
			return NOTIFY_OK;	/* no? not meant for us */

		/* unregister takes one ref away */
        /* 注销从设备 */
		i2c_unregister_device(client);

		/* and put the reference of the find */
		put_device(&client->dev);
		break;
	}

	return NOTIFY_OK;
}

/* 动态设备树 i2c核心层通知器 */
struct notifier_block i2c_of_notifier = {
	.notifier_call = of_i2c_notify,
};
#endif /* CONFIG_OF_DYNAMIC */
