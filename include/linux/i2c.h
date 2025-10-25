/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * i2c.h - definitions for the Linux i2c bus interface
 * Copyright (C) 1995-2000 Simon G. Vogl
 * Copyright (C) 2013-2019 Wolfram Sang <wsa@kernel.org>
 *
 * With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi> and
 * Frodo Looijaard <frodol@dds.nl>
 */
#ifndef _LINUX_I2C_H
#define _LINUX_I2C_H

#include <linux/acpi.h>		/* for acpi_handle */
#include <linux/mod_devicetable.h>
#include <linux/device.h>	/* for struct device */
#include <linux/sched.h>	/* for completion */
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/rtmutex.h>
#include <linux/irqdomain.h>		/* for Host Notify IRQ */
#include <linux/of.h>		/* for struct device_node */
#include <linux/swab.h>		/* for swab16 */
#include <uapi/linux/i2c.h>

/* 定义一条 i2c bus 总线 */
extern struct bus_type i2c_bus_type;
/* 定义 i2c adapter 设备类型 */
extern struct device_type i2c_adapter_type;
/* 定义 i2c client 设备类型 */
extern struct device_type i2c_client_type;

/* --- General options ------------------------------------------------	*/

struct i2c_msg;
struct i2c_algorithm;
struct i2c_adapter;
struct i2c_client;
struct i2c_driver;
struct i2c_device_identity;
union i2c_smbus_data;
struct i2c_board_info;
enum i2c_slave_event;
typedef int (*i2c_slave_cb_t)(struct i2c_client *client,
			      enum i2c_slave_event event, u8 *val);

/* I2C Frequency Modes */
/* I2C 总线不同模式的时钟线频率 */
#define I2C_MAX_STANDARD_MODE_FREQ	100000
#define I2C_MAX_FAST_MODE_FREQ		400000
#define I2C_MAX_FAST_MODE_PLUS_FREQ	1000000
#define I2C_MAX_TURBO_MODE_FREQ		1400000
#define I2C_MAX_HIGH_SPEED_MODE_FREQ	3400000
#define I2C_MAX_ULTRA_FAST_MODE_FREQ	5000000

struct module;
struct property_entry;

#if IS_ENABLED(CONFIG_I2C)
/* Return the Frequency mode string based on the bus frequency */
/* 根据总线速率返回总线模式
 * 这是根据总线模式的最高频率换算的, 并非根据实际速率 */
const char *i2c_freq_mode_string(u32 bus_freq_hz);

/*
 * The master routines are the ones normally used to transmit data to devices
 * on a bus (or read from them). Apart from two basic transfer functions to
 * transmit one message at a time, a more complex version can be used to
 * transmit an arbitrary number of messages without interruption.
 * @count must be less than 64k since msg.len is u16.
 */
/* 主机用于向总线上的设备传输数据(或从设备读取数据).
 * 除了两个基本的一次传输一条消息之外，还有一个更复杂的版本可以不间断地传输任意
 * 数量的消息。
 * @count 必须小于64K，因为 msg.len 是 u16 类型的 */
int i2c_transfer_buffer_flags(const struct i2c_client *client,
			      char *buf, int count, u16 flags);

/**
 * i2c_master_recv - issue a single I2C message in master receive mode
 * @client: Handle to slave device
 * @buf: Where to store data read from slave
 * @count: How many bytes to read, must be less than 64k since msg.len is u16
 *
 * Returns negative errno, or else the number of bytes read.
 */
/* i2c_master_recv - 在主机接收模式下发出一条I2C消息(读从机数据)
 * @client: 需要访问的从机句柄
 * @buf:    从从设备读取的数据存放的缓冲区
 * @count:  读取的数据长度，必须小于64k
 *
 * 读取错误返回负值 errno, 正确则返回读到的数据长度 
 * */
static inline int i2c_master_recv(const struct i2c_client *client,
				  char *buf, int count)
{
	return i2c_transfer_buffer_flags(client, buf, count, I2C_M_RD);
};

/**
 * i2c_master_recv_dmasafe - issue a single I2C message in master receive mode
 *			     using a DMA safe buffer
 * @client: Handle to slave device
 * @buf: Where to store data read from slave, must be safe to use with DMA
 * @count: How many bytes to read, must be less than 64k since msg.len is u16
 *
 * Returns negative errno, or else the number of bytes read.
 */
/* i2c_master_recv_dmasafe - 在主机接收模式下发出一条I2C消息(读从机数据),数据缓
 * 冲区是DMA安全的
 * @client: 需要访问的从机句柄
 * @buf:    从从设备读取的数据存放的缓冲区, 必须使用DMA安全的缓冲区
 * @count:  读取的数据长度，必须小于64k
 *
 * 读取错误返回负值 errno, 正确则返回读到的数据长度 
 * */
static inline int i2c_master_recv_dmasafe(const struct i2c_client *client,
					  char *buf, int count)
{
	return i2c_transfer_buffer_flags(client, buf, count,
					 I2C_M_RD | I2C_M_DMA_SAFE);
};

/**
 * i2c_master_send - issue a single I2C message in master transmit mode
 * @client: Handle to slave device
 * @buf: Data that will be written to the slave
 * @count: How many bytes to write, must be less than 64k since msg.len is u16
 *
 * Returns negative errno, or else the number of bytes written.
 */
/* i2c_master_send - 在主机发送模式下发出一条I2C消息(写从机数据)
 * @client: 需要访问的从机句柄
 * @buf:    向从设备写入的数据的缓冲区
 * @count:  待写入的数据长度，必须小于64k
 *
 * 写入错误返回负值 errno, 正确则返回写入的数据长度
 * */
static inline int i2c_master_send(const struct i2c_client *client,
				  const char *buf, int count)
{
	return i2c_transfer_buffer_flags(client, (char *)buf, count, 0);
};

/**
 * i2c_master_send_dmasafe - issue a single I2C message in master transmit mode
 *			     using a DMA safe buffer
 * @client: Handle to slave device
 * @buf: Data that will be written to the slave, must be safe to use with DMA
 * @count: How many bytes to write, must be less than 64k since msg.len is u16
 *
 * Returns negative errno, or else the number of bytes written.
 */
/* i2c_master_send_dmasafe - 在主机发送模式下发出一条I2C消息(写从机数据),数据缓
 * 冲区是DMA安全的
 * @client: 需要访问的从机句柄
 * @buf:    向从设备写入的数据的缓冲区, 必须使用DMA安全的缓冲区
 * @count:  待写入的数据长度，必须小于64k
 *
 * 写入错误返回负值 errno, 正确则返回写入的数据长度
 * */
static inline int i2c_master_send_dmasafe(const struct i2c_client *client,
					  const char *buf, int count)
{
	return i2c_transfer_buffer_flags(client, (char *)buf, count,
					 I2C_M_DMA_SAFE);
};

/* Transfer num messages.
 */
/* 传输 num 数量的 i2c_msg 消息体 */
int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num);
/* Unlocked flavor */
/* 非加锁版本的 i2c_transfer */
int __i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num);

/* This is the very generalized SMBus access routine. You probably do not
   want to use this, though; one of the functions below may be much easier,
   and probably just as fast.
   Note that we use i2c_adapter here, because you do not need a specific
   smbus adapter to call this function. */
/* 这是非常通用的smbus访问例程。不过，大部分情况可以不使用它；
 * 下面的一些函数是更简单的接口，也可能同样快。
 * 请注意，我们在这里使用 i2c_adapter, 因为不需要特定的 smbus适配器来调用该函数 */
/* 这是读写函数的全功能封装，并且使用的是 i2c_adapter 和 从机地址，
 * 表示访问某个适配器下的某个从机地址，这样更加通用 */
s32 i2c_smbus_xfer(struct i2c_adapter *adapter, u16 addr,
		   unsigned short flags, char read_write, u8 command,
		   int protocol, union i2c_smbus_data *data);

/* Unlocked flavor */
/* 非加锁版本的 i2c_smbus_xfer */
s32 __i2c_smbus_xfer(struct i2c_adapter *adapter, u16 addr,
		     unsigned short flags, char read_write, u8 command,
		     int protocol, union i2c_smbus_data *data);

/* Now follow the 'nice' access routines. These also document the calling
   conventions of i2c_smbus_xfer. */
/* 这是一些更加方便使用的访问例程，是通过调用 i2c_smbus_xfer 实现的 */
/* 注意，下面的这些函数是访问的 从设备 i2c_client, 由内部转换为 i2c_adapter */

u8 i2c_smbus_pec(u8 crc, u8 *p, size_t count);
s32 i2c_smbus_read_byte(const struct i2c_client *client);
s32 i2c_smbus_write_byte(const struct i2c_client *client, u8 value);
s32 i2c_smbus_read_byte_data(const struct i2c_client *client, u8 command);
s32 i2c_smbus_write_byte_data(const struct i2c_client *client,
			      u8 command, u8 value);
s32 i2c_smbus_read_word_data(const struct i2c_client *client, u8 command);
s32 i2c_smbus_write_word_data(const struct i2c_client *client,
			      u8 command, u16 value);

static inline s32
i2c_smbus_read_word_swapped(const struct i2c_client *client, u8 command)
{
	s32 value = i2c_smbus_read_word_data(client, command);

	return (value < 0) ? value : swab16(value);
}

static inline s32
i2c_smbus_write_word_swapped(const struct i2c_client *client,
			     u8 command, u16 value)
{
	return i2c_smbus_write_word_data(client, command, swab16(value));
}

/* Returns the number of read bytes */
s32 i2c_smbus_read_block_data(const struct i2c_client *client,
			      u8 command, u8 *values);
s32 i2c_smbus_write_block_data(const struct i2c_client *client,
			       u8 command, u8 length, const u8 *values);
/* Returns the number of read bytes */
s32 i2c_smbus_read_i2c_block_data(const struct i2c_client *client,
				  u8 command, u8 length, u8 *values);
s32 i2c_smbus_write_i2c_block_data(const struct i2c_client *client,
				   u8 command, u8 length, const u8 *values);
s32 i2c_smbus_read_i2c_block_data_or_emulated(const struct i2c_client *client,
					      u8 command, u8 length,
					      u8 *values);
/* i2c_get_device_id - 获取设备的制造商、器件id 和 修订版本 */
int i2c_get_device_id(const struct i2c_client *client,
		      struct i2c_device_identity *id);
#endif /* I2C */

/**
 * struct i2c_device_identity - i2c client device identification
 * @manufacturer_id: 0 - 4095, database maintained by NXP
 * @part_id: 0 - 511, according to manufacturer
 * @die_revision: 0 - 7, according to manufacturer
 */
/* struct i2c_device_identity - I2C客户端设备识别
 * @manufacturer_id: 0 - 4095, NXP维护的数据库
 * @part_id: 0 - 511, 制造商编码
 * @die_revision: 0 - 7, 制造商编码
 * */
struct i2c_device_identity {
	u16 manufacturer_id;
#define I2C_DEVICE_ID_NXP_SEMICONDUCTORS                0
#define I2C_DEVICE_ID_NXP_SEMICONDUCTORS_1              1
#define I2C_DEVICE_ID_NXP_SEMICONDUCTORS_2              2
#define I2C_DEVICE_ID_NXP_SEMICONDUCTORS_3              3
#define I2C_DEVICE_ID_RAMTRON_INTERNATIONAL             4
#define I2C_DEVICE_ID_ANALOG_DEVICES                    5
#define I2C_DEVICE_ID_STMICROELECTRONICS                6
#define I2C_DEVICE_ID_ON_SEMICONDUCTOR                  7
#define I2C_DEVICE_ID_SPRINTEK_CORPORATION              8
#define I2C_DEVICE_ID_ESPROS_PHOTONICS_AG               9
#define I2C_DEVICE_ID_FUJITSU_SEMICONDUCTOR            10
#define I2C_DEVICE_ID_FLIR                             11
#define I2C_DEVICE_ID_O2MICRO                          12
#define I2C_DEVICE_ID_ATMEL                            13
#define I2C_DEVICE_ID_NONE                         0xffff
	u16 part_id;
	u8 die_revision;
};

/* I2C_PROTOCOL_SMBUS_ALERT 
 *  是从设备向主机发送警报的机制，用于告知主机某种条件的发生 
 * I2C_PROTOCOL_SMBUS_HOST_NOTIFY 
 *  是主机向从设备发送通知的机制，用于指示从设备进行特定操作 */
enum i2c_alert_protocol {
	I2C_PROTOCOL_SMBUS_ALERT,
	I2C_PROTOCOL_SMBUS_HOST_NOTIFY,
};

/**
 * struct i2c_driver - represent an I2C device driver
 * @class: What kind of i2c device we instantiate (for detect)
 * @probe: Callback for device binding - soon to be deprecated
 * @probe_new: New callback for device binding
 * @remove: Callback for device unbinding
 * @shutdown: Callback for device shutdown
 * @alert: Alert callback, for example for the SMBus alert protocol
 * @command: Callback for bus-wide signaling (optional)
 * @driver: Device driver model driver
 * @id_table: List of I2C devices supported by this driver
 * @detect: Callback for device detection
 * @address_list: The I2C addresses to probe (for detect)
 * @clients: List of detected clients we created (for i2c-core use only)
 *
 * The driver.owner field should be set to the module owner of this driver.
 * The driver.name field should be set to the name of this driver.
 *
 * For automatic device detection, both @detect and @address_list must
 * be defined. @class should also be set, otherwise only devices forced
 * with module parameters will be created. The detect function must
 * fill at least the name field of the i2c_board_info structure it is
 * handed upon successful detection, and possibly also the flags field.
 *
 * If @detect is missing, the driver will still work fine for enumerated
 * devices. Detected devices simply won't be supported. This is expected
 * for the many I2C/SMBus devices which can't be detected reliably, and
 * the ones which can always be enumerated in practice.
 *
 * The i2c_client structure which is handed to the @detect callback is
 * not a real i2c_client. It is initialized just enough so that you can
 * call i2c_smbus_read_byte_data and friends on it. Don't do anything
 * else with it. In particular, calling dev_dbg and friends on it is
 * not allowed.
 */
/* struct i2c_driver - 定义一个i2c设备驱动程序
 * @class:  该驱动可以实例化什么样的设备(用于探测机制), 参考 i2c_detect
 * @probe:  用于设备绑定的回调函数 - 即将被弃用
 * @probe_new:  新的用于设备绑定的回调函数
 * @remove: 用于设备解绑的回调函数
 * @shutdown:   设备关闭时的回调函数
 * @alert:  Alert回调函数，例如 SMBus Alert 协议
 * @command:    总线级信令的回调函数(可选) , 参考 i2c_clients_command
 * @driver: 设备驱动模型的驱动程序结构
 * @id_table:   此驱动程序支持的I2C设备列表
 * @detect: 设备探测回调(可选的), 用于探测从设备
 * @address_list:   驱动可以探测的I2C地址列表(用于探测机制)
 * @clients:    驱动创建的，探测到的从设备链表(仅用于I2C核心层)
 * 
 * driver.owner 字段应该设置为该驱动程序的模块所有者，一般为 THIS_MODULE
 * driver.name 字段应该设置为该驱动程序的名称
 *
 * 对于自动设备的探测，必须同时定义 @detect 和 @address_list. @class 也应该设置，
 * 否则只会创建强制使用模块参数的设备。 detect 函数必须在探测成功后，至少填充传
 * 递给它的 i2c_board_info结构体的 name 字段(这里应该是type字段吧，程序中检测是
 * type字段是否被填充了)，可能还需要填充flags字段。
 *
 * 如果没有 @detect, 驱动程序仍然可以在枚举设备上正常工作。但设备检测功能将不受
 * 支持。这对于许多在实际应用中可以一一列举又无法可靠检测的I2C/SMBus器件是有用的。
 *
 * 传给 @detect回调的 i2c_client 结构并不是真正的 i2c_client(构造的临时结构)。
 * 它已经初始化，因此可以调用 i2c_smbus_read_byte及其相关函数。
 * 但需要使用它做任何其它事情。特别的，不允许调用 dev_dbg及相关函数(无name)。
 * */
struct i2c_driver {
	unsigned int class;

	/* Standard driver model interfaces */
	int (*probe)(struct i2c_client *client, const struct i2c_device_id *id);
	int (*remove)(struct i2c_client *client);

	/* New driver model interface to aid the seamless removal of the
	 * current probe()'s, more commonly unused than used second parameter.
	 */
	int (*probe_new)(struct i2c_client *client);

	/* driver model interfaces that don't relate to enumeration  */
	void (*shutdown)(struct i2c_client *client);

	/* Alert callback, for example for the SMBus alert protocol.
	 * The format and meaning of the data value depends on the protocol.
	 * For the SMBus alert protocol, there is a single bit of data passed
	 * as the alert response's low bit ("event flag").
	 * For the SMBus Host Notify protocol, the data corresponds to the
	 * 16-bit payload data reported by the slave device acting as master.
	 */
	void (*alert)(struct i2c_client *client, enum i2c_alert_protocol protocol,
		      unsigned int data);

	/* a ioctl like command that can be used to perform specific functions
	 * with the device.
	 */
	int (*command)(struct i2c_client *client, unsigned int cmd, void *arg);

	struct device_driver driver;
	const struct i2c_device_id *id_table;

	/* Device detection callback for automatic device creation */
	int (*detect)(struct i2c_client *client, struct i2c_board_info *info);
	const unsigned short *address_list;
	struct list_head clients;
};
/* 通过 struct device_driver 结构查找包含它的 i2c_driver 结构 */
#define to_i2c_driver(d) container_of(d, struct i2c_driver, driver)

/**
 * struct i2c_client - represent an I2C slave device
 * @flags: see I2C_CLIENT_* for possible flags
 * @addr: Address used on the I2C bus connected to the parent adapter.
 * @name: Indicates the type of the device, usually a chip name that's
 *	generic enough to hide second-sourcing and compatible revisions.
 * @adapter: manages the bus segment hosting this I2C device
 * @dev: Driver model device node for the slave.
 * @init_irq: IRQ that was set at initialization
 * @irq: indicates the IRQ generated by this device (if any)
 * @detected: member of an i2c_driver.clients list or i2c-core's
 *	userspace_devices list
 * @slave_cb: Callback when I2C slave mode of an adapter is used. The adapter
 *	calls it to pass on slave events to the slave driver.
 * @devres_group_id: id of the devres group that will be created for resources
 *	acquired when probing this device.
 *
 * An i2c_client identifies a single device (i.e. chip) connected to an
 * i2c bus. The behaviour exposed to Linux is defined by the driver
 * managing the device.
 */
/* struct i2c_client - 定义一个i2c从设备
 * @flags:  标志位, I2C_CLIENT_* 标志可用
 * @addr:   连接到父适配器的I2C总线上使用的地址
 * @name:   表示设备的类型，通常是一个足够通用的芯片名称，以隐藏第二源和兼容的修
 * 订。
 * @adapter:指向所属的 adapter
 * @dev:    驱动程序模型 设备结构
 * @init_irq:   初始化时设置的 IRQ
 * @irq:    表示该设备产生的IRQ(如果有)这是实际会注册的irq
 * @detected:   如果该从机设备是通过用户空间注册的, 使用该链表加入到所属适配器的
 * userspace_clients 链表中, 另外，如果是通过驱动程序探测的设备，则加到驱动程序
 * 的 i2c_driver.clients 链表中
 * @slave_cb:   当一个适配器做为I2C从模式使用时的回调。适配器调用该回调将时间传
 * 递给从设备驱动程序
 * @devres_group_id:    探测设备时，为获取的资源创建的 devres 组ID 
 *
 * i2c_client 标识连接到 i2c 总线的单个设备。
 * 暴漏给Linux的行为是由管理设备的驱动程序定义的。
 * */
struct i2c_client {
	unsigned short flags;		/* div., see below		*/
#define I2C_CLIENT_PEC		0x04	/* Use Packet Error Checking */
#define I2C_CLIENT_TEN		0x10	/* we have a ten bit chip address */
					/* Must equal I2C_M_TEN below */
#define I2C_CLIENT_SLAVE	0x20	/* we are the slave */
#define I2C_CLIENT_HOST_NOTIFY	0x40	/* We want to use I2C host notify */
#define I2C_CLIENT_WAKE		0x80	/* for board_info; true iff can wake */
#define I2C_CLIENT_SCCB		0x9000	/* Use Omnivision SCCB protocol */
					/* Must match I2C_M_STOP|IGNORE_NAK */

	unsigned short addr;		/* chip address - NOTE: 7bit	*/
					/* addresses are stored in the	*/
					/* _LOWER_ 7 bits		*/
	char name[I2C_NAME_SIZE];
	struct i2c_adapter *adapter;	/* the adapter we sit on	*/
	struct device dev;		/* the device structure		*/
	int init_irq;			/* irq set at initialization	*/
	int irq;			/* irq issued by device		*/
	struct list_head detected;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	i2c_slave_cb_t slave_cb;	/* callback for slave mode	*/
#endif
	void *devres_group_id;		/* ID of probe devres group	*/
};
/* 通过 struct device 结构查找包含它的 i2c_client 结构 */
#define to_i2c_client(d) container_of(d, struct i2c_client, dev)

/* 返回设备结构所属的 i2c_adapter, 若不是 i2c_adapter设备则返回NULL */
struct i2c_adapter *i2c_verify_adapter(struct device *dev);
/* i2c 设备 match id 表匹配, 返回匹配项的指针 */
const struct i2c_device_id *i2c_match_id(const struct i2c_device_id *id,
					 const struct i2c_client *client);

/* 查找指定 i2c 从设备匹配的驱动数据 */
const void *i2c_get_match_data(const struct i2c_client *client);

/* 通过 kobj 查找包含它的 i2c_client 结构
 * 先查找所属的 struct device 结构，再进一步查找 i2c_client */
static inline struct i2c_client *kobj_to_i2c_client(struct kobject *kobj)
{
	struct device * const dev = kobj_to_dev(kobj);
	return to_i2c_client(dev);
}

/* 获取 i2c_client 从设备的私有数据指针 */
static inline void *i2c_get_clientdata(const struct i2c_client *client)
{
	return dev_get_drvdata(&client->dev);
}

/* 设置 i2c_client 从设备的私有数据指针 */
static inline void i2c_set_clientdata(struct i2c_client *client, void *data)
{
	dev_set_drvdata(&client->dev, data);
}

/* I2C slave support */
/* I2C 从模式支持 */
/* 这是让 Linux内核驱动做为从机，实际场景中非常少
 * 目前内核驱动中仅有少量的驱动支持 */

#if IS_ENABLED(CONFIG_I2C_SLAVE)
enum i2c_slave_event {
	I2C_SLAVE_READ_REQUESTED,
	I2C_SLAVE_WRITE_REQUESTED,
	I2C_SLAVE_READ_PROCESSED,
	I2C_SLAVE_WRITE_RECEIVED,
	I2C_SLAVE_STOP,
};

int i2c_slave_register(struct i2c_client *client, i2c_slave_cb_t slave_cb);
int i2c_slave_unregister(struct i2c_client *client);
bool i2c_detect_slave_mode(struct device *dev);

static inline int i2c_slave_event(struct i2c_client *client,
				  enum i2c_slave_event event, u8 *val)
{
	return client->slave_cb(client, event, val);
}
#else
static inline bool i2c_detect_slave_mode(struct device *dev) { return false; }
#endif

/**
 * struct i2c_board_info - template for device creation
 * @type: chip type, to initialize i2c_client.name
 * @flags: to initialize i2c_client.flags
 * @addr: stored in i2c_client.addr
 * @dev_name: Overrides the default <busnr>-<addr> dev_name if set
 * @platform_data: stored in i2c_client.dev.platform_data
 * @of_node: pointer to OpenFirmware device node
 * @fwnode: device node supplied by the platform firmware
 * @swnode: software node for the device
 * @resources: resources associated with the device
 * @num_resources: number of resources in the @resources array
 * @irq: stored in i2c_client.irq
 *
 * I2C doesn't actually support hardware probing, although controllers and
 * devices may be able to use I2C_SMBUS_QUICK to tell whether or not there's
 * a device at a given address.  Drivers commonly need more information than
 * that, such as chip type, configuration, associated IRQ, and so on.
 *
 * i2c_board_info is used to build tables of information listing I2C devices
 * that are present.  This information is used to grow the driver model tree.
 * For mainboards this is done statically using i2c_register_board_info();
 * bus numbers identify adapters that aren't yet available.  For add-on boards,
 * i2c_new_client_device() does this dynamically with the adapter already known.
 */
/* struct i2c_board_info - 创建 i2c 设备模板
 * @type:   芯片类型, 用于初始化 i2c_client.name
 * @flags:  用于初始化 i2c_client.flags
 * @addr:   从设备器件地址, i2c_client.addr
 * @dev_name:如果设置了，覆盖默认的 <busnr>-<addr> dev_name 
 * @platform_data:  平台相关数据, i2c_client.dev.platform_data
 * @of_node:指向 OpenFirmware(设备树) 节点的指针
 * @fwnode: 由平台固件提供的设备节点
 * @swnode: 设备的软件节点, 这是通过 i2c_board_info 中提供的 
 *          在定义 i2c_board_info 时，提供了一个 .swnode 结构 
 *          该结构中提供了类似设备树属性的字符串及值
 * @resources:  设备关联的资源
 * @num_resources:  @resources 资源数组大小
 * @irq:    中断号, i2c_client.irq
 * 
 * I2C实际上不支持硬件探测，尽管控制器和设备可以使用I2C_SMBUS_QUICK来判断给定地
 * 址是否有设备。驱动程序通常需要更多的信息，例如芯片类型、配置、相关的IRQ，等等。
 *
 * i2c_board_info用于建立信息表，列出存在的I2C设备。这些信息用于增长驱动程序模型
 * 树。对于主板，这是使用i2c_register_board_info（）静态完成的。
 * 没有适配器的 bus 号，是因为从设备的板级信息不需要。
 * 对于扩展板，i2c_new_client_device（）使用已知的适配器动态执行
 * 此操作。
 * */
struct i2c_board_info {
	char		type[I2C_NAME_SIZE];
	unsigned short	flags;
	unsigned short	addr;
	const char	*dev_name;
	void		*platform_data;
	struct device_node *of_node;
	struct fwnode_handle *fwnode;
	const struct software_node *swnode;
	const struct resource *resources;
	unsigned int	num_resources;
	int		irq;
};

/**
 * I2C_BOARD_INFO - macro used to list an i2c device and its address
 * @dev_type: identifies the device type
 * @dev_addr: the device's address on the bus.
 *
 * This macro initializes essential fields of a struct i2c_board_info,
 * declaring what has been provided on a particular board.  Optional
 * fields (such as associated irq, or device-specific platform_data)
 * are provided using conventional syntax.
 */
/* I2C_BOARD_INFO - 用于列出I2C设备及其地址的 简易宏，定义一个从设备信息
 * @dev_type:   标识设备类型
 * @dev_addr:   设备在总线上的地址
 *
 * 该宏初始化 struct i2c_board_info的基本字段，声明特定的板级信息。
 * 可选字段(如关联的irq, platform_data等)使用传统语法提供 */
#define I2C_BOARD_INFO(dev_type, dev_addr) \
	.type = dev_type, .addr = (dev_addr)


#if IS_ENABLED(CONFIG_I2C)
/*
 * Add-on boards should register/unregister their devices; e.g. a board
 * with integrated I2C, a config eeprom, sensors, and a codec that's
 * used in conjunction with the primary hardware.
 */
/* 实例化一个 i2c从设备，设备信息由 i2c_board_info 提供 */
struct i2c_client *
i2c_new_client_device(struct i2c_adapter *adap, struct i2c_board_info const *info);

/* If you don't know the exact address of an I2C device, use this variant
 * instead, which can probe for device presence in a list of possible
 * addresses. The "probe" callback function is optional. If it is provided,
 * it must return 1 on successful probe, 0 otherwise. If it is not provided,
 * a default probing method is used.
 */
/* 如果你不知道I2C设备的确切地址，可以使用这个版本，它可以探测可能地址列表中是否
 * 存在设备。“probe”回调函数是可选的。如果提供了该函数，探测成功时必须返回1，否
 * 则返回0。如果没有提供探测方法，则使用默认探测方法。 */
struct i2c_client *
i2c_new_scanned_device(struct i2c_adapter *adap,
		       struct i2c_board_info *info,
		       unsigned short const *addr_list,
		       int (*probe)(struct i2c_adapter *adap, unsigned short addr));

/* Common custom probe functions */
/* i2c 快速探测函数(读操作) */
int i2c_probe_func_quick_read(struct i2c_adapter *adap, unsigned short addr);

/* i2c_new_dummy_device - 返回一个绑定到虚拟驱动程序的新i2c虚拟设备 */
struct i2c_client *
i2c_new_dummy_device(struct i2c_adapter *adapter, u16 address);

/* devm_i2c_new_dummy_device - 返回一个绑定到虚拟驱动程序的新 i2c 虚拟设备 */
struct i2c_client *
devm_i2c_new_dummy_device(struct device *dev, struct i2c_adapter *adap, u16 address);

/* i2c_new_ancillary_device - 用于获取实例化的从机地址并创建关联的设备 */
struct i2c_client *
i2c_new_ancillary_device(struct i2c_client *client,
			 const char *name,
			 u16 default_addr);

/* 注销 i2c 从设备 */
void i2c_unregister_device(struct i2c_client *client);

/* 检查设备是否为 i2c 设备, 如果是, 则返回 i2c_client 指针 */
struct i2c_client *i2c_verify_client(struct device *dev);
#else
static inline struct i2c_client *i2c_verify_client(struct device *dev)
{
	return NULL;
}
#endif /* I2C */

/* Mainboard arch_initcall() code should register all its I2C devices.
 * This is done at arch_initcall time, before declaring any i2c adapters.
 * Modules for add-on boards must use other calls.
 */
/* 主板的 arch_initcall() 代码中可以注册所有的 i2c 从设备.
 * 这是在 arch_initcall 阶段完成的，然后才是 i2c 适配器的注册.
 * 用于扩展板的模块必须使用其它调用。 
 * */
/* 现在都是使用设备树进行设备定义, 该接口已基本不用了  */
#ifdef CONFIG_I2C_BOARDINFO
int
i2c_register_board_info(int busnum, struct i2c_board_info const *info,
			unsigned n);
#else
static inline int
i2c_register_board_info(int busnum, struct i2c_board_info const *info,
			unsigned n)
{
	return 0;
}
#endif /* I2C_BOARDINFO */

/**
 * struct i2c_algorithm - represent I2C transfer method
 * @master_xfer: Issue a set of i2c transactions to the given I2C adapter
 *   defined by the msgs array, with num messages available to transfer via
 *   the adapter specified by adap.
 * @master_xfer_atomic: same as @master_xfer. Yet, only using atomic context
 *   so e.g. PMICs can be accessed very late before shutdown. Optional.
 * @smbus_xfer: Issue smbus transactions to the given I2C adapter. If this
 *   is not present, then the bus layer will try and convert the SMBus calls
 *   into I2C transfers instead.
 * @smbus_xfer_atomic: same as @smbus_xfer. Yet, only using atomic context
 *   so e.g. PMICs can be accessed very late before shutdown. Optional.
 * @functionality: Return the flags that this algorithm/adapter pair supports
 *   from the ``I2C_FUNC_*`` flags.
 * @reg_slave: Register given client to I2C slave mode of this adapter
 * @unreg_slave: Unregister given client from I2C slave mode of this adapter
 *
 * The following structs are for those who like to implement new bus drivers:
 * i2c_algorithm is the interface to a class of hardware solutions which can
 * be addressed using the same bus algorithms - i.e. bit-banging or the PCF8584
 * to name two of the most common.
 *
 * The return codes from the ``master_xfer{_atomic}`` fields should indicate the
 * type of error code that occurred during the transfer, as documented in the
 * Kernel Documentation file Documentation/i2c/fault-codes.rst.
 */
/* struct i2c_algorithm - 标识I2C数据传输方法
 * @master_xfer:    由I2C适配器发出由msgs数组定义的一组I2C事务，消息的数量为 num
 * @master_xfer_atomic: 与@master_xfer相同。然而，只用于原子上下文，
 * @smbus_xfer: 向给定的I2C适配发出smbus事务。如果不存在，总线层将尝试将SMBus调
 * 用转换为I2C传输。
 * @smbus_xfer_atomic:  与@smbus_xfer相同。然而，只用于原子上下文
 * @functionality:  返回方法/适配器的功能支持标志，参考 I2C_FUNC_* 
 * @reg_slave:  将给定的客户端注册到此适配器的 I2C 从机模式中
 * @unreg_slave:从本适配器的 I2C 从机模式中注销指定客户端
 *
 * 该结构体是为实现新的总线驱动程序使用的，总线驱动程序提供统一的接口
 *
 * "master_xfer{_atomic}" 返回码应该表面在传输器件发生的错误码类型，参考内核文档:
 * Documentation/i2c/fault-codes.rst
 * */
struct i2c_algorithm {
	/*
	 * If an adapter algorithm can't do I2C-level access, set master_xfer
	 * to NULL. If an adapter algorithm can do SMBus access, set
	 * smbus_xfer. If set to NULL, the SMBus protocol is simulated
	 * using common I2C messages.
	 *
	 * master_xfer should return the number of messages successfully
	 * processed, or a negative value on error
	 */
    /* 如果适配器算法不能进行i2c级访问，则将master_xfer设置为NULL。如果适配器算
     * 法可以进行SMBus访问，则设置smbus_xfer。如果设置为NULL，则使用普通I2C消息
     * 模拟SMBus协议。
     *
     * master_xfer 应该返回成功处理的消息数目，如果发生错误则返回负数 */
	int (*master_xfer)(struct i2c_adapter *adap, struct i2c_msg *msgs,
			   int num);
	int (*master_xfer_atomic)(struct i2c_adapter *adap,
				   struct i2c_msg *msgs, int num);
	int (*smbus_xfer)(struct i2c_adapter *adap, u16 addr,
			  unsigned short flags, char read_write,
			  u8 command, int size, union i2c_smbus_data *data);
	int (*smbus_xfer_atomic)(struct i2c_adapter *adap, u16 addr,
				 unsigned short flags, char read_write,
				 u8 command, int size, union i2c_smbus_data *data);

	/* To determine what the adapter supports */
	u32 (*functionality)(struct i2c_adapter *adap);

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	int (*reg_slave)(struct i2c_client *client);
	int (*unreg_slave)(struct i2c_client *client);
#endif
};

/**
 * struct i2c_lock_operations - represent I2C locking operations
 * @lock_bus: Get exclusive access to an I2C bus segment
 * @trylock_bus: Try to get exclusive access to an I2C bus segment
 * @unlock_bus: Release exclusive access to an I2C bus segment
 *
 * The main operations are wrapped by i2c_lock_bus and i2c_unlock_bus.
 */
/* struct i2c_lock_operations - 表示 i2c 锁操作
 * @lock_bus:   获得对I2C总线段的独占访问权
 * @trylock_bus:尝试获得对I2C总线段的独占访问权
 * @unlock_bus: 释放对I2C总线段的独占访问权
 *
 * 主要的操作包装在i2c_lock_bus和i2c_unlock_bus中。
 * */
struct i2c_lock_operations {
	void (*lock_bus)(struct i2c_adapter *adapter, unsigned int flags);
	int (*trylock_bus)(struct i2c_adapter *adapter, unsigned int flags);
	void (*unlock_bus)(struct i2c_adapter *adapter, unsigned int flags);
};

/**
 * struct i2c_timings - I2C timing information
 * @bus_freq_hz: the bus frequency in Hz
 * @scl_rise_ns: time SCL signal takes to rise in ns; t(r) in the I2C specification
 * @scl_fall_ns: time SCL signal takes to fall in ns; t(f) in the I2C specification
 * @scl_int_delay_ns: time IP core additionally needs to setup SCL in ns
 * @sda_fall_ns: time SDA signal takes to fall in ns; t(f) in the I2C specification
 * @sda_hold_ns: time IP core additionally needs to hold SDA in ns
 * @digital_filter_width_ns: width in ns of spikes on i2c lines that the IP core
 *	digital filter can filter out
 * @analog_filter_cutoff_freq_hz: threshold frequency for the low pass IP core
 *	analog filter
 */
/* struct i2c_timings - I2C时序信息
 * 参考 i2c_parse_fw_timings */
struct i2c_timings {
	u32 bus_freq_hz;
	u32 scl_rise_ns;
	u32 scl_fall_ns;
	u32 scl_int_delay_ns;
	u32 sda_fall_ns;
	u32 sda_hold_ns;
	u32 digital_filter_width_ns;
	u32 analog_filter_cutoff_freq_hz;
};

/**
 * struct i2c_bus_recovery_info - I2C bus recovery information
 * @recover_bus: Recover routine. Either pass driver's recover_bus() routine, or
 *	i2c_generic_scl_recovery().
 * @get_scl: This gets current value of SCL line. Mandatory for generic SCL
 *      recovery. Populated internally for generic GPIO recovery.
 * @set_scl: This sets/clears the SCL line. Mandatory for generic SCL recovery.
 *      Populated internally for generic GPIO recovery.
 * @get_sda: This gets current value of SDA line. This or set_sda() is mandatory
 *	for generic SCL recovery. Populated internally, if sda_gpio is a valid
 *	GPIO, for generic GPIO recovery.
 * @set_sda: This sets/clears the SDA line. This or get_sda() is mandatory for
 *	generic SCL recovery. Populated internally, if sda_gpio is a valid GPIO,
 *	for generic GPIO recovery.
 * @get_bus_free: Returns the bus free state as seen from the IP core in case it
 *	has a more complex internal logic than just reading SDA. Optional.
 * @prepare_recovery: This will be called before starting recovery. Platform may
 *	configure padmux here for SDA/SCL line or something else they want.
 * @unprepare_recovery: This will be called after completing recovery. Platform
 *	may configure padmux here for SDA/SCL line or something else they want.
 * @scl_gpiod: gpiod of the SCL line. Only required for GPIO recovery.
 * @sda_gpiod: gpiod of the SDA line. Only required for GPIO recovery.
 * @pinctrl: pinctrl used by GPIO recovery to change the state of the I2C pins.
 *      Optional.
 * @pins_default: default pinctrl state of SCL/SDA lines, when they are assigned
 *      to the I2C bus. Optional. Populated internally for GPIO recovery, if
 *      state with the name PINCTRL_STATE_DEFAULT is found and pinctrl is valid.
 * @pins_gpio: recovery pinctrl state of SCL/SDA lines, when they are used as
 *      GPIOs. Optional. Populated internally for GPIO recovery, if this state
 *      is called "gpio" or "recovery" and pinctrl is valid.
 */
/* struct i2c_bus_recovery_info - I2C总线恢复信息
 * @recover_bus:    恢复程序。传递驱动程序的recover_bus（）例程或
 * i2c_generic_scl_recovery（）。
 * @get_scl:        这得到了SCL线的当前值。对于一般的SCL恢复是强制的。内部填充用
 * 于通用GPIO恢复。
 * @set_scl:        这将设置/清除SCL行。对于一般的SCL恢复是强制的。内部填充用于
 * 通用GPIO恢复。
 * @get_sda:        这将获取SDA线的当前值。这个或set_sda（）对于通用的SCL恢复是
 * 必需的。如果sda_gpio是有效的GPIO，则在内部填充，用于通用GPIO恢复。
 * @set_sda:        这将设置/清除SDA行。这个或get_sda（）对于通用的SCL恢复是必需
 * 的。如果sda_gpio是有效的GPIO，则在内部填充，用于通用GPIO恢复。
 * @get_bus_free:   返回从IP核看到的总线空闲状态，以防内部逻辑比读取SDA更复杂。
 * 可选的。
 * @prepare_recovery:这将在开始恢复之前被调用。平台可以在这里为SDA/SCL线路或其他
 * 他们想要的配置padmux。
 * @unprepare_recovery:这将在完成恢复后调用。平台可以在这里为SDA/SCL线路或其他他
 * 们想要的配置padmux。
 * @scl_gpiod:      SCL线的gpid。仅用于GPIO恢复。
 * @sda_gpiod:      SDA线的gpid。仅用于GPIO恢复。
 * @pinctrl:        用于GPIO恢复改变I2C引脚的状态。
 * @pins_default:   当SCL/SDA线路被分配到I2C总线时，SCL/SDA线路的默认pinctrl状态。
 * 可选的。如果找到名为PINCTRL_STATE_DEFAULT的state，并且pinctrl是有效的，则在内
 * 部填充用于GPIO恢复。
 * @pins_gpio:      SCL/SDA线作为gpio时的恢复平针状态。可选的。如果此状态被称为
 * “GPIO”或“recovery”，并且pinctrl是有效的，则在内部填充用于GPIO恢复。
 * */
struct i2c_bus_recovery_info {
	int (*recover_bus)(struct i2c_adapter *adap);

	int (*get_scl)(struct i2c_adapter *adap);
	void (*set_scl)(struct i2c_adapter *adap, int val);
	int (*get_sda)(struct i2c_adapter *adap);
	void (*set_sda)(struct i2c_adapter *adap, int val);
	int (*get_bus_free)(struct i2c_adapter *adap);

	void (*prepare_recovery)(struct i2c_adapter *adap);
	void (*unprepare_recovery)(struct i2c_adapter *adap);

	/* gpio recovery */
	struct gpio_desc *scl_gpiod;
	struct gpio_desc *sda_gpiod;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_gpio;
};

int i2c_recover_bus(struct i2c_adapter *adap);

/* Generic recovery routines */
/* i2c recovery 操作 */
int i2c_generic_scl_recovery(struct i2c_adapter *adap);

/**
 * struct i2c_adapter_quirks - describe flaws of an i2c adapter
 * @flags: see I2C_AQ_* for possible flags and read below
 * @max_num_msgs: maximum number of messages per transfer
 * @max_write_len: maximum length of a write message
 * @max_read_len: maximum length of a read message
 * @max_comb_1st_msg_len: maximum length of the first msg in a combined message
 * @max_comb_2nd_msg_len: maximum length of the second msg in a combined message
 *
 * Note about combined messages: Some I2C controllers can only send one message
 * per transfer, plus something called combined message or write-then-read.
 * This is (usually) a small write message followed by a read message and
 * barely enough to access register based devices like EEPROMs. There is a flag
 * to support this mode. It implies max_num_msg = 2 and does the length checks
 * with max_comb_*_len because combined message mode usually has its own
 * limitations. Because of HW implementations, some controllers can actually do
 * write-then-anything or other variants. To support that, write-then-read has
 * been broken out into smaller bits like write-first and read-second which can
 * be combined as needed.
 */

/* struct i2c_adapter_quirks - 描述i2c适配器的缺陷
 * @flags:      可能的标志，参考 I2C_AQ_*
 * @max_num_msgs:   每次传输的最大消息数
 * @max_write_len:  写消息的最大长度
 * @max_read_len:   读消息的最大长度
 * @max_comb_1st_msg_len:   组合消息中第一个MSG的最大长度
 * @max_comb_2st_msg_len:   组合消息中第二个MSG的最大长度
 *
 * 关于组合消息的注意事项：有些I2C控制器每次传输只能发送一个消息，加上称为组合消
 * 息或先写后读的消息。这（通常）是一个小的写消息，后面跟着一个读消息，几乎不足
 * 以访问基于寄存器的设备，如eeprom。有一个支持这种模式的标志。它暗示max_num_msg
 * = 2，并使用max_comb_*_len进行长度检查，因为组合消息模式通常有其自身的限制。由
 * 于HW实现，一些控制器实际上可以执行“然后写任何东西”（write-then-anything）或其
 * 他变体。为了支持这一点，先写后读被分解为更小的比特位，如写优先和读优先，它们
 * 可以根据需要组合。
 * */
struct i2c_adapter_quirks {
	u64 flags;
	int max_num_msgs;
	u16 max_write_len;
	u16 max_read_len;
	u16 max_comb_1st_msg_len;
	u16 max_comb_2nd_msg_len;
};

/* struct i2c_adapter_quirks 中的 flags 使用的标记 */
/* enforce max_num_msgs = 2 and use max_comb_*_len for length checks */
#define I2C_AQ_COMB			BIT(0)
/* first combined message must be write */
#define I2C_AQ_COMB_WRITE_FIRST		BIT(1)
/* second combined message must be read */
#define I2C_AQ_COMB_READ_SECOND		BIT(2)
/* both combined messages must have the same target address */
#define I2C_AQ_COMB_SAME_ADDR		BIT(3)
/* convenience macro for typical write-then read case */
#define I2C_AQ_COMB_WRITE_THEN_READ	(I2C_AQ_COMB | I2C_AQ_COMB_WRITE_FIRST | \
					 I2C_AQ_COMB_READ_SECOND | I2C_AQ_COMB_SAME_ADDR)
/* clock stretching is not supported */
#define I2C_AQ_NO_CLK_STRETCH		BIT(4)
/* message cannot have length of 0 */
#define I2C_AQ_NO_ZERO_LEN_READ		BIT(5)
#define I2C_AQ_NO_ZERO_LEN_WRITE	BIT(6)
#define I2C_AQ_NO_ZERO_LEN		(I2C_AQ_NO_ZERO_LEN_READ | I2C_AQ_NO_ZERO_LEN_WRITE)
/* adapter cannot do repeated START */
#define I2C_AQ_NO_REP_START		BIT(7)

/*
 * i2c_adapter is the structure used to identify a physical i2c bus along
 * with the access algorithms necessary to access it.
 */
/* I2c_adapter结构用于标识物理i2c总线以及访问该总线所需的访问算法。
 * 这是一个适配器，扩展的一条 i2c 总线 
 * @owner:  设备的模块所有者
 * @class:  该适配器总线上，可以通过探测实例化的设备类型，参考 i2c_detect
 * @algo:   访问总线的方法
 * @algo_data:  algo私有数据，由适配器驱动程序使用，i2c核心层不使用
 * @lock_ops:   i2c总线的锁操作，由适配驱动 或 使用默认的锁操作
 * @bus_lock:   默认的总线锁操作处理使用的互斥锁
 * @mux_lock:   i2c mux 模块使用的互斥锁
 * @timeout:    总线访问超时机制，由适配器驱动程序提供，或默认1s
 * @retries:    总线访问重传次数，由适配器驱动程序提供
 * @dev:        设备驱动模型的设备结构
 * @locked_flags:   总线挂起标志
 * @nr:         i2c 总线号, 与应用层设备节点有关
 * @name:       i2c 适配器的名字，必须设置
 * @dev_released:   i2c适配器释放完成通知机制
 * @userspace_clients_lock: 在应用空间sysfs创建的从设备链表保护锁
 * @userspace_clients:  在应用空间sysfs创建的从设备链表
 * @bus_recovery_info:  总线恢复信息
 * @quirks:     总线能力, 声明总线的传输能力
 * @host_notify_domain: 主机通知中断域
 * @bus_regulator:  电源相关，但好像没用，或者内核内部使用？？？
 * @debugfs:    该i2c总线/适配器的 debugfs 根目录
 * @addrs_in_instantiation: 位图，7bit地址的每个地址对应一个bit，
 *              用于在使用某个从设备地址时，锁定该地址 
 *
 * 使用 i2c_adapter 定义一个i2c适配器，也就是一条总线，
 * i2c从设备注册时，需要指定所属的i2c适配器
 *
 * 本结构由总线驱动程序使用
 * */
struct i2c_adapter {
	struct module *owner;
	unsigned int class;		  /* classes to allow probing for */
	const struct i2c_algorithm *algo; /* the algorithm to access the bus */
	void *algo_data;

	/* data fields that are valid for all devices	*/
	const struct i2c_lock_operations *lock_ops;
	struct rt_mutex bus_lock;
	struct rt_mutex mux_lock;

	int timeout;			/* in jiffies */
	int retries;
	struct device dev;		/* the adapter device */
    /* 适配器挂起状态标记
     * 使用 I2C_ALF_IS_SUSPENDED 标记是否为挂起状态
     * 使用 I2C_ALF_SUSPEND_REPORTED 控制第一次调用时报告总线已挂起 */
	unsigned long locked_flags;	/* owned by the I2C core */
#define I2C_ALF_IS_SUSPENDED		0
#define I2C_ALF_SUSPEND_REPORTED	1

	int nr;
	char name[48];
	struct completion dev_released;

	struct mutex userspace_clients_lock;
	struct list_head userspace_clients;

	struct i2c_bus_recovery_info *bus_recovery_info;
	const struct i2c_adapter_quirks *quirks;

	struct irq_domain *host_notify_domain;
	struct regulator *bus_regulator;

	struct dentry *debugfs;

	/* 7bit address space */
	DECLARE_BITMAP(addrs_in_instantiation, 1 << 7);
};
/* 通过 设备模型设备结构 找到所属的 i2c 适配器结构 */
#define to_i2c_adapter(d) container_of(d, struct i2c_adapter, dev)

/* 获取 i2c adapter 的私有数据 */
static inline void *i2c_get_adapdata(const struct i2c_adapter *adap)
{
	return dev_get_drvdata(&adap->dev);
}

/* 设置 i2c adapter 的私有数据 */
static inline void i2c_set_adapdata(struct i2c_adapter *adap, void *data)
{
	dev_set_drvdata(&adap->dev, data);
}

/* 检查 i2c adapter 适配器的父设备是否为 i2c adapter 适配器
 * 这种情况出现在系统使用 i2c_mux 时, i2c_mux 做为 i2c 的从设备
 * 但又会注册多个 i2c adapter, 出现 i2c adapter 级联 */
/* 注意, 如果父设备是 i2c adapter, 会返回其指针 */
static inline struct i2c_adapter *
i2c_parent_is_i2c_adapter(const struct i2c_adapter *adapter)
{
    /* 只有启动了 I2C_MUX 才会出现父设备为 i2c adapter 的情况 */
#if IS_ENABLED(CONFIG_I2C_MUX)
	struct device *parent = adapter->dev.parent;

    /* 检查父设备的设备类型, 若为 i2c adapter, 则返回其指针 */
	if (parent != NULL && parent->type == &i2c_adapter_type)
		return to_i2c_adapter(parent);
	else
#endif
		return NULL;
}

/* 遍历 i2c bus 总线的设备，使用 fn 处理 */
int i2c_for_each_dev(void *data, int (*fn)(struct device *dev, void *data));

/* Adapter locking functions, exported for shared pin cases */
/* 适配器锁定功能，指示锁定I2C总线时 的锁定级别 */
#define I2C_LOCK_ROOT_ADAPTER BIT(0)
#define I2C_LOCK_SEGMENT      BIT(1)

/**
 * i2c_lock_bus - Get exclusive access to an I2C bus segment
 * @adapter: Target I2C bus segment
 * @flags: I2C_LOCK_ROOT_ADAPTER locks the root i2c adapter, I2C_LOCK_SEGMENT
 *	locks only this branch in the adapter tree
 */
/* i2c_lock_bus - 获得对I2C总线的独占访问权
 * @adapter:    目标 I2C总线段 
 * @flags:      I2C_LOCK_ROOT_ADAPTER 锁定整个根，I2C_LOCK_SEGMENT 锁定分支
 * */
static inline void
i2c_lock_bus(struct i2c_adapter *adapter, unsigned int flags)
{
	adapter->lock_ops->lock_bus(adapter, flags);
}

/**
 * i2c_trylock_bus - Try to get exclusive access to an I2C bus segment
 * @adapter: Target I2C bus segment
 * @flags: I2C_LOCK_ROOT_ADAPTER tries to locks the root i2c adapter,
 *	I2C_LOCK_SEGMENT tries to lock only this branch in the adapter tree
 *
 * Return: true if the I2C bus segment is locked, false otherwise
 */
/* i2c_lock_bus - 获得对I2C总线的独占访问权,不会阻塞
 * @adapter:    目标 I2C总线段 
 * @flags:      I2C_LOCK_ROOT_ADAPTER 锁定整个根，I2C_LOCK_SEGMENT 锁定分支
 * */
static inline int
i2c_trylock_bus(struct i2c_adapter *adapter, unsigned int flags)
{
	return adapter->lock_ops->trylock_bus(adapter, flags);
}

/**
 * i2c_unlock_bus - Release exclusive access to an I2C bus segment
 * @adapter: Target I2C bus segment
 * @flags: I2C_LOCK_ROOT_ADAPTER unlocks the root i2c adapter, I2C_LOCK_SEGMENT
 *	unlocks only this branch in the adapter tree
 */
/* i2c_lock_bus - 释放对I2C总线的独占访问权
 * @adapter:    目标 I2C总线段 
 * @flags:      I2C_LOCK_ROOT_ADAPTER 锁定整个根，I2C_LOCK_SEGMENT 锁定分支
 * */
static inline void
i2c_unlock_bus(struct i2c_adapter *adapter, unsigned int flags)
{
	adapter->lock_ops->unlock_bus(adapter, flags);
}

/**
 * i2c_mark_adapter_suspended - Report suspended state of the adapter to the core
 * @adap: Adapter to mark as suspended
 *
 * When using this helper to mark an adapter as suspended, the core will reject
 * further transfers to this adapter. The usage of this helper is optional but
 * recommended for devices having distinct handlers for system suspend and
 * runtime suspend. More complex devices are free to implement custom solutions
 * to reject transfers when suspended.
 */
/* i2c_mark_adapter_suspended - 向核心报告适配器处于挂起状态
 * @adap:   标记为挂起状态的适配器
 *
 * 当使用此辅助函数将适配器标记为挂起时，核心将拒绝进一步传输到此适配器。这个辅
 * 助函数的使用是可选的，但对于具有不同的系统挂起和运行时挂起处理程序的设备，建
 * 议使用。更复杂的设备可以自由实现自定义解决方案，在挂起时拒绝传输。
 *
 * 本函数与 i2c_mark_adapter_resumed 函数，适用于各器件的适配器驱动中，当适配需
 * 要挂起/恢复时，使用这两个函数进行标记
 * */
static inline void i2c_mark_adapter_suspended(struct i2c_adapter *adap)
{
	i2c_lock_bus(adap, I2C_LOCK_ROOT_ADAPTER);
	set_bit(I2C_ALF_IS_SUSPENDED, &adap->locked_flags);
	i2c_unlock_bus(adap, I2C_LOCK_ROOT_ADAPTER);
}

/**
 * i2c_mark_adapter_resumed - Report resumed state of the adapter to the core
 * @adap: Adapter to mark as resumed
 *
 * When using this helper to mark an adapter as resumed, the core will allow
 * further transfers to this adapter. See also further notes to
 * @i2c_mark_adapter_suspended().
 */
/* i2c_mark_adapter_resumed - 向核心报告适配器处于恢复(正常)状态
 * @adap:   标记为挂起状态的适配器
 *
 * 当使用这个辅助器将适配器标记为恢复时，核心将允许进一步传输到这个适配器。请参
 * 阅@i2c_mark_adapter_suspended（）的进一步说明。
 * */
static inline void i2c_mark_adapter_resumed(struct i2c_adapter *adap)
{
	i2c_lock_bus(adap, I2C_LOCK_ROOT_ADAPTER);
	clear_bit(I2C_ALF_IS_SUSPENDED, &adap->locked_flags);
	i2c_unlock_bus(adap, I2C_LOCK_ROOT_ADAPTER);
}

/* i2c adapter classes (bitmask) */
/* 适配器 class, 用于 驱动探测设备时 */
#define I2C_CLASS_HWMON		(1<<0)	/* lm_sensors, ... */
#define I2C_CLASS_DDC		(1<<3)	/* DDC bus on graphics adapters */
#define I2C_CLASS_SPD		(1<<7)	/* Memory modules */
/* Warn users that the adapter doesn't support classes anymore */
/* 警告用户 适配器 不支持 class */
#define I2C_CLASS_DEPRECATED	(1<<8)

/* Internal numbers to terminate lists */
/* 用于具有探测功能的驱动程序中
 * 提供探测地址列表 address_list 时，做为地址结束标志 */
#define I2C_CLIENT_END		0xfffeU

/* Construct an I2C_CLIENT_END-terminated array of i2c addresses */
/* 定义一个 i2c 从地址数组列表，使用 I2C_CLIENT_END 结尾
 * 可以用驱动的探测地址列表 address_list */
#define I2C_ADDRS(addr, addrs...) \
	((const unsigned short []){ addr, ## addrs, I2C_CLIENT_END })


/* ----- functions exported by i2c.o */

/* administration...
 */
#if IS_ENABLED(CONFIG_I2C)
/* i2c_add_adapter - 使用动态总线号，声明 i2c 适配器  */
int i2c_add_adapter(struct i2c_adapter *adap);
/* devm_i2c_add_adapter - devm 版本的 i2c_add_adapter() */
int devm_i2c_add_adapter(struct device *dev, struct i2c_adapter *adapter);
/* i2c_del_adapter - 注销 i2c adapter */
void i2c_del_adapter(struct i2c_adapter *adap);
/* 添加一个 i2c adapter, 使用固定分配的 bus 号 */
int i2c_add_numbered_adapter(struct i2c_adapter *adap);

/* 注册一个 i2c_driver */
int i2c_register_driver(struct module *owner, struct i2c_driver *driver);
/* 注销 i2c 驱动 */
void i2c_del_driver(struct i2c_driver *driver);

/* use a define to avoid include chaining to get THIS_MODULE */
/* 驱动程序推荐使用的 i2c_driver 注册接口 */
#define i2c_add_driver(driver) \
	i2c_register_driver(THIS_MODULE, driver)

/* 检测 i2c clinet 是否已绑定驱动 */
static inline bool i2c_client_has_driver(struct i2c_client *client)
{
	return !IS_ERR_OR_NULL(client) && client->dev.driver;
}

/* call the i2c_client->command() of all attached clients with
 * the given arguments */
/* 使用给定的参数，调用所有连接的客户端的 i2c_client->command() 回调 */
void i2c_clients_command(struct i2c_adapter *adap,
			 unsigned int cmd, void *arg);

/* 使用i2c适配器的总线号 获取 i2c_adapter 结构 */
struct i2c_adapter *i2c_get_adapter(int nr);
/* 释放对 i2c_adapter 的引用 */
void i2c_put_adapter(struct i2c_adapter *adap);
/* 获取 i2c adapter 的深度
 * 适用于具有 i2c mux 的情况, 适配器下的设备可能还是适配器 */
unsigned int i2c_adapter_depth(struct i2c_adapter *adapter);

/* i2c_parse_fw_timings - 从固件中获取 i2c 相关的时序参数 */
void i2c_parse_fw_timings(struct device *dev, struct i2c_timings *t, bool use_defaults);

/* Return the functionality mask */
/* 返回 i2c 适配器的支持的功能掩码 */
static inline u32 i2c_get_functionality(struct i2c_adapter *adap)
{
	return adap->algo->functionality(adap);
}

/* Return 1 if adapter supports everything we need, 0 if not. */
/* 检测 i2c 适配器是否支持某功能 */
static inline int i2c_check_functionality(struct i2c_adapter *adap, u32 func)
{
	return (func & i2c_get_functionality(adap)) == func;
}

/**
 * i2c_check_quirks() - Function for checking the quirk flags in an i2c adapter
 * @adap: i2c adapter
 * @quirks: quirk flags
 *
 * Return: true if the adapter has all the specified quirk flags, false if not
 */
/* i2c_check_quirks() - 检测 i2c适配器的缺陷标志
 * @adap:   i2c 适配器
 * @quirks: 缺陷标志
 *
 * 如果适配器具有指定的缺陷标志，则返回 ture, 否则返回 false */
static inline bool i2c_check_quirks(struct i2c_adapter *adap, u64 quirks)
{
	if (!adap->quirks)
		return false;
	return (adap->quirks->flags & quirks) == quirks;
}

/* Return the adapter number for a specific adapter */
/* 获取 i2c adapter 适配器的编号 */
static inline int i2c_adapter_id(struct i2c_adapter *adap)
{
	return adap->nr;
}

/* 从 i2c_msg 中，获取 i2c 的 8bit 从机地址(增加了读写位) */
static inline u8 i2c_8bit_addr_from_msg(const struct i2c_msg *msg)
{
	return (msg->addr << 1) | (msg->flags & I2C_M_RD ? 1 : 0);
}

/* i2c_get_dma_safe_msg_buf - 为给定的 i2c_msg 获取一个DMA安全的缓冲区 */
u8 *i2c_get_dma_safe_msg_buf(struct i2c_msg *msg, unsigned int threshold);
/* i2c_put_dma_safe_msg_buf - 释放DMA安全缓冲区并于i2c_msg同步 */
void i2c_put_dma_safe_msg_buf(u8 *buf, struct i2c_msg *msg, bool xferred);

/* i2c_handle_smbus_host_notify - 将主机通知事件转发给正确的用户 */
int i2c_handle_smbus_host_notify(struct i2c_adapter *adap, unsigned short addr);

/**
 * module_i2c_driver() - Helper macro for registering a modular I2C driver
 * @__i2c_driver: i2c_driver struct
 *
 * Helper macro for I2C drivers which do not do anything special in module
 * init/exit. This eliminates a lot of boilerplate. Each module may only
 * use this macro once, and calling it replaces module_init() and module_exit()
 */
/* module_i2c_driver() - 用于注册模块I2C驱动程序的辅助宏
 * @__i2c_driver:   i2c_driver 结构体 
 *
 * I2C驱动程序的辅助宏，在模块init/exit中不做任何特殊处理。这消除了很多重复的模
 * 块代码。每个模块只能使用此宏一次，调用它将替换module_init（）和module_exit（）
 * */
#define module_i2c_driver(__i2c_driver) \
	module_driver(__i2c_driver, i2c_add_driver, \
			i2c_del_driver)

/**
 * builtin_i2c_driver() - Helper macro for registering a builtin I2C driver
 * @__i2c_driver: i2c_driver struct
 *
 * Helper macro for I2C drivers which do not do anything special in their
 * init. This eliminates a lot of boilerplate. Each driver may only
 * use this macro once, and calling it replaces device_initcall().
 */
/* builtin_i2c_driver() - 用于注册内置I2C驱动程序的辅助宏
 * @__i2c_driver:   i2c_driver 结构体 
 *
 * I2C驱动程序的辅助宏，在其init中不做任何特殊处理。这消除了很多重复的模块代码。
 * 每个驱动程序只能使用这个宏一次，调用它将取代device_initcall（）。
 * */
#define builtin_i2c_driver(__i2c_driver) \
	builtin_driver(__i2c_driver, i2c_add_driver)

#endif /* I2C */

/* must call put_device() when done with returned i2c_client device */
/* i2c_find_device_by_fwnode() - 使用 fwnode 查找所属的 i2c_client */
struct i2c_client *i2c_find_device_by_fwnode(struct fwnode_handle *fwnode);

/* must call put_device() when done with returned i2c_adapter device */
/* i2c_find_adapter_by_fwnode() - 使用 fwnode 查找所属的适配器 */
/* 只增加设备的引用, 使用完后必须使用 put_device() 释放 */
struct i2c_adapter *i2c_find_adapter_by_fwnode(struct fwnode_handle *fwnode);

/* must call i2c_put_adapter() when done with returned i2c_adapter device */
/* i2c_get_adapter_by_fwnode() - 使用 fwnode 查找所属的适配器 */
/* 多增加对 module 的引用, 使用完必须使用 i2c_put_adapter() 释放 */
struct i2c_adapter *i2c_get_adapter_by_fwnode(struct fwnode_handle *fwnode);

#if IS_ENABLED(CONFIG_OF)
/* must call put_device() when done with returned i2c_client device */
/* 使用设备树节点 node 查找所属的 i2c_client 从设备 */
static inline struct i2c_client *of_find_i2c_device_by_node(struct device_node *node)
{
	return i2c_find_device_by_fwnode(of_fwnode_handle(node));
}

/* must call put_device() when done with returned i2c_adapter device */
/* 使用设备树节点 node 查找所属的 i2c_adapter 适配器 */
/* 只增加设备的引用, 使用完后必须使用 put_device() 释放 */
static inline struct i2c_adapter *of_find_i2c_adapter_by_node(struct device_node *node)
{
	return i2c_find_adapter_by_fwnode(of_fwnode_handle(node));
}

/* must call i2c_put_adapter() when done with returned i2c_adapter device */
/* 使用设备树节点 node 查找所属的 i2c_adapter 适配器 */
/* 多增加对 module 的引用, 使用完必须使用 i2c_put_adapter() 释放 */
static inline struct i2c_adapter *of_get_i2c_adapter_by_node(struct device_node *node)
{
	return i2c_get_adapter_by_fwnode(of_fwnode_handle(node));
}

/* i2c_client 尝试匹配 of_device_id  */
const struct of_device_id
*i2c_of_match_device(const struct of_device_id *matches,
		     struct i2c_client *client);

/* 将 i2c 从设备节点转换为 i2c_board_info 结构中的信息 */
int of_i2c_get_board_info(struct device *dev, struct device_node *node,
			  struct i2c_board_info *info);

#else
/* 不支持设备树的内核，设备树相关的空实现 */

static inline struct i2c_client *of_find_i2c_device_by_node(struct device_node *node)
{
	return NULL;
}

static inline struct i2c_adapter *of_find_i2c_adapter_by_node(struct device_node *node)
{
	return NULL;
}

static inline struct i2c_adapter *of_get_i2c_adapter_by_node(struct device_node *node)
{
	return NULL;
}

static inline const struct of_device_id
*i2c_of_match_device(const struct of_device_id *matches,
		     struct i2c_client *client)
{
	return NULL;
}

static inline int of_i2c_get_board_info(struct device *dev,
					struct device_node *node,
					struct i2c_board_info *info)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_OF */

/* acpi 相关， arm 不支持 */
struct acpi_resource;
struct acpi_resource_i2c_serialbus;

#if IS_REACHABLE(CONFIG_ACPI) && IS_REACHABLE(CONFIG_I2C)
bool i2c_acpi_get_i2c_resource(struct acpi_resource *ares,
			       struct acpi_resource_i2c_serialbus **i2c);
int i2c_acpi_client_count(struct acpi_device *adev);
u32 i2c_acpi_find_bus_speed(struct device *dev);
struct i2c_client *i2c_acpi_new_device(struct device *dev, int index,
				       struct i2c_board_info *info);
struct i2c_adapter *i2c_acpi_find_adapter_by_handle(acpi_handle handle);
#else
static inline bool i2c_acpi_get_i2c_resource(struct acpi_resource *ares,
					     struct acpi_resource_i2c_serialbus **i2c)
{
	return false;
}
static inline int i2c_acpi_client_count(struct acpi_device *adev)
{
	return 0;
}
static inline u32 i2c_acpi_find_bus_speed(struct device *dev)
{
	return 0;
}
static inline struct i2c_client *i2c_acpi_new_device(struct device *dev,
					int index, struct i2c_board_info *info)
{
	return ERR_PTR(-ENODEV);
}
static inline struct i2c_adapter *i2c_acpi_find_adapter_by_handle(acpi_handle handle)
{
	return NULL;
}
#endif /* CONFIG_ACPI */

#endif /* _LINUX_I2C_H */
