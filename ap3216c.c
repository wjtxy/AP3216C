#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

/* 111 : all functions once */
#define SystemConfiguration 0x00   
#define INTStatus 0x01
#define ALSConfig 0x10
#define PSConfig 0x20
/* interrupt threshold for PS */
#define PSLThresholdL 0x2A
#define PSLThresholdH 0x2B
#define PSHThresholdL 0x2C
#define PSHThresholdH 0x2D
#define PSIntMode 0x22

#define IRDataLow 0x0A
#define IRDataHigh 0x0B
/* Ambient Light(lux) = data * 0.35 lux/count(default) */
#define ALSDataLow 0x0C
#define ALSDataHigh 0x0D
#define PSDataLow 0x0E
#define PSDataHigh 0x0F


/* cmd */
#define AllFucOnce (1 | (1 << 1) | (1 << 2))
#define AllFunc (1 << 1 | 1 )
#define Reset 1 << 2
/* Hysteresis type (default) */
#define PSIntMode2 1 

struct ap_cmd_desc {
    struct sys_cmd_desc {
        u8 all_func_once;
        u8 reset;
        u8 all_func;
    }sys_cmd;
    struct ps_cmd_desc {
        u8 ps_int_mode2;
    }ps_cmd;
};

static struct ap_cmd_desc ap_cmd = {
    .sys_cmd = {
        .all_func_once = AllFucOnce,
        .reset = Reset,
        .all_func = AllFunc,
    },
    .ps_cmd = {
        .ps_int_mode2 = PSIntMode2,
    }
};
struct ap3216c_desc {
    char name[64];
    struct i2c_client *client;
    struct iio_dev *iio_dev;
    int irq;
    int gpio;
    unsigned long irq_flag;
};
static irqreturn_t irq_handler(int irq, void *data)
{
    dev_info(NULL,"----%s",__func__);

    return IRQ_HANDLED;
}
static void ap3216c_init(struct i2c_client *client)
{
    int ret;
    ret = i2c_smbus_write_i2c_block_data(client, SystemConfiguration, 1, &ap_cmd.sys_cmd.reset);
    if(ret){
        dev_err(&client->dev, "-----ap_cmd.sys_cmd.reset err");
        return ;
    }
    msleep(15);
    ret = i2c_smbus_write_i2c_block_data(client, SystemConfiguration, 1, &ap_cmd.sys_cmd.all_func);
    if(ret){
        dev_err(&client->dev, "-----ap_cmd.sys_cmd.all_func err");
        return ;
    }
}
static struct iio_chan_spec ap_channel[] = {
    {
        .type = IIO_PROXIMITY,
        /*.channel = 0,*/
        /*.indexed = 1,*/
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
    }
};
static int ap_read_ps(struct i2c_client *client)
{
    int ret;
    u8 tmp;
    int val;
    ret = i2c_smbus_read_i2c_block_data(client, PSDataLow, 1, &tmp);
    if(ret < 0){
        dev_err(&client->dev, "-----devm_iio_device_alloc err");
        return -EBUSY;
    }
    if(tmp & (1 << 6))
        dev_err(&client->dev, "-----pa dataL read err");
    val = tmp && 0x0F;
    ret = i2c_smbus_read_i2c_block_data(client, PSDataHigh, 1, &tmp);
    if(ret < 0){
        dev_err(&client->dev, "-----devm_iio_device_alloc err");
        return -EBUSY;
    }
    if(tmp & (1 << 6))
        dev_err(&client->dev, "-----pa dataH read err");
    val |= (tmp & 0x3F) << 4;
    return val;
}
static int ap_read_raw(struct iio_dev *indio_dev,struct iio_chan_spec const *chan,int *val,int *val2,long mask)
{
    struct i2c_client *client = iio_device_get_drvdata(indio_dev);
    switch(mask){
    case IIO_CHAN_INFO_RAW:
        *val = ap_read_ps(client);
        return IIO_VAL_INT;
    }
    return 0;
}
static struct iio_info ap_info = {
    .driver_module = THIS_MODULE,
    .read_raw = ap_read_raw
};

static int ap_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    struct iio_dev *iio_dev;
    struct ap3216c_desc *ap;
    struct device_node *np = client->dev.of_node;
    int ret;

    iio_dev = devm_iio_device_alloc(&client->dev, sizeof(*ap));
    if(!iio_dev){
        dev_err(&client->dev, "-----devm_iio_device_alloc err");
        return -ENOMEM;
    }
    ap = iio_priv(iio_dev);
    ap->client = client;
    ap->iio_dev = iio_dev;

    i2c_set_clientdata(client, ap);
    iio_device_set_drvdata(iio_dev, client);

    ap->gpio = of_get_gpio(np, 0);
    if(!gpio_is_valid(ap->gpio)){
        dev_err(&client->dev, "-----of_get_gpio err");
        return -ENODEV;
    }
    /*dev_info(&client->dev, "----gpio:%d", ap->gpio);*/
    ap->irq = gpio_to_irq(ap->gpio);
    /*ap->irq_flag = of_get_gpio_flags(struct device_node *np, int index, )*/
    ret = devm_request_irq(&client->dev, ap->irq, irq_handler, IRQF_TRIGGER_FALLING, "ap3216c_irq", ap);
    if(ret){
        dev_err(&client->dev, "-----devm_request_irq err");
        return -ENODEV;
    }
    /*dev_info(&client->dev, "----irq:%d", ap->irq);*/

    ap3216c_init(client);

    iio_dev->modes = INDIO_DIRECT_MODE;
    iio_dev->channels = ap_channel;
    iio_dev->info = &ap_info;
    iio_dev->dev.parent = &client->dev;
    iio_dev->num_channels = ARRAY_SIZE(ap_channel);
    iio_dev->name = "ap3216c";

    dev_info(&client->dev,"----%s",__func__);
    ret = devm_iio_device_register(&client->dev, iio_dev);
    if(ret){
        dev_err(&client->dev, "----iio_device_register err");
        return -EBUSY;
    }
    return 0;
}
static int ap_remove(struct i2c_client *client)
{
    return 0;
}
static const struct i2c_device_id mpu_id[] = {
    {"not use", 1},
    {}
};
static const struct of_device_id mpu_match_table[] = {
    {.compatible = "wjt,ap3216c"},
    {}
};
static struct i2c_driver ap_driver = {
    .probe = ap_probe,
    .remove = ap_remove,
    .id_table = mpu_id,
    .driver = {
        .owner = THIS_MODULE,
        .name = "ap3216c",
        /* this table is truely to use match the dtb */
        .of_match_table = mpu_match_table
    },
};
module_i2c_driver(ap_driver);
MODULE_LICENSE("GPL");

