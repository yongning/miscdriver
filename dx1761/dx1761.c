#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/of.h>

#define OFFSET_FENCE 0xff
#define STS_offset1 0x00
#define STS_offset2 0x01
#define CTL_offset1 0x04
#define CTL_offset2 0x05
#define CTL_offset3 0x06
#define CONF_offset1 0x08
#define CONF_offset2 0x09
#define CONF_offset3 0x0a
#define OUTMOD_offset1 0x0c
#define OUTMOD_offset2 0x0d
#define OUTMOD_offset3 0x0e
#define OUTCLR_offset1 0x10
#define OUTCLR_offset2 0x11
#define OUTCLR_offset3 0x12
#define OUTSET_offset1 0x14
#define OUTSET_offset2 0x15
#define OUTSET_offset3 0x16
#define INT_TRIGEDGE_offset 0x20
#define INT_TRIGBOTH_offset 0x24
#define INT_ENABLE_offset 0x28
#define INT_STATUS_offset 0x2c

#define DX1761_GPIO_DIR_OUT 0
#define DX1761_GPIO_DIR_IN BIT(0)
#define GPIO_NUM 24
enum {
    DX1761,
};

static const struct i2c_device_id dx1761_id[] = {
	{"dx1761", DX1761 },
	{},
};
MODULE_DEVICE_TABLE(i2c, dx1761_id);

#ifdef CONFIG_OF
static const struct of_device_id dx1761_of_table[] = {
    {.compatible = "devexel,dx1761" },
    {}
};
MODULE_DEVICE_TABLE(of, dx1761_of_table);
#endif

struct dx1761_chip {
    struct gpio_chip gpio_chip;

    struct i2c_client* client;

    unsigned int dir_input;
    unsigned int dir_output;

    struct mutex lock;
  //  unit8_t reg_out[2];
};

static int dx1761_writeb(struct dx1761_chip* chip, uint8_t offset, uint8_t val)
{
    struct i2c_client* client;
    int ret;

    client = chip->client;
    ret = i2c_smbus_write_byte_data(client, offset, val);
    if (ret < 0) {
        dev_err(&client->dev, "failed to write to i2c device\n");
        return ret;
    }
    
    return 0;    
}

static int dx1761_readb(struct dx1761_chip* chip, uint8_t offset, uint8_t* val)
{
    struct i2c_client* client;
    int ret;

    client = chip->client;
    ret = i2c_smbus_read_byte_data(client, offset);
    if (ret < 0) {
        dev_err(&client->dev, "failed to read to i2c device\n");
	return ret;
    }
    
    *val = ret;
    return 0;
}

static int dx1761_gpio_get_direction(struct gpio_chip* gc, unsigned int off)
{
    uint8_t data, data1;
    uint8_t command;
    int ret;
    struct dx1761_chip* chip = gpiochip_get_data(gc);

    if ((off >= 0) && (off <= 7)) {
        command = CONF_offset1;
	data1 = 1 << off;
    } else if ((off >= 8) && (off <=15)) {
	command = CONF_offset2;
	data1 = 1 << (off - 8);
    } else if ((off >=16) && (off <= 23)) {
	return DX1761_GPIO_DIR_OUT;
    } else {
	return -1;
    }

    mutex_lock(&chip->lock);
    ret = dx1761_readb(chip, command, &data);
    if (ret < 0) {
        mutex_unlock(&chip->lock);
	return -1;
    }
    mutex_unlock(&chip->lock);
    return ((data1 & data) ? DX1761_GPIO_DIR_OUT : DX1761_GPIO_DIR_IN);

}

static int dx1761_gpio_direction_output(struct gpio_chip* gc, unsigned off, int val)
{
    
    uint8_t command1, command2;
    uint8_t data, data2;
    int ret;
    uint8_t tmp;

    struct dx1761_chip* chip = gpiochip_get_data(gc);

    if ((off >= 0) && (off <= 7)) {
	command1 = CONF_offset1;
	tmp = 1 << off;
        if (val == 0) {
	    command2 = OUTCLR_offset1;
	} else if (val == 1) {
	    command2 = OUTSET_offset1;
	} else {
            return -1;
	}
	data2 = 1 << off;
    } else if ((off >= 8) && (off <=15)) {
	command1 = CONF_offset2;
	tmp = 1 << (off - 8);
        if (val == 0) {
	    command2 = OUTCLR_offset2;
	} else if (val == 1) {
	    command2 = OUTSET_offset2;
	} else {
	    return -1;
	}
	data2 = 1 << (off - 8);
    } else if ((off >=16) && (off <=23)) {
	command1 = CONF_offset3;
	tmp = 1 << (off - 16);
	if (val == 0) {
	    command2 = OUTCLR_offset3;
	} else if (val == 1) {
	    command2 = OUTSET_offset3;
	} else {
	    return -1;
	}
	data2 = 1 << (off - 16);
    } else {   
        return -1;
    }
    
    mutex_lock(&chip->lock);
    ret = dx1761_readb(chip, command1, &data);
    if (ret < 0) {
        mutex_unlock(&chip->lock);
        return ret;
    }
    data = data | tmp;
    ret = dx1761_writeb(chip, command1, data);
    if (ret < 0) {
        mutex_unlock(&chip->lock);
	return ret;
    }

    ret = dx1761_writeb(chip, command2, data2);
    mutex_unlock(&chip->lock);

    return ret;
}

static int dx1761_gpio_direction_input(struct gpio_chip* gc, unsigned off)
{
    uint8_t command;
    uint8_t data;
    uint8_t tmp;
    int ret;

    struct dx1761_chip* chip = gpiochip_get_data(gc);

    if ((off >= 0) && (off <= 7)) {
	    command = CONF_offset1;
	    tmp = (1 << off) ^ 0xff;
    } else if ((off >= 8) && (off <=15)) {
	    command = CONF_offset2;
	    tmp = (1 << (off - 8)) ^ 0xff;
    } else {
	return -1;
    }
    
    mutex_lock(&chip->lock);
    ret = dx1761_readb(chip, command, &data);
    if (ret < 0) {
        mutex_unlock(&chip->lock);
	return ret;
    }
    data = data & tmp;
    ret = dx1761_writeb(chip, command, data);
    if (ret < 0) {
        mutex_unlock(&chip->lock);
	return ret;
    }
    mutex_unlock(&chip->lock);

    return ret;
}	

static int dx1761_gpio_get_value(struct gpio_chip* gc, unsigned off)
{
    uint8_t command, command1, command2;
    uint8_t data, data1, data2;
    int ret;
    struct dx1761_chip* chip = gpiochip_get_data(gc);

    if ((off >= 0) && (off <= 7)) {
	    // command = STS_offset1;
	    data1 = 1 << off;
	    command = CONF_offset1;
	    command1 = STS_offset1;
	    command2 = CTL_offset1;
    } else if ((off >= 8) && (off <=15)) {
	    // command = STS_offset2;
	    data1 = 1 << (off - 8);
	    command = CONF_offset2;
	    command1 = STS_offset2;
	    command2 = CTL_offset2;
    } else {
        data1 = 1 << (off - 16);
        command = OFFSET_FENCE;
    }

    mutex_lock(&chip->lock);
    if (command != OFFSET_FENCE) {
        ret = dx1761_readb(chip, command, &data);
        if (data & data1) {
            ret = dx1761_readb(chip, command2, &data2);
        } else {
            ret = dx1761_readb(chip, command1, &data2);
        }
    } else {
        ret = dx1761_readb(chip, command2, &data2);
    }
    mutex_unlock(&chip->lock);
    if (ret < 0) {
        return ret;
    } else {
	return !!(data2 & data1);
    }
}

static void dx1761_gpio_set_value(struct gpio_chip* gc, unsigned off, int val)
{
    uint8_t command;
    uint8_t data;
    struct dx1761_chip *chip = gpiochip_get_data(gc);
     
    if ((off >= 0) && (off <= 7)) {
        if (val == 0) {
	    command = OUTCLR_offset1;
	} else if (val == 1) {
	    command = OUTSET_offset1;
	} else {
	    return;
        }
	data = (1 << off) & 0xff;
    } else if ((off >= 8) && (off <=15)) {
        if (val == 0) {
	    command = OUTCLR_offset2;
	} else if (val == 1){
	    command = OUTSET_offset2;
	} else {
            return;
	}
	data = (1 << (off - 8)) & 0xff;
    } else if ((off >= 16) && (off <=23)) {
        if (val == 0) {
	    command = OUTCLR_offset3;
	} else if (val == 1) {
	    command = OUTSET_offset3;
	} else {
	    return;
	}
	data = (1 << (off - 16)) & 0xff;
    } else {
	return;
    }

    mutex_lock(&chip->lock);
    dx1761_writeb(chip, command, data);
    mutex_unlock(&chip->lock);
    
}

static int dx1761_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
    struct dx1761_chip* dxchip;

    dxchip = devm_kzalloc(&client->dev, sizeof(*dxchip), GFP_KERNEL);
    if (!dxchip)
	return -ENOMEM;

    dxchip->gpio_chip.direction_input = dx1761_gpio_direction_input;
    dxchip->gpio_chip.direction_output = dx1761_gpio_direction_output;
    dxchip->gpio_chip.get_direction = dx1761_gpio_get_direction;
    dxchip->gpio_chip.set = dx1761_gpio_set_value;
    dxchip->gpio_chip.get = dx1761_gpio_get_value;
    dxchip->gpio_chip.ngpio = GPIO_NUM;
    dxchip->gpio_chip.label = client->name;
    dxchip->gpio_chip.parent = &client->dev;
    dxchip->client = client;
    dxchip->gpio_chip.can_sleep = true;
    dxchip->gpio_chip.owner = THIS_MODULE;
    i2c_set_clientdata(client, dxchip);

    return devm_gpiochip_add_data(&client->dev, &dxchip->gpio_chip, dxchip);

}

static int dx1761_remove(struct i2c_client* client)
{
    struct dx1761_chip* dxchip;
    
    dxchip = i2c_get_clientdata(client);
    
    gpiochip_remove(&dxchip->gpio_chip);
    
    return 0;
}

static struct i2c_driver dx1761_i2c_driver = {
    .driver = {
	.name = "dx1761",
	.of_match_table = of_match_ptr(dx1761_of_table),
    },
    .probe = dx1761_probe,
    .remove = dx1761_remove,
    .id_table = dx1761_id,
};

static int __init dx1761_init(void)
{
    return i2c_add_driver(&dx1761_i2c_driver);
}
subsys_initcall(dx1761_init);

static void __exit dx1761_exit(void)
{
    i2c_del_driver(&dx1761_i2c_driver);
}
module_exit(dx1761_exit);

MODULE_AUTHOR("aa@aa.com");
MODULE_DESCRIPTION("i2c GPIO expander for 1761");
MODULE_VERSION("0.8");
MODULE_LICENSE("GPL");




