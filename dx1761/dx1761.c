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
#define INT_TRIGEDGE_offset2 0x21
#define INT_TRIGBOTH_offset 0x24
#define INT_TRIGBOTH_offset2 0x25
#define INT_ENABLE_offset 0x28
#define INT_ENABLE_offset2 0x29
#define INT_STATUS_offset 0x2c
#define INT_STATUS_offset2 0x2d

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
    
    struct mutex lock;
    uint8_t dir[2];
    //  unit8_t reg_out[2];

    struct mutex irq_lock;
    uint8_t irq_mask[2];
    uint8_t irq_edge[2];
    uint8_t irq_edgeboth[2];
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

static void dx1761_irq_mask(struct irq_data* data)
{
    uint8_t irq_mask_reg, irq_mask_val;
    struct gpio_chip* gc = irq_data_get_irq_chip_data(data);
    struct dx1761_chip* chip = gpiochip_get_data(gc);

    if (data->hwirq <8) {
        irq_mask_reg = INT_ENABLE_offset;
        irq_mask_val = chip->irq_mask[0] & (~(1 << data->hwirq));
        chip->irq_mask[0] = irq_mask_val;
    } else {
        irq_mask_reg = INT_ENABLE_offset2;
        irq_mask_val = chip->irq_mask[1] & (~(1 << (data->hwirq - 8)));
        chip->irq_mask[1] = irq_mask_val;
    }

    dx1761_writeb(chip, irq_mask_reg, irq_mask_val);

}

static void dx1761_irq_unmask(struct irq_data* data)
{
    uint8_t irq_mask_reg, irq_mask_val;
    struct gpio_chip* gc = irq_data_get_irq_chip_data(data);
    struct dx1761_chip* chip = gpiochip_get_data(gc);

    if (data->hwirq <8) {
        irq_mask_reg = INT_ENABLE_offset;
        irq_mask_val = chip->irq_mask[0] | 1 << data->hwirq;
        chip->irq_mask[0] = irq_mask_val;
    } else {
        irq_mask_reg = INT_ENABLE_offset2;
        irq_mask_val = chip->irq_mask[1] | 1 << (data->hwirq - 8);
        chip->irq_mask[1] = irq_mask_val;
    }

    dx1761_writeb(chip, irq_mask_reg, irq_mask_val);
}

static void dx1761_irq_bus_lock(struct irq_data* data)
{
    struct gpio_chip* gc = irq_data_get_irq_chip_data(data);
    struct dx1761_chip* chip = gpiochip_get_data(gc);

    mutex_lock(&chip->irq_lock);
}

static void dx1761_irq_bus_sync_unlock(struct irq_data* data)
{
    struct gpio_chip* gc = irq_data_get_irq_chip_data(data);
    struct dx1761_chip* chip = gpiochip_get_data(gc);

    mutex_unlock(&chip->irq_lock);
}

static int dx1761_irq_set_type(struct irq_data* data, unsigned int type)
{
    struct gpio_chip* gc = irq_data_get_irq_chip_data(data);
    struct dx1761_chip* chip = gpiochip_get_data(gc);
    uint8_t off = data->hwirq;
    uint8_t reg, val;

    if (off < 8) {
        switch (type) {
        case IRQ_TYPE_EDGE_BOTH:
            reg = INT_TRIGBOTH_offset;
            val = chip->irq_edgeboth[0] | 1 << off;
            chip->irq_edgeboth[0] = val;
            break;
        case IRQ_TYPE_EDGE_FALLING:
            reg = INT_TRIGEDGE_offset;
            val = chip->irq_edge[0] | 1 << off;
            chip->irq_edge[0] = val;
            break;
        case IRQ_TYPE_EDGE_RISING:
            reg = INT_TRIGEDGE_offset;
            val = chip->irq_edge[0] & (~ (1 << off));
            chip->irq_edge[0] = val;
            break;
        default:
            return -EINVAL;
        }
    } else (off < 16) {
        switch (type) {
        case IRQ_TYPE_EDGE_BOTH:
            reg = INT_TRIGBOTH_offset2;
            val = chip->irq_edgeboth[1] | 1 << (off - 8);
            chip->irq_edgeboth[1] = val;
            break;
        case IRQ_TYPE_EDGE_FALLING:
            reg = INT_TRIGEDGE_offset2;
            val = chip->irq_edge[1] | 1 << (off - 8);
            chip->irq_edge[1] = val;
            break;
        case IRQ_TYPE_EDGE_RISING:
            reg = INT_TRIGEDGE_offset2;
            val = chip->irq_edge[1] & (~ (1 << (off - 8)));
            chip->irq_edge[1] = val;
            break;
        default:
            return -EINVAL;
        }
    } else {
        return -EINVAL;
    }

    dx1761_writeb(chip, reg, val);

    return 0;
}

static struct irq_chip dx1761_irq_chip = {
    .name = "dx1761",
    .irq_mask = dx1761_irq_mask,
    .irq_unmask = dx1761_irq_unmask,
    .irq_bus_lock = dx1761_irq_bus_lock,
    .irq_bus_sync_unlock = dx1761_irq_bus_sync_unlock,
    .irq_set_type = dx1761_irq_set_type,
};

static irqreturn_t dx1761_irq_handler(int irq, void* dev)
{
    struct dx1761_chip* chip = dev;
    uint8_t pend[2];
    uint16_t pending;
    uint8_t irq_num;

    dx1761_readb(chip, INT_STATUS_offset, &pend[0]);
    dx1761_readb(chip, INT_STATUS_offset2, &pend[1]);
    pend[0] = (~irq_mask[0]) & pend[0];
    pend[1] = (~irq_mask[1]) & pend[1];

    pending = pend[1] << 8 | pend[0];
    if (!pending)
        return IRQ_HANDLED;
    
    do {
        irq_num = __ffs(pending);
        handle_nested_irq(irq_find_mapping(chip->gpio_chip.irq.domain, irq_num));
        pending &= ~(1 << irq_num);
    } while (pending);

    return IRQ_HANDLED;
    
}

static void clear_irq(struct dx1761_chip* chip)
{
    dx1761_writeb(chip, INT_TRIGEDGE_offset, 0);
    dx1761_writeb(chip, INT_TRIGEDGE_offset2, 0);
    dx1761_writeb(chip, INT_TRIGBOTH_offset, 0);
    dx1761_writeb(chip, INT_TRIGBOTH_offset2, 0);
    dx1761_writeb(chip, INT_ENABLE_offset, 0);
    dx1761_writeb(chip, INT_ENABLE_offset2, 0);
    dx1761_writeb(chip, INT_STATUS_offset, 0);
    dx1761_writeb(chip, INT_STATUS_offset2, 0);
}

static int dx1761_irq_setup(struct dx1761_chip* chip)
{
    struct i2c_client* client = chip->client;
    struct gpio_irq_chip* girq;
    int ret;
    int irq_base = 0;

    if (!client->irq)
        return 0;
    
    mutex_init(chip->irq_lock);

    ret = devm_request_threaded_irq(client->dev, client->irq,
             NULL, dx1761_irq_handler, IRQF_ONESHOT | IRQF_TRIGGER_LOW | IRQF_SHARED,
            dev_name(&client->dev), chip);
    if (ret) {
        dev_err(&client->dev, "failed to reqeust irq %d\n",
            client->irq);
        return ret;
    }

    girq = &chip->gpio_chip.irq;
    girq->chip = &dx1761_irq_chip;
    girq->parent_handler = handle_level_irq;
    girq->num_parents = 1;
    girq->parents = devm_kcalloc(client->dev, 1,
                        sizeof(*girq->parents), GFP_KERNEL);
    if (!girq->parents)
        return -ENOMEM;
    girq->parents[0] = client->irq;
    girq->default_type = IRQ_TYPE_NONE;
    girq->handler = handle_simple_irq;
    girq->threaded = true;
    girq->first = irq_base;

    clear_irq(chip);
    chip->irq_edge[0] = chip->irq_edge[1] = 0;
    chip->irq_edgeboth[0] = chip->irq_edgeboth[1] = 0;
    chip->irq_mask[0] = chip->irq_mask[1] = 0;

    return 0;
}

static int dx1761_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
    struct dx1761_chip* dxchip;
    int ret;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
        return -EIO;

    dxchip = devm_kzalloc(&client->dev, sizeof(*dxchip), GFP_KERNEL);
    if (!dxchip)
	    return -ENOMEM;

    dxchip->gpio_chip.direction_input = dx1761_gpio_direction_input;
    dxchip->gpio_chip.direction_output = dx1761_gpio_direction_output;
    dxchip->gpio_chip.get_direction = dx1761_gpio_get_direction;
    dxchip->gpio_chip.set = dx1761_gpio_set_value;
    dxchip->gpio_chip.get = dx1761_gpio_get_value;
    dxchip->gpio_chip.ngpio = GPIO_NUM;
    dxchip->gpio_chip.base = -1;
    dxchip->gpio_chip.label = client->name;
    dxchip->gpio_chip.parent = &client->dev;
    dxchip->client = client;
    dxchip->gpio_chip.can_sleep = true;
    dxchip->gpio_chip.owner = THIS_MODULE;

    mutex_init(&dxchip->lock);

    ret = dx1761_irq_setup(dxchip);
    if (ret) {
        return ret;
    }

    ret = devm_gpiochip_add_data(&client->dev, &dxchip->gpio_chip, dxchip);
    if (ret) {
        return ret;
    }

    i2c_set_clientdata(client, dxchip);

    return 0;

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
MODULE_VERSION("0.85");
MODULE_LICENSE("GPL");




