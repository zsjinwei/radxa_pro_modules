/*
 * AD9854 SPI DDS driver
 *
 * Copyright 2015 SYSU SIST JinWei Hwang.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "ad9854.h"

#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>

#define MAX_SPI_FREQ_HZ		10000000

/* ad9854 spi operation register define and initial */
struct ad9854_ser_reg ad9854_ser_reg_tbl[AD9854_REG_SER_SIZE] = {
	[AD9854_REG_SER_PHASE_ADJ_1] = {
		.reg_addr = AD9854_REG_SER_PHASE_ADJ_1,
		.reg_len = 2,
		.reg_val = 0x00,
	},

	[AD9854_REG_SER_PHASE_ADJ_2] = {
		.reg_addr = AD9854_REG_SER_PHASE_ADJ_2,
		.reg_len = 2,
		.reg_val = 0x00,
	},

	[AD9854_REG_SER_FREQ_TUNING_WORD_1] = {
		.reg_addr = AD9854_REG_SER_FREQ_TUNING_WORD_1,
		.reg_len = 6,
		.reg_val = 0x00,
	},

	[AD9854_REG_SER_FREQ_TUNING_WORD_2] = {
		.reg_addr = AD9854_REG_SER_FREQ_TUNING_WORD_2,
		.reg_len = 6,
		.reg_val = 0x00,
	},

	[AD9854_REG_SER_DELTA_FREQ_WORD] = {
		.reg_addr = AD9854_REG_SER_DELTA_FREQ_WORD,
		.reg_len = 6,
		.reg_val = 0x00,
	},

	[AD9854_REG_SER_UPDATE_CLOCK] = {
		.reg_addr = AD9854_REG_SER_UPDATE_CLOCK,
		.reg_len = 4,
		.reg_val = 0x40,
	},

	[AD9854_REG_SER_RAMP_RATE_CLOCK] = {
		.reg_addr = AD9854_REG_SER_RAMP_RATE_CLOCK,
		.reg_len = 3,
		.reg_val = 0x00,
	},

	[AD9854_REG_SER_CTRL] = {
		.reg_addr = AD9854_REG_SER_CTRL,
		.reg_len = 4,
		.reg_val = AD9854_REG_SER_CTRL_DEFAULT_VAL,
	},

	[AD9854_REG_SER_OUTPUT_I_MULTIPLIER] = {
		.reg_addr = AD9854_REG_SER_OUTPUT_I_MULTIPLIER,
		.reg_len = 2,
		.reg_val = 0x00,
	},

	[AD9854_REG_SER_OUTPUT_Q_MULTIPLIER] = {
		.reg_addr = AD9854_REG_SER_OUTPUT_Q_MULTIPLIER,
		.reg_len = 2,
		.reg_val = 0x00,
	},

	[AD9854_REG_SER_OUTPUT_RAMP_RATE] = {
		.reg_addr = AD9854_REG_SER_OUTPUT_RAMP_RATE,
		.reg_len = 1,
		.reg_val = 0x80,
	},

	[AD9854_REG_SER_QDAC] = {
		.reg_addr = AD9854_REG_SER_QDAC,
		.reg_len = 2,
		.reg_val = 0x00,
	},

};

/**
 * convert frequency tuning word to desire frequency
 * @param  st  struct ad9854_state
 * @param  ftw frequency tuning word
 * @return     desire frequency
 */
static unsigned int ad9854_ftw_to_freq(struct ad9854_state *st, u64 ftw)
{
	unsigned int sysclk = (st->pdata->ref_mult) * (st->pdata->ref_clk);
	return (ftw * sysclk) >> 48;
}

/**
 * convert desire frequency to frequency tuning word
 * @param  st   struct ad9854_state
 * @param  freq desire frequency
 * @return      frequency tuning word
 */
static u64 ad9854_freq_to_ftw(struct ad9854_state *st, unsigned int freq)
{
	unsigned int sysclk = (st->pdata->ref_mult) * (st->pdata->ref_clk);
	u64 result = ((u64)freq) << 48;
	do_div(result, sysclk);
	return result;
}

/**
 * do io reset
 * @param  st struct ad9854_state
 * @return    0 - success
 *            -ENODEV - I/O reset Pin is not set
 */
static int ad9854_io_reset(struct ad9854_state *st)
{
	if (gpio_is_valid(st->pdata->gpio_io_reset)) {
		gpio_set_value(st->pdata->gpio_io_reset, 1);
		ndelay(100); /* t_reset >= 100ns */
		gpio_set_value(st->pdata->gpio_io_reset, 0);
		return 0;
	}

	return -ENODEV;
}

/**
 * do master reset
 * @param  st struct ad9854_state
 * @return    0 - success
 *            -ENODEV - master reset Pin is not set
 */
static int ad9854_master_reset(struct ad9854_state *st)
{
	if (gpio_is_valid(st->pdata->gpio_m_reset)) {
		gpio_set_value(st->pdata->gpio_m_reset, 1);
		ndelay(100); /* t_reset >= 100ns */
		gpio_set_value(st->pdata->gpio_m_reset, 0);
		dev_info(&st->spi->dev, "Do master reset success.\n");
		return 0;
	}

	return -ENODEV;
}

/**
 * do I/O Update
 * @param  st struct ad9854_state
 * @return    0 - success
 *            -ENODEV - master reset Pin is not set
 */
static int ad9854_io_update(struct ad9854_state *st)
{
	if (gpio_is_valid(st->pdata->gpio_io_ud_clk)) {
		gpio_set_value(st->pdata->gpio_io_ud_clk, 1);
		ndelay(1000); /* t_reset >= 100ns */
		gpio_set_value(st->pdata->gpio_io_ud_clk, 0);
		return 0;
	}

	return -ENODEV;
}

/**
 * set gpio spi software cs pin to logic-low
 * @param  st struct ad9854_state
 * @return    0 - success
 *            -ENODEV - spi software cs Pin is not define
 */
static int ad9854_spi_scs_low(struct ad9854_state *st)
{
	if (gpio_is_valid(st->pdata->gpio_spi_scs)) {
		gpio_set_value(st->pdata->gpio_spi_scs, 0);
		return 0;
	}

	return -ENODEV;
}

/**
 * set gpio spi software cs pin to logic-high
 * @param  st struct ad9854_state
 * @return    0 - success
 *            -ENODEV - spi software cs Pin is not define
 */
static int ad9854_spi_scs_high(struct ad9854_state *st)
{
	if (gpio_is_valid(st->pdata->gpio_spi_scs)) {
		gpio_set_value(st->pdata->gpio_spi_scs, 1);
		return 0;
	}

	return -ENODEV;
}

/**
 * request gpios defined in ad9854_platform_data
 * @param  st struct ad9854_state
 * @return    request result: 0 - success
 */
static int ad9854_request_gpios(struct ad9854_state *st)
{
	int ret;

	// if (gpio_is_valid(st->spi->cs_gpio)) {
	// 	ret = gpio_request_one(st->spi->cs_gpio,
	// 	                       GPIOF_OUT_INIT_HIGH,
	// 	                       "AD9854_OSK");
	// 	if (ret) {
	// 		dev_err(&st->spi->dev, "failed to request AD9854 SPI CS pin\n");
	// 	}
	// }

	if (gpio_is_valid(st->pdata->gpio_osk)) {
		ret = gpio_request_one(st->pdata->gpio_osk,
		                       GPIOF_OUT_INIT_LOW,
		                       "AD9854_OSK");
		if (ret) {
			dev_err(&st->spi->dev, "failed to request GPIO OSK\n");
		}
	}

	if (gpio_is_valid(st->pdata->gpio_fsk_bpsk_hold)) {
		ret = gpio_request_one(st->pdata->gpio_fsk_bpsk_hold,
		                       GPIOF_OUT_INIT_LOW,
		                       "AD9854_FSK_BPSK_HOLD");
		if (ret) {
			dev_err(&st->spi->dev, "failed to request GPIO FSK/BPSK/HOLD\n");
		}
	}

	if (gpio_is_valid(st->pdata->gpio_io_ud_clk)) {
		ret = gpio_request_one(st->pdata->gpio_io_ud_clk,
		                       GPIOF_OUT_INIT_LOW,
		                       "AD9854_IO_UD_CLK");
		if (ret) {
			dev_err(&st->spi->dev, "failed to request GPIO I/O UD CLK\n");
		}
	}

	if (gpio_is_valid(st->pdata->gpio_sp_select)) {
		ret = gpio_request_one(st->pdata->gpio_sp_select,
		                       GPIOF_OUT_INIT_LOW,
		                       "AD9854_SP_SELECT");
		if (ret) {
			dev_info(&st->spi->dev, "failed to request GPIO S/P SELECT\n");
		}
	}

	if (gpio_is_valid(st->pdata->gpio_spi_scs)) {
		ret = gpio_request_one(st->pdata->gpio_spi_scs,
		                       GPIOF_OUT_INIT_HIGH,
		                       "AD9854_SPI_SCS");
		if (ret) {
			dev_info(&st->spi->dev, "failed to request GPIO SPI Software CS\n");
		}
	}

	if (gpio_is_valid(st->pdata->gpio_m_reset)) {
		ret = gpio_request_one(st->pdata->gpio_m_reset,
		                       GPIOF_OUT_INIT_LOW,
		                       "AD9854_MASTER_RESET");
		if (ret) {
			dev_err(&st->spi->dev, "failed to request GPIO MASTER RESET\n");
			goto error_m_reset;
		}
	} else {
		ret = -EIO;
		goto error_m_reset;
	}

	if (gpio_is_valid(st->pdata->gpio_io_reset)) {
		ret = gpio_request_one(st->pdata->gpio_io_reset,
		                       GPIOF_OUT_INIT_LOW,
		                       "AD9854_IO_RESET");
		if (ret) {
			dev_err(&st->spi->dev, "failed to request GPIO IO RESET\n");
			goto error_io_reset;
		}
	} else {
		ret = -EIO;
		goto error_io_reset;
	}

	dev_err(&st->spi->dev, "GPIOs request success: no error.\n");
	return 0;

error_io_reset:
	if (gpio_is_valid(st->pdata->gpio_m_reset))
		gpio_free(st->pdata->gpio_m_reset);
error_m_reset:
	if (gpio_is_valid(st->pdata->gpio_spi_scs))
		gpio_free(st->pdata->gpio_spi_scs);
	if (gpio_is_valid(st->pdata->gpio_sp_select))
		gpio_free(st->pdata->gpio_sp_select);
	if (gpio_is_valid(st->pdata->gpio_io_ud_clk))
		gpio_free(st->pdata->gpio_io_ud_clk);
	if (gpio_is_valid(st->pdata->gpio_fsk_bpsk_hold))
		gpio_free(st->pdata->gpio_fsk_bpsk_hold);
	if (gpio_is_valid(st->pdata->gpio_osk))
		gpio_free(st->pdata->gpio_osk);

	return ret;
}

/**
 * free gpios defined in ad9854_platform_data
 * @param  st struct ad9854_state
 */
static void ad9854_free_gpios(struct ad9854_state *st)
{
	if (gpio_is_valid(st->pdata->gpio_spi_scs))
		gpio_free(st->pdata->gpio_spi_scs);
	if (gpio_is_valid(st->pdata->gpio_sp_select))
		gpio_free(st->pdata->gpio_sp_select);
	if (gpio_is_valid(st->pdata->gpio_io_reset))
		gpio_free(st->pdata->gpio_io_reset);
	if (gpio_is_valid(st->pdata->gpio_m_reset))
		gpio_free(st->pdata->gpio_m_reset);
	if (gpio_is_valid(st->pdata->gpio_io_ud_clk))
		gpio_free(st->pdata->gpio_io_ud_clk);
	if (gpio_is_valid(st->pdata->gpio_fsk_bpsk_hold))
		gpio_free(st->pdata->gpio_fsk_bpsk_hold);
	if (gpio_is_valid(st->pdata->gpio_osk))
		gpio_free(st->pdata->gpio_osk);
}

/**
 * [ad9854_spi_read_reg read value
 * from device to ad9854_ser_reg struct]
 * @param  dev spi device
 * @param  register which read from
 * @return     0 - no error
 */
static int ad9854_spi_read_reg(struct ad9854_state *st, unsigned int reg_addr)
{
	int ret;
	struct spi_device *spi = st->spi;
	struct ad9854_ser_reg *reg = &st->ser_regs[reg_addr];
	u64 reg_val = 0;
	char *data = (char *) &reg_val;
	char tx_inst = AD9854_INST_ADDR_R(reg->reg_addr);

	data += (8 - reg->reg_len);
	//dev_info(&st->spi->dev, "tx_inst: 0x%X \n", tx_inst);
	//dev_info(&st->spi->dev, "REG : addr:0x%X, len:0x%X, val:0x%X\n", reg->reg_addr, reg->reg_len, reg->reg_val);
	// I/O reset before operation in order to synchronization with the AD9854
	// enable CS pin
	ad9854_spi_scs_low(st);
	// do io reset
	ad9854_io_reset(st);
	// write instruction and read
	ret = spi_write_then_read(spi, &tx_inst, 1, data, reg->reg_len);
	// disable CS pin
	ad9854_spi_scs_high(st);
	if (ret < 0) {
		dev_err(&spi->dev, "SPI read error\n");
		return ret;
	}

	reg->reg_val = be64_to_cpu(reg_val);
	//dev_err(&st->spi->dev, "after read reg->reg_val: 0x%4X \n", reg->reg_val);

	return 0;
}

/**
 * [ad9854_spi_write_reg write value
 * from ad9854_ser_reg struct to device]
 * @param  dev spi device
 * @param  reg register which write to
 * @return     0 - no error
 */
static int ad9854_spi_write_reg(struct ad9854_state *st, unsigned int reg_addr)
{
	int ret, i;
	struct spi_device *spi = st->spi;
	struct ad9854_ser_reg *reg = &st->ser_regs[reg_addr];
	u64 old_val = reg->reg_val;
	u64 reg_val = cpu_to_be64(reg->reg_val);  // use MSB as default
	char *data = (char *) &reg_val;
	char tx_inst = AD9854_INST_ADDR_W(reg->reg_addr);
	data += (8 - reg->reg_len);
	// enable CS pin
	ad9854_spi_scs_low(st);
	// I/O reset before operation in order to synchronization with the AD9854
	ad9854_io_reset(st);
	//dev_err(&spi->dev, "tx_inst: %d \n", tx_inst);
	//dev_err(&spi->dev, "REG : addr:%d, len:%d, val:%lld.\n", reg->reg_addr, reg->reg_len, reg->reg_val);
	for (i = 0; i < reg->reg_len; ++i)
	{
		dev_err(&spi->dev, "data[%d]:0x%X\n", i, data[i]);
	}
	// write instruction
	ret = spi_write(spi, &tx_inst, 1);
	if (ret < 0) {
		dev_err(&spi->dev, "SPI write instruction error\n");
		reg->reg_val = old_val;  // write error, restore old value.
		return ret;
	}

	ret = spi_write(spi, data, reg->reg_len);
	if (ret < 0) {
		dev_err(&spi->dev, "SPI write data error\n");
		reg->reg_val = old_val;  // write error, restore old value.
		return ret;
	}
	// disable CS pin
	ad9854_spi_scs_high(st);
	ad9854_io_update(st);
	return 0;
}

static const struct ad9854_bus_ops ad9854_spi_bops = {
	.read_reg = &ad9854_spi_read_reg,
	.write_reg = &ad9854_spi_write_reg,
};

/**
 * ad9854_read - IIO device FS read call-back function
 * @param  dev  [description]
 * @param  attr [description]
 * @param  buf  [description]
 * @return      [description]
 */
static ssize_t ad9854_read(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad9854_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9854_ser_reg *reg;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32) this_attr->address) {
	case AD9854_REG_SER_FREQ_TUNING_WORD_1:
	case AD9854_REG_SER_FREQ_TUNING_WORD_2:
		reg = &st->ser_regs[this_attr->address];
		ret = ad9854_spi_read_reg(st, this_attr->address);
		if (!ret)
			ret = sprintf(buf, "%d\n", ad9854_ftw_to_freq(st, reg->reg_val));
		break;
	case AD9854_REG_SER_PHASE_ADJ_1:
	case AD9854_REG_SER_PHASE_ADJ_2:
	case AD9854_REG_SER_DELTA_FREQ_WORD:
	case AD9854_REG_SER_UPDATE_CLOCK:
	case AD9854_REG_SER_RAMP_RATE_CLOCK:

	case AD9854_REG_SER_OUTPUT_I_MULTIPLIER:
	case AD9854_REG_SER_OUTPUT_Q_MULTIPLIER:
	case AD9854_REG_SER_OUTPUT_RAMP_RATE:
	case AD9854_REG_SER_QDAC:
		reg = &st->ser_regs[this_attr->address];
		ret = ad9854_spi_read_reg(st, this_attr->address);
		if (!ret)
			ret = sprintf(buf, "%llu\n", reg->reg_val);
		break;
	case AD9854_REG_SER_CTRL:
		reg = &st->ser_regs[AD9854_REG_SER_CTRL];
		ret = ad9854_spi_read_reg(st, AD9854_REG_SER_CTRL);
		if (!ret) {
			ret = sprintf(buf, "CR[31:0] = 0x%llX.\n\
CR[0]: SDO active = %d.\n\
CR[1]: LSB first = %d.\n\
CR[4]: OSK INT = %d.\n\
CR[5]: OSK EN = %d.\n\
CR[6]: Bypass inv sinc = %d.\n\
CR[8]: Int/Ext update clock = %d.\n\
CR[9:11]: Mode = %d.\n\
CR[12]: SRC QDAC = %d.\n\
CR[13]: Triangle = %d.\n\
CR[14]: CLR ACC2 = %d.\n\
CR[15]: CLR ACC1 = %d.\n\
CR[16:20]: Ref Mult = %d.\n\
CR[21]: Bypass PLL = %d.\n\
CR[22]: PLL range = %d.\n\
CR[24]: DIG PD = %d.\n\
CR[25]: DAC PD = %d.\n\
CR[26]: QDAC PD = %d.\n\
CR[27]: Reserved, always low = %d.\n\
CR[28]: Comp PD = %d.\n\
Others: Don't care.\n",
			              reg->reg_val,
			              (unsigned int)(reg->reg_val & CTRL_CR_SDO_ACTIVE) ? 1 : 0,
			              (unsigned int)(reg->reg_val & CTRL_CR_LSB_FIRST) ? 1 : 0,
			              (unsigned int)(reg->reg_val & CTRL_CR_OSK_INT) ? 1 : 0,
			              (unsigned int)(reg->reg_val & CTRL_CR_OSK_EN) ? 1 : 0,
			              (unsigned int)(reg->reg_val & CTRL_CR_BYPASS_INV_SINC) ? 1 : 0,
			              (unsigned int)(reg->reg_val & CTRL_CR_IN_EXT_UP_CLK) ? 1 : 0,
			              (unsigned int)(reg->reg_val & 0x00000E00) >> 9,
			              (unsigned int)(reg->reg_val & CTRL_CR_SRC_QDAC) ? 1 : 0,
			              (unsigned int)(reg->reg_val & CTRL_CR_TRIANGLE) ? 1 : 0,
			              (unsigned int)(reg->reg_val & CTRL_CR_CLR_ACC_2) ? 1 : 0,
			              (unsigned int)(reg->reg_val & CTRL_CR_CLR_ACC_1) ? 1 : 0,
			              (unsigned int)(reg->reg_val & 0x001F0000) >> 16,
			              (unsigned int)(reg->reg_val & CTRL_CR_BYPASS_PLL) ? 1 : 0,
			              (unsigned int)(reg->reg_val & CTRL_CR_PLL_RANGE) ? 1 : 0,
			              (unsigned int)(reg->reg_val & CTRL_CR_DIG_PD) ? 1 : 0,
			              (unsigned int)(reg->reg_val & CTRL_CR_DAC_PD) ? 1 : 0,
			              (unsigned int)(reg->reg_val & CTRL_CR_QDAC_PD) ? 1 : 0,
			              (unsigned int)(reg->reg_val & (1 << 27)) ? 1 : 0,
			              (unsigned int)(reg->reg_val & CTRL_CR_COMP_PD) ? 1 : 0);
		}
		break;
	case AD9854_ATTR_CTRL_COMP_PD:
	case AD9854_ATTR_CTRL_QDAC_PD:
	case AD9854_ATTR_CTRL_DAC_PD:
	case AD9854_ATTR_CTRL_DIG_PD:
	case AD9854_ATTR_CTRL_CLR_ACC1:
	case AD9854_ATTR_CTRL_CLR_ACC2:
	case AD9854_ATTR_CTRL_TRIANGLE:
	case AD9854_ATTR_CTRL_SRC_QDAC:
	case AD9854_ATTR_CTRL_INT_UPDCLK:
	case AD9854_ATTR_CTRL_MODES:
	case AD9854_ATTR_CTRL_BYPASSINVSINC:
	case AD9854_ATTR_CTRL_OSK_EN:
	case AD9854_ATTR_CTRL_OSK_INT:
	case AD9854_ATTR_RESET:
	case AD9854_ATTR_CTRL_REG:
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

/**
 * ad9854_write - IIO device FS write call-back function
 * @param  dev  [description]
 * @param  attr [description]
 * @param  buf  [description]
 * @return      [description]
 */
static ssize_t ad9854_write(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t len)
{
	int ret;
	unsigned long long val, old_val;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad9854_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9854_ser_reg *reg;

	ret = kstrtoull(buf, 10, &val);
	if (ret)
		goto error_ret;

	mutex_lock(&indio_dev->mlock);
	switch ((u32) this_attr->address) {
	case AD9854_REG_SER_FREQ_TUNING_WORD_1:
	case AD9854_REG_SER_FREQ_TUNING_WORD_2:
		if (ad9854_check_ser_reg_val(this_attr->address, val))
		{
			ret = -EINVAL;
			break;
		}
		reg = &st->ser_regs[this_attr->address];
		old_val = reg->reg_val;
		reg->reg_val = ad9854_freq_to_ftw(st, val);
		ret = ad9854_spi_write_reg(st, this_attr->address);
		break;
	case AD9854_REG_SER_PHASE_ADJ_1:
	case AD9854_REG_SER_PHASE_ADJ_2:
	case AD9854_REG_SER_DELTA_FREQ_WORD:
	case AD9854_REG_SER_UPDATE_CLOCK:
	case AD9854_REG_SER_RAMP_RATE_CLOCK:

	case AD9854_REG_SER_OUTPUT_I_MULTIPLIER:
	case AD9854_REG_SER_OUTPUT_Q_MULTIPLIER:
	case AD9854_REG_SER_OUTPUT_RAMP_RATE:
	case AD9854_REG_SER_QDAC:
		if (ad9854_check_ser_reg_val(this_attr->address, val))
		{
			ret = -EINVAL;
			break;
		}
		reg = &st->ser_regs[this_attr->address];
		old_val = reg->reg_val;
		reg->reg_val = val;
		ret = ad9854_spi_write_reg(st, this_attr->address);
		break;
	case AD9854_ATTR_CTRL_COMP_PD:
	case AD9854_ATTR_CTRL_QDAC_PD:
	case AD9854_ATTR_CTRL_DAC_PD:
	case AD9854_ATTR_CTRL_DIG_PD:
	case AD9854_ATTR_CTRL_CLR_ACC1:
	case AD9854_ATTR_CTRL_CLR_ACC2:
	case AD9854_ATTR_CTRL_TRIANGLE:
	case AD9854_ATTR_CTRL_SRC_QDAC:
	case AD9854_ATTR_CTRL_INT_UPDCLK:
	case AD9854_ATTR_CTRL_BYPASSINVSINC:
	case AD9854_ATTR_CTRL_OSK_EN:
	case AD9854_ATTR_CTRL_OSK_INT:
		reg = &st->ser_regs[AD9854_REG_SER_CTRL];
		old_val = reg->reg_val;
		if (val == 0) {
			reg->reg_val &= ~((u32) this_attr->address & 0x1FFFFFFF);
		}
		else {
			reg->reg_val |= ((u32) this_attr->address & 0x1FFFFFFF);
		}
		ret = ad9854_spi_write_reg(st, AD9854_REG_SER_CTRL);
		break;
	case AD9854_ATTR_CTRL_MODES:
		reg = &st->ser_regs[AD9854_REG_SER_CTRL];
		old_val = reg->reg_val;
		if (val < 5) {
			reg->reg_val &= ~(CTRL_CR_MODE_0 | CTRL_CR_MODE_1 | CTRL_CR_MODE_2);
			reg->reg_val |= CTRL_CR_MODE_0 & (val << 9);
			reg->reg_val |= CTRL_CR_MODE_1 & (val << 9);
			reg->reg_val |= CTRL_CR_MODE_2 & (val << 9);
		}
		else {
			dev_err(dev, "Mode 0x%x is not supported.\n", (u32)val);
			ret = -EINVAL;
			break;
		}
		ret = ad9854_spi_write_reg(st, AD9854_REG_SER_CTRL);
		break;
	case AD9854_ATTR_RESET:
		if (!val) {
			ret = -EINVAL;
			break;
		}
		ret = ad9854_master_reset(st);
		if (!ret) {
			st->ser_regs[AD9854_REG_SER_PHASE_ADJ_1].reg_val = 0x00;
			st->ser_regs[AD9854_REG_SER_PHASE_ADJ_2].reg_val = 0x00;
			st->ser_regs[AD9854_REG_SER_FREQ_TUNING_WORD_1].reg_val = 0x00;
			st->ser_regs[AD9854_REG_SER_FREQ_TUNING_WORD_2].reg_val = 0x00;
			st->ser_regs[AD9854_REG_SER_DELTA_FREQ_WORD].reg_val = 0x00;
			st->ser_regs[AD9854_REG_SER_UPDATE_CLOCK].reg_val = 0x40;
			st->ser_regs[AD9854_REG_SER_RAMP_RATE_CLOCK].reg_val = 0x00;
			st->ser_regs[AD9854_REG_SER_CTRL].reg_val = AD9854_REG_SER_CTRL_DEFAULT_VAL;
			st->ser_regs[AD9854_REG_SER_OUTPUT_I_MULTIPLIER].reg_val = 0x00;
			st->ser_regs[AD9854_REG_SER_OUTPUT_Q_MULTIPLIER].reg_val = 0x00;
			st->ser_regs[AD9854_REG_SER_OUTPUT_RAMP_RATE].reg_val = 0x80;
			st->ser_regs[AD9854_REG_SER_QDAC].reg_val = 0x00;
		}
		else {
			break;
		}
		ret = ad9854_ctrl_reg_init(st);
		if (!ret) {
			dev_info(dev, "AD9854 reset success.\n");
		}
		break;
	case AD9854_ATTR_CTRL_REG:
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

error_ret:
	return ret ? ret : len;
}

/**
 * check the value which is ready to write to the register
 * @param  reg_addr register address
 * @param  reg_val  value ready to write
 * @return          0 - valid
 *                  -EINVAL - invalid
 */
static int ad9854_check_ser_reg_val(unsigned int reg_addr, u64 reg_val)
{
	const unsigned int reg_val_bits[AD9854_REG_SER_SIZE] = {14, 14, 48, 48, 48, 32, 20, 32, 12, 12, 8, 12};
	if (reg_addr >= AD9854_REG_SER_SIZE)
		return -EINVAL;
	if (reg_val >= ((u64)1 << reg_val_bits[reg_addr]))
		return -EINVAL;
	return 0;
}

/**
 * initial the control register(0x07)
 * @param  st struct ad9854_state
 * @return    0 - success
 */
static int ad9854_ctrl_reg_init(struct ad9854_state *st)
{
	int ret;
	struct ad9854_ser_reg *reg;
	struct ad9854_platform_data *pdata = st->pdata;
	struct spi_device *spi = st->spi;
	reg = &st->ser_regs[AD9854_REG_SER_CTRL];
	//reset control reg to default value
	reg->reg_val = AD9854_REG_SER_CTRL_DEFAULT_VAL;
	// set SDO active to use as 3-wire SPI mode
	reg->reg_val |= CTRL_CR_SDO_ACTIVE;
	// clear CR[8] to use externel update clk
	reg->reg_val &= ~CTRL_CR_IN_EXT_UP_CLK;
	// turn oof the inv sinc filter
	reg->reg_val |= CTRL_CR_BYPASS_INV_SINC;
	// set ref multiplier
	reg->reg_val &= ~(CTRL_CR_REF_MULT_0 | CTRL_CR_REF_MULT_1 | CTRL_CR_REF_MULT_2 | CTRL_CR_REF_MULT_3);
	reg->reg_val |= CTRL_CR_REF_MULT_0 & ((pdata->ref_mult) << 16);
	reg->reg_val |= CTRL_CR_REF_MULT_1 & ((pdata->ref_mult) << 16);
	reg->reg_val |= CTRL_CR_REF_MULT_2 & ((pdata->ref_mult) << 16);
	reg->reg_val |= CTRL_CR_REF_MULT_3 & ((pdata->ref_mult) << 16);
	reg->reg_val |= CTRL_CR_REF_MULT_4 & ((pdata->ref_mult) << 16);

	// QDAC power down
	reg->reg_val |= CTRL_CR_QDAC_PD;

	// check pll bypass enable
	if (pdata->en_pll_bypass) {
		dev_info(&spi->dev, "PLL byPass is enable.\n");
	}
	else {
		reg->reg_val &= ~CTRL_CR_BYPASS_PLL;
		dev_info(&spi->dev, "PLL byPass is disable.\n");
		// if (refMultiplier * refClk) >=200MHz, set the PLL range to enable VCO
		if ((pdata->ref_clk) * (pdata->ref_mult) >= 200000000) {
			reg->reg_val |= CTRL_CR_PLL_RANGE;
			dev_info(&spi->dev, "PLL range is enable.\n");
		}
	}
	ret = ad9854_spi_write_reg(st, AD9854_REG_SER_CTRL);
	return ret;
}

/* IIO Attr Define */
static IIO_DEV_ATTR_FREQ(0, 0, S_IWUSR | S_IRUSR, ad9854_read, ad9854_write, AD9854_REG_SER_FREQ_TUNING_WORD_1);
static IIO_DEV_ATTR_FREQ(0, 1, S_IWUSR | S_IRUSR, ad9854_read, ad9854_write, AD9854_REG_SER_FREQ_TUNING_WORD_2);
static IIO_CONST_ATTR_FREQ_SCALE(0, "1"); /* 1Hz */

static IIO_DEV_ATTR_PHASE(0, 0, S_IWUSR | S_IRUSR, ad9854_read, ad9854_write, AD9854_REG_SER_PHASE_ADJ_1);
static IIO_DEV_ATTR_PHASE(0, 1, S_IWUSR | S_IRUSR, ad9854_read, ad9854_write, AD9854_REG_SER_PHASE_ADJ_2);
static IIO_CONST_ATTR_PHASE_SCALE(0, "0.000383495197"); /* 2PI/2^14 rad*/

static IIO_DEV_ATTR_DELTAFREQ(0, S_IWUSR | S_IRUSR, ad9854_read, ad9854_write, AD9854_REG_SER_DELTA_FREQ_WORD);
static IIO_DEV_ATTR_UPDATECLK(0, S_IWUSR | S_IRUSR, ad9854_read, ad9854_write, AD9854_REG_SER_UPDATE_CLOCK);
static IIO_DEV_ATTR_RAMPRATECLK(0, S_IWUSR | S_IRUSR, ad9854_read, ad9854_write, AD9854_REG_SER_RAMP_RATE_CLOCK);
static IIO_DEV_ATTR_OSK_IMULTI(0, S_IWUSR | S_IRUSR, ad9854_read, ad9854_write, AD9854_REG_SER_OUTPUT_I_MULTIPLIER);
static IIO_DEV_ATTR_OSK_QMULTI(0, S_IWUSR | S_IRUSR, ad9854_read, ad9854_write, AD9854_REG_SER_OUTPUT_Q_MULTIPLIER);
static IIO_DEV_ATTR_OSK_RAMPRATE(0, S_IWUSR | S_IRUSR, ad9854_read, ad9854_write, AD9854_REG_SER_OUTPUT_RAMP_RATE);
static IIO_DEV_ATTR_QDAC(0, S_IWUSR | S_IRUSR, ad9854_read, ad9854_write, AD9854_REG_SER_QDAC);

static IIO_DEV_ATTR_CTRL_COMP_PD(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_CTRL_COMP_PD);
static IIO_DEV_ATTR_CTRL_QDAC_PD(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_CTRL_QDAC_PD);
static IIO_DEV_ATTR_CTRL_DAC_PD(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_CTRL_DAC_PD);
static IIO_DEV_ATTR_CTRL_DIG_PD(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_CTRL_DIG_PD);
static IIO_DEV_ATTR_CTRL_CLR_ACC1(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_CTRL_CLR_ACC1);
static IIO_DEV_ATTR_CTRL_CLR_ACC2(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_CTRL_CLR_ACC2);
static IIO_DEV_ATTR_CTRL_TRIANGLE(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_CTRL_TRIANGLE);
static IIO_DEV_ATTR_CTRL_SRC_QDAC(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_CTRL_SRC_QDAC);
static IIO_DEV_ATTR_CTRL_INT_UPDCLK(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_CTRL_INT_UPDCLK);
static IIO_DEV_ATTR_CTRL_MODES(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_CTRL_MODES);
static IIO_DEV_ATTR_CTRL_BYPASS_INVSINC(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_CTRL_BYPASSINVSINC);
static IIO_DEV_ATTR_CTRL_OSK_EN(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_CTRL_OSK_EN);
static IIO_DEV_ATTR_CTRL_OSK_INT(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_CTRL_OSK_INT);

static IIO_DEV_ATTR_RESET(0, S_IWUSR, NULL, ad9854_write, AD9854_ATTR_RESET);
static IIO_DEV_ATTR_CTRL_REG        (0, S_IRUSR, ad9854_read, NULL, AD9854_REG_SER_CTRL);

static struct attribute *ad9854_attributes[] = {
	&iio_dev_attr_out_altvoltage0_freq0.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_freq1.dev_attr.attr,
	&iio_const_attr_out_altvoltage0_freq_scale.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_phase0.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_phase1.dev_attr.attr,
	&iio_const_attr_out_altvoltage0_phase_scale.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_deltafreq.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_updateclk.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ramprateclk.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_osk_imulti.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_osk_qmulti.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_osk_ramprate.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_qdac.dev_attr.attr,

	&iio_dev_attr_out_altvoltage0_ctrl_comp_pd.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ctrl_qdac_pd.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ctrl_dac_pd.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ctrl_dig_pd.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ctrl_clr_acc1.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ctrl_clr_acc2.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ctrl_triangle.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ctrl_src_qdac.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ctrl_int_updclk.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ctrl_modes.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ctrl_bypass_invsinc.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ctrl_osk_en.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ctrl_osk_int.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_reset.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_ctrl_reg.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9854_attribute_group = {
	.attrs = ad9854_attributes,
};

static const struct iio_info ad9854_info = {
	.attrs = &ad9854_attribute_group,
	.driver_module = THIS_MODULE,
};

/**
 * parse the pdate defined in device tree file
 * @spi:		spi_device
 * @return:		ad9854_platdata_data filled by parsing dts file
 */
static struct ad9854_platform_data *
ad9854_parse_dt(struct spi_device *spi)
{
	struct device_node *node = spi->dev.of_node;
	struct ad9854_platform_data *pdata;
	unsigned gpio_osk;
	unsigned gpio_fsk_bpsk_hold;
	unsigned gpio_io_ud_clk;
	unsigned gpio_m_reset;
	unsigned gpio_io_reset;
	unsigned gpio_sp_select;
	unsigned gpio_spi_scs;
	pdata = devm_kzalloc(&spi->dev, sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL)
		return NULL; /* out of memory */

	/* no such property */
	if (of_property_read_u32(node, "ad9854,ref_mult", &pdata->ref_mult) != 0)
	{
		dev_err(&spi->dev, "ref_mult property is not defined.\n");
		return NULL;
	}

	/* no such property */
	if (of_property_read_u32(node, "ad9854,ref_clk", &pdata->ref_clk) != 0)
	{
		dev_err(&spi->dev, "ref_clk property is not defined.\n");
		return NULL;
	}

	if (pdata->ref_mult < 4 || pdata->ref_mult > 20) {
		pdata->ref_mult = 4;
		dev_err(&spi->dev, "ref_mult property is invalid, set it to default(4).\n");
	}

	if ((pdata->ref_clk) * (pdata->ref_mult) > 300000000) {
		dev_err(&spi->dev, "Invalid: ref_mult * ref_clk is bigger than 300MHz.\n");
		return NULL;
	}

	/* now get the gpio number*/
	gpio_osk = of_get_named_gpio(node, "ad9854,gpio_osk", 0);
	if (IS_ERR_VALUE(gpio_osk)) {
		dev_warn(&spi->dev, "gpio_osk can not setup, set it to -1.\n");
		pdata->gpio_osk = -1;
	}
	else
	{
		pdata->gpio_osk = gpio_osk;
	}
	/* now get the gpio number*/
	gpio_fsk_bpsk_hold = of_get_named_gpio(node, "ad9854,gpio_fsk_bpsk_hold", 0);
	if (IS_ERR_VALUE(gpio_fsk_bpsk_hold)) {
		dev_warn(&spi->dev, "gpio_fsk_bpsk_hold can not setup, set it to -1.\n");
		pdata->gpio_fsk_bpsk_hold = -1;
	}
	else
	{
		pdata->gpio_fsk_bpsk_hold = gpio_fsk_bpsk_hold;
	}
	/* now get the gpio number*/
	gpio_io_ud_clk = of_get_named_gpio(node, "ad9854,gpio_io_ud_clk", 0);
	if (IS_ERR_VALUE(gpio_io_ud_clk)) {
		dev_warn(&spi->dev, "gpio_io_ud_clk can not setup, set it to -1.\n");
		pdata->gpio_io_ud_clk = -1;
	}
	else
	{
		pdata->gpio_io_ud_clk = gpio_io_ud_clk;
	}
	/* now get the gpio number*/
	gpio_m_reset = of_get_named_gpio(node, "ad9854,gpio_m_reset", 0);
	if (IS_ERR_VALUE(gpio_m_reset)) {
		dev_warn(&spi->dev, "gpio_m_reset can not setup, set it to -1.\n");
		pdata->gpio_m_reset = -1;
	}
	else
	{
		pdata->gpio_m_reset = gpio_m_reset;
	}
	/* now get the gpio number*/
	gpio_io_reset = of_get_named_gpio(node, "ad9854,gpio_io_reset", 0);
	if (IS_ERR_VALUE(gpio_io_reset)) {
		dev_warn(&spi->dev, "gpio_io_reset can not setup, set it to -1.\n");
		pdata->gpio_io_reset = -1;
	}
	else
	{
		pdata->gpio_io_reset = gpio_io_reset;
	}
	/* now get the gpio number*/
	gpio_sp_select = of_get_named_gpio(node, "ad9854,gpio_sp_select", 0);
	if (IS_ERR_VALUE(gpio_sp_select)) {
		dev_warn(&spi->dev, "gpio_sp_select can not setup, set it to -1.\n");
		pdata->gpio_sp_select = -1;
	}
	else
	{
		pdata->gpio_sp_select = gpio_sp_select;
	}
	/* now get the gpio number*/
	gpio_spi_scs = of_get_named_gpio(node, "ad9854,gpio_spi_scs", 0);
	if (IS_ERR_VALUE(gpio_spi_scs)) {
		dev_warn(&spi->dev, "gpio_spi_scs can not setup, set it to -1.\n");
		pdata->gpio_spi_scs = -1;
	}
	else
	{
		pdata->gpio_spi_scs = gpio_spi_scs;
	}
	dev_info(&spi->dev, "DT parse result:\nref_mult: %d.\nref_clk: %d.\ngpio_osk = %d.\ngpio_fsk_bpsk_hold = %d.\ngpio_io_ud_clk = %d.\ngpio_m_reset = %d.\ngpio_io_reset = %d.\ngpio_sp_select = %d.\ngpio_spi_scs = %d.\n",
	         pdata->ref_mult,
	         pdata->ref_clk,
	         pdata->gpio_osk,
	         pdata->gpio_fsk_bpsk_hold,
	         pdata->gpio_io_ud_clk,
	         pdata->gpio_m_reset,
	         pdata->gpio_io_reset,
	         pdata->gpio_sp_select,
	         pdata->gpio_spi_scs);
	/* pdata is filled */
	return pdata;
}

struct iio_dev *ad9854_probe(struct device *dev,
                             unsigned id,
                             const struct ad9854_bus_ops *bops)
{
	struct ad9854_platform_data *pdata = dev->platform_data;
	struct ad9854_state *st;
	struct spi_device *spi = to_spi_device(dev);
	int ret;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return ERR_PTR(-ENOMEM);

	st = iio_priv(indio_dev);

	st->spi = spi;
	st->bops = bops;
	st->ser_regs = ad9854_ser_reg_tbl;
	st->reg = devm_regulator_get(dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			return ERR_PTR(ret);
	}

	st->pdata = pdata;

	indio_dev->dev.parent = dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad9854_info;

	ret = ad9854_request_gpios(st);
	if (ret)
		goto error_disable_reg;

	ret = ad9854_master_reset(st);
	if (ret)
		dev_warn(&st->spi->dev, "Failed to RESET: no MASTER RESET GPIO specified\n");

	ad9854_ctrl_reg_init(st);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_free_gpios;

	return indio_dev;

error_free_gpios:
	ad9854_free_gpios(st);

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);
	return ERR_PTR(ret);
}


static int ad9854_remove(struct iio_dev *indio_dev)
{
	struct ad9854_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	ad9854_free_gpios(st);

	return 0;
}

static int ad9854_spi_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad9854_platform_data *pdata;

	if (spi->dev.of_node != NULL)
	{
		pdata = ad9854_parse_dt(spi);
		if (pdata != NULL)
			spi->dev.platform_data = pdata;
		else {
			dev_dbg(&spi->dev, "no platform data?\n");
			return -ENODEV;
		}
	}

	indio_dev = ad9854_probe(&spi->dev,
	                         spi_get_device_id(spi)->driver_data,
	                         &ad9854_spi_bops);

	if (IS_ERR(indio_dev))
		return PTR_ERR(indio_dev);

	spi_set_drvdata(spi, indio_dev);

	return 0;
}

static int ad9854_spi_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = dev_get_drvdata(&spi->dev);

	return ad9854_remove(indio_dev);
}

#ifdef CONFIG_PM
static int ad9854_spi_suspend(struct device *dev)
{
	// TODO : power manager
	return 0;
}

static int ad9854_spi_resume(struct device *dev)
{
	// TODO : power manager
	return 0;
}

static const struct dev_pm_ops ad9854_pm_ops = {
	.suspend = ad9854_spi_suspend,
	.resume  = ad9854_spi_resume,
};
#define AD9854_SPI_PM_OPS (&ad9854_pm_ops)

#else
#define AD9854_SPI_PM_OPS NULL
#endif

static const struct spi_device_id ad9854_id[] = {
	{"ad9854", ID_AD9854},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9854_id);

#ifdef CONFIG_OF
static const struct of_device_id ad9854_dt_ids[] = {
	{ .compatible = "adi,ad9854" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ad9854_dt_ids);
#endif

static struct spi_driver ad9854_driver = {
	.driver = {
		.name = "ad9854",
		.owner = THIS_MODULE,
		.pm    = AD9854_SPI_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ad9854_dt_ids),
#endif
	},
	.probe = ad9854_spi_probe,
	.remove = ad9854_spi_remove,
	.id_table = ad9854_id,
};
module_spi_driver(ad9854_driver);

MODULE_AUTHOR("JinWei Hwang <zsjinwei@live.com>");
MODULE_DESCRIPTION("Analog Devices AD9854 Driver");
MODULE_LICENSE("GPL v2");
