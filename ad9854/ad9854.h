/*
 * AD9854 SPI DDS driver
 *
 * Copyright 2015 SYSU SIST JinWei Hwang.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_DDS_AD9854_H_
#define IIO_DDS_AD9854_H_

/* Registers */
#define AD9854_REG_SER_SIZE		12
#define AD9854_REG_SER_CTRL_DEFAULT_VAL		0x10640120

#define AD9854_REG_SER_PHASE_ADJ_1		0x00
#define AD9854_REG_SER_PHASE_ADJ_2		0x01
#define AD9854_REG_SER_FREQ_TUNING_WORD_1		0x02
#define AD9854_REG_SER_FREQ_TUNING_WORD_2		0x03
#define AD9854_REG_SER_DELTA_FREQ_WORD		0x04
#define AD9854_REG_SER_UPDATE_CLOCK		0x05
#define AD9854_REG_SER_RAMP_RATE_CLOCK		0x06
#define AD9854_REG_SER_CTRL		0x07
#define AD9854_REG_SER_OUTPUT_I_MULTIPLIER		0x08
#define AD9854_REG_SER_OUTPUT_Q_MULTIPLIER		0x09
#define AD9854_REG_SER_OUTPUT_RAMP_RATE		0x0A
#define AD9854_REG_SER_QDAC		0x0B

/* Attr ID */
#define AD9854_ATTR_CTRL_COMP_PD  		(0xE0000000 | CTRL_CR_COMP_PD)
#define AD9854_ATTR_CTRL_QDAC_PD  		(0xE0000000 | CTRL_CR_QDAC_PD)
#define AD9854_ATTR_CTRL_DAC_PD  		(0xE0000000 | CTRL_CR_DAC_PD)
#define AD9854_ATTR_CTRL_DIG_PD  		(0xE0000000 | CTRL_CR_DIG_PD)
#define AD9854_ATTR_CTRL_CLR_ACC1  		(0xE0000000 | CTRL_CR_CLR_ACC_1)
#define AD9854_ATTR_CTRL_CLR_ACC2  		(0xE0000000 | CTRL_CR_CLR_ACC_2)
#define AD9854_ATTR_CTRL_TRIANGLE  		(0xE0000000 | CTRL_CR_TRIANGLE)
#define AD9854_ATTR_CTRL_SRC_QDAC  		(0xE0000000 | CTRL_CR_SRC_QDAC)
#define AD9854_ATTR_CTRL_INT_UPDCLK 	(0xE0000000 | CTRL_CR_IN_EXT_UP_CLK)
#define AD9854_ATTR_CTRL_MODES  		(0xE0000000 | CTRL_CR_MODE_0)
#define AD9854_ATTR_CTRL_BYPASSINVSINC  (0xE0000000 | CTRL_CR_BYPASS_INV_SINC)
#define AD9854_ATTR_CTRL_OSK_EN  		(0xE0000000 | CTRL_CR_OSK_EN)
#define AD9854_ATTR_CTRL_OSK_INT  		(0xE0000000 | CTRL_CR_OSK_INT)
#define AD9854_ATTR_RESET  				(0xE0000000 | 2)
#define AD9854_ATTR_CTRL_REG  			(0xE0000000 | 1)

/* Instruction byte */
#define AD9854_INST_R		(1<<7)
#define AD9854_INST_W		(0<<7)
#define AD9854_INST_ADDR_R(addr)		(AD9854_INST_R | (addr & (0x0F)))
#define AD9854_INST_ADDR_W(addr)		(AD9854_INST_W | (addr & (0x0F)))

/* control reg bits(31:24) */
#define CTRL_CR_COMP_PD		(1<<28)
#define CTRL_CR_QDAC_PD		(1<<26)
#define CTRL_CR_DAC_PD		(1<<25)
#define CTRL_CR_DIG_PD		(1<<24)
/* control reg bits(24:16) */
#define CTRL_CR_PLL_RANGE		(1<<22)
#define CTRL_CR_BYPASS_PLL		(1<<21)
#define CTRL_CR_REF_MULT_4		(1<<20)
#define CTRL_CR_REF_MULT_3		(1<<19)
#define CTRL_CR_REF_MULT_2		(1<<18)
#define CTRL_CR_REF_MULT_1		(1<<17)
#define CTRL_CR_REF_MULT_0		(1<<16)
/* control reg bits(15:8) */
#define CTRL_CR_CLR_ACC_1		(1<<15)
#define CTRL_CR_CLR_ACC_2		(1<<14)
#define CTRL_CR_TRIANGLE		(1<<13)
#define CTRL_CR_SRC_QDAC		(1<<12)
#define CTRL_CR_MODE_2		(1<<11)
#define CTRL_CR_MODE_1		(1<<10)
#define CTRL_CR_MODE_0		(1<<9)
#define CTRL_CR_IN_EXT_UP_CLK		(1<<8)
/* control reg bits(7:0) */
#define CTRL_CR_BYPASS_INV_SINC		(1<<6)
#define CTRL_CR_OSK_EN		(1<<5)
#define CTRL_CR_OSK_INT		(1<<4)
#define CTRL_CR_LSB_FIRST		(1<<1)
#define CTRL_CR_SDO_ACTIVE		(1<<0)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_freqY
 */
#define IIO_DEV_ATTR_FREQ(_channel, _num, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_freq##_num,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_freqY_scale
 */
#define IIO_CONST_ATTR_FREQ_SCALE(_channel, _string)			\
	IIO_CONST_ATTR(out_altvoltage##_channel##_freq_scale, _string)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_phaseY
 */
#define IIO_DEV_ATTR_PHASE(_channel, _num, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_phase##_num,		\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_phaseY_scale
 */
#define IIO_CONST_ATTR_PHASE_SCALE(_channel, _string)			\
	IIO_CONST_ATTR(out_altvoltage##_channel##_phase_scale, _string)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_deltafreq
 */
#define IIO_DEV_ATTR_DELTAFREQ(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_deltafreq,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_updateclk
 */
#define IIO_DEV_ATTR_UPDATECLK(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_updateclk,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ramprateclk
 */
#define IIO_DEV_ATTR_RAMPRATECLK(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ramprateclk,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_osk_imulti
 */
#define IIO_DEV_ATTR_OSK_IMULTI(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_osk_imulti,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_osk_qmulti
 */
#define IIO_DEV_ATTR_OSK_QMULTI(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_osk_qmulti,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_osk_ramprate
 */
#define IIO_DEV_ATTR_OSK_RAMPRATE(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_osk_ramprate,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_qdac
 */
#define IIO_DEV_ATTR_QDAC(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_qdac,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ctrl_comp_pd
 */
#define IIO_DEV_ATTR_CTRL_COMP_PD(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_comp_pd,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ctrl_qdac_pd
 */
#define IIO_DEV_ATTR_CTRL_QDAC_PD(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_qdac_pd,	\
			_mode, _show, _store, _addr)

/**
* /sys/bus/iio/devices/.../out_altvoltageX_ctrl_dac_pd
*/
#define IIO_DEV_ATTR_CTRL_DAC_PD(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_dac_pd,	\
			_mode, _show, _store, _addr)

/**
* /sys/bus/iio/devices/.../out_altvoltageX_ctrl_dig_pd
*/
#define IIO_DEV_ATTR_CTRL_DIG_PD(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_dig_pd,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ctrl_clr_acc1
 */
#define IIO_DEV_ATTR_CTRL_CLR_ACC1(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_clr_acc1,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ctrl_clr_acc2
 */
#define IIO_DEV_ATTR_CTRL_CLR_ACC2(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_clr_acc2,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ctrl_triangle
 */
#define IIO_DEV_ATTR_CTRL_TRIANGLE(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_triangle,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ctrl_src_qdac
 */
#define IIO_DEV_ATTR_CTRL_SRC_QDAC(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_src_qdac,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ctrl_int_updclk
 */
#define IIO_DEV_ATTR_CTRL_INT_UPDCLK(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_int_updclk,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_modes (just for ad9854)
 */
#define IIO_DEV_ATTR_CTRL_MODES(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_modes,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ctrl_invsinc_en
 */
#define IIO_DEV_ATTR_CTRL_BYPASS_INVSINC(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_bypass_invsinc,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ctrl_osk_en
 */
#define IIO_DEV_ATTR_CTRL_OSK_EN(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_osk_en,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ctrl_osk_int
 */
#define IIO_DEV_ATTR_CTRL_OSK_INT(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_osk_int,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_reset
 */
#define IIO_DEV_ATTR_RESET(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_reset,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ctrl_reg
 */
#define IIO_DEV_ATTR_CTRL_REG(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrl_reg,	\
			_mode, _show, _store, _addr)


/**
 * struct ad9854_platform_data - platform specific information
 * @ref_clk:		OSCCLK frequency on the board
 * @ref_mult:		PLL muitiplier
 * @en_lsb_first:		no use current
 * @en_pll_bypass:		pll bypass enable
 * @gpio_osk:		OSK Pin number, set it to -1 if not uses
 * @gpio_fsk_bpsk_hold:		FSK/BPSK/HOLD Pin number, set it to -1 if not uses
 * @gpio_io_ud_clk:		I/O UD CLK Pin number, set it to -1 if not uses
 * @gpio_m_reset:		master reset Pin number, necessary
 * @gpio_io_reset:		I/O reset Pin number, necessary
 * @gpio_sp_select:		S/P select Pin number, set it to -1 if not uses
 * @gpio_spi_scs:		SPI software CS pin, set it to -1 if not uses
 */
struct ad9854_platform_data {
	unsigned int		ref_clk;
	unsigned int		ref_mult;
	bool		en_lsb_first;
	bool		en_pll_bypass;
	unsigned		gpio_osk;
	unsigned		gpio_fsk_bpsk_hold;
	unsigned		gpio_io_ud_clk;
	unsigned		gpio_m_reset;
	unsigned		gpio_io_reset;
	unsigned		gpio_sp_select;
	unsigned		gpio_spi_scs;
};

/**
 * struct ad9854_state - driver instance specific data
 * @spi:		spi_device
 * @reg:		supply regulator
 * @pdata:		ad9854_platform_data
 * @bops:		ad9854_bus_ops
 * @ser_regs:		ad9854 serial ops register
 */
struct ad9854_state {
	struct spi_device		*spi;
	struct regulator		*reg;
	struct ad9854_platform_data	*pdata;
	const struct ad9854_bus_ops	*bops;
	struct ad9854_ser_reg *ser_regs;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__be16				data ____cacheline_aligned;
};

/**
 * struct ad9854_ser_reg - ad9854 serial ops register
 * @reg_addr:		register address
 * @reg_len:		register size in bytes
 * @reg_val:		register value
 */
struct ad9854_ser_reg
{
	unsigned int reg_addr;
	unsigned int reg_len;
	u64 reg_val;
};

/**
 * struct ad9854_bus_ops - ad9854 spi bus operation
 * @read_reg:		read register value in reg_addr, then save the value in st->ser_regs
 * @read_reg:		write register value in st->ser_regs to device through spi bus
 */
struct ad9854_bus_ops {
	/* more methods added in future? */
	int (*read_reg)(struct ad9854_state *st, unsigned int reg_addr);
	int (*write_reg)(struct ad9854_state *st, unsigned int reg_addr);
};

/**
 * ad9854_supported_device_ids:
 */
enum ad9854_supported_device_ids {
	ID_AD9854,
};

/* functions define */
static unsigned int ad9854_ftw_to_freq(struct ad9854_state *st, u64 ftw);
static u64 ad9854_freq_to_ftw(struct ad9854_state *st, unsigned int freq);
static int ad9854_io_reset(struct ad9854_state *st);
static int ad9854_master_reset(struct ad9854_state *st);
static int ad9854_io_update(struct ad9854_state *st);
static int ad9854_spi_scs_low(struct ad9854_state *st);
static int ad9854_spi_scs_high(struct ad9854_state *st);
static int ad9854_request_gpios(struct ad9854_state *st);
static void ad9854_free_gpios(struct ad9854_state *st);
static int ad9854_spi_read_reg(struct ad9854_state *st, unsigned int reg_addr);
static int ad9854_spi_write_reg(struct ad9854_state *st, unsigned int reg_addr);
static ssize_t ad9854_read(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ad9854_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len);
static int ad9854_check_ser_reg_val(unsigned int reg_addr, u64 reg_val);
static int ad9854_ctrl_reg_init(struct ad9854_state *st);
static struct ad9854_platform_data *ad9854_parse_dt(struct spi_device *spi);
struct iio_dev *ad9854_probe(struct device *dev, unsigned id, const struct ad9854_bus_ops *bops);
static int ad9854_remove(struct iio_dev *indio_dev);
static int ad9854_spi_probe(struct spi_device *spi);
static int ad9854_spi_remove(struct spi_device *spi);

#endif /* IIO_DDS_AD9854_H_ */
