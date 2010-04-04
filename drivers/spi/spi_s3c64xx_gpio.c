#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <plat/regs-gpio.h>
#include <plat/spi-gpio.h>
#include <plat/gpio-cfg.h>

struct s3c64xx_spigpio {
	struct spi_bitbang bitbang;
	struct s3c64xx_spigpio_info *info;
	struct platform_device *dev;
};

static inline struct s3c64xx_spigpio *get_spi(struct spi_device *spi)
{
	return spi_master_get_devdata(spi->master);
}

static inline void setsck(struct spi_device *dev, int on)
{
	struct s3c64xx_spigpio *spi = get_spi(dev);
	s3c_gpdat_setval(spi->info->pin_clk, on ? 1 : 0);
}

static inline void setmosi(struct spi_device *dev, int on)
{
	struct s3c64xx_spigpio *spi = get_spi(dev);
	s3c_gpdat_setval(spi->info->pin_mosi, on ? 1 : 0);
}

static inline u32 getmiso(struct spi_device *dev)
{
	struct s3c64xx_spigpio *spi = get_spi(dev);
	int val;

	s3c_gpdat_getval(spi->info->pin_miso, &val);
	return (val) ? 1 : 0;
}

#define spidelay(x) ndelay(x)

#define	EXPAND_BITBANG_TXRX
#include <linux/spi/spi_bitbang.h>

static u32 s3c64xx_spigpio_txrx_mode0(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, word, bits);
}

static u32 s3c64xx_spigpio_txrx_mode1(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, word, bits);
}

static u32 s3c64xx_spigpio_txrx_mode2(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, word, bits);
}

static u32 s3c64xx_spigpio_txrx_mode3(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 1, word, bits);
}

static void s3c64xx_spigpio_chipselect(struct spi_device *dev, int value)
{
	struct s3c64xx_spigpio *spi = get_spi(dev);

	switch(value) {
	case BITBANG_CS_ACTIVE:
		s3c_gpdat_setval(spi->info->pin_cs, 0);
		break;
	case BITBANG_CS_INACTIVE:
		s3c_gpdat_setval(spi->info->pin_cs, 1);
		break;
	}
}

static int s3c64xx_spigpio_probe(struct platform_device *dev)
{
	struct spi_master *master;
	struct s3c64xx_spigpio *spi;
	int ret;

	master = spi_alloc_master(&dev->dev, sizeof(struct s3c64xx_spigpio));

	if (master == NULL) {
		dev_err(&dev->dev, "allocate spi master failed\n");
		return -ENOMEM;
	}

	spi = spi_master_get_devdata(master);
	spi_master_set_devdata(master, spi);
	
	spi->info = dev->dev.platform_data;
	spi->bitbang.master = spi_master_get(master);
	spi->bitbang.master->num_chipselect = 1;
	spi->bitbang.master->bus_num = spi->info->bus_num;
	spi->bitbang.chipselect = s3c64xx_spigpio_chipselect;

	spi->bitbang.txrx_word[SPI_MODE_0] = s3c64xx_spigpio_txrx_mode0;
	spi->bitbang.txrx_word[SPI_MODE_1] = s3c64xx_spigpio_txrx_mode1;
	spi->bitbang.txrx_word[SPI_MODE_2] = s3c64xx_spigpio_txrx_mode2;
	spi->bitbang.txrx_word[SPI_MODE_3] = s3c64xx_spigpio_txrx_mode3;

	s3c_gpio_cfgpin(spi->info->pin_clk, S3C_GPIO_OUTPUT);
	s3c_gpdat_setval(spi->info->pin_clk, 0);

	s3c_gpio_cfgpin(spi->info->pin_mosi, S3C_GPIO_OUTPUT);
	s3c_gpdat_setval(spi->info->pin_mosi, 1);

	s3c_gpio_cfgpin(spi->info->pin_cs, S3C_GPIO_OUTPUT);
	s3c_gpdat_setval(spi->info->pin_cs, 1);

	s3c_gpio_cfgpin(spi->info->pin_miso, S3C_GPIO_INPUT);

	ret = spi_bitbang_start(&spi->bitbang);
	if (ret)
		goto err_no_bitbang;

	return 0;

err_no_bitbang:
	dev_err(&dev->dev, "start bitbang failed: %d\n", ret);
	spi_master_put(spi->bitbang.master);
	return ret;
}

static int s3c64xx_spigpio_remove(struct platform_device *dev)
{
	struct s3c64xx_spigpio *spi = platform_get_drvdata(dev);

	spi_bitbang_stop(&spi->bitbang);
	spi_master_put(spi->bitbang.master);
	return 0;
}

static struct platform_driver s3c64xx_spigpio_drv = {
	.probe = s3c64xx_spigpio_probe,
	.remove = s3c64xx_spigpio_remove,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name = "spi-s3c64xx-gpio",
		.owner = THIS_MODULE,
	},
};

static int __init s3c64xx_spigpio_init(void)
{
	return platform_driver_register(&s3c64xx_spigpio_drv);
}

static void __exit s3c64xx_spigpio_exit(void)
{
	platform_driver_unregister(&s3c64xx_spigpio_drv);
}

module_init(s3c64xx_spigpio_init);
module_exit(s3c64xx_spigpio_exit);

MODULE_DESCRIPTION("S3C64XX SPI GPIO Driver");
MODULE_AUTHOR("David Bolcsfoldi, <dbolcsfoldi@intrinsyc.com>");
MODULE_LICENSE("GPL");

