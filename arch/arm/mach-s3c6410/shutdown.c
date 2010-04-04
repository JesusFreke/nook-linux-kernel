#include <linux/delay.h>
#include <linux/io.h>

#include <asm/io.h>
#include <plat/regs-iic.h>
#include <mach/map.h>

#define I2C_TIMEOUT (1)
#define I2C_OK (0)
#define I2C_NOK (1)
#define I2C_NACK (2)
#define I2C_NOK_LA (3)
#define I2C_NOK_TOUT (4)

#define I2C_WRITE (0)
#define I2C_READ (1)

static void __iomem *regs;

static void i2c_bitbang_init(int slave_addr)
{
	int i, status;
	unsigned long tmp;

	regs = ioremap(S3C_PA_IIC, (S3C_PA_IIC + SZ_4K -1) - S3C_PA_IIC);

	/* wait for some time to give previous transfer a chance to finish */

	i = I2C_TIMEOUT * 1000;
	status = readl(regs + S3C2410_IICSTAT);
	
	while ((i > 0) && (status & S3C2410_IICSTAT_BUSBUSY)) {
		udelay (1000);
		status = readl(regs + S3C2410_IICSTAT);
		i--;
	}

	/* set prescaler, divisor according to freq, also set
	* ACKGEN, IRQ */
	writel(S3C2410_IICCON_TXDIV_16 | S3C2410_IICCON_IRQEN | (7 & S3C2410_IICCON_SCALEMASK), regs + S3C2410_IICCON);

	/* init to SLAVE RECEIVE and set slaveaddr */
	writel(0, regs + S3C2410_IICSTAT);
	writel(slave_addr, regs + S3C2410_IICADD);
	
	/* program Master Transmit (and implicit STOP) */
	writel(S3C2410_IICSTAT_MASTER_TX | S3C2410_IICSTAT_TXRXEN, regs + S3C2410_IICSTAT);
}

static inline int is_ack(void)
{
	return (!(readl(regs + S3C2410_IICSTAT) & S3C2410_IICSTAT_LASTBIT));
}

static inline void xfer_byte(void)
{
	writel(readl(regs + S3C2410_IICCON) & (~S3C2410_IICCON_IRQPEND), regs + S3C2410_IICCON);
}

static int wait_for_xfer(void)
{
	int i;
	unsigned long status;

	i = I2C_TIMEOUT * 10000;
	status = readl(regs + S3C2410_IICCON);

	while ((i > 0) && !(status & S3C2410_IICCON_IRQPEND)) {
		udelay (100);
		status = readl(regs + S3C2410_IICCON);
		i--;
	}

	return (status & S3C2410_IICCON_IRQPEND) ? I2C_OK : I2C_NOK_TOUT;
}

static int i2c_transfer(unsigned char cmd_type,
						unsigned char chip,
			            unsigned char addr[],
						unsigned char addr_len,
						unsigned char data[], unsigned short data_len)
{
	int i, status, result;

	if (data == 0 || data_len == 0) {
		/*Don't support data transfer of no length or to address 0 */
		return I2C_NOK;
	}

	/* Check I2C bus idle */
	i = I2C_TIMEOUT * 1000;
	status = readl(regs + S3C2410_IICSTAT);
	while ((i > 0) && (status & S3C2410_IICSTAT_BUSBUSY)) {
		udelay (1000);
		status = readl(regs + S3C2410_IICSTAT);
		i--;
	}

	if (status & S3C2410_IICSTAT_BUSBUSY) {
		return I2C_NOK_TOUT;
	}

	writel(readl(regs + S3C2410_IICCON) | 0x80, regs + S3C2410_IICCON);
	result = I2C_OK;

	switch (cmd_type) {
	case I2C_WRITE:
		if (addr && addr_len) {
			writel(chip, regs + S3C2410_IICDS);
			/* send START */
			writel(S3C2410_IICSTAT_MASTER_TX | S3C2410_IICSTAT_TXRXEN | S3C2410_IICSTAT_START, regs + S3C2410_IICSTAT);
			i = 0;
			while ((i < addr_len) && (result == I2C_OK)) {
				result = wait_for_xfer();
				writel(addr[i], regs + S3C2410_IICDS);
				xfer_byte();
				i++;
			}
			i = 0;
			while ((i < data_len) && (result == I2C_OK)) {
				result = wait_for_xfer();
				writel(data[i], regs + S3C2410_IICDS);
				xfer_byte();
				i++;
			}
		} else {
			writel(chip, regs + S3C2410_IICDS);
			/* send START */
			writel(S3C2410_IICSTAT_MASTER_TX | S3C2410_IICSTAT_TXRXEN | S3C2410_IICSTAT_START, regs + S3C2410_IICSTAT);
			i = 0;
			while ((i < data_len) && (result == I2C_OK)) {
				result = wait_for_xfer();
				writel(data[i], regs + S3C2410_IICDS);
				xfer_byte();
				i++;
			}
		}

		if (result == I2C_OK)
			result = wait_for_xfer();

		/* send STOP */
		writel(S3C2410_IICSTAT_MASTER_TX | S3C2410_IICSTAT_TXRXEN, regs + S3C2410_IICSTAT);
		xfer_byte();
		break;

	case I2C_READ:
		if (addr && addr_len) {
			writel(S3C2410_IICSTAT_MASTER_TX | S3C2410_IICSTAT_TXRXEN, regs + S3C2410_IICSTAT);
			writel(chip, regs + S3C2410_IICDS);
			/* send START */
			writel(readl(regs + S3C2410_IICSTAT) | S3C2410_IICSTAT_START, regs + S3C2410_IICSTAT);
			
			result = wait_for_xfer();
			if (is_ack()) {
				i = 0;
				while ((i < addr_len) && (result == I2C_OK)) {
					writel(addr[i], regs + S3C2410_IICDS);
					xfer_byte();
					result = wait_for_xfer();
					i++;
				}

				writel(chip, regs + S3C2410_IICDS);
				/* resend START */
				writel(S3C2410_IICSTAT_MASTER_RX | S3C2410_IICSTAT_TXRXEN | S3C2410_IICSTAT_START, regs + S3C2410_IICSTAT);
				xfer_byte();
				result = wait_for_xfer();
				i = 0;
				while ((i < data_len) && (result == I2C_OK)) {
					/* disable ACK for final READ */
					if (i == data_len - 1)
						writel(readl(regs + S3C2410_IICCON) & (~0x80), regs + S3C2410_IICCON);

					xfer_byte();
					result = wait_for_xfer();
					data[i] = readl(regs + S3C2410_IICDS);
					i++;
				}
			} else {
				result = I2C_NACK;
			}

		} else {
			writel(S3C2410_IICSTAT_MASTER_RX | S3C2410_IICSTAT_TXRXEN, regs + S3C2410_IICSTAT);
			writel(chip, regs + S3C2410_IICDS);
			
			/* send START */
			writel(readl(regs + S3C2410_IICSTAT) | S3C2410_IICSTAT_START, regs + S3C2410_IICSTAT);
			result = wait_for_xfer();

			if (is_ack()) {
				i = 0;
				while ((i < data_len) && (result == I2C_OK)) {
					/* disable ACK for final READ */
					if (i == data_len - 1)
						writel(readl(regs + S3C2410_IICCON) & (~0x80), regs + S3C2410_IICCON);

					xfer_byte();
					result = wait_for_xfer();
					data[i] = readl(regs + S3C2410_IICDS);
					i++;
				}
			} else {
				result = I2C_NACK;
			}
		}

		/* send STOP */
		writel(S3C2410_IICSTAT_MASTER_RX | S3C2410_IICSTAT_TXRXEN, regs + S3C2410_IICSTAT);
		xfer_byte();
		break;

	default:
		result = I2C_NOK;
		break;
	}

	return (result);
}

static void i2c_bitbang_write(u8 chip, u32 addr, u16 *data)
{
	u8 xaddr[4];
	
	xaddr[0] = (addr >> 24) & 0xFF;
	xaddr[1] = (addr >> 16) & 0xFF;
	xaddr[2] = (addr >> 8) & 0xFF;
	xaddr[3] = addr & 0xFF;
		
	*data = cpu_to_be16(*data);
	i2c_transfer(I2C_WRITE, chip << 1, &xaddr[4 - 1], 1, (u8 *) data, 2);
}

static void i2c_bitbang_read(u8 chip, u32 addr, u16 *data)
{
	u8 xaddr[4];
	
	xaddr[0] = (addr >> 24) & 0xFF;
	xaddr[1] = (addr >> 16) & 0xFF;
	xaddr[2] = (addr >> 8) & 0xFF;
	xaddr[3] = addr & 0xFF;
	
	i2c_transfer (I2C_READ, chip << 1, &xaddr[4 - 1], 1, (u8 *) data, 2);
	*data = be16_to_cpu(*data);
}

/* Called from arch/arm/mach-s3c6410/include/mach/system.h on restart
	this in turn is called from arch_reset in arch/arm/kernel/process.c */
void bravo_reset(void)
{
	u16 reg;
	u16 val = 0x13;

	i2c_bitbang_init(0x10);
	i2c_bitbang_read(0x1a, 0x03, &reg);
	reg |= (1 << 14);

	i2c_bitbang_write(0x1a, 0xdb, &val);
	i2c_bitbang_write(0x1a, 0x03, &reg);
}

/* Called from arch/arm/kernel/process.c on machine_power_off */
void bravo_power_off(void)
{
	u16 reg;

	i2c_bitbang_init(0x10);
	i2c_bitbang_read(0x1a, 0x03, &reg);
	reg &= ~(0x8000);

	i2c_bitbang_write(0x1a, 0x03, &reg);
}

