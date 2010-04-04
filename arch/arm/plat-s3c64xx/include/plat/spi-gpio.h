#ifndef __ASM_ARCH_SPIGPIO_H
#define __ASM_ARCH_SPIGPIO_H __FILE__

struct s3c64xx_spigpio_info {
	unsigned long pin_clk;
    unsigned long pin_mosi;
    unsigned long pin_miso;
	unsigned long pin_cs;
	int bus_num;
};

#endif /* __ASM_ARCH_SPIGPIO_H */

