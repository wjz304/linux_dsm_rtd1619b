// Copyright (c) 2000-2008 Synology Inc. All rights reserved.

#ifdef CONFIG_SYNO_MPC85XX_COMMON
#include <asm/io.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
extern phys_addr_t get_immrbase(void);
#define SYNO_POWERPC_EUMB_BASE	get_immrbase()
#else
#include <linux/types.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <asm/io.h>
#define MPC10X_MAPB_EUMB_BASE 0xfc000000
#define SYNO_POWERPC_EUMB_BASE	MPC10X_MAPB_EUMB_BASE
#endif
#define SYNO_POWERPC_I2C_OFFSET 0x3000

#define MPC_I2C_ADDR  		0x00
#define MPC_I2C_FDR 		0x04
#define MPC_I2C_CR		0x08
#define MPC_I2C_SR		0x0c
#define MPC_I2C_DR		0x10
#define MPC_I2C_DFSRR 		0x14
#define MPC_I2C_REGION 		0x20

#define CCR_MEN     		0x80
#define CCR_MIEN 		0x40
#define CCR_MSTA 		0x20
#define CCR_MTX  		0x10
#define CCR_TXAK 		0x08
#define CCR_RSTA 		0x04

#define CSR_MCF  		0x80
#define CSR_MAAS 		0x40
#define CSR_MBB  		0x20
#define CSR_MAL  		0x10
#define CSR_SRW  		0x04
#define CSR_MIF  		0x02
#define CSR_RXAK 		0x01

#define	MPC8245_CFG_EUMBBAR		0x78
#define	MPC8245_VENDOR_ID		0x1057
#define	MPC8245_EUMB_I2C_SIZE	0x20

char *i2c_base;
struct semaphore gsemI2c;
//*************************************************************************
//		I2C control for MPC8245
//*************************************************************************


static void writeccr(u32 x)
{
	writel(x, i2c_base + MPC_I2C_CR);
}

static int mpc_i2c_wait(unsigned timeout, int writing)
{
	unsigned long orig_jiffies = jiffies;
	u32 x;
	while (((x = readl(i2c_base + MPC_I2C_SR)) & (CSR_MCF | CSR_MIF)) != (CSR_MCF | CSR_MIF)) {
		schedule();
		if (signal_pending(current)) {
			printk("I2C: Interrupted\n");
			return -EINTR;
		}
		if(time_after(jiffies, orig_jiffies + timeout)){
			printk("I2C: timeout\n");
			return -EIO;
		}
	}

	if (x & CSR_MAL) {
		printk("I2C: MAL\n");
		return -EIO;
	}

	if (writing && (x & CSR_RXAK)) {
		printk("I2C: No RXAK\n");
		/* generate stop */
		writeccr(CCR_MEN);
		return -EIO;
	}
	writel(0, i2c_base + MPC_I2C_SR);
	return 0;
}

static void mpc_i2c_stop(void)
{
	writeccr(CCR_MEN);
}

int mpc_i2c_wait_bus_idle(unsigned int timeout)
{
	unsigned long orig_jiffies = jiffies;
	u32 x;
    
	while (((x = readl(i2c_base + MPC_I2C_SR)) & CSR_MBB) != 0) {
		schedule();
		if (signal_pending(current)) {
			printk("I2C: Interrupted\n");
			return -EINTR;
		}
		if (time_after(jiffies, orig_jiffies + timeout)) {
			printk("I2C: timeout\n");
			return -EIO;
		}
	}
	return 0;
}

int mpc_i2c_write(int target, const u8 *data, int length, int restart, int offset)
{
	int i;
	unsigned timeout = HZ;
	u32 flags = restart ? CCR_RSTA : 0;
	int err = -1;

	down(&gsemI2c);
	if (mpc_i2c_wait_bus_idle(timeout)) {
		goto ABORT;
	}

	/* Start with MEN */
	if (! restart)
		writeccr(CCR_MEN);
	/* Start as master */
	writeccr(CCR_MEN | CCR_MSTA | CCR_MTX | flags);
	/* Write target byte */
	writel((target << 1), i2c_base + MPC_I2C_DR);

	if (mpc_i2c_wait(timeout, 1) < 0) {
		printk("%s(%s)(%d) write target %x time out\n", __FILE__, __FUNCTION__, __LINE__, target);
		goto END;
	}

	if( offset >= 0 ) {
		u8 slave_addr = ((u8)offset) << 4;
		writel(slave_addr, i2c_base + MPC_I2C_DR);
		if (mpc_i2c_wait(timeout, 1) < 0) {
			printk("%s(%s)(%d) write offset %x time out\n", __FILE__, __FUNCTION__, __LINE__, slave_addr);
			goto END;
		}
	}
			
	for(i = 0; i < length; i++) {
		/* Write data byte */
		writel(data[i], i2c_base + MPC_I2C_DR);

		if (mpc_i2c_wait(timeout, 1) < 0) {
			printk("%s(%s)(%d) write byte %d value %x time out\n", __FILE__, __FUNCTION__, __LINE__, i, data[i]);
			goto END;
		}
	}

	err = 0;
END:
	mpc_i2c_stop();
ABORT:
	up(&gsemI2c);
	return err;
}

int mpc_i2c_read(int target, u8 *data, int length, int restart, int offset)
{
	unsigned timeout = HZ;
	int i;
	u32 flags = restart ? CCR_RSTA : 0;
	u32 temp = 0;
	int err = -1;

	down(&gsemI2c);
	if (mpc_i2c_wait_bus_idle(timeout)) {
		goto ABORT;
	}

	/* Start with MEN */
	if (! restart)
		writeccr(CCR_MEN);
	/* Start as master */
	writeccr(CCR_MEN | CCR_MSTA | CCR_MTX | flags);
	/* Write target byte */
	writel((target << 1), i2c_base + MPC_I2C_DR);

	if (mpc_i2c_wait(timeout, 1) < 0) {
		printk("%s(%s)(%d) write target %x timeout\n", __FILE__, __FUNCTION__, __LINE__, target);
		goto END;
	}

	if( offset >= 0 ) {
		u8 slave_addr = ((u8)offset) << 4;
		writel(slave_addr, i2c_base + MPC_I2C_DR);
		if (mpc_i2c_wait(timeout, 1) < 0) {
			printk("%s(%s)(%d) write offset %x timeout\n", __FILE__, __FUNCTION__, __LINE__, slave_addr);
			goto END;
		}
	}

	/* Switch to read - restart */
	writeccr(CCR_MEN | CCR_MSTA | CCR_MTX | CCR_RSTA);
	/* Write  target byte - this time with the read flag set */
	writel((target<<1) | 1, i2c_base + MPC_I2C_DR);

	if (mpc_i2c_wait(timeout, 0) < 0) {
		printk("%s(%s)(%d) write target %x timeout\n", __FILE__, __FUNCTION__, __LINE__, target);
		goto END;
	}

	if (length == 1)
		writeccr(CCR_MEN | CCR_MSTA | CCR_TXAK);
	else
		writeccr(CCR_MEN | CCR_MSTA);
		/* Dummy read */
	readl(i2c_base + MPC_I2C_DR);

	for(i = 0; i < length; i++) {
		if (mpc_i2c_wait(timeout, 0) < 0) {
			printk("%s(%s)(%d) read byte %d timeout\n", __FILE__, __FUNCTION__, __LINE__, i);
			goto END;
		}

		/* Generate stop on last byte */
		if (i == length - 1)
			writeccr(CCR_MEN | CCR_TXAK);
		//data[i] = readl(i2c_base + MPC_I2C_DR);
		temp = readl(i2c_base + MPC_I2C_DR);
		data[i] = (u8)temp;
	}
	err = 0;
END: 
	mpc_i2c_stop();
ABORT:
	up(&gsemI2c);
	return length;
}

int mpc_i2c_init(void)
{
#ifdef CONFIG_SYNO_MPC85XX_COMMON
	u32 x;
#endif

	i2c_base = (unsigned char __iomem *)(SYNO_POWERPC_EUMB_BASE + SYNO_POWERPC_I2C_OFFSET);
	sema_init(&gsemI2c, 1);

#ifdef CONFIG_SYNO_MPC85XX_COMMON
#ifdef CONFIG_SYNO_MPC854X
	// ((CCB/2)/1920) ~= 100 KHz (CPU_MPC8548ERM.pdf p.531)
	// initialize i2c frequency by set register I2CFDR
	x = 0x0B;
#elif CONFIG_SYNO_MPC8533
	// ((CCB/3)/1152) = (400/3)/1152 ~= 100 KHz (MPC8533ERM.pdf p.11-7)
	x = 0x08;
#endif
	writel(x, i2c_base + MPC_I2C_FDR);
#endif

	return 0;
}


