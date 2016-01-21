#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
//#include <linux/waitqueue.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/can.h>
#include <linux/can/core.h>
#include <linux/can/dev.h>
#include "mcp2515.h"


static dev_t devno;
static int mcp2515_major = MCP2515_MAJOR;
static int mcp2515_minor = MCP2515_MINOR;


typedef enum{
	BandRate_10kbps,
	BandRate_20kbps,
	BandRate_40kbps,
	BandRate_80kbps,
	BandRate_125kbps,
	BandRate_250kbps,
	BandRate_500kbps,
}CanBandRate;


struct mcp2515_chip {
	canid_t own_id;		// CAN ID
	canid_t broadcast_id;	// Broadcast ID
	CanBandRate bandrate;	// 波特率

	struct cdev cdev;	
	struct spi_device *spi;
	struct class *class;

	struct work_struct irq_work;

	uint32_t count;

	uint8_t *spi_transfer_buf;

	struct can_frame spi_tx_buf[MCP2515_BUF_LEN];
	struct can_frame spi_rx_buf[MCP2515_BUF_LEN];

	uint32_t rxbin;
	uint32_t rxbout;
	uint32_t txbin;
	uint32_t txbout;

	wait_queue_head_t rwq;
};


static uint8_t mcp2515_read_state(struct spi_device *spi, uint8_t cmd)
{
	uint8_t tx_buf[2], rx_buf[1] = {0};
	uint8_t val;
	int ret;

	tx_buf[0] = cmd;

	ret = spi_write_then_read(spi, tx_buf, 1, rx_buf, 1);
	if (ret < 0) {
		val = 0;
		printk("MCP2515: Read register error!\n");
	} else
		val = rx_buf[0];

	return val;
}


static uint8_t mcp2515_read_reg(struct spi_device *spi, uint8_t reg)
{
	uint8_t tx_buf[2], rx_buf[1] = {0};
	uint8_t val;
	int ret;

	tx_buf[0] = INSTRUCTION_READ;
	tx_buf[1] = reg;

	ret = spi_write_then_read(spi, tx_buf, 2, rx_buf, 1);
	if (ret < 0) {
		val = 0;
		printk("MCP2515: Read register error!\n");
	} else
		val = rx_buf[0];

	return val;
}


static void mcp2515_write_reg(struct spi_device *spi, uint8_t reg, uint8_t val)
{
	uint8_t tx_buf[3];
	int ret;

	tx_buf[0] = INSTRUCTION_WRITE;
	tx_buf[1] = reg;
	tx_buf[2] = val;

	ret = spi_write(spi, tx_buf, 3);
	if (ret < 0)
		printk("MCP2515: Write register error!\n");
}


static void mcp2515_write_bits(struct spi_device *spi, uint8_t reg, uint8_t mask, uint8_t val)
{
	uint8_t tx_buf[4];
	int ret;

	tx_buf[0] = INSTRUCTION_BIT_MODIFY;
	tx_buf[1] = reg;
	tx_buf[2] = mask;
	tx_buf[3] = val;

	ret = spi_write(spi, tx_buf, 4);
	if (ret < 0)
		printk("MCP2515: mcp2515_write_bit error!\n");
}


static void mcp2515_write_can_id(struct spi_device *spi, uint32_t address, canid_t can_id,
		uint8_t ext, uint8_t add_reg)
{
	canid_t tbufdata;
	uint8_t canctrl;
	int i;

	if (ext) {
		can_id &= 0x1fffffff;

		/*							MCP2515 EXTENDED DATA FRAME						*/
		/* 					7 - 0								9 - 8				*/
		/* 					 11								   15 - 13				*/
		/* 				   23 - 16							   31 - 24				*/

		tbufdata = 	((can_id & 0x1fe00000) >> 21) 	| ((can_id & 0x00030000) >> 8)	|	 
					((0x1) << 11) 					| ((can_id & 0x001c0000) >> 5) 	| 
					((can_id & 0x0000ff00) << 8) 	| ((can_id & 0x000000ff) << 24)	;

	} else {
		can_id &= 0x7ff;
		tbufdata = ((can_id >> 3) | ((can_id & 0x7) << 13)) & 0xf7ff;  
	}

	if (add_reg) {		//add_reg = 1 write address
		memcpy((canid_t *)address, &tbufdata, sizeof(canid_t));
	} else {				//add_reg = 0 write reg

		canctrl = mcp2515_read_reg(spi, CANCTRL);
		mcp2515_write_bits(spi, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_CONF);

		for (i = 0; i < 4; i++)	{

			mcp2515_write_reg(spi, address + i, (tbufdata >> (i*8)) & 0xFF);
			mdelay(1);
			//printk("%d = %#x\n", i, mcp2515_read_reg(spi, address + i));
		}

		mcp2515_write_reg(spi, CANCTRL, canctrl);
		mcp2515_write_bits(spi, RXBCTRL(0), 1 << 6, 1 << 6);
		mcp2515_write_bits(spi, RXBCTRL(1), 1 << 6, 1 << 6);
	}
}


static void mcp2515_write_can_bandrate(struct spi_device *spi)
{
	struct mcp2515_chip *chip = dev_get_drvdata(&spi->dev);
	uint8_t cnf1, cnf2, cnf3;

	cnf1 = cnf2 = cnf3 = 0;

	switch(chip->bandrate) {
	case BandRate_500kbps:
		cnf1 = 0x00; cnf2 = 0x91; cnf3 = 0x01;
		break;
	case BandRate_250kbps:
		cnf1 = 0x00; cnf2 = 0x96; cnf3 = 0x04;
		break;
	case BandRate_125kbps:
		cnf1 = 0x00; cnf2 = 0x97; cnf3 = 0x03;
		break;
	case BandRate_80kbps:
		cnf1 = 0x01; cnf2 = 0xBF; cnf3 = 0x07;
		break;
	case BandRate_40kbps:
		cnf1 = 0x04; cnf2 = 0xB6; cnf3 = 0x04;
		break;
	case BandRate_20kbps:
		cnf1 = 0x09; cnf2 = 0xBD; cnf3 = 0x04;
		break;
	case BandRate_10kbps:
		cnf1 = 0x13; cnf2 = 0xBD; cnf3 = 0x04;
		break;
	}

	mcp2515_write_reg(spi, CNF1, cnf1);
	mdelay(1);
	mcp2515_write_reg(spi, CNF2, cnf2);
	mdelay(1);
	mcp2515_write_reg(spi, CNF3, cnf3);
	mdelay(1);
}


static void mcp2515_tx(struct spi_device *spi, int pos)
{
	struct mcp2515_chip *chip = dev_get_drvdata(&spi->dev);
	struct can_frame *frame;
	uint8_t *tx_buf;
	int ret;

	tx_buf = chip->spi_transfer_buf;

	if (chip->txbout != chip->txbin) {

		frame = &chip->spi_tx_buf[chip->txbout];

		if (frame->can_dlc > CAN_FRAME_MAX_DATA_LEN)
			frame->can_dlc = CAN_FRAME_MAX_DATA_LEN;

		tx_buf[0] = INSTRUCTION_LOAD_TXB(pos);
		mcp2515_write_can_id(spi, (uint32_t)tx_buf+1, frame->can_id, 1, 1);
		tx_buf[5] = frame->can_dlc & 0xF & ~(1 << 6);
		memcpy(tx_buf + 6, frame->data, frame->can_dlc);

		ret = spi_write(spi, tx_buf, SPI_TRANSFER_BUF_LEN);
		if (ret < 0)
			printk("MCP2515: MCP2515 transfer failed!\n");

		chip->txbout++;
		if (chip->txbout >= MCP2515_BUF_LEN)
			chip->txbout = 0;

		mcp2515_write_reg(spi, TXBCTRL(pos), TXBCTRL_TXREQ);
	}

	return ;
}


static void mcp2515_rx(struct spi_device *spi, int pos)
{
	struct mcp2515_chip *chip = dev_get_drvdata(&spi->dev);
	struct can_frame *frame;
	uint8_t *rx_buf;
	uint8_t tx_buf[2] = {0};
	int ret;

	rx_buf = chip->spi_transfer_buf;
	memset(rx_buf, 0, SPI_TRANSFER_BUF_LEN);

	frame = &chip->spi_rx_buf[chip->rxbin];

	tx_buf[0] = INSTRUCTION_READ_RXB(pos);

	ret = spi_write_then_read(spi, tx_buf, 1, rx_buf, SPI_TRANSFER_BUF_LEN - 1);
	if (ret < 0)
		printk("MCP2515: MCP2515 receive data error!\n");

	frame->can_id = (rx_buf[0] << 24) | (rx_buf[1] << 16) | (rx_buf[2] << 8) | rx_buf[3];
	frame->can_dlc = rx_buf[4] & 0x0F;
	memcpy(frame->data, rx_buf + 5, frame->can_dlc);

	chip->rxbin++;
	if (chip->rxbin >= MCP2515_BUF_LEN)
		chip->rxbin = 0;

	return ;
}


static int mcp2515_hw_reset(struct spi_device *spi)
{
	uint8_t tx_buf[1];
	int st1 = 0, st2 = 0;
	int ret;

	tx_buf[0] = INSTRUCTION_RESET;

	ret = spi_write(spi, tx_buf, 1);
	if (ret < 0)
		printk("MCP2515: mcp2515_reset error!\n");

	mdelay(10);

	st1 = mcp2515_read_reg(spi, CANSTAT) & 0xEE;
	st2 = mcp2515_read_reg(spi, CANCTRL) & 0x17;

	return (st1 == 0x80 && st2 == 0x07) ? 1 : 0;
}


static void mcp2515_hw_sleep(struct spi_device *spi)
{
	mcp2515_write_reg(spi, CANCTRL, CANCTRL_REQOP_SLEEP);
}


static void mcp2515_hw_wakeup(struct spi_device *spi)
{
	mcp2515_write_bits(spi, CANINTE, CANINTE_WAKIE, CANINTE_WAKIE);
	mcp2515_write_bits(spi, CANINTF, CANINTF_WAKIF, CANINTF_WAKIF);
	mdelay(1);
}

static irqreturn_t mcp2515_irq(int irq, void *dev_id)
{
	struct spi_device *spi = dev_id;
	struct mcp2515_chip *chip = dev_get_drvdata(&spi->dev);

	schedule_work(&chip->irq_work);

	return IRQ_HANDLED;
}


static void mcp2515_irq_handler(struct work_struct *work)
{
	struct mcp2515_chip *chip = container_of(work, struct mcp2515_chip, irq_work);
	struct spi_device *spi = chip->spi;
	int intf;

	while (1) {

		intf = mcp2515_read_reg(spi, CANINTF);
		if (!intf)
			break;

		//if (intf & CANINTF_WAKIF)
		if (intf & CANINTF_MERRF)
			mcp2515_hw_reset(spi);
		if (intf & CANINTF_ERRIF)
			mcp2515_write_reg(spi, EFLG, 0x00);
		if (intf & CANINTF_TX2IF)
			mcp2515_tx(spi, 2);
		if (intf & CANINTF_TX1IF)
			mcp2515_tx(spi, 1);
		if (intf & CANINTF_TX0IF)
			mcp2515_tx(spi, 0);
		if (intf & CANINTF_RX1IF)
			mcp2515_rx(spi, 1);
		if (intf & CANINTF_RX0IF)
			mcp2515_rx(spi, 0);

		mcp2515_write_bits(spi, CANINTF, intf, 0x00);	// Clean CANINTF bits.

		if (chip->rxbin != chip->rxbout)
			wake_up_interruptible(&chip->rwq);
	}
}


static int mcp2515_open (struct inode *inode, struct file *filp)
{
	struct mcp2515_chip *chip = container_of(inode->i_cdev, struct mcp2515_chip, cdev);
	struct spi_device *spi = chip->spi;
	int i;

	filp->private_data = chip;

	mcp2515_hw_wakeup(spi);
	mcp2515_hw_reset(spi);

	mcp2515_write_reg(spi, CANINTE,
			  CANINTE_ERRIE | CANINTE_TX2IE
			| CANINTE_TX1IE | CANINTE_TX0IE
			| CANINTE_RX1IE | CANINTE_RX0IE);

	mcp2515_write_reg(spi, CANCTRL, CANCTRL_REQOP_CONF);

	for(i = 0; i < 2; i++) {
		mcp2515_write_reg(spi, RXMSIDH(i), 0xFF);
		mdelay(1);
		mcp2515_write_reg(spi, RXMSIDL(i), 0xE3);
		mdelay(1);
		mcp2515_write_reg(spi, RXMEID8(i), 0xFF);
		mdelay(1);
		mcp2515_write_reg(spi, RXMEID0(i), 0xFF);
		mdelay(1);
	}

	mcp2515_write_can_bandrate(spi);

	/* Set Normal mode, open ACK tick */
	mcp2515_write_reg(spi, CANCTRL, CANCTRL_REQOP_NORMAL | (1 << 2));

	/* If RXB0 if full, RXB0 message will rollover and be written to RXB1. */
	//mcp2515_write_reg(spi, RXBCTRL(0), RXBCTRL_BUKT);		


	chip->count ++;

	return 0;
}


static int mcp2515_release (struct inode *inode, struct file *filp)
{
	struct mcp2515_chip *chip = container_of(inode->i_cdev, struct mcp2515_chip, cdev);
	struct spi_device *spi = chip->spi;

	chip->count --;
	//if (chip->count)
	//	return 0;

	/* Clear pendind interrupts */
	mcp2515_write_reg(spi, CANINTE, 0x00);
	mcp2515_write_reg(spi, CANINTF, 0x00);

	/* Go to sleep */
	mcp2515_hw_sleep(spi);

	//printk("MCP2515: MCP2515 release OK!\n");

	return 0;
}


static void mcp2515_set_id(struct spi_device *spi, canid_t id, int8_t flag)
{
	struct mcp2515_chip *chip = dev_get_drvdata(&spi->dev);

	if (flag == Own_ID) {
		mcp2515_write_can_id(spi, (uint32_t)&chip->own_id, id, 1, 1);
		mcp2515_write_can_id(spi, RXFSIDH(0), id, 1, 0);
		mcp2515_write_can_id(spi, RXFSIDH(3), id, 1, 0);
		mcp2515_write_can_id(spi, RXFSIDH(4), id, 1, 0);
		mcp2515_write_can_id(spi, RXFSIDH(5), id, 1, 0);
	}

	if (flag == Broadcast_ID) {
		mcp2515_write_can_id(spi, (uint32_t)&chip->broadcast_id, id, 1, 1);
		mcp2515_write_can_id(spi, RXFSIDH(1), id, 1, 0);
		mcp2515_write_can_id(spi, RXFSIDH(2), id, 1, 0);
	}
}


static int mcp2515_set_bandrate(struct spi_device *spi, int bandrate)
{
	struct mcp2515_chip *chip = dev_get_drvdata(&spi->dev);

	switch(bandrate) {
	case 500:
		chip->bandrate = BandRate_500kbps;
		break;
	case 250:
		chip->bandrate = BandRate_250kbps;
		break;
	case 125:
		chip->bandrate = BandRate_125kbps;
		break;
	case 80:
		chip->bandrate = BandRate_80kbps;
		break;
	case 40:
		chip->bandrate = BandRate_40kbps;
		break;
	case 20:
		chip->bandrate = BandRate_20kbps;
		break;
	case 10:
		chip->bandrate = BandRate_10kbps;
		break;
	default:
		return -1;
	}

	mcp2515_write_can_bandrate(spi);

	return 0;
}


static long mcp2515_unlocked_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct mcp2515_chip *chip = filp->private_data;
	struct spi_device *spi = chip->spi;
	int ret = 0;

	switch(cmd) {
	case CAN_Set_Own_ID:
		mcp2515_set_id(spi, (canid_t)arg, Own_ID);
		break;
	case CAN_Set_Broadcast_ID:
		mcp2515_set_id(spi, (canid_t)arg, Broadcast_ID);
		break;
	case CAN_Set_Bandrate:
		ret = mcp2515_set_bandrate(spi, arg);
		break;
	default:
		break;
	}
	return ret;
}


static ssize_t mcp2515_read(struct file *filp, char __user *buf, size_t count, loff_t * lof)
{
	struct mcp2515_chip *chip = filp->private_data;
	int nbytes = 0;
	int ret;
	struct can_frame *frame;

	if(count != sizeof(struct can_frame))
		return -EINVAL;

	//printk("rxbin = %d, rxbout = %d\n", chip->rxbin, chip->rxbout);

	while (chip->rxbin == chip->rxbout) {

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(chip->rwq, (chip->rxbin != chip->rxbout)))
			return -ERESTARTSYS;
	}

	frame = &chip->spi_rx_buf[chip->rxbout];

	ret = copy_to_user(buf, frame, sizeof(struct can_frame));
	if (0 != ret)
		return -EFAULT;

	chip->rxbout++;
	if (chip->rxbout >= MCP2515_BUF_LEN)
		chip->rxbout = 0;

	nbytes = frame->can_dlc & 0x0F;

	return nbytes;
}


static ssize_t mcp2515_write (struct file *filp, const char __user *buf, size_t count, loff_t *lof)
{
	struct mcp2515_chip *chip = filp->private_data;
	struct spi_device *spi = chip->spi;
	struct can_frame *frame;
	int txreq;
	int nbytes = 0;
	int ret;
	
	if (count != sizeof(struct can_frame))
		return -EINVAL;

	frame = &chip->spi_tx_buf[chip->txbin];

	ret = copy_from_user(frame, buf, sizeof(struct can_frame));
	if (0 != ret)
		return -EFAULT;

	chip->txbin++;
	if (chip->txbin >= MCP2515_BUF_LEN)
		chip->txbin = 0;

	nbytes = frame->can_dlc & 0x0F;
		
	txreq = mcp2515_read_state(spi, INSTRUCTION_CAN_STATE);

	if (!(txreq & CAN_STATE_TX0REQ))
		mcp2515_tx(spi, 0);

	if (!(txreq & CAN_STATE_TX1REQ))
		mcp2515_tx(spi, 1);

	if (!(txreq & CAN_STATE_TX2REQ))
		mcp2515_tx(spi, 2);

	return nbytes;
}


static struct file_operations mcp2515_fops = {
	.open			= mcp2515_open,
	.release		= mcp2515_release,
	.write			= mcp2515_write,
	.read			= mcp2515_read,
	.unlocked_ioctl	= mcp2515_unlocked_ioctl,
};


static int __devinit mcp2515_probe(struct spi_device *spi)
{
	struct mcp2515_chip *chip;
	int ret;

	chip = kmalloc(sizeof(struct mcp2515_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto error_alloc;
	}

	dev_set_drvdata(&spi->dev, chip);

	chip->spi = spi;
	chip->count = 0;
	chip->txbin = 0;
	chip->rxbin = 0;
	chip->txbout = 0;
	chip->rxbout = 0;
	chip->own_id = 0;
	chip->broadcast_id = 0;
	chip->bandrate = BandRate_125kbps;

	INIT_WORK(&chip->irq_work, mcp2515_irq_handler);

    ret = request_irq(IRQ_EINT(1), mcp2515_irq, IRQF_DISABLED | IRQF_TRIGGER_FALLING, DEVICE_NAME, spi);
	if (ret < 0) {
		printk("MCP2515: Request_irq() Error!\n");
		goto error_irq;
	}

	init_waitqueue_head(&chip->rwq);

	chip->spi_transfer_buf = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
	if (!chip->spi_transfer_buf) {
		ret = -ENOMEM;
		goto error_buf;
	}

	if (mcp2515_minor > MCP2515_MAX_DEV)
		goto error_register;

	if (mcp2515_major) {
		devno = MKDEV(mcp2515_major, ++mcp2515_minor);
		ret = register_chrdev_region(devno, 0, DEVICE_NAME);
	} else {
		ret = alloc_chrdev_region(&devno, mcp2515_major, 0, DEVICE_NAME);
		mcp2515_major = MAJOR(devno);
	}
	
	if (ret < 0) {
		printk("MCP2515: Register char device region (%d: %d) failed! (ret = %d)\n", MAJOR(devno), MINOR(devno), ret);
		goto error_register;
	}

	cdev_init(&chip->cdev, &mcp2515_fops);
	chip->cdev.owner = THIS_MODULE;
	ret = cdev_add(&chip->cdev, devno, 1);
	if (ret < 0) {
		printk("MCP2515: Register char device failed! (ret = %d)\n", ret);
		goto error_devadd;
	}

	chip->class = class_create(THIS_MODULE, "mcp2515_class");
	if (IS_ERR(chip->class)) {
		printk("MCP2515: Failed in creating class.\n");
		goto error_class_reg;
	}
	device_create(chip->class, NULL, devno, NULL, DEVICE_NAME);

	ret = mcp2515_hw_reset(spi);
	if (!ret)
		goto error_reset;

	printk ("MCP2515: MCP2515 Can Device Driver.\n");

	return 0;

error_reset:
error_class_reg:  
		cdev_del(&chip->cdev);  
error_devadd:
		unregister_chrdev_region(devno, 0);
error_register:
		free_irq(spi->irq, spi);
error_buf:
		kfree(chip);
error_irq:
error_alloc:
		return ret;
}


static int __devexit mcp2515_remove(struct spi_device *spi)
{
	struct mcp2515_chip *chip = dev_get_drvdata(&spi->dev);
	
	device_destroy(chip->class, devno);
	class_destroy(chip->class);

    unregister_chrdev_region(devno, 1);

	cdev_del(&chip->cdev);

	free_irq(IRQ_EINT(1), spi);

	kfree(chip->spi_transfer_buf);

	//kfree(chip);

	return 0;
}


static int mcp2515_suspend(struct spi_device *spi, pm_message_t mesg)
{
	mcp2515_hw_sleep(spi);

	return 0;
}


static int mcp2515_resume(struct spi_device *spi)
{
	mcp2515_hw_wakeup(spi);

	return 0;
}


static struct spi_driver mcp2515_driver = {
	.driver = {
		.name	= DEVICE_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},

	.probe		= mcp2515_probe,
	.remove		= __devexit_p(mcp2515_remove),
	.suspend	= mcp2515_suspend,
	.resume		= mcp2515_resume,
};


static int __init mcp2515_can_init(void)
{
	return spi_register_driver(&mcp2515_driver);
}


static void __exit mcp2515_can_exit(void)
{
	spi_unregister_driver(&mcp2515_driver);
}


module_init(mcp2515_can_init);
module_exit(mcp2515_can_exit);

MODULE_DESCRIPTION("MCP2515 Can Controller Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lee Sheen");
