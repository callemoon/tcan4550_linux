// SPDX-License-Identifier: GPL-2.0
// CAN driver for TI TCAN4550
// No CAN FD support
// Copyright (C) 2023 Carl-Magnus Moon

#include <linux/bitfield.h>
#include <linux/can/dev.h>
#include <linux/ethtool.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <linux/gpio/consumer.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/interrupt.h>

#include <linux/gpio.h> 

#include "tcan4550.h"

// Registers
const static uint32_t DEVICE_ID1 = 0x0;
const static uint32_t DEVICE_ID2 = 0x4;

const static uint32_t STATUS = 0x0C;
const static uint32_t SPI_MASK = 0x10;

const static uint32_t MODES_OF_OPERATION = 0x0800;
const static uint32_t INTERRUPT_FLAGS = 0x0820;
const static uint32_t INTERRUPT_ENABLE = 0x0830;

const static uint32_t CCCR = 0x1018;
const static uint32_t NBTP = 0x101C;
const static uint32_t RXF0C = 0x10A0;
const static uint32_t RXF0S = 0x10A4;
const static uint32_t RXF0A = 0x10A8;
const static uint32_t TXBC = 0x10C0;
const static uint32_t TXESC = 0x10C8;
const static uint32_t RXESC = 0x10BC;
const static uint32_t TXQFS = 0x10C4;
const static uint32_t TXBAR = 0x10D0;

const static uint32_t IR = 0x1050;  // interrupt register
const static uint32_t IE = 0x1054;  // interrupt enable
const static uint32_t ILE = 0x105C; // interrupt line enable

const static uint32_t RF0N = (0x1UL << 0);   // rx fifo 0 new data
const static uint32_t TC = (0x1UL << 9);  // transmission complete
const static uint32_t TFE = (0x1UL << 11);    // transmit fifo empty
const static uint32_t EW = (0x1UL << 24);    // error warning
const static uint32_t BO = (0x1UL << 25);    // bus off

const static uint32_t INIT = 0x1;   // init
const static uint32_t CCE = 0x2;    // configuration change enable
const static uint32_t CSR = 0x10;   // clock stop request

const static uint32_t MODESEL_1 = 0x40;
const static uint32_t MODESEL_2 = 0x80;


// MRAM config
const static uint32_t RX_SLOT_SIZE = 16;
const static uint32_t TX_SLOT_SIZE = 16;
const static uint32_t TX_MSG_BOXES = 16;
const static uint32_t TX_FIFO_START_ADDRESS = 0x0;
const static uint32_t RX_MSG_BOXES = 16;
const static uint32_t RX_FIFO_START_ADDRESS = 0x200;

const static uint32_t MRAM_BASE = 0x8000;

// GPIO
const static uint32_t GPIO_RESET = 21;  // GPIO pin for chip reset

struct tcan4550_priv
{
    struct can_priv can;    // must be located first in private struct
    struct device *dev;
    struct net_device *ndev;
    struct spi_device *spi;

    struct mutex spi_lock; /* SPI device lock */
};

static struct spi_device *spi = 0;  // global spi handle

// tcan function headers
static int tcan4550_sendMsg(struct canfd_frame *msg, uint32_t *index);
static void tcan4550_set_normal_mode(void);
static void tcan4550_set_standby_mode(void);
static void tcan4550_configure_mram(void);
static void tcan4550_unlock(void);
static bool tcan4550_readIdentification(void);
static bool tcan4550_setBitRate(uint32_t bitRate);
static int tcan4550_setupInterrupts(struct net_device *dev);
static void tcan4550_hwReset(void);
static void tcan4550_setupIo(void);
static void tcan4550_freeIo(void);
static irqreturn_t tcan4450_handleInterrupts(int irq, void *dev);

// spi function headers
static uint32_t spi_read32(uint32_t address);
static int spi_write32(uint32_t address, uint32_t data);
static int spi_read128(uint32_t address, uint32_t data[4]);
static int spi_write128(uint32_t address, uint32_t data[4]);

static const struct can_bittiming_const tcan_bittiming_const = {
    .name = KBUILD_MODNAME,
    .tseg1_min = 1,
    .tseg1_max = 255,
    .tseg2_min = 1,
    .tseg2_max = 127,
    .sjw_max = 127,
    .brp_min = 1,
    .brp_max = 511,
    .brp_inc = 1,
};

static int tcan4550_spi_trans(int len, unsigned char *rxBuf, unsigned char *txBuf)
{
//    struct tcan4550_priv *priv;
//    priv = spi_get_drvdata(spi);
    struct spi_transfer t = {
        .tx_buf = txBuf,
        .rx_buf = rxBuf,
        .len = len,
        .cs_change = 0,
    };
    struct spi_message m;
    int ret;

    if (spi == 0)
    {
        return 0;
    }

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    ret = spi_sync(spi, &m);
    if (ret)
    {
        dev_err(&spi->dev, "spi transfer failed: ret = %d\n", ret);
    }

    return ret;
}

static uint32_t spi_read32(uint32_t address)
{
    unsigned char txBuf[8];
    unsigned char rxBuf[8];

    int ret;

    txBuf[0] = 0x41;
    txBuf[1] = address >> 8;
    txBuf[2] = address & 0xFF;
    txBuf[3] = 1;

    ret = tcan4550_spi_trans(8, rxBuf, txBuf);

    return (rxBuf[4] << 24) + (rxBuf[5] << 16) + (rxBuf[6] << 8) + rxBuf[7];
}

static int spi_read128(uint32_t address, uint32_t data[4])
{
    unsigned char txBuf[20];
    unsigned char rxBuf[20];

    int ret;

    txBuf[0] = 0x41;
    txBuf[1] = address >> 8;
    txBuf[2] = address & 0xFF;
    txBuf[3] = 4;

    ret = tcan4550_spi_trans(20, rxBuf, txBuf);

    data[0] = rxBuf[7] + (rxBuf[6] << 8) + (rxBuf[5] << 16) + (rxBuf[4] << 24);
    data[1] = rxBuf[11] + (rxBuf[10] << 8) + (rxBuf[9] << 16) + (rxBuf[8] << 24);
    data[2] = rxBuf[15] + (rxBuf[14] << 8) + (rxBuf[13] << 16) + (rxBuf[12] << 24);
    data[3] = rxBuf[19] + (rxBuf[18] << 8) + (rxBuf[17] << 16) + (rxBuf[16] << 24);

    return ret;
}

static int spi_write32(uint32_t address, uint32_t data)
{
    unsigned char txBuf[8];
    unsigned char rxBuf[8];

    int ret;

    txBuf[0] = 0x61;
    txBuf[1] = address >> 8;
    txBuf[2] = address & 0xFF;
    txBuf[3] = 1;
    txBuf[4] = (data >> 24) & 0xFF;
    txBuf[5] = (data >> 16) & 0xFF;
    txBuf[6] = (data >> 8) & 0xFF;
    txBuf[7] = data & 0xFF;

    ret = tcan4550_spi_trans(8, rxBuf, txBuf);

    return ret;
}

static int spi_write128(uint32_t address, uint32_t data[4])
{
    unsigned char txBuf[20];
    unsigned char rxBuf[20];

    int ret;

    txBuf[0] = 0x61;
    txBuf[1] = address >> 8;
    txBuf[2] = address & 0xFF;
    txBuf[3] = 4;
    txBuf[4] = (data[0] >> 24) & 0xFF;
    txBuf[5] = (data[0] >> 16) & 0xFF;
    txBuf[6] = (data[0] >> 8) & 0xFF;
    txBuf[7] = data[0] & 0xFF;
    txBuf[8] = (data[1] >> 24) & 0xFF;
    txBuf[9] = (data[1] >> 16) & 0xFF;
    txBuf[10] = (data[1] >> 8) & 0xFF;
    txBuf[11] = data[1] & 0xFF;
    txBuf[12] = (data[2] >> 24) & 0xFF;
    txBuf[13] = (data[2] >> 16) & 0xFF;
    txBuf[14] = (data[2] >> 8) & 0xFF;
    txBuf[15] = data[2] & 0xFF;
    txBuf[16] = (data[3] >> 24) & 0xFF;
    txBuf[17] = (data[3] >> 16) & 0xFF;
    txBuf[18] = (data[3] >> 8) & 0xFF;
    txBuf[19] = data[3] & 0xFF;

    ret = tcan4550_spi_trans(20, rxBuf, txBuf);

    return ret;
}

static void tcan4550_set_standby_mode(void)
{
    uint32_t val;

    val = spi_read32(MODES_OF_OPERATION);

    val |= MODESEL_1;
    val &= ~((uint32_t)MODESEL_2);

    spi_write32(MODES_OF_OPERATION, val);
}

static void tcan4550_set_normal_mode(void)
{
    uint32_t val;

    val = spi_read32(MODES_OF_OPERATION);

    val |= MODESEL_2;
    val &= ~((uint32_t)MODESEL_1);

    spi_write32(MODES_OF_OPERATION, val);
}

static bool tcan4550_readIdentification(void)
{
    uint32_t id1;
    uint32_t id2;

    id1 = spi_read32(DEVICE_ID1);
    id2 = spi_read32(DEVICE_ID2);

    // TCAN4550 in ascii
    if ((id1 == 0x4E414354) && (id2 == 0x30353534))
    {
        return true;
    }

    return false;
}

static bool tcan4550_setBitRate(uint32_t bitRate)
{
    spi_write32(NBTP, bitRate);

    return true;
}

static void tcan4550_configure_mram()
{
    // clear MRAM to avoid risk of ECC errors 2kB = 512 words
    for (uint32_t i = 0; i < 512; i++)
    {
        spi_write32(MRAM_BASE + (i * 4), 0);
    }

    // configure tx-fifo
    spi_write32(TXBC, TX_FIFO_START_ADDRESS + (TX_MSG_BOXES << 24));

    // configure rx-fifo
    spi_write32(RXF0C, RX_FIFO_START_ADDRESS + (RX_MSG_BOXES << 16));

    // 8-byte transmit size
    spi_write32(TXESC, 0);

    // 8-byte receive size
    spi_write32(RXESC, 0);
}

static void tcan4550_unlock()
{
    uint32_t val = spi_read32(CCCR);

    val |= (CCE + INIT);		  // set CCE and INIT bits
    val &= ~((uint32_t)CSR); // clear CSR

    spi_write32(CCCR, val);
}

static int tcan4550_sendMsg(struct canfd_frame *msg, uint32_t *index)
{
    // check for free buffers
    uint32_t txqfs = spi_read32(TXQFS);
    uint32_t freeBuffers = txqfs & 0x3F;
    uint32_t writeIndex = (txqfs >> 16) & 0x1F;

    if (freeBuffers > 0)
    {
        uint32_t baseAddress = MRAM_BASE + TX_FIFO_START_ADDRESS + (writeIndex * TX_SLOT_SIZE);
        uint32_t buffer[4];

        buffer[0] = (msg->can_id << 18);
        buffer[1] = (msg->len << 16);
        buffer[2] = msg->data[0] + (msg->data[1] << 8) + (msg->data[2] << 16) + (msg->data[3] << 24);
        buffer[3] = msg->data[4] + (msg->data[5] << 8) + (msg->data[6] << 16) + (msg->data[7] << 24);

        spi_write128(baseAddress, buffer);  // write message id,len and data

        spi_write32(TXBAR, (1 << writeIndex)); // request buffer transmission
    }

    return freeBuffers;
}

bool tcan4550_recMsg(struct canfd_frame *msg)
{
	uint32_t rxf0s = spi_read32(RXF0S);

	uint32_t fillLevel = rxf0s & 0xFF;
	uint32_t getIndex = (rxf0s >> 8) & 0xFF;
	//uint32_t putIndex = (rxf0s >> 16) & 0xFF;

	if(fillLevel > 0)
	{
		uint32_t data[4];
		uint32_t baseAddress = MRAM_BASE + RX_FIFO_START_ADDRESS + (getIndex * RX_SLOT_SIZE);

		spi_read128(baseAddress, data);

		spi_write32(RXF0A, getIndex);	// acknowledge that we have read the message

		msg->can_id = data[0] >> 18;
		msg->len = (data[1] >> 16) & 0x7F;

		msg->data[0] = data[2] & 0xFF;
		msg->data[1] = (data[2] >> 8) & 0xFF;
		msg->data[2] = (data[2] >> 16) & 0xFF;
		msg->data[3] = (data[2] >> 24) & 0xFF;
		msg->data[4] = data[3] & 0xFF;
		msg->data[5] = (data[3] >> 8) & 0xFF;
		msg->data[6] = (data[3] >> 16) & 0xFF;
		msg->data[7] = (data[3] >> 24) & 0xFF;

		return true;
	}

	return false;
}

static irqreturn_t tcan4450_handleInterrupts(int irq, void *dev)
{
    struct tcan4550_priv *priv = netdev_priv(dev);
    struct net_device_stats *stats = &((struct net_device *)dev)->stats;
    uint32_t ir;
   
    mutex_lock(&priv->spi_lock);

    ir = spi_read32(IR);
    spi_write32(IR, ir);    // acknowledge interrupts

//    if(ir == 0)
//    {
//        return IRQ_NONE;
//    }

    //spi_write32(STATUS, 0xFFFFFFFF);
    //spi_write32(INTERRUPT_FLAGS, 0xFFFFFFFF);

    // rx fifo 0 new message
    if(ir & RF0N)
    {
        struct canfd_frame msg;
        struct sk_buff *skb;
        struct canfd_frame *cf;

        if(tcan4550_recMsg(&msg))
        {
            // no need to keep mutex during this phase
            mutex_unlock(&priv->spi_lock);

            skb = alloc_can_skb(dev, (struct can_frame **)&cf);

            if(skb)
            {
                cf->len = msg.len;
                cf->can_id = msg.can_id;

                cf->data[0] = msg.data[0];
                cf->data[1] = msg.data[1];
                cf->data[2] = msg.data[2];
                cf->data[3] = msg.data[3];
                cf->data[4] = msg.data[4];
                cf->data[5] = msg.data[5];
                cf->data[6] = msg.data[6];
                cf->data[7] = msg.data[7];

                netif_receive_skb(skb);

                stats->rx_packets++;
                stats->rx_bytes+=msg.len;
            }

            mutex_lock(&priv->spi_lock);
        }
    }

    // Tx fifo empty
    if(ir & TFE)
    {
       // can_free_echo_skb(dev, 0, 0);

//        if(netif_tx_queue_stopped(dev))
        {
            netif_wake_queue(dev);
        }
    }

    if(ir & BO)
    {

    }

    if(ir & EW)
    {

    }

    mutex_unlock(&priv->spi_lock);

    return IRQ_HANDLED;
}

static int tcan4550_setupInterrupts(struct net_device *dev)
{
    int err;

    spi_write32(IE, RF0N + TFE + BO + EW);  // rx fifo 0 new message + tx fifo empty + bus off + warning

    spi_write32(ILE, 0x1);  // enable interrupt line 1

    // mask all spi errors
    spi_write32(SPI_MASK, 0xFFFFFFFF);

    // clear spi status register
    spi_write32(STATUS, 0xFFFFFFFF);

    // clear interrupts
    spi_write32(INTERRUPT_FLAGS, 0xFFFFFFFF);

    // interrupt enables
    spi_write32(INTERRUPT_ENABLE, 0);

    err = request_threaded_irq(spi->irq, NULL, tcan4450_handleInterrupts, IRQF_ONESHOT, dev->name, dev);
    if(err)
    {
        return err;
    }

    return 0;
}

void tcan4550_setupIo(void)
{
    gpio_request(GPIO_RESET,"GPIO_RESET");
    gpio_direction_output(GPIO_RESET, 0);
    gpio_set_value(GPIO_RESET, 0);
}

void tcan4550_freeIo(void)
{
    gpio_free(GPIO_RESET);
}

void tcan4550_hwReset(void)
{
    gpio_set_value(GPIO_RESET, 1);
    usleep_range(30, 100);  // toggle pin for at least  30us
    gpio_set_value(GPIO_RESET, 0);

    usleep_range(700, 1000);    // we need to wait at least 700us for chip to become ready
}

static bool tcan4550_init(struct net_device *dev, uint32_t bitRateReg)
{
    tcan4550_hwReset();

    if (!tcan4550_readIdentification())
    {
        netdev_err(dev, "failed to read TCAN4550 identification\n");

        return false;
    }

    tcan4550_set_standby_mode();
    tcan4550_unlock();
    tcan4550_setBitRate(bitRateReg);
    tcan4550_configure_mram();
    
    if(tcan4550_setupInterrupts(dev))
    {
        netdev_err(dev, "failed to register interrupt\n");
        
        return false;
    }

    tcan4550_set_normal_mode();

    return true;
}

/*------------------------------------------------------------*/
/* Linux CAN Driver standard functions                        */
/*------------------------------------------------------------*/

static int tcan_open(struct net_device *dev)
{
    struct tcan4550_priv *priv = netdev_priv(dev);
    const struct can_bittiming *bt = &priv->can.bittiming;
    uint32_t bitRateReg = (bt->phase_seg2 - 1) + ((bt->prop_seg + bt->phase_seg1 - 1) << 8) + ((bt->brp - 1) << 16) + ((bt->sjw - 1) << 25);
    int err;

    tcan4550_setupIo();

    /* open the can device */
    err = open_candev(dev);
    if (err)
    {
        netdev_err(dev, "failed to open can device\n");
        return err;
    }

    if(!tcan4550_init(dev, bitRateReg))
    {
        netdev_err(dev, "failed to init tcan\n");
        return -1;
    }

    mutex_init(&priv->spi_lock);

    netif_start_queue(dev);

    return 0;
}

static int tcan_close(struct net_device *dev)
{
//    struct tcan4550_priv *priv = netdev_priv(dev);

    netif_stop_queue(dev);

    close_candev(dev);

    free_irq(spi->irq, dev);

    tcan4550_freeIo();

    return 0;
}

static netdev_tx_t t_can_start_xmit(struct sk_buff *skb,
                                    struct net_device *dev)
{
    struct net_device_stats *stats = &dev->stats;
    struct can_frame *frame = (struct can_frame *)skb->data;
    struct canfd_frame msg;
    uint32_t index;
    struct tcan4550_priv *priv;

    priv = netdev_priv(dev);   // get the private

    // drop invalid can msgs
    if (can_dev_dropped_skb(dev, skb))
    {
        return NETDEV_TX_OK;
    }

    msg.len = frame->len;
    msg.can_id = frame->can_id;
    msg.data[0] = frame->data[0];
    msg.data[1] = frame->data[1];
    msg.data[2] = frame->data[2];
    msg.data[3] = frame->data[3];
    msg.data[4] = frame->data[4];
    msg.data[5] = frame->data[5];
    msg.data[6] = frame->data[6];
    msg.data[7] = frame->data[7];

    mutex_lock(&priv->spi_lock);

    // If sending is ok, also copy to echo buffer
    if(tcan4550_sendMsg(&msg, &index))  
    {
        can_put_echo_skb(skb, dev, index, 0);
        can_free_echo_skb(dev, 0, 0);

        stats->tx_packets++;
        stats->tx_bytes+=msg.len;
    }
    else
    {
        netif_stop_queue(dev);  // queue will wake up when FIFO is empty.

        mutex_unlock(&priv->spi_lock);

        return NETDEV_TX_BUSY;
    }

    mutex_unlock(&priv->spi_lock);

    return NETDEV_TX_OK;
}

static const struct net_device_ops m_can_netdev_ops = {
    .ndo_open = tcan_open,
    .ndo_stop = tcan_close,
    .ndo_start_xmit = t_can_start_xmit,
    .ndo_change_mtu = can_change_mtu,
};

static int tcan_probe(struct spi_device *_spi)
{
    struct net_device *ndev;
    int err;
    struct tcan4550_priv *priv;

    spi = _spi; // store spi handle

    ndev = alloc_candev(sizeof(struct tcan4550_priv), 16);
    if (!ndev)
    {
        return -ENOMEM;
    }

    priv = netdev_priv(ndev);   // get the private
    spi_set_drvdata(spi, ndev);
    SET_NETDEV_DEV(ndev, &spi->dev);

    priv->dev = &spi->dev;
    priv->ndev = ndev;
    priv->spi = spi;
    priv->can.bittiming_const = &tcan_bittiming_const;
    priv->can.clock.freq = 40000000;

    ndev->netdev_ops = &m_can_netdev_ops;
    ndev->flags |= IFF_ECHO;

    spi->bits_per_word = 8;
    spi->max_speed_hz = 18000000;

    err = spi_setup(spi);
    if (err)
    {
        goto exit_free;
    }

    err = register_candev(ndev);
    if (err)
    {
        dev_err(&spi->dev, "registering netdev failed\n");
        goto exit_free;
    }

    return 0;

exit_free:
    free_candev(ndev);

    return err;
}

void tcan_remove(struct spi_device *spi)
{
    struct net_device *ndev = spi_get_drvdata(spi);

    unregister_candev(ndev);

    free_candev(ndev);
}

static const struct of_device_id tcan4550_of_match[] = {
    {
        .compatible = "ti,tcan4550",
        .data = (void *)4550,
    },
    {}};
MODULE_DEVICE_TABLE(of, tcan4550_of_match);

static const struct spi_device_id tcan4550_id_table[] = {
    {
        .name = "tcan4550",
        .driver_data = (kernel_ulong_t)4550,
    },
    {}};
MODULE_DEVICE_TABLE(spi, tcan4550_id_table);

static struct spi_driver tcan4550_can_driver = {
    .driver = {
        .name = "tcan4550",
        .of_match_table = tcan4550_of_match,
        .pm = NULL,
    },
    .id_table = tcan4550_id_table,
    .probe = tcan_probe,
    .remove = tcan_remove,
};
module_spi_driver(tcan4550_can_driver);

MODULE_AUTHOR("Carl-Magnus Moon <>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN bus driver for TI TCAN4550 controller");
