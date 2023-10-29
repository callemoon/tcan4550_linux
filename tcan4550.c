// SPDX-License-Identifier: GPL-2.0
// CAN driver for TI TCAN4550
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

#include <linux/gpio.h>
#include <linux/spinlock.h>

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

const static uint32_t RF0N = (0x1UL << 0); // rx fifo 0 new data
const static uint32_t TC = (0x1UL << 9);   // transmission complete
const static uint32_t TFE = (0x1UL << 11); // transmit fifo empty
const static uint32_t EP = (0x1UL << 23);  // error passive
const static uint32_t EW = (0x1UL << 24);  // error warning
const static uint32_t BO = (0x1UL << 25);  // bus off

const static uint32_t INIT = 0x1; // init
const static uint32_t CCE = 0x2;  // configuration change enable
const static uint32_t CSR = 0x10; // clock stop request

const static uint32_t MODESEL_1 = 0x40;
const static uint32_t MODESEL_2 = 0x80;

// MRAM config
const static uint32_t RX_SLOT_SIZE = 16;
const static uint32_t TX_SLOT_SIZE = 16;
const static uint32_t TX_MSG_BOXES = 32;
const static uint32_t TX_FIFO_START_ADDRESS = 0x0;
const static uint32_t RX_MSG_BOXES = 16;
const static uint32_t RX_FIFO_START_ADDRESS = 0x200;

const static uint32_t MRAM_BASE = 0x8000;

#define MAX_SPI_MESSAGES    16  // Max CAN messages in an SPI pacakge
#define MAX_BURST_RX_PACKAGES   4

#define TX_BUFFER_SIZE 15+1 // one slot is used to keep track of empty queue

struct sk_buff *tx_skb[TX_BUFFER_SIZE];
static int head = 0;
static int tail = 0;

unsigned long flags;
static DEFINE_SPINLOCK(mLock); // spinlock protecting tx_skb buffer
static DEFINE_MUTEX(spi_lock); // mutex protecting SPI access

struct tcan4550_priv
{
    struct can_priv can; // must be located first in private struct
    struct device *dev;
    struct net_device *ndev;
    struct spi_device *spi;

    struct workqueue_struct *wq;
    struct work_struct tx_work;
};

static struct spi_device *spi = 0; // global spi handle
static struct gpio_desc *reset_gpio;

// tcan function headers
static int tcan4550_sendMsg(struct canfd_frame *msg, uint32_t *index, bool extended, bool rtr);
static void tcan4550_set_normal_mode(void);
static void tcan4550_set_standby_mode(void);
static void tcan4550_configure_mram(void);
static void tcan4550_unlock(void);
static bool tcan4550_readIdentification(void);
static bool tcan4550_setBitRate(uint32_t bitRate);
static int tcan4550_setupInterrupts(struct net_device *dev);
static void tcan4550_hwReset(void);
void tcan4550_setupIo(struct device *dev);
static irqreturn_t tcan4450_handleInterrupts(int irq, void *dev);
void tcan4550_composeMessage(struct sk_buff *skb, uint32_t *buffer);

static void tcan4550_tx_work_handler(struct work_struct *ws);
bool tcan4550_recMsgs(struct net_device *dev);

// spi function headers
static uint32_t spi_read32(uint32_t address);
static int spi_write32(uint32_t address, uint32_t data);
static int spi_write_len(uint32_t address, int32_t msgs, uint32_t *data);
static int spi_read_len(uint32_t address, int32_t msgs, uint32_t *data);

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

/*------------------------------------------------------------*/
/* SPI helper functions                                       */
/*------------------------------------------------------------*/
static int spi_trans(struct spi_device *_spi, int len, unsigned char *rxBuf, unsigned char *txBuf)
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

    if (_spi == 0)
    {
        return -EINVAL;
    }

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    mutex_lock(&spi_lock);

    ret = spi_sync(_spi, &m);
    if (ret)
    {
        dev_err(&_spi->dev, "spi transfer failed: ret = %d\n", ret);
    }

    mutex_unlock(&spi_lock);

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

    ret = spi_trans(spi, 8, rxBuf, txBuf);

    return (rxBuf[4] << 24) + (rxBuf[5] << 16) + (rxBuf[6] << 8) + rxBuf[7];
}

static int spi_read_len(uint32_t address, int32_t msgs, uint32_t *data)
{
    unsigned char txBuf[4+(MAX_SPI_MESSAGES*16)];
    unsigned char rxBuf[4+(MAX_SPI_MESSAGES*16)];

    int ret;
    uint32_t i;

    if(msgs > MAX_SPI_MESSAGES)
    {
        return 0;
    }

    txBuf[0] = 0x41;
    txBuf[1] = address >> 8;
    txBuf[2] = address & 0xFF;
    txBuf[3] = msgs*4;

    ret = spi_trans(spi, 4 + (msgs * 16), rxBuf, txBuf);

    for(i = 0; i < (msgs*4); i++)
    {
        data[0 + (i*4)] = rxBuf[7+(i*16)] + (rxBuf[6 + (i*16)] << 8) + (rxBuf[5 + (i*16)] << 16) + (rxBuf[4 + (i*16)] << 24);
        data[1 + (i*4)] = rxBuf[11 + (i*16)] + (rxBuf[10 + (i*16)] << 8) + (rxBuf[9 + (i*16)] << 16) + (rxBuf[8 + (i*16)] << 24);
        data[2 + (i*4)] = rxBuf[15 + (i*16)] + (rxBuf[14 + (i*16)] << 8) + (rxBuf[13 + (i*16)] << 16) + (rxBuf[12 + (i*16)] << 24);
        data[3 + (i*4)] = rxBuf[19 + (i*16)] + (rxBuf[18 + (i*16)] << 8) + (rxBuf[17 + (i*16)] << 16) + (rxBuf[16 + (i*16)] << 24);
    }

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

    ret = spi_trans(spi, 8, rxBuf, txBuf);

    return ret;
}

static int spi_write_len(uint32_t address, int32_t msgs, uint32_t *data)
{
    unsigned char txBuf[4+(MAX_SPI_MESSAGES*16)];
    unsigned char rxBuf[4+(MAX_SPI_MESSAGES*16)];
    uint32_t i;
    int ret;

    if(msgs > MAX_SPI_MESSAGES)
    {
        return 0;
    }

    txBuf[0] = 0x61;
    txBuf[1] = address >> 8;
    txBuf[2] = address & 0xFF;
    txBuf[3] = msgs*4;

    for(i = 0; i < (msgs * 4); i++)
    {
        txBuf[4 + (i*4)] = ((data[i] >> 24) & 0xFF);
        txBuf[5 + (i*4)] = ((data[i] >> 16) & 0xFF);
        txBuf[6 + (i*4)] = ((data[i] >> 8) & 0xFF);
        txBuf[7 + (i*4)] = (data[i] & 0xFF);
    }

    ret = spi_trans(spi, 4+(msgs*16), rxBuf, txBuf);

    return ret;
}

/*------------------------------------------------------------*/
/* TCAN4550 functions                                         */
/*------------------------------------------------------------*/
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
    uint32_t i;

    // clear MRAM to avoid risk of ECC errors 2kB = 512 words
    for (i = 0; i < 512; i++)
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

    val |= (CCE + INIT);     // set CCE and INIT bits
    val &= ~((uint32_t)CSR); // clear CSR

    spi_write32(CCCR, val);
}

// convert a struct sk_buff to a tcan4550 msg
void tcan4550_composeMessage(struct sk_buff *skb, uint32_t *buffer)
{
    struct can_frame *frame;
    bool extended = false;
    bool rtr = false;
    uint32_t len;
    uint32_t id;

    frame = (struct can_frame *)skb->data;

    if(frame->can_id & CAN_EFF_FLAG)
    {
        extended = true;
    }

    if(frame->can_id & CAN_RTR_FLAG)
    {
        rtr = true;
    }

    len = frame->len;
    if (len > 8)
    {
        len = 8;
    }
    
    if(extended)
    {
        id = frame->can_id & CAN_EFF_MASK;
        buffer[0] = id;
    }
    else
    {
        id = frame->can_id & CAN_SFF_MASK; 
        buffer[0] = (id << 18);
    }

    buffer[0] += (rtr << 29) + (extended << 30);    // add extended and rtr flags
    buffer[1] = (len << 16);
    buffer[2] = frame->data[0] + (frame->data[1] << 8) + (frame->data[2] << 16) + (frame->data[3] << 24);
    buffer[3] = frame->data[4] + (frame->data[5] << 8) + (frame->data[6] << 16) + (frame->data[7] << 24);
}

static void tcan4550_tx_work_handler(struct work_struct *ws)
{
    struct tcan4550_priv *priv = container_of(ws, struct tcan4550_priv,
                                              tx_work);

    struct net_device_stats *stats = &((struct net_device *)priv->ndev)->stats;

    // check for free buffers
    uint32_t txqfs = spi_read32(TXQFS);
    uint32_t freeBuffers = txqfs & 0x3F;
    uint32_t writeIndex = (txqfs >> 16) & 0x1F;
    uint32_t writeIndexTmp = writeIndex;

    uint32_t buffer[64];
    uint32_t requestMask = 0;

    int msgs = 0;

    uint32_t baseAddress = MRAM_BASE + TX_FIFO_START_ADDRESS + (writeIndex * TX_SLOT_SIZE);

    spin_lock_irqsave(&mLock, flags);

    // build an spi message consisting of up to 16 CAN messges
    while((head != tail) && (msgs < freeBuffers) && (writeIndexTmp < TX_MSG_BOXES))
    {
        tcan4550_composeMessage(tx_skb[tail], &buffer[msgs*4]);

        can_put_echo_skb(tx_skb[tail], priv->ndev, 0, 0);
        can_free_echo_skb(priv->ndev, 0, 0);

        requestMask += (1 << writeIndexTmp);    // add current message to request mask

        msgs++;
        writeIndexTmp++;

        tail++;
        if(tail >= TX_BUFFER_SIZE)
        {
            tail = 0;
        }

        //stats->tx_bytes += cf->len;
    }

    spin_unlock_irqrestore(&mLock, flags);

    if(msgs > 0)
    {
        spi_write_len(baseAddress, msgs, buffer);   // write message data

        spi_write32(TXBAR, requestMask); // request buffer transmission
    }

    stats->tx_packets++;

}

bool tcan4550_recMsgs(struct net_device *dev)
{
    uint32_t rxBuf[64];
    uint32_t i;
    struct net_device_stats *stats = &((struct net_device *)dev)->stats;

    uint32_t rxf0s = spi_read32(RXF0S);

    uint32_t fillLevel = (rxf0s & 0x7F);          // 0-64
    uint32_t getIndex = ((rxf0s >> 8) & 0x3F);    // 0-63
    // uint32_t putIndex = (rxf0s >> 16) & 0xFF;

    uint32_t msgsToGet = fillLevel;
    
    // after wrap around of rx buffer we need a new spi request
    if(msgsToGet > (RX_MSG_BOXES - getIndex))
    {
        msgsToGet = (RX_MSG_BOXES - getIndex);
    }

    // do not read too many packages in one go as we also need to ack rx packages to give room for new rx and perform tx
    if(msgsToGet > MAX_BURST_RX_PACKAGES)
    {
        msgsToGet = MAX_BURST_RX_PACKAGES;
    }
    
    uint32_t baseAddress = MRAM_BASE + RX_FIFO_START_ADDRESS + (getIndex * RX_SLOT_SIZE);

    spi_read_len(baseAddress, msgsToGet, rxBuf);

    spi_write32(RXF0A, (getIndex + msgsToGet - 1)); // acknowledge the last message we have read, that will automatically free all read

    for(i = 0; i < msgsToGet; i++)
    {
        struct canfd_frame *cf;
        struct sk_buff *skb;

        skb = alloc_can_skb(dev, (struct can_frame **)&cf);

        if (skb)
        {
            uint32_t data[4];

            data[0] = rxBuf[0+(i*4)];
            data[1] = rxBuf[1+(i*4)];
            data[2] = rxBuf[2+(i*4)];
            data[3] = rxBuf[3+(i*4)];

            cf->len = (data[1] >> 16) & 0x7F;
        
            if(data[0] & (1 << 30)) // extended
            {
                cf->can_id = (data[0] & CAN_EFF_MASK) | CAN_EFF_FLAG;
            }
            else
            {
                cf->can_id = (data[0] >> 18) & CAN_SFF_MASK; 
            }

            cf->data[0] = data[2] & 0xFF;
            cf->data[1] = (data[2] >> 8) & 0xFF;
            cf->data[2] = (data[2] >> 16) & 0xFF;
            cf->data[3] = (data[2] >> 24) & 0xFF;
            cf->data[4] = data[3] & 0xFF;
            cf->data[5] = (data[3] >> 8) & 0xFF;
            cf->data[6] = (data[3] >> 16) & 0xFF;
            cf->data[7] = (data[3] >> 24) & 0xFF;

            netif_rx(skb);

            stats->rx_packets++;
            stats->rx_bytes += cf->len;
        }
    }

    return false;
}

// main interrupt handler - run as an irq thread
static irqreturn_t tcan4450_handleInterrupts(int irq, void *dev)
{
    struct tcan4550_priv *priv = netdev_priv(dev);
    struct net_device_stats *stats = &((struct net_device *)dev)->stats;
    uint32_t ir;

    ir = spi_read32(IR);
    spi_write32(IR, ir); // acknowledge interrupts

    //    if(ir == 0)
    //    {
    //        return IRQ_NONE;
    //    }

    // spi_write32(STATUS, 0xFFFFFFFF);
    // spi_write32(INTERRUPT_FLAGS, 0xFFFFFFFF);

    // rx fifo 0 new message
    if (ir & RF0N)
    {
        tcan4550_recMsgs(dev);
    }

    // Tx fifo empty
    if (ir & TFE)
    {
        //        if(netif_tx_queue_stopped(dev))
        {
            netdev_err(dev, "waking queue\n");
            netif_wake_queue(dev);
        }
    }

    if (ir & BO)
    {
        // stats->bus_off++;
        can_bus_off(dev);
    }

    if (ir & EW)
    {
        // stats->error_warning++;
    }

    if (ir & EP)
    {
        // stats->error_passive++;
    }

    return IRQ_HANDLED;
}

static int tcan4550_setupInterrupts(struct net_device *dev)
{
    int err;

    spi_write32(IE, RF0N + TFE + BO + EW + EP); // rx fifo 0 new message + tx fifo empty + bus off + warning + error passive

    spi_write32(ILE, 0x1); // enable interrupt line 1

    // mask all spi errors
    spi_write32(SPI_MASK, 0xFFFFFFFF);

    // clear spi status register
    spi_write32(STATUS, 0xFFFFFFFF);

    // clear interrupts
    spi_write32(INTERRUPT_FLAGS, 0xFFFFFFFF);

    // interrupt enables
    spi_write32(INTERRUPT_ENABLE, 0);

    // as SPI is slow, run irq in a kernel thread
    err = request_threaded_irq(spi->irq, NULL, tcan4450_handleInterrupts, IRQF_ONESHOT, dev->name, dev);
    if (err)
    {
        return err;
    }

    return 0;
}

void tcan4550_setupIo(struct device *dev)
{
    reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW); // get reset gpio from device-tree reset-gpio property, set to output low
    if (IS_ERR(reset_gpio))
    {
        dev_err(dev, "could not get reset gpio\n");

        reset_gpio = NULL;
    }
}

void tcan4550_hwReset(void)
{
    gpiod_set_value(reset_gpio, 1);
    usleep_range(30, 100); // toggle pin for at least  30us
    gpiod_set_value(reset_gpio, 0);

    usleep_range(700, 1000); // we need to wait at least 700us for chip to become ready
}

static bool tcan4550_init(struct net_device *dev, uint32_t bitRateReg)
{
    tcan4550_hwReset();

    if (!tcan4550_readIdentification())
    {
        if (!tcan4550_readIdentification())
        {
            netdev_err(dev, "failed to read TCAN4550 identification\n");

            return false;
        }
    }

    tcan4550_set_standby_mode();
    tcan4550_unlock();
    tcan4550_setBitRate(bitRateReg);
    tcan4550_configure_mram();

    if (tcan4550_setupInterrupts(dev))
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

    /* open the can device */
    err = open_candev(dev);
    if (err)
    {
        netdev_err(dev, "failed to open can device\n");
        return err;
    }

    if (!tcan4550_init(dev, bitRateReg))
    {
        netdev_err(dev, "failed to init tcan\n");
        return -ENXIO;
    }

    netif_start_queue(dev);

    return 0;
}

static int tcan_close(struct net_device *dev)
{
    //    struct tcan4550_priv *priv = netdev_priv(dev);

    netif_stop_queue(dev);

    close_candev(dev);

    free_irq(spi->irq, dev);

    head = 0;
    tail = 0;

    return 0;
}

static netdev_tx_t t_can_start_xmit(struct sk_buff *skb,
                                    struct net_device *dev)
{
    struct tcan4550_priv *priv;
    priv = netdev_priv(dev); // get the private
    uint32_t tmpHead;

    // drop invalid can msgs
    if (can_dropped_invalid_skb(dev, skb))
    {
        return NETDEV_TX_OK;
    }

    spin_lock_irqsave(&mLock, flags);

    tmpHead = head;
    tmpHead++;
    if(tmpHead >= TX_BUFFER_SIZE)
    {
        tmpHead = 0;
    }
    
    if(tmpHead == tail)
    {
        netif_stop_queue(dev);

        spin_unlock_irqrestore(&mLock, flags);

        netdev_err(dev, "stopping queue\n");

        queue_work(priv->wq, &priv->tx_work);

        return NETDEV_TX_BUSY;
    }

    tx_skb[head] = skb;
    head=tmpHead;

    spin_unlock_irqrestore(&mLock, flags);

    queue_work(priv->wq, &priv->tx_work);

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
    struct spi_delay delay =
    {
        .unit = SPI_DELAY_UNIT_USECS,
        .value = 0
    };

    spi = _spi; // store spi handle

    ndev = alloc_candev(sizeof(struct tcan4550_priv), 16);
    if (!ndev)
    {
        return -ENOMEM;
    }

    priv = netdev_priv(ndev); // get the private
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
    spi->cs_setup = delay;
    spi->cs_hold = delay;
    spi->cs_inactive = delay;
    spi->word_delay = delay;

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

    tcan4550_setupIo(&spi->dev);

    priv->wq = alloc_workqueue("tcan4550_wq", WQ_FREEZABLE | WQ_MEM_RECLAIM, 1);
    if (!priv->wq)
    {
        err = -ENOMEM;
        goto exit_free;
    }
    INIT_WORK(&priv->tx_work, tcan4550_tx_work_handler);

    return 0;

exit_free:
    free_candev(ndev);

    return err;
}

void tcan_remove(struct spi_device *spi)
{
    struct net_device *ndev = spi_get_drvdata(spi);
    struct tcan4550_priv *priv = netdev_priv(ndev);
    
    unregister_candev(ndev);

    free_candev(ndev);

    destroy_workqueue(priv->wq);

//    return 0;
}

static const struct of_device_id tcan4550_of_match[] = {
    {
        .compatible = "ti,tcan4x5x",
        .data = (void *)4550,
    },
    {}};
MODULE_DEVICE_TABLE(of, tcan4550_of_match);

static const struct spi_device_id tcan4550_id_table[] = {
    {
        .name = "tcan4x5x",
        .driver_data = (kernel_ulong_t)4550,
    },
    {}};
MODULE_DEVICE_TABLE(spi, tcan4550_id_table);

static struct spi_driver tcan4550_can_driver = {
    .driver = {
        .name = "tcan4x5x",
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
