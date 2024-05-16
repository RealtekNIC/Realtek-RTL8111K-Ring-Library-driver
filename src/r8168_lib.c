/*
################################################################################
#
# r8168 is the Linux device driver released for Realtek Gigabit Ethernet
# controllers with PCI-Express interface.
#
# Copyright(c) 2022 Realtek Semiconductor Corp. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>.
#
# Author:
# Realtek NIC software team <nicfae@realtek.com>
# No. 2, Innovation Road II, Hsinchu Science Park, Hsinchu 300, Taiwan
#
################################################################################
*/

/************************************************************************************
 *  This product is covered by one or more of the following patents:
 *  US6,570,884, US6,115,776, and US6,327,625.
 ***********************************************************************************/

#include <linux/pci.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include "r8168.h"
#include "r8168_lib.h"

void __iomem * db_tx;
void __iomem * db_rx;

static void
rtl8168_map_to_asic(struct rtl8168_private *tp,
                    struct rtl8168_ring *ring,
                    struct RxDesc *desc,
                    dma_addr_t mapping,
                    u32 rx_buf_sz,
                    const u32 cur_rx)
{
        desc->addr = cpu_to_le64(mapping);
        wmb();
        rtl8168_mark_to_asic(desc, rx_buf_sz);
}

static void
rtl8168_lib_tx_fill(struct rtl8168_ring *ring)
{
        struct TxDesc *descs = ring->desc_addr;
        u32 i;

        for (i = 0; i < ring->ring_size; i++) {
                struct TxDesc *desc = &descs[i];

                desc->addr = cpu_to_le64(ring->bufs[i].dma_addr);
        }
        descs[ring->ring_size - 1].opts1 |= cpu_to_le32(RingEnd);
}

static inline void
rtl8168_mark_as_last_descriptor(struct rtl8168_private *tp,
                                struct RxDesc *desc)
{
        desc->opts1 |= cpu_to_le32(RingEnd);
}

static void
rtl8168_lib_rx_fill(struct rtl8168_ring *ring)
{
        struct rtl8168_private *tp = ring->private;
        struct RxDesc *desc;
        u32 i;

        for (i = 0; i < ring->ring_size; i++) {
                desc = rtl8168_get_rxdesc(tp, tp->RxDescArray, i, 0);
                rtl8168_map_to_asic(tp, ring, desc,
                                    ring->bufs[i].dma_addr,
                                    ring->buff_size, i);
        }

        desc = rtl8168_get_rxdesc(tp, tp->RxDescArray, ring->ring_size - 1,
                                  0);
        rtl8168_mark_as_last_descriptor(tp, desc);
}

static struct rtl8168_ring *rtl8168_get_tx_ring(struct rtl8168_private *tp)
{
        struct rtl8168_ring *ring;

        WARN_ON_ONCE(tp->num_tx_rings < 1);

        ring = &tp->lib_tx_ring[1];
        if (ring->allocated)
                return NULL;

        ring->allocated = true;

        return ring;
}

static struct rtl8168_ring *rtl8168_get_rx_ring(struct rtl8168_private *tp)
{
        struct rtl8168_ring *ring;

        ring = &tp->lib_rx_ring[1];
        if (ring->allocated)
                return NULL;

        ring->allocated = true;

        return ring;
}

static void rtl8168_put_ring(struct rtl8168_ring *ring)
{
        if (!ring)
                return;

        ring->allocated = false;
}

static void rtl8168_init_rx_ring(struct rtl8168_ring *ring)
{
        if (!ring->allocated)
                return;

        rtl8168_lib_rx_fill(ring);
}

static void rtl8168_init_tx_ring(struct rtl8168_ring *ring)
{
        struct rtl8168_private *tp = ring->private;
        u16 tdsar_reg;

        if (!ring->allocated)
                return;

        rtl8168_lib_tx_fill(ring);

        tdsar_reg = TxHDescStartAddrLow;
        RTL_W32(tp, tdsar_reg, ((u64)ring->desc_daddr & DMA_BIT_MASK(32)));
        RTL_W32(tp, tdsar_reg + 4, ((u64)ring->desc_daddr >> 32));
}

static void rtl8168_free_ring_mem(struct rtl8168_ring *ring)
{
        unsigned i;
        struct rtl8168_private *tp = ring->private;
        struct pci_dev *pdev = tp->pci_dev;

        if (ring->direction == RTL8168_CH_DIR_TX) {
                if (ring->desc_addr) {
                        dma_free_coherent(&pdev->dev, ring->desc_size,
                                          ring->desc_addr, ring->desc_daddr);

                        ring->desc_addr = NULL;
                }
        } else if (ring->direction == RTL8168_CH_DIR_RX) {
                ring->desc_addr = NULL;
        }

        if (ring->bufs) {
                if (ring->flags & RTL8168_CONTIG_BUFS) {
                        struct rtl8168_buf *rtl_buf = &ring->bufs[0];
                        if (rtl_buf->addr) {
                                dma_free_coherent(
                                        &pdev->dev,
                                        ring->ring_size * ring->buff_size,
                                        rtl_buf->addr,
                                        rtl_buf->dma_addr);
                                rtl_buf->addr = NULL;
                        }
                } else {
                        for (i=0; i<ring->ring_size; i++) {
                                struct rtl8168_buf *rtl_buf = &ring->bufs[i];
                                if (rtl_buf->addr) {
                                        dma_free_coherent(
                                                &pdev->dev,
                                                rtl_buf->size,
                                                rtl_buf->addr,
                                                rtl_buf->dma_addr);
                                        rtl_buf->addr = NULL;
                                }
                        }
                }
                kfree(ring->bufs);
                ring->bufs = 0;
        }
}

static int rtl8168_alloc_ring_mem(struct rtl8168_ring *ring)
{
        int i;
        struct rtl8168_private *tp = ring->private;
        struct pci_dev *pdev = tp->pci_dev;

        ring->bufs = kzalloc(sizeof(struct rtl8168_buf) * ring->ring_size, GFP_KERNEL);
        if (!ring->bufs)
                return -ENOMEM;

        if (ring->mem_ops == NULL) {
                /* Use dma_alloc_coherent() and dma_free_coherent() below */
                if (ring->direction == RTL8168_CH_DIR_TX) {
                        ring->desc_size = ring->ring_size * sizeof(struct TxDesc);
                        ring->desc_addr = dma_alloc_coherent(
                                                  &pdev->dev,
                                                  ring->desc_size,
                                                  &ring->desc_daddr,
                                                  GFP_KERNEL);
                } else if (ring->direction == RTL8168_CH_DIR_RX) {
                        if (ring->ring_size != tp->num_rx_desc) {
                                netif_err(tp, drv, tp->dev,
                                          "lib rx desc num not equal to sw path!\n");
                                goto error_out;
                        }

                        ring->desc_size = ring->ring_size * tp->RxDescLength;
                        ring->desc_addr = rtl8168_get_rxdesc(tp, tp->RxDescArray,
                                                             0, 0);
                        /*
                         * ipa will calculate the rx q0 daddr
                         * based on RxPhyAddr.
                         */
                        ring->desc_daddr = tp->RxPhyAddr;
                }

                if (!ring->desc_addr)
                        goto error_out;

                if (ring->direction == RTL8168_CH_DIR_TX)
                        memset(ring->desc_addr, 0x0, ring->desc_size);

                if (ring->flags & RTL8168_CONTIG_BUFS) {
                        struct rtl8168_buf *rtl_buf = &ring->bufs[0];

                        rtl_buf->size = ring->buff_size;
                        rtl_buf->addr = dma_alloc_coherent(
                                                &pdev->dev,
                                                ring->ring_size * ring->buff_size,
                                                &rtl_buf->dma_addr,
                                                GFP_KERNEL);
                        if (!rtl_buf->addr)
                                goto error_out;

                        for (i = 1; i < ring->ring_size; i++) {
                                struct rtl8168_buf *rtl_buf = &ring->bufs[i];
                                struct rtl8168_buf *rtl_buf_prev = &ring->bufs[i-1];
                                rtl_buf->size = ring->buff_size;
                                rtl_buf->addr = rtl_buf_prev->addr + ring->buff_size;
                                rtl_buf->dma_addr = rtl_buf_prev->dma_addr + ring->buff_size;
                        }
                } else {
                        for (i = 0; i < ring->ring_size; i++) {
                                struct rtl8168_buf *rtl_buf = &ring->bufs[i];

                                rtl_buf->size = ring->buff_size;
                                rtl_buf->addr = dma_alloc_coherent(
                                                        &pdev->dev,
                                                        rtl_buf->size,
                                                        &rtl_buf->dma_addr,
                                                        GFP_KERNEL);
                                if (!rtl_buf->addr)
                                        goto error_out;

                                memset(rtl_buf->addr, 0x0, rtl_buf->size);
                        }
                }
        }
#if 0
        /* Validate parameters */
        /* Allocate descs */
        mem_ops->alloc_descs(...);

        /* Allocate buffers */
        if (R8168B_CONTIG_BUFS) {
                mem_ops->alloc_buffs(...);
        } else {
                /* Call mem_ops->alloc_buffs(...) for each descriptor. */
        }
#endif

        return 0;

error_out:
        rtl8168_free_ring_mem(ring);

        return -ENOMEM;
}

struct rtl8168_ring *rtl8168_request_ring(struct net_device *ndev,
                unsigned int ring_size, unsigned int buff_size,
                enum rtl8168_channel_dir direction, unsigned int flags,
                struct rtl8168_mem_ops *mem_ops)
{
        struct rtl8168_private *tp = netdev_priv(ndev);
        struct rtl8168_ring * ring = 0;

        rtnl_lock();

        if (direction == RTL8168_CH_DIR_TX)
                ring = rtl8168_get_tx_ring(tp);
        else if (direction == RTL8168_CH_DIR_RX)
                ring = rtl8168_get_rx_ring(tp);

        if (!ring)
                goto error_out;

        ring->ring_size = ring_size;
        ring->buff_size = buff_size;
        ring->mem_ops = mem_ops;
        ring->flags = flags;

        if (rtl8168_alloc_ring_mem(ring))
                goto error_put_ring;

        /* initialize descriptors to point to buffers allocated */
        if (direction == RTL8168_CH_DIR_TX)
                rtl8168_init_tx_ring(ring);
        /* rx ring will set in enable ring */
        /*else if (direction == RTL8168_CH_DIR_RX)
                rtl8168_init_rx_ring(ring);
        */

        rtnl_unlock();

        return ring;

error_put_ring:
        rtl8168_put_ring(ring);
error_out:
        rtnl_unlock();
        return NULL;
}
EXPORT_SYMBOL(rtl8168_request_ring);

void rtl8168_release_ring(struct rtl8168_ring *ring)
{
        struct rtl8168_private *tp;

        if (!ring)
                return;

        tp = ring->private;

        rtnl_lock();

        rtl8168_free_ring_mem(ring);
        rtl8168_put_ring(ring);
        if (rtl8168_lib_all_ring_released(tp)) {
                struct net_device *dev = tp->dev;

                if (netif_running(dev)) {
                        rtl8168_close(dev);
                        rtl8168_open(dev);
                }
        }

        rtnl_unlock();
}
EXPORT_SYMBOL(rtl8168_release_ring);

int rtl8168_enable_ring(struct rtl8168_ring *ring)
{
        struct rtl8168_private *tp;
        struct net_device *dev;

        if (!ring)
                return -EINVAL;

        if (!(ring->direction == RTL8168_CH_DIR_TX ||
              ring->direction == RTL8168_CH_DIR_RX))
                return -EINVAL;

        rtnl_lock();

        tp = ring->private;
        dev = tp->dev;

        if (!netif_running(dev)) {
                netif_warn(tp, drv, dev, "closed not enable ring. \n");
                goto out_unlock;
        }

        /* Start the ring if needed */
        netif_tx_disable(dev);
        _rtl8168_wait_for_quiescence(dev);
        rtl8168_hw_reset(dev);
        rtl8168_tx_clear(tp);
        rtl8168_rx_clear(tp);
        if (ring->direction == RTL8168_CH_DIR_RX)
                tp->num_rx_rings = 0;
        rtl8168_init_ring(dev);

        ring->enabled = true;

        rtl8168_hw_config(dev);
        rtl8168_hw_start(dev);

#ifdef CONFIG_R8168_NAPI
        rtl8168_enable_napi(tp);
#endif//CONFIG_R8168_NAPI

        netif_tx_start_all_queues(dev);

out_unlock:
        rtnl_unlock();

        return 0;
}
EXPORT_SYMBOL(rtl8168_enable_ring);

void rtl8168_disable_ring(struct rtl8168_ring *ring)
{
        struct rtl8168_private *tp;
        struct net_device *dev;

        /* Stop the ring if possible. IPA do not want to receive or transmit
        packets beyond this point.
        */

        if (!ring)
                return;

        if (!(ring->direction == RTL8168_CH_DIR_TX ||
              ring->direction == RTL8168_CH_DIR_RX))
                return;

        tp = ring->private;
        dev = tp->dev;

        rtnl_lock();

        rtl8168_hw_reset(dev);
        //rtl8168_tx_clear(tp);
        //rtl8168_rx_clear(tp);
        //rtl8168_init_ring(dev);

        ring->enabled = false;

        if (ring->direction == RTL8168_CH_DIR_RX)
                tp->num_rx_rings = 1;

        //rtl8168_hw_config(dev);
        //rtl8168_hw_start(dev);

        rtnl_unlock();
}
EXPORT_SYMBOL(rtl8168_disable_ring);

int rtl8168_request_event(struct rtl8168_ring *ring, unsigned long flags,
                          dma_addr_t addr, u64 data, phys_addr_t paddr)
{
        void __iomem *doorbell;
        struct rtl8168_private *tp;
        struct pci_dev *pdev;
        u32 message_id;

        if (!ring)
                return -EINVAL;

        if (!(ring->direction == RTL8168_CH_DIR_TX ||
              ring->direction == RTL8168_CH_DIR_RX))
                return -EINVAL;

        if (ring->event.allocated)
                return -EEXIST;

        /* map doorbell address */
        doorbell = ioremap(paddr, sizeof(data));
        if (!doorbell)
                return -EIO;

        if (ring->direction == RTL8168_CH_DIR_TX) {
                db_tx = doorbell;

                /*
                * IPA tx interrupt is handled by sw path. But if driver
                * return non-zero here, IPA will fail to init.
                */
                message_id = 0;

                goto out;
        } else
                db_rx = doorbell;

        message_id = 0;

        tp = ring->private;
        pdev = tp->pci_dev;

        if (flags & MSIX_event_type) {
                /* Update MSI-X table entry with @addr and @data */
                /* Initialize any MSI-X/interrupt related register in HW */
                /* Interrupt all controlled by sw path.
                u16 reg = message_id * 0x10;

                rtnl_lock();

                ring->event.addr = rtl8168_eri_read(tp, reg, 4, ERIAR_MSIX);
                ring->event.addr |= (u64)rtl8168_eri_read(tp, reg + 4, 4, ERIAR_MSIX) << 32;
                ring->event.data = rtl8168_eri_read(tp, reg + 8, 4, ERIAR_MSIX);
                ring->event.data |= (u64)rtl8168_eri_read(tp, reg + 8, 4, ERIAR_MSIX) << 32;

                rtl8168_eri_write(tp, reg, 4, (u64)addr & DMA_BIT_MASK(32), ERIAR_MSIX);
                rtl8168_eri_write(tp, reg + 4, 4, (u64)addr >> 32, ERIAR_MSIX);
                rtl8168_eri_write(tp, reg + 8, 4, data, ERIAR_MSIX);
                rtl8168_eri_write(tp, reg + 12, 4, data >> 32, ERIAR_MSIX);

                rtnl_unlock();
                */
        }

out:
        ring->event.message_id = message_id;
        ring->event.allocated = 1;

        return 0;
}
EXPORT_SYMBOL(rtl8168_request_event);

void rtl8168_release_event(struct rtl8168_ring *ring)
{
        struct rtl8168_private *tp;
        void __iomem *doorbell;
        dma_addr_t addr;
        u64 data;
        u16 reg;

        /* Reverse request_event() */
        if (!ring)
                return;

        if (!(ring->direction == RTL8168_CH_DIR_TX ||
              ring->direction == RTL8168_CH_DIR_RX))
                return;

        if (!ring->event.allocated)
                return;

        /* unmap doorbell address */
        if (ring->direction == RTL8168_CH_DIR_TX)
                doorbell = db_tx;
        else
                doorbell = db_rx;

        if (doorbell)
                iounmap(doorbell);

        if (ring->direction == RTL8168_CH_DIR_TX)
                db_tx = NULL;
        else
                db_rx = NULL;

        /*
        * IPA tx interrupt is handled by sw path.
        * Driver does not need to restore msix table.
        */
        if (ring->direction == RTL8168_CH_DIR_TX)
                goto out;

        tp = ring->private;

        reg = ring->event.message_id * 0x10;

        addr = ring->event.addr;
        data = ring->event.data;

        /* Interrupt all controlled by sw path.
        rtnl_lock();

        rtl8168_eri_write(tp, reg, 4, (u64)addr & DMA_BIT_MASK(32), ERIAR_MSIX);
        rtl8168_eri_write(tp, reg + 4, 4, (u64)addr >> 32, ERIAR_MSIX);
        rtl8168_eri_write(tp, reg + 8, 4, data, ERIAR_MSIX);
        rtl8168_eri_write(tp, reg + 12, 4, data >> 32, ERIAR_MSIX);

        rtnl_unlock();
        */

out:
        ring->event.allocated = 0;

        return;
}
EXPORT_SYMBOL(rtl8168_release_event);

static int _rtl8168_enable_event(struct rtl8168_ring *ring)
{
        struct rtl8168_private *tp = ring->private;

        if (!ring->event.allocated)
                return -EINVAL;

        if (ring->direction == RTL8168_CH_DIR_TX)
                goto out;

        /* Set interrupt moderation timer */
        //rtl8168_set_ring_intr_mod(ring, ring->event.delay);

        /* Enable interrupt */
        rtl8168_enable_interrupt_by_vector(tp, ring->event.message_id);

out:
        ring->event.enabled = 1;

        return 0;
}

int rtl8168_enable_event(struct rtl8168_ring *ring)
{
        struct rtl8168_private *tp;
        struct net_device *dev;

        if (!ring)
                return -EINVAL;

        rtnl_lock();

        tp = ring->private;
        dev = tp->dev;

        if (!netif_running(dev))
                netif_warn(tp, drv, dev, "closed not enable event. \n");
        else
                _rtl8168_enable_event(ring);

        rtnl_unlock();

        return 0;
}
EXPORT_SYMBOL(rtl8168_enable_event);

int rtl8168_disable_event(struct rtl8168_ring *ring)
{
        struct rtl8168_private *tp = ring->private;

        if (!ring->event.allocated)
                return -EINVAL;

        if (ring->direction == RTL8168_CH_DIR_TX)
                goto out;

        rtnl_lock();

        /* Disable interrupt */
        rtl8168_disable_interrupt_by_vector(tp, ring->event.message_id);

        rtnl_unlock();

out:
        ring->event.enabled = 0;

        return 0;
}
EXPORT_SYMBOL(rtl8168_disable_event);

int rtl8168_rss_redirect(struct net_device *ndev,
                         unsigned long flags,
                         struct rtl8168_ring *ring)
{
#ifdef ENABLE_RSS_SUPPORT
        struct rtl8168_private *tp = ring->private;
        int i;

        if (!tp->EnableRss)
                return 0;

        /* Disable RSS if needed */
        /* Update RSS hash table to set all entries point to ring->queue */
        /* Set additional flags as needed. Ex. hash_type */
        /* Enable RSS */

        for (i = 0; i < rtl8168_rss_indir_tbl_entries(tp); i++)
                tp->rss_indir_tbl[i] = 0;

        _rtl8168_config_rss(tp);
#endif
        return 0;
}
EXPORT_SYMBOL(rtl8168_rss_redirect);

int rtl8168_rss_reset(struct net_device *ndev)
{
#ifdef ENABLE_RSS_SUPPORT
        struct rtl8168_private *tp = netdev_priv(ndev);

        if (!tp->EnableRss)
                return 0;

        /* Disable RSS */
        /* Reset RSS hash table */
        /* Enable RSS if that is the default config for driver */

        rtl8168_init_rss(tp);
        _rtl8168_config_rss(tp);
#endif
        return 0;
}
EXPORT_SYMBOL(rtl8168_rss_reset);

struct net_device *rtl8168_get_netdev(struct device *dev)
{
        struct pci_dev *pdev = to_pci_dev(dev);

        /* Get device private data from @dev */
        /* Retrieve struct net_device * from device private data */

        return pci_get_drvdata(pdev);
}
EXPORT_SYMBOL(rtl8168_get_netdev);

int rtl8168_receive_skb(struct net_device *net_dev, struct sk_buff *skb, bool napi)
{
        /* Update interface stats - rx_packets, rx_bytes */
        skb->protocol = eth_type_trans(skb, net_dev);
        return napi ? netif_receive_skb(skb) : netif_rx(skb);
}
EXPORT_SYMBOL(rtl8168_receive_skb);

int rtl8168_register_notifier(struct net_device *net_dev,
                              struct notifier_block *nb)
{
        struct rtl8168_private *tp = netdev_priv(net_dev);

        return blocking_notifier_chain_register(&tp->lib_nh, nb);
}
EXPORT_SYMBOL(rtl8168_register_notifier);

int rtl8168_unregister_notifier(struct net_device *net_dev,
                                struct notifier_block *nb)
{
        struct rtl8168_private *tp = netdev_priv(net_dev);

        return blocking_notifier_chain_unregister(&tp->lib_nh, nb);
}
EXPORT_SYMBOL(rtl8168_unregister_notifier);

void rtl8168_lib_reset_prepare(struct rtl8168_private *tp)
{
        blocking_notifier_call_chain(&tp->lib_nh,
                                     RTL8168_NOTIFY_RESET_PREPARE, NULL);
}
EXPORT_SYMBOL(rtl8168_lib_reset_prepare);

void rtl8168_lib_reset_complete(struct rtl8168_private *tp)
{
        blocking_notifier_call_chain(&tp->lib_nh,
                                     RTL8168_NOTIFY_RESET_COMPLETE, NULL);
}
EXPORT_SYMBOL(rtl8168_lib_reset_complete);

#define rtl8168_statistics rtl8168_counters
int rtl8168_lib_get_stats(struct net_device *ndev, struct rtl8168_statistics *stats)
{
        struct rtl8168_private *tp = netdev_priv(ndev);
        struct rtl8168_counters *counters;
        dma_addr_t paddr;
        int rc = -1;

        if (!stats)
                goto out;

        counters = tp->tally_vaddr;
        paddr = tp->tally_paddr;
        if (!counters)
                goto out;

        rc = rtl8168_dump_tally_counter(tp, paddr);
        if (rc < 0)
                goto out;

        *stats = *counters;

out:
        return rc;
}
EXPORT_SYMBOL(rtl8168_lib_get_stats);

int rtl8168_lib_save_regs(struct net_device *ndev, struct rtl8168_regs_save *stats)
{
        struct rtl8168_private *tp = netdev_priv(ndev);
        u32 *pdword;
        int i, max;

        rtnl_lock();

        //macio
        max = R8168_MAC_REGS_SIZE;
        for (i = 0; i < max; i++)
                stats->mac_io[i] = RTL_R8(tp, i);

        //pcie_phy
        max = R8168_EPHY_REGS_SIZE/2;
        for (i = 0; i < max; i++)
                stats->pcie_phy[i] = rtl8168_ephy_read(tp, i);

        //eth_phy
        max = R8168_PHY_REGS_SIZE/2;
        rtl8168_mdio_write(tp, 0x1f, 0x0000);
        for (i = 0; i < max; i++)
                stats->eth_phy[i] = rtl8168_mdio_read(tp, i);

        //eri
        max = R8168_ERI_REGS_SIZE/4;
        for (i = 0; i < max; i++)
                stats->eri_reg[i] = rtl8168_eri_read(tp, i, 4, ERIAR_ExGMAC);

        //pci_reg
        max = R8168_PCI_REGS_SIZE/4;
        for (i = 0; i < max; i++)
                pci_read_config_dword(tp->pci_dev, i, &stats->pci_reg[i]);
#if 0
        //int_miti
        stats->int_miti_rxq0 = RTL_R8(tp, INT_MITI_V2_0_RX);
        stats->int_miti_txq0 = RTL_R8(tp, INT_MITI_V2_0_TX);
        stats->int_miti_rxq1 = RTL_R8(tp, INT_MITI_V2_1_RX);
        stats->int_miti_txq1 = RTL_R8(tp, INT_MITI_V2_1_TX);

        //imr/isr
        stats->imr_new = RTL_R32(tp, IMR0_8168);
        stats->isr_new = RTL_R32(tp, ISR0_8168);

        //tdu/rdu
        stats->tdu_status = RTL_R8(tp, TDU_STA_8168);
        stats->rdu_status = RTL_R16(tp, RDU_STA_8168);
#endif
        //rss rtl8168_eri_write(tp, rss_key_reg + i, 4, *rss_key++, ERIAR_ExGMAC);
        stats->rss_ctrl = rtl8168_eri_read(tp, RSS_CTRL_8168, 4, ERIAR_ExGMAC);
        pdword = (u32*)&stats->rss_key[0];
        for (i=0; i < RTL8168_RSS_KEY_SIZE; i+=4)
                *pdword++ = rtl8168_eri_read(tp, RSS_KEY_8168 + i, 4, ERIAR_ExGMAC);

        pdword = (u32*)&stats->rss_i_table[0];
        for (i=0; i < RTL8168_RSS_INDIR_TBL_SIZE; i++)
                *pdword++ = RTL_R32(tp, Rss_indir_tbl + i*4);

        //stats->rss_queue_num_sel_r = RTL_R16(tp, Q_NUM_CTRL_8168);

        rtnl_unlock();

        return 0;
}
EXPORT_SYMBOL(rtl8168_lib_save_regs);

void rtl8168_init_lib_ring(struct rtl8168_private *tp)
{
        struct rtl8168_ring *ring;

        /* init lib tx ring */
        ring = &tp->lib_tx_ring[1];
        if (ring->allocated) {
                rtl8168_init_tx_ring(ring);
                if (ring->event.enabled)
                        _rtl8168_enable_event(ring);
        }

        /* init lib rx ring */
        ring = &tp->lib_rx_ring[1];
        if (ring->allocated) {
                rtl8168_init_rx_ring(ring);
                if (ring->event.enabled)
                        _rtl8168_enable_event(ring);
        }
}

void rtl8168_lib_tx_interrupt(struct rtl8168_private *tp)
{
        if (tp->lib_tx_ring[1].enabled && db_tx)
                writel_relaxed(1, db_tx);
}

void rtl8168_lib_rx_interrupt(struct rtl8168_private *tp)
{
        if (tp->lib_rx_ring[1].enabled && db_rx)
                writel_relaxed(1, db_rx);
}

/*
int rtl8168_lib_printf_macio_regs(struct net_device *ndev, struct rtl8168_regs_save *stats)
{
        struct rtl8168_private *tp = netdev_priv(ndev);
        int i;

        //00
        for(i=0; i<6; i++)
                printk("mac_id[6] = 0x%x\n", stats->mac_reg.mac_id[i]);
        printk("reg_06 = 0x%x\n", stats->mac_reg.reg_06);
        for(i=0; i<8; i++)
                printk("mar[8] = 0x%x\n", stats->mac_reg.mar[i]);
        //10
        printk("dtccr = 0x%llx\n", stats->mac_reg.dtccr);
        printk("ledsel0 = 0x%x\n", stats->mac_reg.ledsel0);
        printk("legreg = 0x%x\n", stats->mac_reg.legreg);
        printk("tctr3 = 0x%x\n", stats->mac_reg.tctr3);
        //20
        printk("txq0_desc_addr = 0x%llx\n", stats->mac_reg.txq0_desc_addr);
        printk("reg_28 = 0x%llx\n", stats->mac_reg.reg_28);
        //30
        printk("rit = 0x%x\n", stats->mac_reg.rit);
        printk("ritc = 0x%x\n", stats->mac_reg.ritc);
        printk("reg_34 = 0x%x\n", stats->mac_reg.reg_34);
        printk("cr = 0x%x\n", stats->mac_reg.cr);
        printk("imr0 = 0x%x\n", stats->mac_reg.imr0);
        printk("isr0 = 0x%x\n", stats->mac_reg.isr0);
        //40
        printk("tcr = 0x%x\n", stats->mac_reg.tcr);
        printk("rcr = 0x%x\n", stats->mac_reg.rcr);
        printk("tctr0 = 0x%x\n", stats->mac_reg.tctr0);
        printk("tctr1 = 0x%x\n", stats->mac_reg.tctr1);
        //50
        printk("cr93c46 = 0x%x\n", stats->mac_reg.cr93c46);
        printk("config0 = 0x%x\n", stats->mac_reg.config0);
        printk("config1 = 0x%x\n", stats->mac_reg.config1);
        printk("config2 = 0x%x\n", stats->mac_reg.config2);
        printk("config3 = 0x%x\n", stats->mac_reg.config3);
        printk("config4 = 0x%x\n", stats->mac_reg.config4);
        printk("config5 = 0x%x\n", stats->mac_reg.config5);
        printk("tdfnr = 0x%x\n", stats->mac_reg.tdfnr);
        printk("timer_int0 = 0x%x\n", stats->mac_reg.timer_int0);
        printk("timer_int1 = 0x%x\n", stats->mac_reg.timer_int1);
        //60
        printk("gphy_mdcmdio = 0x%x\n", stats->mac_reg.gphy_mdcmdio);
        printk("csidr = 0x%x\n", stats->mac_reg.csidr);
        printk("csiar = 0x%x\n", stats->mac_reg.csiar);
        printk("phy_status = 0x%x\n", stats->mac_reg.phy_status);
        printk("config6 = 0x%x\n", stats->mac_reg.config6);
        printk("pmch = 0x%x\n", stats->mac_reg.pmch);
        //70
        printk("eridr = 0x%x\n", stats->mac_reg.eridr);
        printk("eriar = 0x%x\n", stats->mac_reg.eriar);
        printk("config7 = 0x%x\n", stats->mac_reg.config7);
        printk("reg_7a = 0x%x\n", stats->mac_reg.reg_7a);
        printk("ephy_rxerr_cnt = 0x%x\n", stats->mac_reg.ephy_rxerr_cnt);
        //80
        printk("ephy_mdcmdio = 0x%x\n", stats->mac_reg.ephy_mdcmdio);
        printk("ledsel2 = 0x%x\n", stats->mac_reg.ledsel2);
        printk("ledsel1 = 0x%x\n", stats->mac_reg.ledsel1);
        printk("tctr2 = 0x%x\n", stats->mac_reg.tctr2);
        printk("timer_int2 = 0x%x\n", stats->mac_reg.timer_int2);
        //90
        printk("tppoll0 = 0x%x\n", stats->mac_reg.tppoll0);
        printk("reg_91 = 0x%x\n", stats->mac_reg.reg_91);
        printk("reg_92 = 0x%x\n", stats->mac_reg.reg_92);
        printk("led_feature = 0x%x\n", stats->mac_reg.led_feature);
        printk("ledsel3 = 0x%x\n", stats->mac_reg.ledsel3);
        printk("eee_led_config = 0x%x\n", stats->mac_reg.eee_led_config);
        printk("reg_9a = 0x%x\n", stats->mac_reg.reg_9a);
        printk("reg_9c = 0x%x\n", stats->mac_reg.reg_9c);
        //a0
        printk("reg_a0 = 0x%x\n", stats->mac_reg.reg_a0);
        printk("reg_a4 = 0x%x\n", stats->mac_reg.reg_a4);
        printk("reg_a8 = 0x%x\n", stats->mac_reg.reg_a8);
        printk("reg_ac = 0x%x\n", stats->mac_reg.reg_ac);
        //b0
        printk("patch_dbg = 0x%x\n", stats->mac_reg.patch_dbg);
        printk("reg_b4 = 0x%x\n", stats->mac_reg.reg_b4);
        printk("gphy_ocp = 0x%x\n", stats->mac_reg.gphy_ocp);
        printk("reg_bc = 0x%x\n", stats->mac_reg.reg_bc);
        //c0
        printk("reg_c0 = 0x%x\n", stats->mac_reg.reg_c0);
        printk("reg_c4 = 0x%x\n", stats->mac_reg.reg_c4);
        printk("reg_c8 = 0x%x\n", stats->mac_reg.reg_c8);
        printk("otp_cmd = 0x%x\n", stats->mac_reg.otp_cmd);
        printk("otp_pg_config = 0x%x\n", stats->mac_reg.otp_pg_config);
        //d0
        printk("phy_pwr = 0x%x\n", stats->mac_reg.phy_pwr);
        printk("twsi_ctrl = 0x%x\n", stats->mac_reg.twsi_ctrl);
        printk("oob_ctrl = 0x%x\n", stats->mac_reg.oob_ctrl);
        printk("mac_dbgo = 0x%x\n", stats->mac_reg.mac_dbgo);
        printk("mac_dbg = 0x%x\n", stats->mac_reg.mac_dbg);
        printk("reg_d8 = 0x%x\n", stats->mac_reg.reg_d8);
        printk("rms = 0x%x\n", stats->mac_reg.rms);
        printk("efuse_data = 0x%x\n", stats->mac_reg.efuse_data);
        //e0
        printk("cpcr = 0x%x\n", stats->mac_reg.cpcr);
        printk("reg_e2 = 0x%x\n", stats->mac_reg.reg_e2);
        printk("rxq0_desc_addr = 0x%llx\n", stats->mac_reg.rxq0_desc_addr);
        printk("reg_ec = 0x%x\n", stats->mac_reg.reg_ec);
        printk("tx10midle_cnt = 0x%x\n", stats->mac_reg.tx10midle_cnt);
        //f0
        printk("misc0 = 0x%x\n", stats->mac_reg.misc0);
        printk("misc1 = 0x%x\n", stats->mac_reg.misc1);
        printk("timer_int3 = 0x%x\n", stats->mac_reg.timer_int3);
        printk("cmac_ib = 0x%x\n", stats->mac_reg.cmac_ib);
        printk("reg_fc = 0x%x\n", stats->mac_reg.reg_fc);
        printk("sw_rst = 0x%x\n", stats->mac_reg.sw_rst);

        return 0;
}
*/

unsigned int rtl8168_lib_get_num_rx_rings(struct net_device *ndev)
{
        struct rtl8168_private *tp = netdev_priv(ndev);

        return rtl8168_num_lib_rx_rings(tp);
}
EXPORT_SYMBOL(rtl8168_lib_get_num_rx_rings);

unsigned int rtl8168_lib_get_num_tx_rings(struct net_device *ndev)
{
        struct rtl8168_private *tp = netdev_priv(ndev);

        return rtl8168_num_lib_tx_rings(tp);
}
EXPORT_SYMBOL(rtl8168_lib_get_num_tx_rings);

unsigned int rtl8168_lib_get_num_sw_path_rx_descs(struct net_device *ndev)
{
        struct rtl8168_private *tp = netdev_priv(ndev);

        return tp->num_rx_desc;
}
EXPORT_SYMBOL(rtl8168_lib_get_num_sw_path_rx_descs);
