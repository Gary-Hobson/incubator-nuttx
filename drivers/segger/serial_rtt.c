/****************************************************************************
 * drivers/segger/serial_rtt.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/list.h>
#include <nuttx/serial/serial.h>

#include <SEGGER_RTT.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rtt_dev_s
{
  int channel;
  FAR char *txbuf;
  FAR char *rxbuf;
  struct list_node node;
  struct uart_dev_s uart;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int serial_rtt_setup(FAR struct uart_dev_s *dev);
static void serial_rtt_shutdown(FAR struct uart_dev_s *dev);
static int serial_rtt_attach(FAR struct uart_dev_s *dev);
static void serial_rtt_detach(FAR struct uart_dev_s *dev);
static int serial_rtt_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
static int serial_rtt_receive(FAR struct uart_dev_s *dev,
                              unsigned int *status);
static void serial_rtt_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool serial_rtt_rxavailable(FAR struct uart_dev_s *dev);
static void serial_rtt_send(FAR struct uart_dev_s *dev, int ch);
static void serial_rtt_txint(FAR struct uart_dev_s *dev, bool enable);
static bool serial_rtt_txready(FAR struct uart_dev_s *dev);
static bool serial_rtt_txempty(FAR struct uart_dev_s *dev);
static void serial_rtt_dmasend(FAR struct uart_dev_s *dev);
static void serial_rtt_dmareceive(FAR struct uart_dev_s *dev);
static void serial_rtt_dmarxfree(FAR struct uart_dev_s *dev);
static void serial_rtt_dmatxavail(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct list_node g_rtt_dev_list = LIST_INITIAL_VALUE(g_rtt_dev_list);

static const struct uart_ops_s g_serial_rtt_ops =
{
  .setup = serial_rtt_setup,
  .shutdown = serial_rtt_shutdown,
  .attach = serial_rtt_attach,
  .detach = serial_rtt_detach,
  .ioctl = serial_rtt_ioctl,
  .receive = serial_rtt_receive,
  .rxint = serial_rtt_rxint,
  .dmasend = serial_rtt_dmasend,
  .dmareceive = serial_rtt_dmareceive,
  .dmarxfree = serial_rtt_dmarxfree,
  .dmatxavail = serial_rtt_dmatxavail,
  .rxavailable = serial_rtt_rxavailable,
  .send = serial_rtt_send,
  .txint = serial_rtt_txint,
  .txready = serial_rtt_txready,
  .txempty = serial_rtt_txempty,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: serial_rtt_setup
 ****************************************************************************/

static int serial_rtt_setup(FAR struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: serial_rtt_shutdown
 ****************************************************************************/

static void serial_rtt_shutdown(FAR struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: serial_rtt_attach
 ****************************************************************************/

static int serial_rtt_attach(FAR struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: serial_rtt_detach
 ****************************************************************************/

static void serial_rtt_detach(FAR struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: serial_rtt_ioctl
 ****************************************************************************/

static int serial_rtt_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: serial_rtt_rxint
 ****************************************************************************/

static void serial_rtt_rxint(FAR struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: serial_rtt_receive
 ****************************************************************************/

static int serial_rtt_receive(FAR struct uart_dev_s *dev,
                              unsigned int *status)
{
  return -1;
}

/****************************************************************************
 * Name: serial_rtt_dmarxfree
 ****************************************************************************/

static void serial_rtt_dmarxfree(FAR struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: serial_rtt_txint
 ****************************************************************************/

static void serial_rtt_txint(FAR struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: serial_rtt_txready
 ****************************************************************************/

static bool serial_rtt_txready(FAR struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: serial_rtt_send
 ****************************************************************************/

static void serial_rtt_send(FAR struct uart_dev_s *dev, int ch)
{
}

/****************************************************************************
 * Name: serial_rtt_txempty
 ****************************************************************************/

static bool serial_rtt_txempty(FAR struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: serial_rtt_dmasend
 ****************************************************************************/

static void serial_rtt_dmasend(FAR struct uart_dev_s *dev)
{
  SEGGER_RTT_BUFFER_UP *pRing;
  FAR struct rtt_dev_s *rtt = (FAR struct rtt_dev_s *)dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmatx;
  size_t len = xfer->length + xfer->nlength;
  pRing = (SEGGER_RTT_BUFFER_UP *)((char *)&_SEGGER_RTT.aUp[rtt->channel]
                                   + SEGGER_RTT_UNCACHED_OFF);

  if (rtt->channel)
    {
      pRing->WrOff = (pRing->WrOff + len) % dev->xmit.size;
    }
  else
    {
      SEGGER_RTT_Write(rtt->channel, xfer->buffer, xfer->length);
      SEGGER_RTT_Write(rtt->channel, xfer->nbuffer, xfer->nlength);
    }

  xfer->nbytes = len;
  uart_xmitchars_done(dev);
}

/****************************************************************************
 * Name: serial_rtt_dmareceive
 ****************************************************************************/

static void serial_rtt_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct rtt_dev_s *rtt = (FAR struct rtt_dev_s *)dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  uint32_t len = SEGGER_RTT_HasData(rtt->channel);
  size_t space = xfer->length + xfer->nlength;

  if (len > space)
    {
      len = space;
    }

  if (!rtt->channel)
    {
      SEGGER_RTT_Read(rtt->channel, xfer->buffer, xfer->length);
      SEGGER_RTT_Read(rtt->channel, xfer->nbuffer, xfer->nlength);
    }

  xfer->nbytes = len;
  uart_recvchars_done(dev);
}

/****************************************************************************
 * Name: serial_rtt_dmatxavail
 ****************************************************************************/

static void serial_rtt_dmatxavail(FAR struct uart_dev_s *dev)
{
  uart_xmitchars_dma(dev);
}

/****************************************************************************
 * Name: serial_rtt_rxavailable
 ****************************************************************************/

static bool serial_rtt_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct rtt_dev_s *rtt = (FAR struct rtt_dev_s *)dev->priv;
  return !!SEGGER_RTT_HasData(rtt->channel);
}

/****************************************************************************
 * Name: serial_rtt_loop
 ****************************************************************************/

static int serial_rtt_loop(int argc, FAR char *argv[])
{
  FAR struct rtt_dev_s *rtt;

  while (1)
    {
      int has_data = false;
      list_for_every_entry(&g_rtt_dev_list, rtt, struct rtt_dev_s, node)
        {
          if (SEGGER_RTT_HasData(rtt->channel))
            {
              has_data = true;
              uart_recvchars_dma(&rtt->uart);
            }
        }

      if (!has_data)
        {
          usleep(CONFIG_SERIAL_RTT_POLLING_INTERVAL);
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: serial_rtt_initialize
 ****************************************************************************/

static void serial_rtt_initialize(FAR const char *name,
                                  FAR struct rtt_dev_s *rtt)
{
  if (list_is_empty(&g_rtt_dev_list))
    {
      kthread_create("serial_rtt_daemon", SCHED_PRIORITY_MAX,
                     CONFIG_DEFAULT_TASK_STACKSIZE, serial_rtt_loop, NULL);
    }

  list_add_tail(&g_rtt_dev_list, &rtt->node);
  uart_register(name, &rtt->uart);
}

void console_rtt_initialize(void)
{
  static struct rtt_dev_s rtt;
  rtt.channel = 0;
  rtt.uart.isconsole = true;
  rtt.uart.ops = &g_serial_rtt_ops;
  rtt.uart.priv = &rtt;
  rtt.uart.xmit.size = CONFIG_CONSOLE_RTT_TXBUF_SIZE;
  rtt.uart.xmit.buffer = kmm_malloc(CONFIG_CONSOLE_RTT_TXBUF_SIZE);
  DEBUGASSERT(rtt.uart.xmit.buffer);
  rtt.uart.recv.size = CONFIG_CONSOLE_RTT_RXBUF_SIZE;
  rtt.uart.recv.buffer = kmm_malloc(CONFIG_CONSOLE_RTT_RXBUF_SIZE);
  DEBUGASSERT(rtt.uart.recv.buffer);
  serial_rtt_initialize("/dev/console", &rtt);
}

void serial_rtt_register(FAR const char *name, int channel, size_t txsize,
                         size_t rxsize)
{
  FAR struct rtt_dev_s *rtt = kmm_zalloc(sizeof(struct rtt_dev_s));
  rtt->channel = channel;
  rtt->uart.isconsole = false;
  rtt->uart.ops = &g_serial_rtt_ops;
  rtt->uart.priv = rtt;
  rtt->uart.xmit.size = txsize;
  rtt->txbuf = kmm_zalloc(txsize);
  DEBUGASSERT(rtt->txbuf);
  rtt->uart.xmit.buffer = rtt->txbuf;
  rtt->uart.recv.size = rxsize;
  rtt->rxbuf = kmm_zalloc(rxsize);
  DEBUGASSERT(rtt->rxbuf);
  rtt->uart.recv.buffer = rtt->rxbuf;
  SEGGER_RTT_ConfigDownBuffer(channel, name, rtt->rxbuf, rxsize,
                              SEGGER_RTT_MODE_DEFAULT);
  SEGGER_RTT_ConfigUpBuffer(channel, name, rtt->txbuf, txsize,
                            SEGGER_RTT_MODE_DEFAULT);
  serial_rtt_initialize(name, rtt);
}
