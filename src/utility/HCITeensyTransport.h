/*
  This file is part of the ArduinoBLE library.
  Copyright (c) 2018 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "HCITransport.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <USBHost_t36.h>


class HCITeensyTransportClass : public HCITransportInterface, public USBDriver {
public:
  HCITeensyTransportClass();
  virtual ~HCITeensyTransportClass();

  // From HCITransportInterface
  virtual int begin();
  virtual void end();

  virtual void wait(unsigned long timeout);

  virtual int available();
  virtual int peek();
  virtual int read();
  virtual size_t write(const uint8_t* data, size_t length);

  // From USBDriver
  virtual bool claim(Device_t *device, int type, const uint8_t *descriptors, uint32_t len);
  virtual void control(const Transfer_t *transfer);
  virtual void disconnect();
  virtual void timer_event(USBDriverTimer *whichTimer);

  // methods for processing the USBDriver.
  static void rx_callback(const Transfer_t *transfer);
  static void rx2_callback(const Transfer_t *transfer);
  static void tx_callback(const Transfer_t *transfer);
  void rx_data(const Transfer_t *transfer);
  void rx2_data(const Transfer_t *transfer);
  void tx_data(const Transfer_t *transfer);
  void init();
  // HCI support functions...
  void sendHCICommand(uint16_t hciCommand, uint16_t cParams, const uint8_t* data);
  inline bool add_to_rxring(uint8_t c);

  setup_t setup;
  Pipe_t mypipes[4] __attribute__ ((aligned(32)));
  Transfer_t mytransfers[7] __attribute__ ((aligned(32)));
  strbuf_t mystring_bufs[2];      // 2 string buffers - one for our device - one for remote device...
  uint16_t        pending_control_ = 0;
  uint16_t        rx_size_ = 0;
  uint16_t        rx2_size_ = 0;
  uint16_t        tx_size_ = 0;
  Pipe_t          *rxpipe_;
  Pipe_t          *rx2pipe_;
  Pipe_t          *txpipe_;
  uint8_t         rxbuf_[64];    // used to receive data from RX, which may come with several packets...

  uint8_t         rx_ring_buffer_[256];
  volatile uint16_t        rx_ring_head_ = 0;
  volatile uint16_t        rx_ring_tail_ = 0;

  uint8_t         rx_packet_data_remaining_ = 0; // how much data remaining
  uint8_t         rx2_packet_data_remaining_ = 0; // how much data remaining
  uint8_t         txbuf_[256];    // buffer to use to send commands to bluetooth
  uint8_t         rx2buf_[64];    // receive buffer from Bulk end point
  uint8_t         rx2buf2_[256];   // receive buffer from Bulk end point
  typedef struct {
      uint16_t    idVendor;
      uint16_t    idProduct;
  } product_vendor_mapping_t;
  static product_vendor_mapping_t pid_vid_mapping[];


};