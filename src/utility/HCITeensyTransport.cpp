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

#if defined(TEENSYDUINO) && (__IMXRT1062__) || (ARDUINO_TEENSY36)

#include "HCITeensyTransport.h"

//#define DEBUG_BT
//#define DEBUG_BT_VERBOSE

#ifndef DEBUG_BT
#undef DEBUG_BT_VERBOSE
void inline DBGPrintf(...) {};
#else
#define DBGPrintf Serial.printf
#endif

USBHost myusb;
USBHub hub1(myusb);

HCITeensyTransportClass HCITeensyTransport;
HCITransportInterface& HCITransport = HCITeensyTransport;

static elapsedMillis em_rx_tx2 = 0;
static elapsedMillis em_rx_tx = 0;

#define HCI_COMMAND_PKT   0x01
#define HCI_ACLDATA_PKT   0x02
#define HCI_EVENT_PKT     0x04
#define HCI_SECURITY_PKT  0x06

HCITeensyTransportClass::HCITeensyTransportClass()
{
  init();
}

HCITeensyTransportClass::~HCITeensyTransportClass()
{
}

void HCITeensyTransportClass::init()
{
    contribute_Pipes(mypipes, sizeof(mypipes) / sizeof(Pipe_t));
    contribute_Transfers(mytransfers, sizeof(mytransfers) / sizeof(Transfer_t));
    contribute_String_Buffers(mystring_bufs, sizeof(mystring_bufs) / sizeof(strbuf_t));
    driver_ready_for_device(this);
}


int HCITeensyTransportClass::begin()
{
  // make sure we are initialized. 
  //btStarted(); // this somehow stops the arduino ide from initializing bluedroid

  DBGPrintf("HCITeensyTransportClass::begin() called\n");
  myusb.begin();

  elapsedMillis em;
  while (em < 1000) {
    myusb.Task();
    if (device) return 1;
  }

  return -1;
}

//12 01 00 02 FF 01 01 40 5C 0A E8 21 12 01 01 02 03 01
//VendorID = 0A5C, ProductID = 21E8, Version = 0112
//Class/Subclass/Protocol = 255 / 1 / 1
HCITeensyTransportClass::product_vendor_mapping_t HCITeensyTransportClass::pid_vid_mapping[] = {
    { 0xA5C, 0x21E8 }
};



bool HCITeensyTransportClass::claim(Device_t *dev, int type, const uint8_t *descriptors, uint32_t len)
{
    DBGPrintf("HCITeensyTransportClass::claim(%p, %d, %p, %u) called\n", dev, type, descriptors, len);
    DBGPrintf("\tclass:%x this=%x vid:pid=%x:%x\n", dev->bDeviceClass, (uint32_t)this, dev->idVendor,  dev->idProduct);
    // only claim at device level

    if (type != 0) return false; // claim at the device level

    // Lets try to support the main USB Bluetooth class...
    // http://www.usb.org/developers/defined_class/#BaseClassE0h
    if (dev->bDeviceClass != 0xe0)  {
        bool special_case_device = false;
        for (uint8_t i = 0; i < (sizeof(pid_vid_mapping) / sizeof(pid_vid_mapping[0])); i++) {
            if ((pid_vid_mapping[i].idVendor == dev->idVendor) && (pid_vid_mapping[i].idProduct == dev->idProduct)) {
                special_case_device = true;
                break;
            }
        }
        if (!special_case_device) return false;
    }
    if ((dev->bDeviceSubClass != 1) || (dev->bDeviceProtocol != 1)) return false; // Bluetooth Programming Interface

    DBGPrintf("HCITeensyTransportClass claim this=%x vid:pid=%x:%x\n    ", (uint32_t)this, dev->idVendor,  dev->idProduct);
    if (len > 512) {
        DBGPrintf("  Descriptor length %d only showing first 512\n    ");
        len = 512;
    }
    for (uint16_t i = 0; i < len; i++) {
        DBGPrintf("%02x ", descriptors[i]);
        if ((i & 0x3f) == 0x3f) DBGPrintf("\n    ");
    }
    DBGPrintf("\n  ");

    // Lets try to process the first Interface and get the end points...
    // Some common stuff for both XBoxs
    uint32_t count_end_points = descriptors[4];
    if (count_end_points < 2) return false;
    uint32_t rxep = 0;
    uint32_t rx2ep = 0;
    uint32_t txep = 0;
    uint8_t rx_interval = 0;
    uint8_t rx2_interval = 0;
    uint8_t tx_interval = 0;
    rx_size_ = 0;
    rx2_size_ = 0;
    tx_size_ = 0;
    uint32_t descriptor_index = 9;
    while (count_end_points-- /*&& ((rxep == 0) || txep == 0) */) {
        if (descriptors[descriptor_index] != 7) return false; // length 7
        if (descriptors[descriptor_index + 1] != 5) return false; // ep desc
        if ((descriptors[descriptor_index + 4] <= 64)
                && (descriptors[descriptor_index + 5] == 0)) {
            // have a bulk EP size
            if (descriptors[descriptor_index + 2] & 0x80 ) {
                if (descriptors[descriptor_index + 3] == 3)     { // Interrupt
                    rxep = descriptors[descriptor_index + 2];
                    rx_size_ = descriptors[descriptor_index + 4];
                    rx_interval = descriptors[descriptor_index + 6];
                } else if  (descriptors[descriptor_index + 3] == 2)     { // bulk
                    rx2ep = descriptors[descriptor_index + 2];
                    rx2_size_ = descriptors[descriptor_index + 4];
                    rx2_interval = descriptors[descriptor_index + 6];
                }
            } else {
                txep = descriptors[descriptor_index + 2];
                tx_size_ = descriptors[descriptor_index + 4];
                tx_interval = descriptors[descriptor_index + 6];
            }
        }
        descriptor_index += 7;  // setup to look at next one...
    }
    if ((rxep == 0) || (txep == 0)) {
        USBHDBGSerial.printf("Bluetooth end points not found: %d %d\n", rxep, txep);
        return false; // did not find two end points.
    }
    DBGPrintf("    rxep=%d(%d) txep=%d(%d) rx2ep=%d(%d)\n", rxep & 15, rx_size_, txep, tx_size_,
              rx2ep & 15, rx2_size_);

    rxpipe_ = new_Pipe(dev, 3, rxep & 15, 1, rx_size_, rx_interval);
    if (!rxpipe_) return false;
    txpipe_ = new_Pipe(dev, 3, txep, 0, tx_size_, tx_interval);
    if (!txpipe_) {
        //free_Pipe(rxpipe_);
        return false;
    }
    rx2pipe_ = new_Pipe(dev, 2, rx2ep & 15, 1, rx2_size_, rx2_interval);
    if (!rx2pipe_)  {
        // Free other pipes...
        return false;
    }

    rxpipe_->callback_function = rx_callback;
    queue_Data_Transfer(rxpipe_, rxbuf_, rx_size_, this);

    rx2pipe_->callback_function = rx2_callback;
    queue_Data_Transfer(rx2pipe_, rx2buf_, rx2_size_, this);

    txpipe_->callback_function = tx_callback;

    // Send out the reset
    device = dev; // yes this is normally done on return from this but should not hurt if we do it here.
    //sendResetHCI();
    //pending_control_ = PC_RESET;

    return true;
}
void HCITeensyTransportClass::disconnect()
{
    USBHDBGSerial.printf("Bluetooth Disconnect");
    // lets clear out any active connecitons
}

void HCITeensyTransportClass::timer_event(USBDriverTimer *whichTimer)
{
    //if (timer_connection_) timer_connection_->timer_event();
}


void HCITeensyTransportClass::control(const Transfer_t *transfer)
{
#ifdef DEBUG_BT_VERBOSE
    DBGPrintf("    Control callback (bluetooth): %d : ", pending_control_);
    uint8_t *buffer = (uint8_t*)transfer->buffer;
    for (uint8_t i = 0; i < transfer->length; i++) DBGPrintf("%02x ", buffer[i]);
    DBGPrintf("\n");
#endif
}

void HCITeensyTransportClass::rx_callback(const Transfer_t *transfer)
{
    if (!transfer->driver) return;
    ((HCITeensyTransportClass *)(transfer->driver))->rx_data(transfer);
}

void HCITeensyTransportClass::rx2_callback(const Transfer_t *transfer)
{
    if (!transfer->driver) return;
    ((HCITeensyTransportClass *)(transfer->driver))->rx2_data(transfer);
}

void HCITeensyTransportClass::tx_callback(const Transfer_t *transfer)
{
    if (!transfer->driver) return;
    ((HCITeensyTransportClass *)(transfer->driver))->tx_data(transfer);
}


bool  HCITeensyTransportClass::add_to_rxring(uint8_t c)
{

  uint16_t newhead = rx_ring_head_ + 1;
  if (newhead >= sizeof(rx_ring_buffer_)) newhead = 0;
  if (newhead == rx_ring_tail_) return false; // no room
  rx_ring_head_ = newhead;
  rx_ring_buffer_[rx_ring_head_] = c;
  return true;
}


void HCITeensyTransportClass::rx_data(const Transfer_t *transfer)
{
  uint32_t len = transfer->length - ((transfer->qtd.token >> 16) & 0x7FFF);
  DBGPrintf("<<(EVT, %u, H:%u T:%u)", (uint32_t)em_rx_tx, rx_ring_head_, rx_ring_tail_);
  em_rx_tx = 0;
  uint8_t *buffer = (uint8_t*)transfer->buffer;
  for (uint8_t i = 0; i < len; i++) DBGPrintf("%02X ", buffer[i]);
  DBGPrintf("\n");

  //HCI EVENT RX <- 040E0405012000
  // We are simply going to stash this into big buffer
  // that the other side can read from
  if (len > 0) { 
    // bugbug should check for failure
    // Add EVENT indicator at beginning of each event
    if (rx_packet_data_remaining_ == 0) {
      add_to_rxring(HCI_EVENT_PKT);
      rx_packet_data_remaining_ = rxbuf_[1] + 2;  // length of data plus the two bytes at start...
    }
    rx_packet_data_remaining_ -= len;
    for (uint8_t i = 0; i < len; i++) add_to_rxring(buffer[i]);
  }

  // And quueue the rx back up
  queue_Data_Transfer(rxpipe_, rxbuf_, rx_size_, this);
}

#define ATT_OP_ERROR              0x01
#define ATT_OP_MTU_REQ            0x02
#define ATT_OP_MTU_RESP           0x03
#define ATT_OP_FIND_INFO_REQ      0x04
#define ATT_OP_FIND_INFO_RESP     0x05
#define ATT_OP_FIND_BY_TYPE_REQ   0x06
#define ATT_OP_FIND_BY_TYPE_RESP  0x07
#define ATT_OP_READ_BY_TYPE_REQ   0x08
#define ATT_OP_READ_BY_TYPE_RESP  0x09
#define ATT_OP_READ_REQ           0x0a
#define ATT_OP_READ_RESP          0x0b
#define ATT_OP_READ_BLOB_REQ      0x0c
#define ATT_OP_READ_BLOB_RESP     0x0d
#define ATT_OP_READ_MULTI_REQ     0x0e
#define ATT_OP_READ_MULTI_RESP    0x0f
#define ATT_OP_READ_BY_GROUP_REQ  0x10
#define ATT_OP_READ_BY_GROUP_RESP 0x11
#define ATT_OP_WRITE_REQ          0x12
#define ATT_OP_WRITE_RESP         0x13
#define ATT_OP_WRITE_CMD          0x52
#define ATT_OP_PREP_WRITE_REQ     0x16
#define ATT_OP_PREP_WRITE_RESP    0x17
#define ATT_OP_EXEC_WRITE_REQ     0x18
#define ATT_OP_EXEC_WRITE_RESP    0x19
#define ATT_OP_HANDLE_NOTIFY      0x1b
#define ATT_OP_HANDLE_IND         0x1d
#define ATT_OP_HANDLE_CNF         0x1e
#define ATT_OP_SIGNED_WRITE_CMD   0xd2

#define ATT_ECODE_INVALID_HANDLE       0x01
#define ATT_ECODE_READ_NOT_PERM        0x02
#define ATT_ECODE_WRITE_NOT_PERM       0x03
#define ATT_ECODE_INVALID_PDU          0x04
#define ATT_ECODE_AUTHENTICATION       0x05
#define ATT_ECODE_REQ_NOT_SUPP         0x06
#define ATT_ECODE_INVALID_OFFSET       0x07
#define ATT_ECODE_AUTHORIZATION        0x08
#define ATT_ECODE_PREP_QUEUE_FULL      0x09
#define ATT_ECODE_ATTR_NOT_FOUND       0x0a
#define ATT_ECODE_ATTR_NOT_LONG        0x0b
#define ATT_ECODE_INSUFF_ENCR_KEY_SIZE 0x0c
#define ATT_ECODE_INVAL_ATTR_VALUE_LEN 0x0d
#define ATT_ECODE_UNLIKELY             0x0e
#define ATT_ECODE_INSUFF_ENC           0x0f
#define ATT_ECODE_UNSUPP_GRP_TYPE      0x10
#define ATT_ECODE_INSUFF_RESOURCES     0x11

void decode_att_data(const uint8_t *data) {
#ifdef DEBUG_BT
  //                 0  1  2  3  4  5  6  7 MT
  //>>(ACLDATA, 26):40 00 0B 00 07 00 04 00 10 01 00 FF FF 00 28 
  //<<(ACLDATA, 30):40 20 12 00 0E 00 04 00 11 06 01 00 07 00 00 18 08 00 08 00 01 18 
  //>>(ACLDATA, 14):40 00 0B 00 07 00 04 00 10 09 00 FF FF 00 28 
  //<<(ACLDATA, 30):40 20 12 00 0E 00 04 00 11 06 09 00 11 00 0A 18 12 00 15 00 0F 18 
  //>>(ACLDATA, 30):40 00 0B 00 07 00 04 00 10 16 00 FF FF 00 28 
  //<<(ACLDATA, 30):40 20 0C 00 08 00 04 00 11 06 16 00 23 00 12 18 
  //>>(ACLDATA, 14):40 00 0B 00 07 00 04 00 10 24 00 FF FF 00 28 
  //<<(ACLDATA, 30):40 20 09 00 05 00 04 00 01 10 24 00 80 
  //>>(ACLDATA, 14):40 00 09 00 05 00 04 00 04 04 00 05 00 

  uint16_t data_len = data[4] + (data[5] << 8);
  uint8_t mtu = data[8];
  switch (mtu) {
    case ATT_OP_ERROR:
      //                 0  1  2  3  4  5  6  7  8  9 10 11 12
      //<<(ACLDATA, 30):40 20 09 00 05 00 04 00 01 10 24 00 80 
      DBGPrintf("\t** ATT_ERROR **: OP:%x handle:%x error:%02x ", data[9], data[10] + (data[11] << 8), data[12]);
      switch (data[12]) {
        case ATT_ECODE_INVALID_HANDLE: DBGPrintf("- INVALID_HANDLE\n"); break;
        case ATT_ECODE_READ_NOT_PERM: DBGPrintf("- READ_NOT_PERM\n"); break;
        case ATT_ECODE_WRITE_NOT_PERM: DBGPrintf("- WRITE_NOT_PERM\n"); break;
        case ATT_ECODE_INVALID_PDU: DBGPrintf("- INVALID_PDU\n"); break;
        case ATT_ECODE_AUTHENTICATION: DBGPrintf("- AUTHENTICATION\n"); break;
        case ATT_ECODE_REQ_NOT_SUPP: DBGPrintf("- REQ_NOT_SUPP\n"); break;
        case ATT_ECODE_INVALID_OFFSET: DBGPrintf("- INVALID_OFFSET\n"); break;
        case ATT_ECODE_AUTHORIZATION: DBGPrintf("- AUTHORIZATION\n"); break;
        case ATT_ECODE_PREP_QUEUE_FULL: DBGPrintf("- PREP_QUEUE_FULL\n"); break;
        case ATT_ECODE_ATTR_NOT_FOUND: DBGPrintf("- ATTR_NOT_FOUND\n"); break;
        case ATT_ECODE_ATTR_NOT_LONG: DBGPrintf("- ATTR_NOT_LONG\n"); break;
        case ATT_ECODE_INSUFF_ENCR_KEY_SIZE: DBGPrintf("- INSUFF_ENCR_KEY_SIZE\n"); break;
        case ATT_ECODE_INVAL_ATTR_VALUE_LEN: DBGPrintf("- INVAL_ATTR_VALUE_LEN\n"); break;
        case ATT_ECODE_UNLIKELY: DBGPrintf("- UNLIKELY\n"); break;
        case ATT_ECODE_INSUFF_ENC: DBGPrintf("- INSUFF_ENC\n"); break;
        case ATT_ECODE_UNSUPP_GRP_TYPE: DBGPrintf("- UNSUPP_GRP_TYPE\n"); break;
        case ATT_ECODE_INSUFF_RESOURCES: DBGPrintf("- INSUFF_RESOURCES\n"); break;
        default: DBGPrintf("\n");
      }
      break;
    case ATT_OP_MTU_REQ:
      DBGPrintf("\t** MTU_REQ **: MTU: %x\n", data[9] + (data[10] << 8));
      break;
    case ATT_OP_MTU_RESP:
      DBGPrintf("\t** MTU_RESP **: MTU: %x\n", data[9] + (data[10] << 8));
      break;
    case ATT_OP_FIND_INFO_REQ:
      //>>(ACLDATA, 14):40 00 09 00 05 00 04 00 04 04 00 05 00 
      DBGPrintf("\t** FIND_INFO_REQ **: Starting handle:%04x ending handle:%04x\n", data[9] + (data[10] << 8), data[11] + (data[12] << 8));
      break;
    case ATT_OP_FIND_INFO_RESP:
      //<<(ACLDATA, 30):40 20 0A 00 06 00 04 00 05 01 04 00 03 28 
      DBGPrintf("\t** FIND_INFO_RESP **: Format:%x\n", data[9]);
      {
        uint8_t i = 10;
        while (i < (data_len + 8)) {
          if (data[9] == 1) {
            DBGPrintf("\t\tHandle:%02x UUID:%04x\n", data[i] + (data[i+1] << 8), data[2] + (data[i+3] << 8));
            i += 4;
          } else {
            DBGPrintf("\t\tHandle:%x UUID:", data[i] + (data[i+1] << 8));
            i += 2;
            for (uint8_t j = 0; j < 16; j++) DBGPrintf("%02x", data[i++]);
            DBGPrintf("\n");
          }
        }
      }
      break;
    case ATT_OP_FIND_BY_TYPE_REQ:
      DBGPrintf("\t** FIND_BY_TYPE_REQ **:");
      break;
    case ATT_OP_FIND_BY_TYPE_RESP:
      DBGPrintf("\t** FIND_BY_TYPE_RESP **:");
      break;

    case ATT_OP_READ_BY_TYPE_REQ:
      //                 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14
      //>>(ACLDATA, 45):40 00 0B 00 07 00 04 00 08 01 00 07 00 03 28 
      DBGPrintf("\t** READ_BY_TYPE_REQ **: Starting handle:%04x ending handle:%04x Attribute Type:", data[9] + (data[10] << 8), data[11] + (data[12] << 8));
      if (data_len == 7) DBGPrintf("%04x\n", data[13] + (data[14] << 8)); 
      else {
        for (uint8_t i = 0; i < 16; i++) DBGPrintf("%02x\n", data[13+i]);
        DBGPrintf("\n");
      }
      break;

    case ATT_OP_READ_BY_TYPE_RESP:
      //                 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 
      //<<(ACLDATA, 30):40 20 14 00 10 00 04 00 09 07 02 00 02 03 00 00 2A 04 00 02 05 00 01 2A 
      DBGPrintf("\t** READ_BY_TYPE_RESP **: len per:%02x\n", data[9]);
      {
        uint8_t i = 10;
        while (i < (data_len + 8)) {
          DBGPrintf("\t\tAtr Handle:%04x Data:", data[i] + (data[i+1] << 8));
          i += 2;
          for (uint8_t j = 2; j < data[9]; j++) DBGPrintf(" %02x", data[i++]);
          DBGPrintf("\n");
        }
      }
      break;

    case ATT_OP_READ_REQ:
      DBGPrintf("\t** READ_REQ **:");
      break;
    case ATT_OP_READ_RESP:
      DBGPrintf("\t** READ_RESP **:");
      break;
    case ATT_OP_READ_BLOB_REQ:
      DBGPrintf("\t** READ_BLOB_REQ **:");
      break;
    case ATT_OP_READ_BLOB_RESP:
      DBGPrintf("\t** READ_BLOB_RESP **:");
      break;
    case ATT_OP_READ_MULTI_REQ:
      DBGPrintf("\t** READ_MULTI_REQ **:");
      break;
    case ATT_OP_READ_MULTI_RESP:
      DBGPrintf("\t** READ_MULTI_RESP **:");
      break;

    case ATT_OP_READ_BY_GROUP_REQ:
      //>>(ACLDATA, 26):40 00 0B 00 07 00 04 00 10 01 00 FF FF 00 28 
      DBGPrintf("\t** READ_BY_GROUP_REQ **: Starting handle:%04x ending handle:%04x Group Type:", data[9] + (data[10] << 8), data[11] + (data[12] << 8));
      if (data_len == 7) DBGPrintf("%04x\n", data[13] + (data[14] << 8)); 
      else {
        for (uint8_t i = 0; i < 16; i++) DBGPrintf("%02x\n", data[13+i]);
        DBGPrintf("\n");
      }
      break;

    case ATT_OP_READ_BY_GROUP_RESP:
      //                 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21
      //<<(ACLDATA, 30):40 20 12 00 0E 00 04 00 11 06 01 00 07 00 00 18 08 00 08 00 01 18 
      DBGPrintf("\t** READ_MULTI_RESP **: len per:%02x\n", data[9]);
      {
        uint8_t i = 10;
        while (i < (data_len + 8)) {
          DBGPrintf("\t\tAtr Handle:%04x End Group Handle:%04x Data:", data[i] + (data[i+1] << 8), data[2] + (data[i+3] << 8));
          i += 4;
          for (uint8_t j = 4; j < data[9]; j++) DBGPrintf(" %02x", data[i++]);
          DBGPrintf("\n");
        }
      }
      break;
    default: 
      //                 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21
      //<<(ACLDATA, 30):40 20 12 00 0E 00 04 00 11 06 01 00 07 00 00 18 08 00 08 00 01 18 
      DBGPrintf("\t** ??? %0xx ??? **: len per:%02x\n", mtu);
      break;
  }


#endif
}


void HCITeensyTransportClass::rx2_data(const Transfer_t *transfer)
{
  uint32_t len = transfer->length - ((transfer->qtd.token >> 16) & 0x7FFF);
  DBGPrintf("<<(ACLDATA, %u):", (uint32_t)em_rx_tx2);
  em_rx_tx2 = 0;
  uint8_t *buffer = (uint8_t*)transfer->buffer;
  for (uint8_t i = 0; i < len; i++) DBGPrintf("%02X ", buffer[i]);
  DBGPrintf("\n");
  decode_att_data(buffer);
  // Not sure what to do with it now...
  // 40 20 07 00 03 00 04 00 03 12 00 
  if (len > 0) { 
    // bugbug should check for failure
    // Add EVENT indicator at beginning of each event
    // Not sure yet if these split up or not.  Assume not for start
    add_to_rxring(HCI_ACLDATA_PKT);
    rx2_packet_data_remaining_ = rxbuf_[1] + 2;  // length of data plus the two bytes at start...
    for (uint8_t i = 0; i < len; i++) add_to_rxring(buffer[i]);
  }

  queue_Data_Transfer(rx2pipe_, rx2buf_, rx2_size_, this);

}

void HCITeensyTransportClass::tx_data(const Transfer_t *transfer)
{
  uint32_t len = transfer->length - ((transfer->qtd.token >> 16) & 0x7FFF);
  uint8_t *buffer = (uint8_t*)transfer->buffer;
  DBGPrintf("tx_data callback (bluetooth):");
  for (uint8_t i = 0; i < len; i++) DBGPrintf("%02X ", buffer[i]);
  DBGPrintf("\n");
}



void HCITeensyTransportClass::end()
{
}

void HCITeensyTransportClass::wait(unsigned long timeout)
{
  elapsedMillis em;
  while ((rx_ring_head_ == rx_ring_tail_) && (em < timeout)) {
    myusb.Task();
    yield();
  }
}

int HCITeensyTransportClass::available()
{
  // BUGBUG - not right, but they only check for 0 or not 0...
  return (rx_ring_head_ == rx_ring_tail_)? 0 : 1;
}

// never called
int HCITeensyTransportClass::peek()
{
  return -1;
}

int HCITeensyTransportClass::read()
{
  if (!device) return -1;
  if (rx_ring_head_ == rx_ring_tail_) return -1;
  if (++rx_ring_tail_ >= sizeof(rx_ring_buffer_)) rx_ring_tail_ = 0;
  return rx_ring_buffer_[rx_ring_tail_];
}

size_t HCITeensyTransportClass::write(const uint8_t* data, size_t length)
{
  // guessing the first byte tells us something like channel. 
  if (data[0] == HCI_COMMAND_PKT) {
    //HCI COMMAND TX -> 01030C00
    //HCI COMMAND TX -> 01011000
    // lets try to send it out
    length--;
    memcpy(txbuf_, data + 1, length);
    DBGPrintf(">>(CMD, %u):", (uint32_t)em_rx_tx);
    em_rx_tx = 0;
    for (uint8_t i = 0; i < length; i++) DBGPrintf("%02X ", txbuf_[i]);
    DBGPrintf("\n");
    if (device == nullptr) {
        // something wrong:
        USBHDBGSerial.printf("\n !!!!!!!!!!! HCITeensyTransportClass::sendHCICommand called with device == nullptr\n");
        return 0; // don't send it. 
    }
    mk_setup(setup, 0x20, 0x0, 0, 0, length);
    queue_Control_Transfer(device, &setup, txbuf_, this);
    return length+1;
  } else if (data[0] == HCI_ACLDATA_PKT) {

    // This should go to the BULK end point
    length--;
    memcpy(txbuf_, data + 1, length);
    DBGPrintf(">>(ACLDATA, %u):", (uint32_t)em_rx_tx);
    em_rx_tx = 0;
    for (uint8_t i = 0; i < length; i++) DBGPrintf("%02X ", txbuf_[i]);
    DBGPrintf("\n");
    decode_att_data(txbuf_);
    queue_Data_Transfer(txpipe_, txbuf_, length, this);
    return length+1;

  }
  return 0;
}


#endif
