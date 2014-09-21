// Copyright (c) 2014, Primiano Tucci (www.primianotucci.com)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include <USB/usb.h>
#include <USB/usb_function_hid.h>

#pragma romdata

/* Device Descriptor */
ROM USB_DEVICE_DESCRIPTOR device_dsc = {
    0x12,                   // Size of this descriptor in bytes
    USB_DESCRIPTOR_DEVICE,  // DEVICE descriptor type
    0x0200,                 // USB Spec Release Number in BCD format
    0x00,                   // Class Code
    0x00,                   // Subclass code
    0x00,                   // Protocol code
    USB_EP0_BUFF_SIZE,      // Max packet size for EP0, see usb_config.h
    MY_VID,                 // Vendor ID
    MY_PID,                 // Product ID: Keyboard fw demo
    0x0001,                 // Device release number in BCD format
    0x01,                   // Manufacturer string index
    0x02,                   // Product string index
    0x00,                   // Device serial number string index
    0x01                    // Number of possible configurations
};

/* Configuration 1 Descriptor */
ROM BYTE configDescriptor1[] = {
    /* Configuration Descriptor */
    0x09,  // sizeof(USB_CFG_DSC),    // Size of this descriptor in bytes
    USB_DESCRIPTOR_CONFIGURATION,  // CONFIGURATION descriptor type
    DESC_CONFIG_WORD(0x0029),      // Total length of data for this cfg
    1,                             // Number of interfaces in this cfg
    1,                             // Index value of this configuration
    0,                             // Configuration string index
    _DEFAULT | _SELF | _RWU,       // Attributes, see usb_device.h
    50,                            // Max power consumption (2X mA)

    /* Interface Descriptor */
    0x09,  // sizeof(USB_INTF_DSC),   // Size of this descriptor in bytes
    USB_DESCRIPTOR_INTERFACE,  // INTERFACE descriptor type
    0,                         // Interface Number
    0,                         // Alternate Setting Number
    2,                         // Number of endpoints in this intf
    HID_INTF,                  // Class code
    BOOT_INTF_SUBCLASS,        // Subclass code
    HID_PROTOCOL_KEYBOARD,     // Protocol code
    0x02,                      // Interface string index

    /* HID Class-Specific Descriptor */
    0x09,                      // Size of this descriptor in bytes
    DSC_HID,                   // HID descriptor type
    DESC_CONFIG_WORD(0x0111),  // HID Spec Release Number in BCD format (1.11)
    0x00,                      // Country Code (0x00 for Not supported)
    HID_NUM_OF_DSC,            // Number of class descriptors, see usbcfg.h
    DSC_RPT,                   // Report descriptor type
    DESC_CONFIG_WORD(63),      // Size of the report descriptor

    /* Endpoint Descriptor */
    0x07, /*sizeof(USB_EP_DSC)*/
    USB_DESCRIPTOR_ENDPOINT,  // Endpoint Descriptor
    HID_EP | _EP_IN,  // EndpointAddress
    _INTERRUPT,  // Attributes
    DESC_CONFIG_WORD(8),  // size
    0x01,  // Interval

    /* Endpoint Descriptor */
    0x07, /*sizeof(USB_EP_DSC)*/
    USB_DESCRIPTOR_ENDPOINT,  // Endpoint Descriptor
    _INTERRUPT,  // Attributes
    HID_EP | _EP_OUT,  // EndpointAddress
    DESC_CONFIG_WORD(8),  // size
    0x01  // Interval
};

// Language code string descriptor
ROM struct {
  BYTE bLength;
  BYTE bDscType;
  WORD string[1];
} sd000 = {sizeof(sd000), USB_DESCRIPTOR_STRING, {0x0409}};

// Manufacturer string descriptor
ROM struct {
  BYTE bLength;
  BYTE bDscType;
  WORD string[18];
} sd001 = {sizeof(sd001),
           USB_DESCRIPTOR_STRING,
           {'p', 'r', 'i', 'm', 'i', 'a', 'n', 'o', 't', 'u', 'c', 'c', 'i',
            '.', 'c', 'o', 'm'}};

// Product string descriptor
ROM struct {
  BYTE bLength;
  BYTE bDscType;
  WORD string[9];
} sd002 = {sizeof(sd002),
           USB_DESCRIPTOR_STRING,
           {'L', 'G', 'T', 'M', '-', 'H', 'I', 'D'}};

// Class specific descriptor - HID Keyboard
ROM struct {
  BYTE report[HID_RPT01_SIZE];
} hid_rpt01 = {
      {0x05, 0x01,  // USAGE_PAGE (Generic Desktop)
       0x09, 0x06,  // USAGE (Keyboard)
       0xa1, 0x01,  // COLLECTION (Application)
       0x05, 0x07,  //   USAGE_PAGE (Keyboard)
       0x19, 0xe0,  //   USAGE_MINIMUM (Keyboard LeftControl)
       0x29, 0xe7,  //   USAGE_MAXIMUM (Keyboard Right GUI)
       0x15, 0x00,  //   LOGICAL_MINIMUM (0)
       0x25, 0x01,  //   LOGICAL_MAXIMUM (1)
       0x75, 0x01,  //   REPORT_SIZE (1)
       0x95, 0x08,  //   REPORT_COUNT (8)
       0x81, 0x02,  //   INPUT (Data,Var,Abs)
       0x95, 0x01,  //   REPORT_COUNT (1)
       0x75, 0x08,  //   REPORT_SIZE (8)
       0x81, 0x03,  //   INPUT (Cnst,Var,Abs)
       0x95, 0x05,  //   REPORT_COUNT (5)
       0x75, 0x01,  //   REPORT_SIZE (1)
       0x05, 0x08,  //   USAGE_PAGE (LEDs)
       0x19, 0x01,  //   USAGE_MINIMUM (Num Lock)
       0x29, 0x05,  //   USAGE_MAXIMUM (Kana)
       0x91, 0x02,  //   OUTPUT (Data,Var,Abs)
       0x95, 0x01,  //   REPORT_COUNT (1)
       0x75, 0x03,  //   REPORT_SIZE (3)
       0x91, 0x03,  //   OUTPUT (Cnst,Var,Abs)
       0x95, 0x06,  //   REPORT_COUNT (6)
       0x75, 0x08,  //   REPORT_SIZE (8)
       0x15, 0x00,  //   LOGICAL_MINIMUM (0)
       0x25, 0x65,  //   LOGICAL_MAXIMUM (101)
       0x05, 0x07,  //   USAGE_PAGE (Keyboard)
       0x19, 0x00,  //   USAGE_MINIMUM (Reserved (no event indicated))
       0x29, 0x65,  //   USAGE_MAXIMUM (Keyboard Application)
       0x81, 0x00,  //   INPUT (Data,Ary,Abs)
       0xc0}        // End Collection
};

// Array of configuration descriptors
ROM BYTE* ROM USB_CD_Ptr[] = {(ROM BYTE * ROM) & configDescriptor1};

// Array of string descriptors
ROM BYTE* ROM USB_SD_Ptr[] = {(ROM BYTE * ROM) & sd000,
                              (ROM BYTE * ROM) & sd001,
                              (ROM BYTE * ROM) & sd002};
