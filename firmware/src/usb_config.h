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
#ifndef USB_CONFIG_H
#define USB_CONFIG_H

#define USB_EP0_BUFF_SIZE 8  // Valid Options: 8, 16, 32, or 64 bytes.
                             // Using larger options take more SRAM, but
                             // does not provide much advantage in most types
// of applications.  Exceptions to this, are applications
// that use EP0 IN or OUT for sending large amounts of
// application related data.

#define USB_MAX_NUM_INT 1  // For tracking Alternate Setting
#define USB_MAX_EP_NUMBER 1

//#define USB_PING_PONG_MODE USB_PING_PONG__NO_PING_PONG
#define USB_PING_PONG_MODE USB_PING_PONG__FULL_PING_PONG
//#define USB_PING_PONG_MODE USB_PING_PONG__EP0_OUT_ONLY
//#define USB_PING_PONG_MODE USB_PING_PONG__ALL_BUT_EP0		//NOTE: This
// mode is not supported in PIC18F4550 family rev A3 devices

#define USB_POLLING
//#define USB_INTERRUPT

#define USB_PULLUP_OPTION USB_PULLUP_ENABLE
//#define USB_PULLUP_OPTION USB_PULLUP_DISABLED

#define USB_TRANSCEIVER_OPTION USB_INTERNAL_TRANSCEIVER

//#define USB_SPEED_OPTION USB_FULL_SPEED
#define USB_SPEED_OPTION USB_LOW_SPEED  //(not valid option for PIC24F devices)

#define MY_VID 0x04D8
#define MY_PID 0x0055

// Option to enable auto-arming of the status stage of control transfers, if no
//"progress" has been made for the USB_STATUS_STAGE_TIMEOUT value.
// If progress is made (any successful transactions completing on EP0 IN or OUT)
// the timeout counter gets reset to the USB_STATUS_STAGE_TIMEOUT value.
//
// During normal control transfer processing, the USB stack or the application
// firmware will call USBCtrlEPAllowStatusStage() as soon as the firmware is
// finished
// processing the control transfer.  Therefore, the status stage completes as
// quickly as is physically possible.  The USB_ENABLE_STATUS_STAGE_TIMEOUTS
// feature, and the USB_STATUS_STAGE_TIMEOUT value are only relevant, when:
// 1.  The application uses the USBDeferStatusStage() API function, but never
// calls USBCtrlEPAllowStatusStage().  Or:
// 2.  The application uses host to device (OUT) control transfers with data
// stage, and some abnormal error occurs, where the host might try to abort the
// control transfer, before it has sent all of the data it claimed it was going
// to send.
//
// If the application firmware never uses the USBDeferStatusStage() API
// function,
// and it never uses host to device control transfers with data stage, then
// it is not required to enable the USB_ENABLE_STATUS_STAGE_TIMEOUTS feature.

#define USB_ENABLE_STATUS_STAGE_TIMEOUTS  // Comment this out to disable this
                                          // feature.

// Section 9.2.6 of the USB 2.0 specifications indicate that:
// 1.  Control transfers with no data stage: Status stage must complete within
//      50ms of the start of the control transfer.
// 2.  Control transfers with (IN) data stage: Status stage must complete within
//      50ms of sending the last IN data packet in fullfilment of the data
//      stage.
// 3.  Control transfers with (OUT) data stage: No specific status stage timing
//      requirement.  However, the total time of the entire control transfer
//      (ex:
//      including the OUT data stage and IN status stage) must not exceed 5
//      seconds.
//
// Therefore, if the USB_ENABLE_STATUS_STAGE_TIMEOUTS feature is used, it is
// suggested
// to set the USB_STATUS_STAGE_TIMEOUT value to timeout in less than 50ms.  If
// the
// USB_ENABLE_STATUS_STAGE_TIMEOUTS feature is not enabled, then the
// USB_STATUS_STAGE_TIMEOUT
// parameter is not relevant.

#define USB_STATUS_STAGE_TIMEOUT \
  (BYTE)45  // Approximate timeout in milliseconds, except when
// USB_POLLING mode is used, and USBDeviceTasks() is called at < 1kHz
// In this special case, the timeout becomes approximately:
// Timeout(in milliseconds) = ((1000 * (USB_STATUS_STAGE_TIMEOUT - 1)) /
// (USBDeviceTasks() polling frequency in Hz))

#define USB_SUPPORT_DEVICE

#define USB_NUM_STRING_DESCRIPTORS 3

#define USB_USE_HID

#define HID_INTF_ID 0x00
#define HID_EP 1
#define HID_INT_OUT_EP_SIZE 1
#define HID_INT_IN_EP_SIZE 8
#define HID_NUM_OF_DSC 1
#define HID_RPT01_SIZE 63
#define USER_SET_REPORT_HANDLER USBHIDCBSetReportHandler

#endif  // USB_CONFIG_H
