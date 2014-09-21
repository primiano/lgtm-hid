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
#include "HardwareProfile.h"
#include <USB/usb.h>
#include <USB/usb_function_hid.h>
#include "ascii_to_hid.h"

#pragma config PLLDIV = 5  // (20 MHz crystal on PICDEM FS USB board)
#if (USB_SPEED_OPTION == USB_FULL_SPEED)
#pragma config CPUDIV = OSC1_PLL2
#else
#pragma config CPUDIV = OSC3_PLL4
#endif
#pragma config USBDIV = 2
#pragma config FOSC = HSPLL_HS
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config PWRT = ON
#pragma config BOR = ON
#pragma config BORV = 3
#pragma config VREGEN = ON
#pragma config WDT = OFF
#pragma config WDTPS = 32768
#pragma config MCLRE = OFF
#pragma config LPT1OSC = OFF
#pragma config PBADEN = OFF
#pragma config STVREN = ON
#pragma config LVP = OFF
#pragma config XINST = OFF

#pragma udata USB_VARIABLES = 0x500
unsigned char hid_report_in[HID_INT_IN_EP_SIZE];
volatile unsigned char hid_report_out[HID_INT_OUT_EP_SIZE];

#pragma udata
USB_HANDLE g_usb_handle_in;
USB_HANDLE g_usb_handle_out;
BOOL g_blink_status_valid;
DWORD g_usb_led_timer;
ROM const char* g_lgtm_ascii_ptr;
BOOL g_config_mode;
BYTE g_lgtm_strings_idx;
char g_tx_queue[128];

#pragma code
const ROM char CFG_MODE_STR[] = "*-*-* LGTM-HID config *-*-*    ";
const ROM char* ROM LGTM_STRINGS[] = {"LGTM ", "RS-LGTM ",
                                      "2,3-dimethyl-1,3-butadiene-stamp LGTM ",
                                      "stampty stamp LGTM "};
#define NUM_LGTM_STRINGS 4

void BlinkUSBStatus(void);
BOOL SwitchIsPressed(void);
void InitializeSystem(void);
void ProcessIO(void);
void Keyboard(void);
void USBHIDCBSetReportComplete(void);

void main(void) {
  long i = 0;

  memset(g_tx_queue, 0, sizeof(g_tx_queue));
  EEADR = 0;
  EECON1bits.EEPGD = 0;
  EECON1bits.CFGS = 0;
  EECON1bits.RD = 1;
  g_lgtm_strings_idx = EEDATA;
  if (g_lgtm_strings_idx >= NUM_LGTM_STRINGS)
    g_lgtm_strings_idx = 0;
  g_lgtm_ascii_ptr = LGTM_STRINGS[g_lgtm_strings_idx];
  InitializeSystem();

  // If the switch is pressed on boot, enter system config mode.
  g_config_mode = (sw == 0);

  while (1) {
    USBDeviceTasks();   
    ProcessIO();
  }
}

void InitializeSystem(void) {
  ADCON1 |= 0x0F;  // Default all pins to digital
  INTCON2 = 0;
  mInitAllLEDs();
  mInitAllSwitches();

  g_blink_status_valid = TRUE;
  g_usb_handle_in = 0;
  g_usb_handle_out = 0;

  USBDeviceInit();  // usb_device.c.  Initializes USB module SFRs and firmware
                    // variables to known states.
}

void ProcessIO(void) {
  // Blink the LEDs according to the USB device status
  // However, the LEDs are also used temporarily for showing the Num Lock
  // keyboard LED status.  If the host sends an LED state update interrupt
  // out report, or sends it by a SET_REPORT control transfer, then
  // the demo board LEDs are temporarily taken over to show the Num Lock
  // LED state info.  After a countdown timout, the firmware will switch
  // back to showing the USB state on the LEDs, instead of the num lock status.
  if (g_blink_status_valid == TRUE) {
    BlinkUSBStatus();
  } else {
    g_usb_led_timer--;
    if (g_usb_led_timer == 0) {
      g_blink_status_valid = TRUE;
    }
  }

  // User Application USB tasks
  if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1))
    return;

  // Call the function that behaves like a keyboard
  Keyboard();

}

void Keyboard(void) {
  static char* c = g_tx_queue;

  if (SwitchIsPressed()) {
    g_tx_queue[0] = 0;
    if (g_config_mode) {
      ++g_lgtm_strings_idx;
      if (g_lgtm_strings_idx >= NUM_LGTM_STRINGS)
        g_lgtm_strings_idx = 0;
      EEADR = 0;
      EEDATA = g_lgtm_strings_idx;
      EECON1bits.EEPGD = 0;
      EECON1bits.CFGS = 0;
      EECON1bits.WREN = 1;
      EECON2 = 0x55;
      EECON2 = 0xAA;
      EECON1bits.WR = 1;
      strcatpgm2ram(g_tx_queue, CFG_MODE_STR);
    }
    strcatpgm2ram(g_tx_queue, LGTM_STRINGS[g_lgtm_strings_idx]);
    c = g_tx_queue;
  }

  // Check if the IN endpoint is not busy, and if it isn't check if we want to
  // send keystroke data to the host.
  if (!HIDTxHandleBusy(g_usb_handle_in)) {
    memset(hid_report_in, 0, sizeof(hid_report_in));
    if (*c) {
      HIDKey key;
      key = AsciiToHid(*c);
      ++c;
      hid_report_in[0] = key.modifier;
      hid_report_in[2] = key.code;
    }
    // Send the 8 byte packet over USB to the host.
    g_usb_handle_in = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
  }

  // Check if any data was sent from the PC to the keyboard device.  Report
  // descriptor allows
  // host to send 1 byte of data.  Bits 0-4 are LED states, bits 5-7 are unused
  // pad bits.
  // The host can potentially send this OUT report data through the HID OUT
  // endpoint (EP1 OUT),
  // or, alternatively, the host may try to send LED state information by
  // sending a
  // SET_REPORT control transfer on EP0.  See the USBHIDCBSetReportHandler()
  // function.
  if (!HIDRxHandleBusy(g_usb_handle_out)) {
    // Do something useful with the data now.  Data is in the OutBuffer[0].
    // Num Lock LED state is in Bit0.
    if (hid_report_out[0] & 0x01)  // Make LED1 and LED2 match Num Lock state.
    {
      mLED_1_On();
      mLED_2_On();
    } else {
      mLED_1_Off();
      mLED_2_Off();
    }

    // Stop toggling the LEDs, so you can temporily see the Num lock LED state
    // instead.
    // Once the g_usb_led_timer reaches 0, the LEDs will go back to showing USB
    // state instead.
    g_blink_status_valid = FALSE;
    g_usb_led_timer = 140000;

    g_usb_handle_out = HIDRxPacket(HID_EP, (BYTE*)&hid_report_out, 1);
  }

  return;
}

BOOL SwitchIsPressed(void) {
  static long count = 0;
  static BOOL last_state = TRUE;

  if (sw == 0)
    ++count;
  else {
    count = 0;
    last_state = FALSE;
  }

  if (count == 4000 && last_state == FALSE) {
    last_state = TRUE;
    return TRUE;
  }
  return FALSE;
}

void BlinkUSBStatus(void) {
  static WORD led_count = 0;

  if (led_count == 0)
    led_count = 10000U;
  led_count--;

#define mLED_Both_Off() \
  {                     \
    mLED_1_Off();       \
    mLED_2_Off();       \
  }
#define mLED_Both_On() \
  {                    \
    mLED_1_On();       \
    mLED_2_On();       \
  }
#define mLED_Only_1_On() \
  {                      \
    mLED_1_On();         \
    mLED_2_Off();        \
  }
#define mLED_Only_2_On() \
  {                      \
    mLED_1_Off();        \
    mLED_2_On();         \
  }

  if (USBSuspendControl == 1) {
    if (led_count == 0) {
      mLED_1_Toggle();
      if (mGetLED_1()) {
        mLED_2_On();
      } else {
        mLED_2_Off();
      }
    }
  } else {
    if (USBDeviceState == DETACHED_STATE) {
      mLED_Both_Off();
    } else if (USBDeviceState == ATTACHED_STATE) {
      mLED_Both_On();
    } else if (USBDeviceState == POWERED_STATE) {
      mLED_Only_1_On();
    } else if (USBDeviceState == DEFAULT_STATE) {
      mLED_Only_2_On();
    } else if (USBDeviceState == ADDRESS_STATE) {
      if (led_count == 0) {
        mLED_1_Toggle();
        mLED_2_Off();
      }
    } else if (USBDeviceState == CONFIGURED_STATE) {
      if (led_count == 0) {
        mLED_1_Toggle();
        if (mGetLED_1()) {
          mLED_2_Off();
        } else {
          mLED_2_On();
        }
      }
    }
  }

}

// ******************************************************************************************************
// ************** USB Callback Functions
// ****************************************************************
// ******************************************************************************************************
void USBCBSuspend(void) {
  // Example power saving code.  Insert appropriate code here for the desired
  // application behavior.  If the microcontroller will be put to sleep, a
  // process similar to that shown below may be used:

  // ConfigureIOPinsForLowPower();
  // SaveStateOfAllInterruptEnableBits();
  // DisableAllInterruptEnableBits();
  // EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();  //should enable at
  // least USBActivityIF as a wake source
  // Sleep();
  // RestoreStateOfAllPreviouslySavedInterruptEnableBits();  //Preferrably, this
  // should be done in the USBCBWakeFromSuspend() function instead.
  // RestoreIOPinsToNormal();                  //Preferrably, this should be
  // done in the USBCBWakeFromSuspend() function instead.

  // Alternatively, the microcontorller may use clock switching to reduce
  // current consumption.
  // Configure device for low power consumption
  mLED_1_Off();
  mLED_2_Off();
  // Should also configure all other I/O pins for lowest power consumption.
  // Typically this is done by driving unused I/O pins as outputs and driving
  // them high or low.
  // In this example, this is not done however, in case the user is expecting
  // the I/O pins
  // to remain tri-state and has hooked something up to them.
  // Leaving the I/O pins floating will waste power and should not be done in a
  // real application.

  // Note: The clock switching code needed is processor specific, as the
  // clock trees and registers aren't identical accross all PIC18 USB device
  // families.
  // Sleep on sleep, 125kHz selected as microcontroller clock source
  OSCCON = 0x13;
  // IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit
  // is
  // cleared inside the usb_device.c file.  Clearing USBActivityIF here will
  // cause
  // things to not work as intended.
}

void USBCBWakeFromSuspend(void) {
  // If clock switching or other power savings measures were taken when
  // executing the USBCBSuspend() function, now would be a good time to
  // switch back to normal full power run mode conditions.  The host allows
  // 10+ milliseconds of wakeup time, after which the device must be
  // fully back to normal, and capable of receiving and processing USB
  // packets.  In order to do this, the USB module must receive proper
  // clocking (IE: 48MHz clock must be available to SIE for full speed USB
  // operation).
  // Make sure the selected oscillator settings are consistant with USB
  // operation
  // before returning from this function.

  // Switch clock back to main clock source necessary for USB operation
  // Previous clock source was something low frequency as set in the
  // USBCBSuspend() function.
  OSCCON = 0x60;  // Primary clock source selected.
  // Adding a software start up delay will ensure
  // that the primary oscillator and PLL are running before executing any other
  // code.  If the PLL isn't being used, (ex: primary osc = 48MHz externally
  // applied EC)
  // then this code adds a small unnecessary delay, but it is harmless to
  // execute anyway.
  {
    unsigned int pll_startup_counter =
        800;  // Long delay at 31kHz, but ~0.8ms at 48MHz
    while (pll_startup_counter--)
      ;  // Clock will switch over while executing this delay loop
  }
}

void USBCBCheckOtherReq(void) {
  USBCheckHIDRequest();
}

void USBCBInitEP(void) {
  // enable the HID endpoint
  USBEnableEndpoint(HID_EP,
                    USB_IN_ENABLED | USB_OUT_ENABLED | USB_HANDSHAKE_ENABLED |
                        USB_DISALLOW_SETUP);
  // Arm OUT endpoint so we can receive caps lock, num lock, etc. info from host
  g_usb_handle_out = HIDRxPacket(HID_EP, (BYTE*)&hid_report_out, 1);
}

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void* pdata, WORD size) {
  switch (event) {
    case EVENT_TRANSFER:
      // Add application specific callback task or callback function here if
      // desired.
      break;
    case EVENT_SOF:
      break;
    case EVENT_SUSPEND:
      USBCBSuspend();
      break;
    case EVENT_RESUME:
      USBCBWakeFromSuspend();
      break;
    case EVENT_CONFIGURED:
      USBCBInitEP();
      break;
    case EVENT_SET_DESCRIPTOR:
      break;
    case EVENT_EP0_REQUEST:
      USBCBCheckOtherReq();
      break;
    case EVENT_BUS_ERROR:
      break;
    case EVENT_TRANSFER_TERMINATED:
      // Add application specific callback task or callback function here if
      // desired.
      // The EVENT_TRANSFER_TERMINATED event occurs when the host performs a
      // CLEAR
      // FEATURE (endpoint halt) request on an application endpoint which was
      // previously armed (UOWN was = 1).  Here would be a good place to:
      // 1.  Determine which endpoint the transaction that just got terminated
      // was
      //    on, by checking the handle value in the *pdata.
      // 2.  Re-arm the endpoint if desired (typically would be the case for OUT
      //    endpoints).
      break;
    default:
      break;
  }
  return TRUE;
}

// *****************************************************************************
// ************** USB Class Specific Callback Function(s) **********************
// *****************************************************************************
void USBHIDCBSetReportHandler(void) {
  // Prepare to receive the keyboard LED state data through a SET_REPORT
  // control transfer on endpoint 0.  The host should only send 1 byte,
  // since this is all that the report descriptor allows it to send.
  USBEP0Receive(
      (BYTE*)&CtrlTrfData, USB_EP0_BUFF_SIZE, USBHIDCBSetReportComplete);
}

// Secondary callback function that gets called when the above
// control transfer completes for the USBHIDCBSetReportHandler()
void USBHIDCBSetReportComplete(void) {
  // 1 byte of LED state data should now be in the CtrlTrfData buffer.

  // Num Lock LED state is in Bit0.
  if (CtrlTrfData[0] & 0x01)  // Make LED1 and LED2 match Num Lock state.
  {
    mLED_1_On();
    mLED_2_On();
  } else {
    mLED_1_Off();
    mLED_2_Off();
  }

  // Stop toggling the LEDs, so you can temporily see the Num lock LED state
  // instead.
  // Once the g_usb_led_timer reaches 0, the LEDs will go back to showing USB
  // state instead.
  g_blink_status_valid = FALSE;
  g_usb_led_timer = 140000;
}
