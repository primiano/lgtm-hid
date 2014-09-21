lgtm-hid: The ultimate code review gadget
====

This is a simple, Uber-geek USB gadget for ultimate code reviewers.  
It is essentially a single-key USB-HID keyboard implemented using a Microchip PIC 18F4550.  
When pressed, the gadget emits the scan code for "LGTM", saving up to 75% of your productivity as a code reviewer for rubber-stamp LGTMs.

**Seriously?**  
You might be wondering if I am serious. Yes, this is not a joke. This is an actual gadget (schematics and firmware available below).  
If you have done code-reviews for a while, you probably feel the need of hacking one yourself at this point :)

![lgtm-hid](https://cloud.githubusercontent.com/assets/7137473/4348920/b12e2b40-41a1-11e4-9b87-4795c3a420c3.JPG)

**Configuration**  
If the key is pressed while the gadget is being inserted in the USB port, it enters a "configuration mode".
This mode allows to change the generated scan codes (using the only gadget button) to emit one of the following:

* LGTM
* RS-LGTM
* 2,3-dimethyl-1,3-butadiene-stamp LGTM
* stampty stamp LGTM

Just keep pressing the key to select the new default string and unplug the device afterwards. The setting will be stored persistently in the device's EEPROM.

![lgtm-hid pcb](https://cloud.githubusercontent.com/assets/7137473/4348919/b126375a-41a1-11e4-9440-baf54be22acb.JPG)
