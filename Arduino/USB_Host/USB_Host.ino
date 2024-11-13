#include "main.h"
#include "fatfs.h"
#include "usb_host.h"

void setup() {
  // put your setup code here, to run once:

  MX_USB_HOST_Init();
}

void loop() {
  // put your main code here, to run repeatedly:

  MX_USB_HOST_Process();
}
