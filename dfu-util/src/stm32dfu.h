#ifndef _STM32DFU_H
#define _STM32DFU_H

#include "param.h"

int stm32dfu_do_dnload(struct usb_dev_handle *usb_handle, int interface,
		      const parameters param);
//		      const int xfer_size, const char *fname,const unsigned int address,const int do_execute);
int stm32dfu_erase_all(struct usb_dev_handle *usb_handle,int interface);
int in_array(unsigned char state,unsigned char* states,int states_count);

#endif
