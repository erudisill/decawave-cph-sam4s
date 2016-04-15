/*
 * cph_usb.h
 *
 *  Created on: Apr 14, 2016
 *      Author: jcobb
 */

#ifndef SRC_CPH_CPH_USB_H_
#define SRC_CPH_CPH_USB_H_

volatile bool cph_usb_rx_ready;
void cph_usb_rx_notify(void);


void cph_usb_init(void);
bool cph_usb_data_ready(void);
void cph_usb_data_read(uint8_t *data);




#endif /* SRC_CPH_CPH_USB_H_ */
