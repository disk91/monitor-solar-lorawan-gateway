#ifndef __LORACOM_H__
#define __LORACOM_H__

void loraSetup(void);
void do_send(uint8_t * data, uint8_t sz );
void loraLoop(void);
boolean canLoraSleep(void);

#endif
