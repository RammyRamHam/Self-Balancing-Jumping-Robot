#ifndef _SPEAKER_H_
#define _SPEAKER_H_

#include "stdint.h"

void speakerInit(void);
void speakerReset(void);
void speakerSetFilePlayMode(void);
int speakerDataScale(uint8_t* d_buff, uint8_t* s_buff, uint32_t len);
void speakerPlay(void*arg);

#endif