#ifndef PPM_h
#define PPM_h

#include "stm32f105xc.h"
#include "GlobalObjects.h"




// standard PWM 50 Hz
#define PWM_ARR 0xFFFF
#define PWM_PSC 0x16
#define MIN_PWM_VAL 0x0CCD //1 ms
#define PPM_PAUSE_VAL 0x051F //0.4 ms
#define PPM_SYNC_PULSE 0xFAE0 //end of last PPM pulse


//#define MAX_THROTTLE_VAL 0x0CCD

// PWM 36.5 Hz
/*
#define PWM_ARR 0xFFFF
#define PWM_PSC 0x1E
#define MIN_PWM_VAL 0x097B//0x0960
#define PPM_PAUSE_VAL 0x03CB//0X2D0 //0x03D7
#define PPM_SYNC_PULSE 0xFC34 //end of last PPM pulse
*/


#define SCALE_FACTOR MIN_PWM_VAL/2

//#define PPM_CHANNELS 8


void PPMinit (void);
void PPMupdate (int32_t  * ar, SaveDomain* Params);




#endif
