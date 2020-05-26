//#include <stdio.h>
//#include <string.h>
//
//#include "setup.h"
//
////This works for the Test board DigiMoboV5
//
//void setPreampGain(uint32_t chan_mask0, uint32_t chan_mask1, uint32_t chan_mask2, uint16_t value)
//{
//    uint8_t errors = 0;
//    for (int i=0;i<32;i++){
//    	if ((0x1<<i) & chan_mask0){
//    		int channel = i;
//    		LTC2634_write(strawsCal[i]._ltc, strawsCal[i]._gain, value);
//    	}
//    	if ((0x1<<i) & chan_mask1){
//    		int channel = i+32;
//    		LTC2634_write(strawsCal[i]._ltc, strawsCal[i]._gain, value);
//    	}
//    	if ((0x1<<i) & chan_mask2){
//    		int channel = i+64;
//    		LTC2634_write(strawsCal[i]._ltc, strawsCal[i]._gain, value);
//    	}
//    }
//}
//
//void setPreampThresh(uint32_t chan_mask0, uint32_t chan_mask1, uint32_t chan_mask2, uint16_t value)
//{
//    uint8_t errors = 0;
//    for (int i=0;i<32;i++){
//    	if ((0x1<<i) & chan_mask0){
//    		int channel = i;
//    		LTC2634_write(strawsCal[i]._ltc, strawsCal[i]._thresh, value);
//    	}
//    	if ((0x1<<i) & chan_mask1){
//    		int channel = i+32;
//    		LTC2634_write(strawsCal[i]._ltc, strawsCal[i]._thresh, value);
//    	}
//    	if ((0x1<<i) & chan_mask2){
//    		int channel = i+64;
//    		LTC2634_write(strawsCal[i]._ltc, strawsCal[i]._thresh, value);
//    	}
//    }
//}
//
//
//
