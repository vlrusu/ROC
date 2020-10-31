#include "setup.h"
//#include "Commands.h"

#include "utils.h"

void autobitslip()
{
	/*
	uint8_t clock = (uint8_t) buffer[4];
	uint8_t dophase = (uint8_t) buffer[5];
	channel_mask[0] = readU32fromBytes(&buffer[6]);
	channel_mask[1] = readU32fromBytes(&buffer[10]);
	channel_mask[2] = readU32fromBytes(&buffer[14]);
	int8_t adc_mode = 1;

	hvcal = 1;
	get_mapped_channels();

	uint16_t mask_ADC_in_use = 0;
	for (uint8_t iadc=0;iadc<12;iadc++){
		uint8_t this_8ch_mask = ((mapped_channel_mask[iadc/4])>>((iadc*8)%32)) & 0xff;
		if (this_8ch_mask)
			mask_ADC_in_use |= (0x1 << iadc);
	}

	digi_write(DG_ADDR_SAMPLE,1,0);
	digi_write(DG_ADDR_LOOKBACK,1,0);

	digi_write(DG_ADDR_TRIGGER_MODE,0,0);
	digi_write(DG_ADDR_ENABLE_PULSER,1,0);

	*(registers_0_addr + REG_ROC_FIFO_HOWMANY) = 1;

	bufcount = 0;
	outBuffer[bufcount++] = AUTOBITSLIPCMDID;
	bufWrite(outBuffer, &bufcount, 902, 2);

	// we keep track of where we are in memory and how many bytes we need to send
	uint16_t bufcount_start = bufcount;
	uint16_t bufcount_end = bufcount + 902;

	//bufWrite(outBuffer, &bufcount, 0, 902);
	for (size_t i=0;i<902;i++)
		outBuffer[bufcount_start+i] = 0;


	outBuffer[bufcount_start] = dophase;
	bufcount_start += 1;

	///////////////////////////////////
	// DO PHASE                      //
	///////////////////////////////////
	// 9 bytes per channel, 9*96 total
	if (dophase){
		hvcal = 1;
		int8_t phases[96];
		for (uint8_t ichan=0;ichan<96;ichan++){
			phases[ichan] = -1;
			if (ichan > 47)
				hvcal =2;
			uint16_t adc_number = adc_map[ichan/8];
			uint16_t adcclk_number = adcclk_map[ichan/8];
			thischanmask = (((uint32_t) 0x1)<<(ichan%32));
			if ( ((ichan<32) && ((thischanmask & mapped_channel_mask[0]) == 0x0))||
					((ichan>=32) && (ichan<64) && ((thischanmask & mapped_channel_mask[1]) == 0x0))||
					((ichan>=64) && ((thischanmask & mapped_channel_mask[2]) == 0x0))	){
				//bufWrite(outBuffer, &bufcount, 0, 9);
				continue;
			}
			if (((0x1<<(adc_number)) & ENABLED_ADCS) == 0x0){
				//bufWrite(outBuffer, &bufcount, 0, 9);
				continue;
			}
			digi_write(DG_ADDR_MASK1, 0x0, 0);
			digi_write(DG_ADDR_MASK2, 0x0, 0);
			digi_write(DG_ADDR_MASK3, 0x0, 0);
			if ((ichan%48) < 16)
				digi_write(DG_ADDR_MASK1, (uint16_t) (0x1<<(ichan%48)), hvcal);
			else if ((ichan%48) < 32)
				digi_write(DG_ADDR_MASK2, (uint16_t) (0x1<<((ichan%48)-16)), hvcal);
			else
				digi_write(DG_ADDR_MASK3, (uint16_t) (0x1<<((ichan%48)-32)), hvcal);
			uint16_t results[12];

			uint8_t iphase=0;
			for (iphase=0;iphase<12;iphase++){
				adc_write(ADC_ADDR_PHASE,iphase,(0x1<<(adcclk_number)));
				adc_write(ADC_ADDR_TESTIO,9,(0x1<<(adc_number)));
				//for (int iadc=0;iadc<12;iadc++){
					//	adc_write(ADC_ADDR_PHASE,iphase,(0x1<<(iadc)));
					//	adc_write(ADC_ADDR_TESTIO,9,(0x1<<(iadc)));
					//}

				// reset fifo
				resetFIFO();

				*(registers_0_addr + REG_ROC_EWW_PULSER) = 1;

				readout_obloc = 0;
				readout_maxDelay = 50;
				readout_mode = 0;
				readout_wordsPerTrigger = NUMTDCWORDS+1;
				readout_numTriggers = 11;

				int delay_count = 0;
				int trigger_count = 0;

				uint16_t lasthit[readout_wordsPerTrigger];

				read_data2(&delay_count,&trigger_count,lasthit);
				if (trigger_count != 11){

					outBuffer[bufcount_start + 9*96 + 12 + 12] = ichan;
					outBufSend(g_uart, outBuffer, bufcount_end);
					return;

					//									sprintf(outBuffer,"Didn't get enough triggers: %d\n",trigger_count);
					//									UART_polled_tx_string( &g_uart, outBuffer );
					break;
				}
				results[iphase] = lasthit[readout_wordsPerTrigger-1];
			}
			bufWriteN(outBuffer, bufcount_start + ichan*9, ichan, 1);
			bufWriteN(outBuffer, bufcount_start + ichan*9+1, iphase, 1);

			uint8_t maxdist = 0;
			//int bestclock = -1;
			uint8_t bestclock = 0xff;
			for (uint8_t i=0;i<12;i++){
				if (results[i] != 0x2AA && results[i] != 0x155){
					continue;
				} //find first occurrence of 0x2AA or 0x155
				uint8_t thisdist = 1;
				for (uint8_t j=1;j<12;j++){
					uint8_t i2 = (i+j) % 12;
					if ((results[i2] != 0x2AA && results[i2] != 0x155) || results[i2] != results[i]){
						break;
					}
					thisdist++;
				}
				uint8_t thisdist2 = 1;
				for (uint8_t j=1;j<12;j++){
					uint8_t i2 = (i-j+12) % 12;
					if ((results[i2] != 0x2AA && results[i2] != 0x155) || results[i2] != results[i]){
						break;
					}
					thisdist2++;
				}
				if (thisdist2 < thisdist)
					thisdist = thisdist2;
				if (thisdist > maxdist){
					maxdist = thisdist;
					bestclock = i;
				}
			}

			if (maxdist == 12){
				bestclock = 0xFF;
			}

			//							sprintf(outBuffer,"%d: Best clock phase: %d\n",ichan,bestclock);
			//							UART_polled_tx_string( &g_uart, outBuffer );

			outBuffer[bufcount_start + ichan*9+2] = bestclock;

			//if (bestclock > -1)
			if (bestclock < 12){
				phases[ichan] = bestclock;
//				bufWrite(outBuffer, &bufcount, 0, 6);
			}
			else{
				//								sprintf(outBuffer,"%d: Could not find useable clock phase %02x %02x %02x\n",ichan,results[0],results[3],results[6]);
				//								UART_polled_tx_string( &g_uart, outBuffer );
				bufWriteN(outBuffer, bufcount_start + ichan*9+3, results[0], 2);
				bufWriteN(outBuffer, bufcount_start + ichan*9+5, results[3], 2);
				bufWriteN(outBuffer, bufcount_start + ichan*9+7, results[6], 2);

			}
		}

		// get best phase for each adc
		uint8_t i=0;
		while (i<12){
			//for (uint8_t i=0;i<12;i++){
			int phasecount[12] = {0};
			uint8_t thisclk = adcclk_map[i];
			uint8_t j=1;
			while (adcclk_map[i+j] == adcclk_map[i]){j++;if (i+j >=12){break;}};
			for (uint8_t ichan=i*8;ichan<(i+j)*8;ichan++)
				if (phases[ichan] >= 0)
					phasecount[phases[ichan]]++;
			int maxcount = 0;
			adc_phases[adc_map[i]] = 0;
			for (uint8_t iphase=0;iphase<12;iphase++)
				if (phasecount[iphase] > maxcount){
					maxcount = phasecount[iphase];
					for (int k=i;k<i+j;k++)
						adc_phases[adc_map[k]] = iphase;
				}
			i += j;
		}
		//						for (uint8_t i=0;i<12;i++){
		//							int phasecount[12] = {0};
		//							for (uint8_t j=0;j<8;j++)
		//								if (phases[i*8+j] >= 0)
		//									phasecount[phases[i*8+j]]++;
		//							int maxcount = 0;
		//							adc_phases[adc_map[i]] = 0;
		//							for (uint8_t j=0;j<12;j++){
		//								if (phasecount[j] > maxcount){
		//									maxcount = phasecount[j];
		//									adc_phases[adc_map[i]] = j;
		//								}
		//							}
		//						}
//	}else{
//		bufWrite(outBuffer, &bufcount, 0, 9*96);
	}


	///////////////////////////////////
	// DO BITSLIP                    //
	///////////////////////////////////
	// 12 bytes total, one per adc

	// now do bitslip
	for (uint8_t i=0;i<12;i++){

		if ((0x1<<i) & mask_ADC_in_use){
			if (clock < 99){
				adc_write(ADC_ADDR_PHASE,clock,(0x1<<i));
				adc_write(ADC_ADDR_TESTIO,1,(0x1<<i));
			}else{
				adc_write(ADC_ADDR_PHASE,adc_phases[i],(0x1<<i));
				adc_write(ADC_ADDR_TESTIO,1,(0x1<<i));
			}
		}
	}
	uint8_t ichan;
	hvcal = 1;
	for (ichan=0;ichan<96;ichan++){
		uint16_t bufchanaddr = bufcount_start + 9*96 + (int) (ichan/8);
		uint8_t chanmask = (uint8_t)(1 << (ichan % 8));
		if (ichan > 47)
			hvcal =2;
		uint8_t adc_number = adc_map[ichan/8];
		thischanmask = (((uint32_t) 0x1)<<(ichan%32));
		if ( ((ichan<32) && ((thischanmask & mapped_channel_mask[0]) == 0x0))||
				((ichan>=32) && (ichan<64) && ((thischanmask & mapped_channel_mask[1]) == 0x0))||
				((ichan>=64) && ((thischanmask & mapped_channel_mask[2]) == 0x0))	){
		//	outBuffer[bufchanaddr] = outBuffer[bufchanaddr] & (~chanmask);
		//	//if (((ichan+1)%8)==0) bufcount++;
			continue;
		}
		if (((0x1<<(adc_number)) & ENABLED_ADCS) == 0x0){
		//	outBuffer[bufchanaddr] = outBuffer[bufchanaddr] & (~chanmask);
		//	//if (((ichan+1)%8)==0) bufcount++;
			continue;
		}
		digi_write(DG_ADDR_MASK1, 0x0, 0);
		digi_write(DG_ADDR_MASK2, 0x0, 0);
		digi_write(DG_ADDR_MASK3, 0x0, 0);
		if ((ichan%48) < 16)
			digi_write(DG_ADDR_MASK1,(uint16_t) (0x1<<(ichan%48)), hvcal);
		else if ((ichan%48) < 32)
			digi_write(DG_ADDR_MASK2,(uint16_t) (0x1<<((ichan%48)-16)), hvcal);
		else
			digi_write(DG_ADDR_MASK3,(uint16_t) (0x1<<((ichan%48)-32)), hvcal);
		uint8_t success = 0;
		for (uint8_t i=0;i<10;i++){
			resetFIFO();

			*(registers_0_addr + REG_ROC_EWW_PULSER) = 1;

			readout_obloc = 0;
			readout_maxDelay = 50;
			readout_mode = 0;
			readout_wordsPerTrigger = NUMTDCWORDS+1;
			readout_numTriggers = 11;

			int delay_count = 0;
			int trigger_count = 0;

			uint16_t lasthit[readout_wordsPerTrigger];

			read_data2(&delay_count,&trigger_count,lasthit);
			if (trigger_count != 11){

				outBuffer[bufcount_start + 9*96 + 12 + 12] = ichan;
				outBufSend(g_uart, outBuffer, bufcount_end);
				return;
				//								sprintf(outBuffer,"Didn't get enough triggers: %d\n",trigger_count);
				//								UART_polled_tx_string( &g_uart, outBuffer );
				break;
			}
			if (lasthit[readout_wordsPerTrigger-1] == 0x001){
				success = 1;
				break;
			}
			//sprintf(outBuffer,"%d: %d: got %02x\n",ichan,i,lasthit[12]);
			//UART_polled_tx_string( &g_uart, outBuffer );

			if ((ichan%48) < 8){
				digi_write(DG_ADDR_BITSLIP0,((0x1<<((ichan%48)-0))), hvcal);
			}else if ((ichan%48) < 16){
				digi_write(DG_ADDR_BITSLIP1,((0x1<<((ichan%48)-8))), hvcal);
			}else if ((ichan%48) < 24){
				digi_write(DG_ADDR_BITSLIP2,((0x1<<((ichan%48)-16))), hvcal);
			}else if ((ichan%48) < 32){
				digi_write(DG_ADDR_BITSLIP3,((0x1<<((ichan%48)-24))), hvcal);
			}else if ((ichan%48) < 40){
					digi_write(DG_ADDR_BITSLIP4,((0x1<<((ichan%48)-32))), hvcal);
			}else if ((ichan%48) < 48){
					digi_write(DG_ADDR_BITSLIP5,((0x1<<((ichan%48)-40))), hvcal);
			}
			digi_write(DG_ADDR_BITSLIP0,0x0,hvcal);
			digi_write(DG_ADDR_BITSLIP1,0x0,hvcal);
			digi_write(DG_ADDR_BITSLIP2,0x0,hvcal);
			digi_write(DG_ADDR_BITSLIP3,0x0,hvcal);
			digi_write(DG_ADDR_BITSLIP4,0x0,hvcal);
			digi_write(DG_ADDR_BITSLIP5,0x0,hvcal);
		}

		//if (success){
		//	sprintf(outBuffer,"%d: Found correct word alignment\n",ichan);
		//	UART_polled_tx_string( &g_uart, outBuffer );
		//}else{
		//	sprintf(outBuffer,"%d: FAILED to find correct word alignment\n",ichan);
		//	UART_polled_tx_string( &g_uart, outBuffer );
		//}
		if (success)
			outBuffer[bufchanaddr] = outBuffer[bufchanaddr] | chanmask;
		//else
		//	outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
		//if (((ichan+1)%8)==0) bufcount++; //gives 12 bytes describing whether the operation is successful
	}

	//if (ichan<96){
	//	for (uint8_t i=ichan; i<96; i++) {
	//		outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (i % 8))));
	//		if (((i+1)%8)==0) bufcount++;
	//	}
	//} //match the format if break in between



	///////////////////////////////////
	// CHECK RESULT                  //
	///////////////////////////////////

	// check at the end with mixed frequency ADC
	for (uint8_t i=0;i<12;i++){

		if ((0x1<<i) & mask_ADC_in_use){

			if (clock < 99){
				adc_write(ADC_ADDR_PHASE,clock,(0x1<<i));
				adc_write(ADC_ADDR_TESTIO,0xC,(0x1<<i));
			}else{
				adc_write(ADC_ADDR_PHASE,adc_phases[i],(0x1<<i));
				adc_write(ADC_ADDR_TESTIO,0xC,(0x1<<i));
			}
		}
	}
	uint32_t error_mask[3] = {0};

	hvcal = 1;
	for (ichan=0;ichan<96;ichan++){
		uint16_t bufchanaddr = bufcount_start + 9*96 + 12 + (int) (ichan/8);
		uint8_t chanmask = (uint8_t)(1 << (ichan % 8));
		if (ichan > 47)
			hvcal =2;
		uint8_t adc_number = adc_map[ichan/8];
		thischanmask = (((uint32_t) 0x1)<<(ichan%32));
		if ( ((ichan<32) && ((thischanmask & mapped_channel_mask[0]) == 0x0))||
				((ichan>=32) && (ichan<64) && ((thischanmask & mapped_channel_mask[1]) == 0x0))||
				((ichan>=64) && ((thischanmask & mapped_channel_mask[2]) == 0x0))	){
		//	outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
		//	if (((ichan+1)%8)==0) bufcount++; //place holder, unchecked also gives 0
			continue;
		}
		if (((0x1<<(adc_number)) & ENABLED_ADCS) == 0x0){
		//	outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
		//	if (((ichan+1)%8)==0) bufcount++; //place holder, unchecked also gives 0
			continue;
		}
		digi_write(DG_ADDR_MASK1, 0x0, 0);
		digi_write(DG_ADDR_MASK2, 0x0, 0);
		digi_write(DG_ADDR_MASK3, 0x0, 0);
		if ((ichan%48) < 16)
			digi_write(DG_ADDR_MASK1,(uint16_t) (0x1<<(ichan%48)), hvcal);
		else if ((ichan%48) < 32)
			digi_write(DG_ADDR_MASK2,(uint16_t) (0x1<<((ichan%48)-16)), hvcal);
		else
			digi_write(DG_ADDR_MASK3,(uint16_t) (0x1<<((ichan%48)-32)), hvcal);

		resetFIFO();

		*(registers_0_addr + REG_ROC_EWW_PULSER) = 1;

		readout_obloc = 0;
		readout_maxDelay = 50;
		readout_mode = 0;
		readout_wordsPerTrigger = NUMTDCWORDS+1;
		readout_numTriggers = 11;

		int delay_count = 0;
		int trigger_count = 0;

		uint16_t lasthit[readout_wordsPerTrigger];

		read_data2(&delay_count,&trigger_count,lasthit);
		if (trigger_count != 11){

			outBuffer[bufcount_start + 9*96 + 12 + 12] = ichan;
			outBufSend(g_uart, outBuffer, bufcount_end);
			return;
			//sprintf(outBuffer,"Didn't get enough triggers: %d\n",trigger_count);
			//UART_polled_tx_string( &g_uart, outBuffer );
			break;
		}
		if (lasthit[readout_wordsPerTrigger-1] != 0x319){
			error_mask[ichan/32]|= (((uint32_t)0x1)<<(ichan%32));
		//	//sprintf(outBuffer,"%d FAILED final check: %02x (instead of 0x319)\n",ichan,lasthit[12]);
		//	//UART_polled_tx_string( &g_uart, outBuffer );
		//	outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
		//	if (((ichan+1)%8)==0) bufcount++; //gives 12 bytes describing whether final check is successful
		}
		else{
			outBuffer[bufchanaddr] = outBuffer[bufchanaddr] | chanmask;
			//if (((ichan+1)%8)==0) bufcount++;
		}
	}

	//if (ichan<96){
	//	for (uint8_t i=ichan; i<96; i++) {
	//		outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
	//		if (((i+1)%8)==0) bufcount++;
	//	}
	//} //match the format if break in between

	outBuffer[bufcount_start + 9*96 + 12 + 12] = 96;

	//sprintf(outBuffer,"Error mask: %04x%08x\n",((uint32_t)(error_mask>>32)),(uint32_t)(error_mask&0xFFFFFFFF));
	//UART_polled_tx_string( &g_uart, outBuffer );

	for (uint8_t i=0; i<3; i++)
		bufWriteN(outBuffer, bufcount_start + 9*96 + 12 + 12 + 4*i, error_mask[i], 4);

	outBufSend(g_uart, outBuffer, bufcount_end);

	*/
	uint8_t checksamples = 1;

	uint8_t eye_monitor_width = (uint8_t) buffer[4];
	uint8_t init_adc_phase = (uint8_t) buffer[5];
	channel_mask[0] = readU32fromBytes(&buffer[6]);
	channel_mask[1] = readU32fromBytes(&buffer[10]);
	channel_mask[2] = readU32fromBytes(&buffer[14]);
	uint8_t ifcheck = (uint8_t) buffer[18];
	get_mapped_channels();

	outBuffer[bufcount++] = AUTOBITSLIPCMDID;
	bufcount_place_holder = bufcount;
	bufWrite(outBuffer, &bufcount, 0, 2);
	bufWrite(outBuffer, &bufcount, eye_monitor_width, 1);
	bufWrite(outBuffer, &bufcount, ifcheck, 1);

	uint8_t iteration = 0;
	uint16_t faulted_adc;

	while (1)
	{
		faulted_adc = 0;

		digi_write(DG_ADDR_BITALIGN_RSETN, 0, 0);
		delayUs(8);
		digi_write(DG_ADDR_BITALIGN_RSETN, 1, 0);

		digi_write(DG_ADDR_SAMPLE,1,0);
		digi_write(DG_ADDR_LOOKBACK,1,0);

		digi_write(DG_ADDR_TRIGGER_MODE,0,0);
		digi_write(DG_ADDR_ENABLE_PULSER,1,0);

		uint16_t activeADC = 0;
		for (uint8_t i=0;i<12;i++){
			if ((mapped_channel_mask[i/4] >> ((i%4)*8)) & 0xff){
				activeADC |= (((uint16_t)0x1) << i);
			}
		}

		//using pattern #9 (0x155/0x2aa) for alignment
		for (uint8_t i=0;i<12;i++){
			if (((0x1<<i) & ENABLED_ADCS) & activeADC) {
				//fix all adc at phase = 3 (default 180 deg)
				//adc_write(ADC_ADDR_PHASE,3,(0x1<<i));
				adc_write(ADC_ADDR_PHASE,init_adc_phase,(0x1<<i));
				adc_write(ADC_ADDR_TESTIO,0x9,(0x1<<i));
			}
			else //turn off the channels not in use
				mapped_channel_mask[i/4] &= (~((uint32_t)0xff << ((i%4)*8)));
		}
		bufWrite(outBuffer, &bufcount, init_adc_phase, 1);

		//turn on pulser, read one word a time
		*(registers_0_addr + REG_ROC_FIFO_HOWMANY) = 1;
		*(registers_0_addr + REG_ROC_EWW_PULSER) = 1;

		//set minimum eye monitor width
		digi_write(DG_ADDR_BITALIGN_EWM_WIDTH, (uint16_t)eye_monitor_width,0);

		uint16_t active_ch[6] = {0};
		active_ch[0] = (uint16_t)(mapped_channel_mask[0] & 0xffff);
		active_ch[1] = (uint16_t)((mapped_channel_mask[0] >> 16) & 0xffff);
		active_ch[2] = (uint16_t)(mapped_channel_mask[1] & 0xffff);
		active_ch[3] = (uint16_t)((mapped_channel_mask[1] >> 16) & 0xffff);
		active_ch[4] = (uint16_t)(mapped_channel_mask[2] & 0xffff);
		active_ch[5] = (uint16_t)((mapped_channel_mask[2] >> 16) & 0xffff);

		digi_write(DG_ADDR_MASK1, active_ch[0], 1);
		digi_write(DG_ADDR_MASK2, active_ch[1], 1);
		digi_write(DG_ADDR_MASK3, active_ch[2], 1);
		digi_write(DG_ADDR_MASK1, active_ch[3], 2);
		digi_write(DG_ADDR_MASK2, active_ch[4], 2);
		digi_write(DG_ADDR_MASK3, active_ch[5], 2);

		for (uint8_t i = 0; i < 6; i++){
			digi_write(DG_ADDR_RX_CH_MASK1+i%3, active_ch[i], 1+i/3);
			bufWrite(outBuffer, &bufcount, active_ch[i], 2);
		}

		//start bit align, set corresponding channel's restart to 1, then back to 0 after 8 ticks
		digi_write(DG_ADDR_BITALIGN_RSTRT, 1, 0);
		delayUs(8);
		digi_write(DG_ADDR_BITALIGN_RSTRT, 0, 0);

		volatile uint16_t completion[6] = {0};
		volatile uint16_t error[6] = {0};

		//wait until all channels are completed; timeout after 30 secs
		for (uint8_t i=0; i<30; i++){
			hwdelay(50000000);//check every 1 second if is done
			uint8_t bitalign_done = 1;
			for (uint8_t j = 0; j < 6; j++){
				completion[j] = digi_read(DG_ADDR_BITALIGN_CMP1+j%3, 1+j/3);
				error[j] = digi_read(DG_ADDR_BITALIGN_ERR1+j%3, 1+j/3);
				if ((completion[j] & active_ch[j]) != active_ch[j])
					bitalign_done = 0;
			}

			if (bitalign_done) break;
		}

		for (uint8_t i=0; i<6; i++){
			bufWrite(outBuffer, &bufcount, completion[i], 2);
			bufWrite(outBuffer, &bufcount, error[i], 2);

			if ((~completion[i]) & 0x00ff)
				faulted_adc |= ((uint16_t)0x1 << (2*i));
			if ((~completion[i]) & 0xff00)
				faulted_adc |= ((uint16_t)0x1 << (2*i+1));
		}

		///////////////////////////////////
		// 	DOING BITSLIP                //
		///////////////////////////////////

		//switch to pattern #1 for bitslip
		for (uint8_t i=0;i<12;i++){
			if ((0x1<<i) & ENABLED_ADCS)
				adc_write(ADC_ADDR_TESTIO,0x1,(0x1<<i));
		}

		uint8_t bitslipstep[96];
		for (uint8_t ichan=0; ichan<96; ichan++) bitslipstep[ichan] = 0xff;
		volatile uint16_t bitstlip_done[6] = {0};

		for (uint8_t i=0; i<10; i++){
			hwdelay(50);

			digi_write(DG_ADDR_BSC_OPERATION_TYPE, 0, 0);
			digi_write(DG_ADDR_BITSLIP_STRT, 1, 0);
			delayUs(10);
			digi_write(DG_ADDR_BITSLIP_STRT, 0, 0);

			for (uint8_t j = 0; j < 6; j++)
				bitstlip_done[j] = digi_read(DG_ADDR_BITSLIP_DONE1+j%3, 1+j/3);

			for (uint8_t ichan=0; ichan<96; ichan++){
				uint16_t this_chan_mask = (uint16_t) 0x1 << (ichan%16);
				if (((bitstlip_done[ichan/16] & this_chan_mask) != 0) && (bitslipstep[ichan] == 0xff))
					bitslipstep[ichan] = i;
			}
		}
		for (uint8_t i=0; i<6; i++){
			bufWrite(outBuffer, &bufcount, bitstlip_done[i], 2);
			if ((~bitstlip_done[i]) & 0x00ff)
				faulted_adc |= ((uint16_t)0x1 << (2*i));
			if ((~bitstlip_done[i]) & 0xff00)
				faulted_adc |= ((uint16_t)0x1 << (2*i+1));
		}
		for (uint8_t i=0; i<96; i++)
			bufWrite(outBuffer, &bufcount, bitslipstep[i], 1);

		///////////////////////////////////
		// CHECK RESULT                  //
		///////////////////////////////////

		// check with mixed frequency ADC

		uint16_t pattern_match[6] = {0};
		if (ifcheck){
			for (uint8_t i=0;i<12;i++){
				if ((0x1<<i) & ENABLED_ADCS)
					adc_write(ADC_ADDR_TESTIO,0xC,(0x1<<i));
			}

			digi_write(DG_ADDR_BSC_OPERATION_TYPE, 1, 0);
			digi_write(DG_ADDR_BITSLIP_STRT, 1, 0);
			delayUs(10);
			digi_write(DG_ADDR_BITSLIP_STRT, 0, 0);

			for (uint8_t j = 0; j < 6; j++){
				pattern_match[j] = digi_read(DG_ADDR_BSC_PATTERN_MATCH1+j%3, 1+j/3);
				bufWrite(outBuffer, &bufcount, pattern_match[j], 2);
				if ((pattern_match[j] & active_ch[j] & 0x00ff) != (active_ch[j] & 0x00ff))
					faulted_adc |= ((uint16_t)0x1 << (2*j));
				if ((pattern_match[j] & active_ch[j] & 0xff00) != (active_ch[j] & 0xff00))
					faulted_adc |= ((uint16_t)0x1 << (2*j+1));
			}
		}
		else{
			for (uint8_t i=0;i<6;i++) bufWrite(outBuffer, &bufcount, 0, 2);
		}

		iteration ++;
		if (faulted_adc == 0) break;
		if (iteration == 6) break; //maximum: 3 trials at phase 0, 3 for faulted adc at different phase
		if (iteration == 3) init_adc_phase = (init_adc_phase+3)%12;
		if (iteration >= 3){
			for (uint8_t i=0; i<3; i++) mapped_channel_mask[i] = 0;
			for (uint8_t i=0; i<12; i++){
				if (faulted_adc & ((uint16_t)0x1 << (i))){
					mapped_channel_mask[i/4] |= ((uint32_t)0x000000ff)<<((i%4)*8);
				}
			}
		}
	}

	bufWrite(outBuffer, &bufcount_place_holder, (bufcount-3), 2);
	outBufSend(g_uart, outBuffer, bufcount);
}
