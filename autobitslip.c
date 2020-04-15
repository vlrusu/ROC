#include "setup.h"
//#include "Commands.h"

#include "utils.h"

void autobitslip()
{
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
}