/*
**************************************************************************

rec_fifo_hd_speed.cpp                          (c) Spectrum GmbH , 02/2007

**************************************************************************

this example supports all acquistion cards

Does FIFO acquistion to hard disk to test the maximum writing performance
of the hard disk

This program only runs under Windows as it uses some windows specific API
calls for data writing, time measurement and key checking
**************************************************************************
*/



// ----- standard c include files -----
#include <stdio.h>
#include <string.h>
#include <conio.h>

// ----- new include files -----
#include <Windows.h> // for Sleep func
#include <math.h>
#include "engine.h" // for matlab engine
#include <typeinfo>

//#include <Eigen/Dense> // for eigen
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <iostream>

// ----- include of common example librarys -----
#include "../common/spcm_lib_card.h"
#include "../common/spcm_lib_data.h"
#include "../common/spcm_lib_thread.h"


// ----- global setup for the run (can be changed interactively) -----
int32   g_lSamplingRate = MEGA(20);
int32   g_lNotifySize = KILO_B(1024*16);
int32   g_lBufferSize = MEGA_B(1024*32);
bool    g_bThread = false;
uint64  g_qwChannelEnable = 1;
uint32  g_dwUpdateBuffers = 1;
uint32  g_dwUpdateCount = 0;
enum    { eStandard, eHDSpeedTest, eSpeedTest } g_eMode = eStandard;

#define FILENAME "500mVPP_500MHz_Squares"
#define DATA_LENGTH 16777216

//define getting average function
double average(int16_t* array, int length) {
	double sum = 0.0;
	for (int i = 0; i < length; i++)
		sum += *(array+i);
	return sum / length;
}

/*
**************************************************************************
bDoCardSetup: setup matching the calculation routine
**************************************************************************
*/

static int initialized = 0;

bool bDoCardSetup(ST_SPCM_CARDINFO *pstCard)
{


	// FIFO mode setup, we run continuously and have 16 samples of pre data before trigger event
	// all available channels are activated
	bSpcMSetupModeRecFIFOSingle(pstCard, g_qwChannelEnable, 16);

	// we try to set the samplerate on internal PLL, no clock output
	if (g_lSamplingRate > pstCard->lMaxSamplerate)
		g_lSamplingRate = pstCard->lMaxSamplerate;
	bSpcMSetupClockPLL(pstCard, g_lSamplingRate, false);
	printf("Sampling rate set to %.1f MHz\n", (float)pstCard->lSetSamplerate / MEGA(1));

	// we set software trigger, no trigger output
	bSpcMSetupTrigSoftware(pstCard, false);

	return pstCard->bSetError;
}



/*
**************************************************************************
Working routine data
**************************************************************************
*/


struct ST_WORKDATA
{
	int64           llWritten;
	HANDLE          hFile;
	char            szFileName[100];
	LARGE_INTEGER   uStartTime;
	LARGE_INTEGER   uLastTime;
	LARGE_INTEGER   uHighResFreq;
};



/*
**************************************************************************
Setup working routine
**************************************************************************
*/

bool bWorkInit(void * pvWorkData, ST_BUFFERDATA * pstBufferData)
{
	ST_WORKDATA* pstWorkData = (ST_WORKDATA *)pvWorkData;

	// setup for the transfer, to avoid overrun we use quite large blocks as this has a better throughput to hard disk
	pstBufferData->dwDataBufLen = g_lBufferSize;
	pstBufferData->dwDataNotify = g_lNotifySize;

	// setup for the work
	pstWorkData->llWritten = 0;

	sprintf(pstWorkData->szFileName, "%s.bin", FILENAME);

	printf("\n");
	printf("Written      HW-Buf      SW-Buf   Average   Current\n-----------------------------------------------------\n");
	if ((g_eMode == eStandard) || (g_eMode == eHDSpeedTest))
		pstWorkData->hFile = CreateFile(pstWorkData->szFileName,
		GENERIC_WRITE,
		FILE_SHARE_READ | FILE_SHARE_WRITE,
		NULL,
		CREATE_ALWAYS,
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_NO_BUFFERING,
		NULL);

	QueryPerformanceFrequency(&pstWorkData->uHighResFreq);
	pstWorkData->uStartTime.QuadPart = 0;

	return ((pstWorkData->hFile != NULL) || (g_eMode == eSpeedTest));
}



/*
**************************************************************************
bWorkDo: stores data to hard disk
**************************************************************************
*/

bool bWorkDo(void * pvWorkData, ST_BUFFERDATA * pstBufferData)
{
	ST_WORKDATA*    pstWorkData = (ST_WORKDATA *)pvWorkData;
	uint32          dwWritten;
	int64           llBufferFillPromille;
	LARGE_INTEGER   uTime;
	double          dAverageTime = 0;
	double          dLastTime = 0;
	double          dAverageSpeed = 0;
	double          dLastSpeed = 0;

	// first call will initialize all time measurings
	
	
	QueryPerformanceCounter(&uTime);
	if (pstWorkData->uStartTime.QuadPart == 0)
	{
		pstWorkData->uStartTime.QuadPart = uTime.QuadPart;
		pstWorkData->uLastTime.QuadPart = uTime.QuadPart;
	}
	
	// calc transfer speed
	else
	{
		dAverageTime = (double)(uTime.QuadPart - pstWorkData->uStartTime.QuadPart) / pstWorkData->uHighResFreq.QuadPart;
		dLastTime = (double)(uTime.QuadPart - pstWorkData->uLastTime.QuadPart) / pstWorkData->uHighResFreq.QuadPart;
		dAverageSpeed = (double)pstWorkData->llWritten / dAverageTime / MEGA_B(1);
		dLastSpeed = (double)pstBufferData->dwDataNotify / dLastTime / MEGA_B(1);
		pstWorkData->uLastTime.QuadPart = uTime.QuadPart;
	}

	// new delcarations
	//int16_t buffer;
	//int count = 1;

	// write the data and count the samples
	if (g_eMode == eSpeedTest)
		dwWritten = pstBufferData->dwDataNotify;
	else {
		//test
		//std::cout << typeid(pstBufferData->pvDataCurrentBuf).name() << '\n';
		//printf("pvDataCurrentBuf(pointer which points the real data) : \n");

		/*************************** Cpp to matlab  ***********************/
		printf("Cpp to matlab part start\n");
		int16_t* signal_int16;


		//open matlab
		Engine *ep;
		mxArray *T = NULL;

		if (!(ep = engOpen(""))) {
			fprintf(stderr, "\nCan't start MATLAB engine\n");
			return EXIT_FAILURE;
		}

		//define T(matlab array)
		const size_t dims[2] = { pstBufferData->dwDataNotify / sizeof(int16_t), 1 };
		T = mxCreateNumericArray(1, dims, mxINT16_CLASS, mxREAL);

		//copy signal_int16 to T
		
		signal_int16 = (int16_t *)malloc((pstBufferData->dwDataNotify));
		int16_t* signal_int16_addr = signal_int16;
		signal_int16 = (int16_t *)mxGetData(T);

		memcpy(signal_int16, pstBufferData->pvDataCurrentBuf, (pstBufferData->dwDataNotify));

		//print signal_int16 for test
		//static int cnt_signal = 0;
		//cnt_signal++;

		//if (cnt_signal == 4) {
		//	for (int i = 0; i <= 500; i++) {
		//		printf("\n signal_int16 = %hd ", *(signal_int16 + i));

		//	}
		//}

		//print T using matlab
		//engEvalString(ep, "T = zeros(1000000,1)");
		engPutVariable(ep, "T", T);

		engEvalString(ep, "control_val = 50");
		engEvalString(ep, "x1 = T(1:16*control_val:end);");
		engEvalString(ep, "x2 = T(51:16*control_val:end);");
		engEvalString(ep, "x3 = T(101:16*control_val:end);");
		engEvalString(ep, "x4 = T(151:16*control_val:end);");
		engEvalString(ep, "x5 = T(201:16*control_val:end);");
		engEvalString(ep, "x6 = T(251:16*control_val:end);");
		engEvalString(ep, "x7 = T(301:16*control_val:end);");
		engEvalString(ep, "x8 = T(351:16*control_val:end);");


		engEvalString(ep, "x9 = T(401:16*control_val:end);");
		engEvalString(ep, "x10 = T(451:16*control_val:end);");
		engEvalString(ep, "x11 = T(501:16*control_val:end);");
		engEvalString(ep, "x12 = T(551:16*control_val:end);");
		engEvalString(ep, "x13 = T(601:16*control_val:end);");
		engEvalString(ep, "x14 = T(651:16*control_val:end);");
		engEvalString(ep, "x15 = T(701:16*control_val:end);");
		engEvalString(ep, "x16 = T(751:16*control_val:end);");


		engEvalString(ep, "figure(1)");
		engEvalString(ep, "subplot(2,4,1);");
		engEvalString(ep, "plot(x1);");

		engEvalString(ep, "subplot(2,4,2)");
		engEvalString(ep, "plot(x2)");

		engEvalString(ep, "subplot(2,4,3)");
		engEvalString(ep, "plot(x3)");

		engEvalString(ep, "subplot(2,4,4)");
		engEvalString(ep, "plot(x4)");

		engEvalString(ep, "subplot(2,4,5)");
		engEvalString(ep, "plot(x5)");

		engEvalString(ep, "subplot(2,4,6)");
		engEvalString(ep, "plot(x6)");

		engEvalString(ep, "subplot(2,4,7)");
		engEvalString(ep, "plot(x7)");

		engEvalString(ep, "subplot(2,4,8)");
		engEvalString(ep, "plot(x8)");

		engEvalString(ep, "figure(2)");
		engEvalString(ep, "subplot(2,4,1)");
		engEvalString(ep, "plot(x9)");

		engEvalString(ep, "subplot(2,4,2)");
		engEvalString(ep, "plot(x10)");

		engEvalString(ep, "subplot(2,4,3)");
		engEvalString(ep, "plot(x11)");

		engEvalString(ep, "subplot(2,4,4)");
		engEvalString(ep, "plot(x12)");

		engEvalString(ep, "subplot(2,4,5)");
		engEvalString(ep, "plot(x13)");

		engEvalString(ep, "subplot(2,4,6)");
		engEvalString(ep, "plot(x14)");

		engEvalString(ep, "subplot(2,4,7)");
		engEvalString(ep, "plot(x15)");

		engEvalString(ep, "subplot(2,4,8)");
		engEvalString(ep, "plot(x16)");

		//engEvalString(ep, "plot(T(1:end), 'r');");
		//engEvalString(ep, "hold on");
		//engEvalString(ep, "plot(T(1:end), 'o');");

		engEvalString(ep, "drawnow");
		engEvalString(ep, "hold off");


		/************************  signal process  ****************************/
		//signal_int16(=input_signal) exist from 0 to 8388607
		
		// define variables
		static int loop_count = 1; //loop_count starting at 1
		int num_samples_from_prev = 0;
		int num_remain_samples = 0;

		if (loop_count == 1) {
			num_samples_from_prev = 0;
		}
		else {
			num_samples_from_prev = num_remain_samples;
		}

		int16_t* samples_from_prev = (int16_t*)malloc(num_samples_from_prev * sizeof(int16_t));

		const int number_of_samples = DATA_LENGTH / 2;
		const int down_sampling_rate = 10;
		const int looking_window_size = 16;
		int moving_flag = 0;
		double th = 0;
		const int processed_signal_size = floor(number_of_samples / down_sampling_rate);
		int16_t* out_signal = (int16_t*)malloc(processed_signal_size * sizeof(int16_t));
		int16_t* tmp_signal = (int16_t*)malloc(number_of_samples * sizeof(int16_t));
		int16_t* answer_signal = (int16_t*)malloc(processed_signal_size * sizeof(int16_t));

		// regenerate input_signal using prev signal
		memcpy(tmp_signal, signal_int16, number_of_samples * sizeof(int16_t));
		memcpy(signal_int16, samples_from_prev, num_samples_from_prev * sizeof(int16_t));
		memcpy(signal_int16 + num_samples_from_prev, tmp_signal, number_of_samples * sizeof(int16_t));

		// save remainder signal to next loop's prev signal
		num_remain_samples = (num_samples_from_prev + number_of_samples) % down_sampling_rate;
		int16_t* remain_samples = (int16_t*)malloc(num_remain_samples * sizeof(int16_t));
		memcpy(remain_samples, signal_int16 + num_samples_from_prev + number_of_samples - num_remain_samples, num_remain_samples*sizeof(int16_t));
		
		// change input signal to be multiple of 10, move signal_int16 to input_signal
		int16_t* input_signal = (int16_t*)malloc((number_of_samples - num_remain_samples) * sizeof(int16_t));
		memcpy(input_signal, signal_int16, (number_of_samples - num_remain_samples) * sizeof(int16_t));

		//print signal
		static int cnt_signal = 0;
		cnt_signal++;

		if (loop_count == 2) {
			for (int i = 0; i < 30; i++) {
				printf("\n\n %d th input_signal = %hd", i, *(input_signal + i));
			}
			printf("\n--------------------------------\n");
			for (int i = 8388590; i < 8388610; i++) {
				printf("\n\n %d th input_signal = %hd", i, *(input_signal + i));
			}
		}
		

		for (int i = 0; i < processed_signal_size; i++) {
			answer_signal[i] = 0;
		}

		//main procedure
		for (int i = 0; i < processed_signal_size; i++) {
			if (i % looking_window_size == 0) {
				if (i*down_sampling_rate + 1 <= number_of_samples && (i + 1)*down_sampling_rate + down_sampling_rate*looking_window_size <= number_of_samples){
					th = average(input_signal + i*down_sampling_rate, down_sampling_rate*looking_window_size);
					//printf("\n\n if case, i = %d and th = %f", i, th);
				}
				else {
					th = average(input_signal + down_sampling_rate*(processed_signal_size - looking_window_size), down_sampling_rate*looking_window_size);
					//printf("\n\n else case, i = %d and th = %f", i, th);
				}
			}

			int num_amend = 1000;
			if (i % num_amend == 0) {
				if (i < 838000) {
					if (average(input_signal + down_sampling_rate*i, down_sampling_rate * 1) >= th) {
						for (int ii = 0; ii < num_amend; ii++) {
							if (ii % 2 == 0) {
								answer_signal[i + ii] = 1;
							}
						}
					}
					else {
						for (int ii = 0; ii < num_amend; ii++) {
							if (ii % 2 == 1) {
								answer_signal[i + ii] = 1;
							}
						}
					}
					//printf("\n th = %f \n", th);
					//printf("\n average = %f \n", average(input_signal + down_sampling_rate*i, down_sampling_rate * 1));
				}

				// when i >= 830000
				else {
					if (average(input_signal + down_sampling_rate*i, down_sampling_rate * 1) >= th) {
						for (int ii = i; ii < processed_signal_size; ii++) {
							if (ii % 2 == 0) {
								answer_signal[ii] = 1;
							}
						}
					}
					else {
						for (int ii = i; ii < processed_signal_size; ii++) {
							if (ii % 2 == 1) {
								answer_signal[ii] = 1;
							}
						}
					}
				}
			}
			if (average(input_signal + down_sampling_rate*i, down_sampling_rate * 1) >= th)
				out_signal[i] = 1;
			else
				out_signal[i] = 0;
		}
		
		//print the answer_signal and input_signal togheter
		for (int i = 838800; i < 838810; i++) {
			printf("\n---------------------------------\n");
			printf("%d th output_signal = %d and answer_signal = %d", i, out_signal[i], answer_signal[i]);
			printf("\n\n");
		}

		//print the accuracy
		int number_of_correct_samples = 0;
		for (int i = 0; i < processed_signal_size; i++) {
			if (out_signal[i] == answer_signal[i])
				number_of_correct_samples++;
		}

		double whole_accuracy = ((double)number_of_correct_samples / (double)processed_signal_size) * 100;
		printf("\n %d th correct samples = %d \n", loop_count, number_of_correct_samples);
		printf("\n %d th processed_signal_size = %d \n", loop_count, processed_signal_size);
		printf("\n\n %d th loop's whole accuracy = %f \n", loop_count, whole_accuracy);

		//count : counts the loop
		loop_count++;

		//write file
		WriteFile(pstWorkData->hFile, pstBufferData->pvDataCurrentBuf, pstBufferData->dwDataNotify, &dwWritten, NULL);

		//free allocated array and pointers
		mxDestroyArray(T);
		free(samples_from_prev);
		free(out_signal);
		free(tmp_signal);
		free(remain_samples);
		free(input_signal);
		free(answer_signal);
		free(signal_int16_addr);
	}

	pstWorkData->llWritten += dwWritten;
	if (dwWritten != pstBufferData->dwDataNotify)
	{
		printf("\nData Write error\n");
		return false;
	}

	// current status
	if (--g_dwUpdateCount == 0)
	{
		g_dwUpdateCount = g_dwUpdateBuffers;
		spcm_dwGetParam_i64(pstBufferData->pstCard->hDrv, SPC_FILLSIZEPROMILLE, &llBufferFillPromille);

		printf("\r");
		if (pstBufferData->llDataTransferred > GIGA_B(1))
			printf("%7.2lf GB", (double)pstBufferData->llDataTransferred / GIGA_B(1));
		else
			printf("%7.2lf MB", (double)pstBufferData->llDataTransferred / MEGA_B(1));

		printf(" %6.1lf %%", (double)llBufferFillPromille / 10.0);

		printf("    %6.1lf %%", 100.0 * (double)pstBufferData->dwDataAvailBytes / pstBufferData->dwDataBufLen);

		// print transfer speed
		printf("   %6.2lf MB/s", dAverageSpeed);
		printf("   %6.2lf MB/s", dLastSpeed);
	}

	pstBufferData->dwDataAvailBytes = pstBufferData->dwDataNotify;

	return true;
}




/*
**************************************************************************
vWorkClose: Close the work and clean up
For speed reason is the bKeyAbortCheck function (with
kbhit() inside) not used!
**************************************************************************
*/

void vWorkClose(void * pvWorkData, ST_BUFFERDATA * pstBufferData)
{
	ST_WORKDATA* pstWorkData = (ST_WORKDATA *)pvWorkData;

	if (pstWorkData->hFile && (g_eMode != eSpeedTest))
		CloseHandle(pstWorkData->hFile);
}


/*
**************************************************************************
bKeyCheckAsync
**************************************************************************
*/
int g_nKeyPress;
bool bKeyCheckAsync(void *, ST_BUFFERDATA *)
{
	return (g_nKeyPress != GetAsyncKeyState(VK_ESCAPE));
}



/*
**************************************************************************
bSetup: returns true if start, false if abort
**************************************************************************
*/

bool bSetup(ST_SPCM_CARDINFO * pstCard)
{
	double dTmp;
	uint32 dwTmp;
	int32  lChannels;
	char   szErrorText[ERRORTEXTLEN], szNameBuffer[100];
	uint64 qwContBufLen;

	// read out cont buf len and set default buffer size to it
	void* pvTmp;
	spcm_dwSetParam_i64(pstCard->hDrv, SPC_M2CMD, M2CMD_CARD_RESET);
	spcm_dwGetContBuf_i64(pstCard->hDrv, SPCM_BUF_DATA, &pvTmp, &qwContBufLen);
	if (qwContBufLen > 0)
		g_lBufferSize = (int32)qwContBufLen;

	while (1)
	{
		printf("\n\n");
		printf("Current Setup\n-------------\n");
		printf("          Card Selection:   %s", pszSpcMPrintCardInfo(pstCard, szNameBuffer, sizeof (szNameBuffer), false));
		printf("I ....... Interface speed:  ");
		switch (g_eMode)
		{
		case eStandard:    printf("Normal FIFO mode to HD\n"); break;
		case eHDSpeedTest: printf("Max PCI/PCIe interface speed to HD\n"); break;
		case eSpeedTest:   printf("Max PCI/PCIe interface speed only\n"); break;
		}
		printf("B ....... Buffer Size:      %.2lf MByte (Continuous Buffer: %d MByte)\n", (double)g_lBufferSize / MEGA_B(1), (int32)(qwContBufLen / MEGA_B(1)));
		printf("N ....... Notify Size:      %d kByte\n", g_lNotifySize / KILO_B(1));
		if (g_eMode == eStandard)
		{
			printf("S ....... Sampling Rate:    %.2lf MS/s\n", (double)g_lSamplingRate / MEGA(1));
			printf("T ....... Thread Mode:      %s\n", g_bThread ? "on" : "off");
			printf("C ....... Channel Enable:   %x\n", g_qwChannelEnable);
		}
		printf("Enter ... Start Test\n");
		printf("Esc ..... Abort\n");

		spcm_dwSetParam_i64(pstCard->hDrv, SPC_CHENABLE, g_qwChannelEnable);
		spcm_dwSetParam_i64(pstCard->hDrv, SPC_SAMPLERATE, g_lSamplingRate);
		spcm_dwGetParam_i32(pstCard->hDrv, SPC_CHCOUNT, &lChannels);
		spcm_dwSetParam_i32(pstCard->hDrv, SPC_TEST_FIFOSPEED, (g_eMode != eStandard) ? 1 : 0);

		if (spcm_dwGetErrorInfo_i32(pstCard->hDrv, NULL, NULL, szErrorText) != ERR_OK)
			printf("\nSetup Error:\n------------\n%s\n\n", szErrorText);
		else
		{
			double dTransferSpeed;
			if (pstCard->eCardFunction == AnalogIn)
				dTransferSpeed = (double)g_lSamplingRate * lChannels * pstCard->lBytesPerSample;
			else
				dTransferSpeed = (double)g_lSamplingRate * lChannels / 8;

			if (g_eMode == eStandard)
				printf("          Transfer Speed: %.2lf MByte/s\n", dTransferSpeed / MEGA_B(1));
			else
				printf("          Transfer Speed: max\n");

			// calc the display update rate in buffers to x/second to keep display overhead small
			g_dwUpdateBuffers = (uint32)(dTransferSpeed / g_lNotifySize / 4);
			if (g_dwUpdateBuffers < 1)
				g_dwUpdateBuffers = 1;
			g_dwUpdateCount = g_dwUpdateBuffers;
		}
		printf("\n");


		switch (_getch())
		{
		case 27: return false;
		case 13:
			if (g_lBufferSize <= (int32)qwContBufLen)
				printf("\n***** Continuous Buffer from Kernel Driver used *****\n\n");
			return true;

		case 'i':
		case 'I':
			switch (g_eMode)
			{
			case eStandard:     g_eMode = eHDSpeedTest; break;
			case eHDSpeedTest:  g_eMode = eSpeedTest; break;
			case eSpeedTest:    g_eMode = eStandard; break;
			}
			break;

		case 't':
		case 'T':
			g_bThread = !g_bThread;
			break;

		case 's':
		case 'S':
			printf("Sampling Rate (MS/s): ");
			scanf("%lf", &dTmp);
			g_lSamplingRate = (int32)(dTmp * MEGA(1));
			break;

		case 'b':
		case 'B':
			printf("Buffer Size (MByte): ");
			scanf("%lf", &dTmp);
			g_lBufferSize = (int32)(dTmp * MEGA_B(1));
			break;

		case 'n':
		case 'N':
			printf("Notify Size (kByte): ");
			scanf("%lf", &dTmp);
			g_lNotifySize = (int32)(dTmp * KILO_B(1));
			break;

		case 'c':
		case 'C':
			printf("Channel Enable Mask (hex): ");
			scanf("%x", &dwTmp);
			g_qwChannelEnable = dwTmp;
			break;

		}
	}
}



/*
**************************************************************************
main
**************************************************************************
*/

int main()
{
	char                szBuffer[1024];     // a character buffer for any messages
	ST_SPCM_CARDINFO    astCard[MAXBRD];    // info structure of my card
	ST_BUFFERDATA       stBufferData;       // buffer and transfer definitions
	ST_WORKDATA         stWorkData;         // work data for the working functions
	int32               lCardIdx = 0;
	int32               lCardCount = 0;

	// ------------------------------------------------------------------------
	// init cards, get some information and print it
	for (lCardIdx = 0; lCardIdx < MAXBRD; lCardIdx++)
	{
		if (bSpcMInitCardByIdx(&astCard[lCardCount], lCardIdx))
		{
			printf(pszSpcMPrintCardInfo(&astCard[lCardCount], szBuffer, sizeof (szBuffer)));
			printf("\n");
			lCardCount++;
		}
	}
	if (lCardCount == 0)
	{
		printf("No Spectrum card found...\n");
		return 0;
	}

	// if we have more than one card we make the selection now
	if (lCardCount > 1)
	{
		do
		{
			printf("\n");
			printf("Please select the card to test:\n");
			printf("-------------------------------\n");
			for (lCardIdx = 0; lCardIdx < lCardCount; lCardIdx++)
				printf("%d ..... M2i.%04x sn %05d\n", lCardIdx, astCard[lCardIdx].lCardType & TYP_VERSIONMASK, astCard[lCardIdx].lSerialNumber);

			int16 nSelection = _getch();
			if (nSelection == 27)
				return 1;
			if ((nSelection >= '0') && (nSelection < ('0' + lCardIdx)))
				lCardIdx = (nSelection - '0');
		} while (lCardIdx == lCardCount);

		// close all the other cards allowing a second instance of the program to run
		for (int32 lCloseIdx = 0; lCloseIdx < lCardCount; lCloseIdx++)
		if (lCloseIdx != lCardIdx)
			vSpcMCloseCard(&astCard[lCloseIdx]);
	}
	else
		lCardIdx = 0;



	// check whether we support this card type in the example
	if ((astCard[lCardIdx].eCardFunction != AnalogIn) && (astCard[lCardIdx].eCardFunction != DigitalIn) && (astCard[lCardIdx].eCardFunction != DigitalIO))
		return nSpcMErrorMessageStdOut(&astCard[lCardIdx], "Error: Card function not supported by this example\n", false);


	// we start with 16 bit acquisition as this is supported by all cards
	switch (astCard[lCardIdx].eCardFunction)
	{
	case AnalogIn:
		switch (astCard[lCardIdx].lBytesPerSample)
		{
		case 1: g_qwChannelEnable = 0x03; break;
		case 2: g_qwChannelEnable = 0x01; break;
		}
		break;

	case DigitalIn:
	case DigitalIO:
		g_qwChannelEnable = 0xffff;
		break;
	}

	//------------------ code not in the original example -------------------//
	//int16_t* signal_int16;

	////open matlab engine
	//Engine *ep;
	//mxArray *T = NULL, *result = NULL;
	//mxArray *T_uint16 = NULL;

	//if (!(ep = engOpen(""))) {
	//     fprintf(stderr, "\nCan't start MATLAB engine\n");
	//     return EXIT_FAILURE;
	//}
	int cnt = 0;
	float sleep_t = 0;
	printf("loop %d st \n", cnt);
	std::cout << "Sleep time in ms : ";
	std::cin >> sleep_t;
	std::cout << '\n';
	
	// ------------------------------------------------------------------------
	// do the card setup, error is routed in the structure so we don't care for the return values
	while (bSetup(&astCard[lCardIdx]))
	{
		//Sleep(sleep_t);
		if (!astCard[lCardIdx].bSetError)
			bDoCardSetup(&astCard[lCardIdx]);
		

		// ------------------------------------------------------------------------
		// setup the data transfer thread and start it, we use atimeout of 5 s in the example
		memset(&stBufferData, 0, sizeof(stBufferData));
		stBufferData.pstCard = &astCard[lCardIdx];
		stBufferData.bStartCard = true;
		stBufferData.bStartData = true;
		stBufferData.lTimeout = 5000;

		// setup for async esc check
		g_nKeyPress = GetAsyncKeyState(VK_ESCAPE);

		// start the threaded version if g_bThread is defined
		if (!astCard[lCardIdx].bSetError && g_bThread)
			vDoThreadMainLoop(&stBufferData, &stWorkData, bWorkInit, bWorkDo, vWorkClose, bKeyCheckAsync);

		// start the unthreaded version with a smaller timeout of 100 ms to gain control about the FIFO loop
		stBufferData.lTimeout = 100;


		if (!astCard[lCardIdx].bSetError && !g_bThread)
		{
			
			vDoMainLoop(&stBufferData, &stWorkData, bWorkInit, bWorkDo, vWorkClose, bKeyCheckAsync);
			cnt++;
		}

		// ------------------------------------------------------------------------
		// print error information if an error occured
		if (astCard[lCardIdx].bSetError)
			return nSpcMErrorMessageStdOut(&astCard[lCardIdx], "An error occured while programming the card:\n", true);

	} // if (bStart)



	// clean up and close the driver
	vSpcMCloseCard(&astCard[lCardIdx]);

	return 1;
}

