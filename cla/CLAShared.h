/** Copyright (c) 2009-2011, Texas Instruments Incorporated
Copyright (c) 2012-2014, International Business Machines Corp.
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met: 
1) Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer: 2) Redistributions in binary 
form must reproduce the above copyright notice, this list of conditions and the 
following disclaimer in the documentation and/or other materials provided with the 
distribution, and; 3) Neither the name of the Texas Instruments nor the names of its 
contributors may be used to endorse or promote products derived from this software 
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS, 
STATUTORY OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#ifndef CLASHARED_H_
#define CLASHARED_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "DSP28x_Project.h"

#define MAX_CHANNELS 16

//; Implementing the Transposed Direct II form of the
//; IIR filter. Each Second order Section (biquad) of the filter
//; looks like the following:
//;
//; xn------>(x)------>(+)--------------->yn
//;      |    ^         ^             |
//;      |    |         |q3           |
//;      |    a0       (z)            |
//;      |              ^             |
//;      |              |q2           |
//;      |-->(x)------>(+)<-----(x)---|
//;      |    ^         ^        ^    |
//;      |    |         |q1      |    |
//;      |    a1       (z)       b1   |
//;      |              ^             |
//;      |              |q0           |
//;      --->(x)------>(+)<-----(x)----
//;           ^                  ^
//;           |                  |
//;           a2                 b2
//;

typedef struct {
  float xn ;		// input sample to IIR biquad
  float yn ;		// output sample from IIR biquad
  float A[3] ;		// 
  float B[3] ;		// 
  float Q[4] ;		// delay line for IIR biquad
} FILTER ;

typedef struct {
  float NumberOfChannels ;
  float CurrentChannel ;
  FILTER Channels[MAX_CHANNELS] ;
} FILTERSET ;


// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them
// .global and the main CPU can make use of them.

  extern Uint32 Cla1Task1;
  extern Uint32 Cla1Task2;
  extern Uint32 Cla1Task3;
  extern Uint32 Cla1Task4;
  extern Uint32 Cla1Task5;
  extern Uint32 Cla1Task6;
  extern Uint32 Cla1Task7;
  extern Uint32 Cla1Task8;
  extern Uint32 Cla1Prog_asm_Start;
  extern Uint32 Cla1T1End;
  extern Uint32 Cla1T2End;
  extern Uint32 Cla1T3End;
  extern Uint32 Cla1T4End;
  extern Uint32 Cla1T5End;
  extern Uint32 Cla1T6End;
  extern Uint32 Cla1T7End;
  extern Uint32 Cla1T8End;
  
//Task 1 (ASM) Variables
extern FILTER CHX ;
extern FILTERSET IIRFILTERSET ;
extern FILTERSET *pIIRFILTERSET ;
extern FILTER *pCURRENTIIRFILTER ;
extern float *pyn_array ;
extern float *pxn_array ;
extern float xn_array[MAX_CHANNELS]; //Sample input
extern float yn_array[MAX_CHANNELS]; //Sample output
//extern volatile Uint16 *pADCResult ;
extern Uint16 *pADCResult ;
extern FILTER *pINITIALIIRFILTER ;
extern Uint16 *pInitialADCResult ;
extern float *pInitialyn_array ;
extern float *pInitialxn_array ;

//Task 2 (ASM) Variables

//Task 3 (ASM) Variables

//Task 4 (ASM) Variables

//Task 5 (ASM) Variables

//Task 6 (ASM) Variables

//Task 7 (ASM) Variables

//Task 8 (ASM) Variables

//Common (ASM) Variables


#ifdef __cplusplus
}
#endif /* extern "C" */
#endif /*CLASHARED_H_*/
