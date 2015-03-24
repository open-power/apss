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
#include "DSP2803x_Device.h"
#include "CLAShared.h"

//typedef struct {
//  float xn ;
//  float yn ;
//  float A[3] ;
//  float B[3] ;
//  float Q[4] ;
//} FILTER ;

#define IIR_COEFF_20KHZ_SAMPLE_2KHZ_CUTOFF 1

#ifdef IIR_COEFF_20KHZ_SAMPLE_2KHZ_CUTOFF
/* @brief IIR Filter Coefficients @ 20kHz sample rate, 2kHz (-3dB) cutoff freq
 *
 * !! Important !!  
 * If you use anything to calculate new IIR coefficients, you
 * will need to negate the 'B' coefficients since the CLA IIR algorithm does 
 * all adds and no subtracts.  In doing this, it assuming the B*y sums are 
 * all negative, which will only happen if the B coefficients are negated. 
 *
 */
#define IIR_COEFF_A0  0.06745508395870334
#define IIR_COEFF_A1  0.13491016791740668
#define IIR_COEFF_A2  0.06745508395870334
#define IIR_COEFF_B0  -1
#define IIR_COEFF_B1  1.1429772843080923
#define IIR_COEFF_B2  -0.41279762014290533
#else

#endif

#define CH0 \
 {0.0,\
  0.0,\
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH1 \
 {0.0,\
  0.0,\
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH2 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH3 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH4 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH5 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH6 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH7 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}


#define CH8 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH9 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH10 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH11 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH12 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH13 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH14 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}

#define CH15 \
 {0.0, \
  0.0, \
  {IIR_COEFF_A0, IIR_COEFF_A1, IIR_COEFF_A2},\
  {IIR_COEFF_B0, IIR_COEFF_B1, IIR_COEFF_B2},\
  {0, 0, 0, 0}}


//
// The DATA_SECTION pragma statements are used to place the
// variables in specific named sections.  These sections
// are linked to the message and data RAMs for the CLA in the
// linker command (.cmd) file.
//
// The following will be placed in the CLA to CPU
// message RAM.  The CLA can write to and read from
// this RAM.  The main CPU can only read from it

#pragma DATA_SECTION(pINITIALIIRFILTER,"CpuToCla1MsgRAM")
FILTER *pINITIALIIRFILTER = &IIRFILTERSET.Channels[0] ;
#pragma DATA_SECTION(pInitialADCResult,"CpuToCla1MsgRAM")
Uint16 *pInitialADCResult = (Uint16 *)&AdcResult.ADCRESULT0 ;
#pragma DATA_SECTION(pInitialyn_array,"CpuToCla1MsgRAM")
float *pInitialyn_array = &yn_array[0] ;
#pragma DATA_SECTION(pInitialxn_array,"CpuToCla1MsgRAM")
float *pInitialxn_array = &xn_array[0] ;

#pragma DATA_SECTION(yn_array,"Cla1ToCpuMsgRAM")
float yn_array[MAX_CHANNELS] ;
#pragma DATA_SECTION(xn_array,"Cla1ToCpuMsgRAM")
float xn_array[MAX_CHANNELS];

// CLA Task Data
// Ensure that all data is placed in the data rams
// Task Variables

// CHX is a scratchpad filter space for the CLA.
// This is the area which the CLA IIR task will operate on to simplify
// the CLA software.  Another task on the CLA will context switch this
// scratchpad to the appropriate, active channel.

#pragma DATA_SECTION(CHX,"ClaDataRam1")
FILTER CHX = {
  0.0,							// xn
  0.0,							// yn
  {0, 0, 0},					// A
  {0, 0, 0},					// B
  {0, 0, 0, 0}					// Q
} ;

#pragma DATA_SECTION(IIRFILTERSET,"ClaDataRam1")
FILTERSET IIRFILTERSET = {
  (float)MAX_CHANNELS,					// NumberOfChannels
  (float)0,							// CurrentChannel
  {CH0,						    // Channels[]
   CH1,
   CH2,
   CH3,
   CH4,
   CH5,
   CH6,
   CH7,
   CH8,
   CH9,
   CH10,
   CH11,
   CH12,
   CH13,
   CH14,
   CH15
  }
} ;

#pragma DATA_SECTION(pIIRFILTERSET,"ClaDataRam1")
FILTERSET *pIIRFILTERSET = &IIRFILTERSET ;
#pragma DATA_SECTION(pCURRENTIIRFILTER,"ClaDataRam1")
FILTER *pCURRENTIIRFILTER = &IIRFILTERSET.Channels[0] ;
#pragma DATA_SECTION(pyn_array,"ClaDataRam1")
float *pyn_array = &yn_array[0] ;
#pragma DATA_SECTION(pxn_array,"ClaDataRam1")
float *pxn_array = &xn_array[0] ;
#pragma DATA_SECTION(pADCResult,"ClaDataRam1")
Uint16 *pADCResult = (Uint16 *)&AdcResult.ADCRESULT0 ;
