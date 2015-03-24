; Copyright (c) 2009-2011, Texas Instruments Incorporated
; Copyright (c) 2012-2014, International Business Machines Corp.
; All rights reserved.
; Redistribution and use in source and binary forms, with or without modification, 
; are permitted provided that the following conditions are met: 
; 1) Redistributions of source code must retain the above copyright notice, 
; this list of conditions and the following disclaimer: 2) Redistributions in binary 
; form must reproduce the above copyright notice, this list of conditions and the 
; following disclaimer in the documentation and/or other materials provided with the 
; distribution, and; 3) Neither the name of the Texas Instruments nor the names of its 
; contributors may be used to endorse or promote products derived from this software 
; without specific prior written permission.

; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS, 
; STATUTORY OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
; MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
; COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
; EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
; HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
; OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


;// Include variables and constants that will be shared in the
;// C28x C-code and CLA assembly code.  This is accomplished by
;// ucosg .cdecls to include a C-code header file that contains
;// these variables and constants

      .cdecls   C,LIST,"CLAShared.h"

  
;// To include an MDEBUGSTOP (CLA breakpoint) as the first instruction
;// of each task, set CLA_DEBUG to 1.  Use any other value to leave out
;// the MDEBUGSTOP instruction.

CLA_DEBUG .set 0
T1_DEBUG  .set 0
T11_DEBUG  .set 1
T2_DEBUG  .set 0
T3_DEBUG  .set 0
T4_DEBUG  .set 0 
T5_DEBUG  .set 0
T6_DEBUG  .set 0
T7_DEBUG  .set 0
T8_DEBUG  .set 0

NUMBEROFCHANNELSOFFSET .set 0
CURRENTCHANNELOFFSET   .set 2
CHANNELSOFFSET         .set 4

; Intermediate results
_NumberOfChannels  .set _IIRFILTERSET + NUMBEROFCHANNELSOFFSET
_CurrentChannel    .set _IIRFILTERSET + CURRENTCHANNELOFFSET
_Channels          .set _IIRFILTERSET + CHANNELSOFFSET

_xn              .set _CHX + 0
_yn              .set _CHX + 2
_A0              .set _CHX + 4   
_A1              .set _CHX + 6
_A2              .set _CHX + 8
_B0              .set _CHX + 10  ; not used
_B1              .set _CHX + 12
_B2              .set _CHX + 14
_Q0              .set _CHX + 16
_Q1              .set _CHX + 18
_Q2              .set _CHX + 20
_Q3              .set _CHX + 22

;// CLA code must be within its own assembly section and must be
;// even aligned.  Note: cosce all CLA instructions are 32-bit
;// this alignment naturally occurs and the .align 2 is most likely
;// redundant

       .sect        "Cla1Prog"
_Cla1Prog_asm_Start
       .align       2
              
_Cla1Task1:
   .if T1_DEBUG
   MDEBUGSTOP
   .endif 
   ; Place input in current filter context
   MMOV16	    MAR1,@_pADCResult
   MNOP
   MNOP
   MNOP
   MUI16TOF32   MR0, *MAR1[1]++
   MNOP
   MNOP
   MNOP
   MMOV32       @_xn, MR0
   MMOV16       @_pADCResult, MAR1
   MNOP
   MNOP
   MNOP
   .if CLA_DEBUG
   MDEBUGSTOP   ; tgh
   .endif
   MCCNDD       _IIRBiquad, UNC
   MNOP
   MNOP
   MNOP

   ; IIR Biquad Filter is complete.
   ; Need to update the output array.
   ; The scratchpad Filter context needs to be placed back into the data structure for the next time.
   ; The next active filter needs to be copied into the scratchpad filter context.
   
   ; Update the output array yn_array[] with current filter output
   MMOV16       MAR1, @_pyn_array
   MMOV32       MR0, @_yn          ;MR0 = yn  
   MNOP
   MNOP
   ;MF32TOUI16R  MR0,MR0           ;tgh20120820
   MMOV32       *MAR1[2]++, MR0   
   MMOV16       @_pyn_array, MAR1

   ; Switch completed IIR filter context back to data structure
   MMOV16       MAR1, @_pCURRENTIIRFILTER
   MNOP
   MNOP
   MNOP
   MCCNDD       _DeActivateChannel, UNC
   MNOP
   MNOP
   MNOP

   .if T11_DEBUG
   MDEBUGSTOP
   .endif 
   ; Update current channel number to next filter
   MMOV32       MR1, @_CurrentChannel
   MADDF32      MR1, #1.0, MR1
   MMOV32       @_CurrentChannel, MR1
   MMOV32       MR0, @_NumberOfChannels
   MCMP32       MR0, MR1
   MNOP
   MNOP
   MNOP
   MBCNDD       _Cla1Task1Done, EQ
   MBCNDD       _Cla1Task1Done, LT
   MNOP
   MNOP
   MNOP
   MMOV16       @_pCURRENTIIRFILTER, MAR1
   ; Place next active filter into current context
   MCCNDD       _ActivateChannel, UNC
   MNOP
   MNOP
   MNOP
   MBCNDD       _Cla1Task1, UNC
   MNOP
   MNOP
   MNOP

_Cla1Task1Done:
   MNOP

   MSTOP
_Cla1Task1End:


_Cla1Task2:
   .if T2_DEBUG
   MDEBUGSTOP
   .endif
   MSTOP
  
_Cla1Task2End:


_Cla1Task3:
   .if T3_DEBUG
   MDEBUGSTOP
   .endif
   MSTOP
  
_Cla1Task3End:

_Cla1Task4:
   .if T4_DEBUG
   MDEBUGSTOP
   .endif
   MSTOP
  
_Cla1Task4End:

_Cla1Task5:
   .if T5_DEBUG
   MDEBUGSTOP
   .endif
   MSTOP
   
_Cla1Task5End:

_Cla1Task6:
   .if T6_DEBUG
   MDEBUGSTOP
   .endif
   MSTOP
_Cla1Task6End:


_Cla1Task7:
   .if T7_DEBUG
   MDEBUGSTOP
   .endif
   MSTOP
_Cla1Task7End:


_Cla1Task8:
; Variable Initialization
   .if T8_DEBUG
   MDEBUGSTOP
   .endif
   MMOV32        MR0, @_pINITIALIIRFILTER
   MMOV32        @_pCURRENTIIRFILTER, MR0
   MMOV32        MR0, @_pInitialADCResult
   MMOV32        @_pADCResult, MR0
   ;MMOV32        MR0, @_pInitialyn_array
   ;MMOV32        @_yn_array, MR0           ;tgh
   .if CLA_DEBUG
   MDEBUGSTOP
   .endif
   MMOV32        MR0, @_pInitialyn_array ;tgh
   MMOV32        @_pyn_array, MR0 ;tgh
   .if CLA_DEBUG
   MDEBUGSTOP
   .endif
   MMOV32        MR0, @_pInitialxn_array
   MMOV32        @_pxn_array, MR0  ;tgh
   MMOVIZ        MR0,#0.0
   MMOV32        @_CurrentChannel, MR0
   
   MMOV16        MAR1, @_pCURRENTIIRFILTER
   MNOP
   MNOP
   MNOP
   MCCNDD        _ActivateChannel, UNC
   MNOP
   MNOP
   MNOP
   MSTOP
  
_ActivateChannel:
   ; Move Current Channel into scratchpad CHX
   MMOV32        MR0, *MAR1[2]++
   MMOV32        @_xn, MR0
   MMOV32        MR0, *MAR1[2]++
   MMOV32        @_yn, MR0
   MMOV32        MR0, *MAR1[2]++
   MMOV32        @_A0, MR0
   MMOV32        MR0, *MAR1[2]++
   MMOV32        @_A1, MR0
   MMOV32        MR0, *MAR1[2]++
   MMOV32        @_A2, MR0
   MMOV32        MR0, *MAR1[2]++
   MMOV32        @_B0, MR0
   MMOV32        MR0, *MAR1[2]++
   MMOV32        @_B1, MR0
   MMOV32        MR0, *MAR1[2]++
   MMOV32        @_B2, MR0
   MMOV32        MR0, *MAR1[2]++
   MMOV32        @_Q0, MR0
   MMOV32        MR0, *MAR1[2]++
   MMOV32        @_Q1, MR0
   MMOV32        MR0, *MAR1[2]++
   MMOV32        @_Q2, MR0
   MMOV32        MR0, *MAR1[2]++
   MMOV32        @_Q3, MR0
   MRCNDD
   MNOP
   MNOP
   MNOP
  
_DeActivateChannel:
   ; Move scratchpad CHX into Current Channel
   MMOV32        MR0, @_xn
   MMOV32        *MAR1[2]++, MR0
   MMOV32        MR0, @_yn
   MMOV32        *MAR1[2]++, MR0
   MMOV32        MR0, @_A0
   MMOV32        *MAR1[2]++, MR0
   MMOV32        MR0, @_A1
   MMOV32        *MAR1[2]++, MR0
   MMOV32        MR0, @_A2
   MMOV32        *MAR1[2]++, MR0
   MMOV32        MR0, @_B0
   MMOV32        *MAR1[2]++, MR0
   MMOV32        MR0, @_B1
   MMOV32        *MAR1[2]++, MR0
   MMOV32        MR0, @_B2
   MMOV32        *MAR1[2]++, MR0
   MMOV32        MR0, @_Q0
   MMOV32        *MAR1[2]++, MR0
   MMOV32        MR0, @_Q1
   MMOV32        *MAR1[2]++, MR0
   MMOV32        MR0, @_Q2
   MMOV32        *MAR1[2]++, MR0
   MMOV32        MR0, @_Q3
   MMOV32        *MAR1[2]++, MR0
   MRCNDD
   MNOP
   MNOP
   MNOP

_IIRBiquad:  
; Implementing the Transposed Direct II form of the
; IIR filter. Each Second order Section (biquad) of the filter
; looks like the following:
;
; xn------>(x)------>(+)--------------->yn
;      |    ^         ^             |
;      |    |         |q3           |
;      |    a0       (z)            |
;      |              ^             |
;      |              |q2           |
;      |-->(x)------>(+)<-----(x)---|
;      |    ^         ^        ^    |
;      |    |         |q1      |    |
;      |    a1       (z)       b1   |
;      |              ^             |
;      |              |q0           |
;      --->(x)------>(+)<-----(x)----
;           ^                  ^
;           |                  |
;           a2                 b2
;
; The operations for each biquad, in order, is as follows 
; 1. yn = xn*a0 + q3 
; 2. q2 = xn*a1 + yn*b1 + q1
; 3. q0 = xn*a2 + yn*b2
; 4. q3 = q2
; 5. q1 = q0
;
   MMOV32	    MR0,@_xn               ;Sample input in MR0

   MMOV32	    MR1,@_A0               ;MR1 = a0

   MMPYF32	    MR3,MR0,MR1 ||         ;MR3 = xn*a0
   MMOV32	    MR2,@_Q3               ;MR2 = q3

   MMOV32	    MR1,@_A1               ;MR1 = a1

   MMACF32	    MR3,MR2,MR2,MR1,MR0 || ;MR3 =xn*b1+q3, MR2=xn*a1
   MMOV32       MR1,@_B1               ;MR1 = b1

   MMOV32	    @_yn,MR3               ;yn = MR3

   MMPYF32	    MR3,MR1,MR3	||         ;MR3 = yn*b1
   MMOV32	    MR1,@_A2               ;MR1 = a2

   MMACF32	    MR3,MR2,MR0,MR1,MR0	|| ;MR3 =yn*b1+xn*a1,MR0=xn*a2
   MMOV32	    MR2,@_Q1               ;MR2 = q1

   MMOV32	    @_Q1,MR0               ;Temporarily save off MR0 in q1
   MMOV32	    MR1,@_B2               ;MR1 = b2
   MMOV32	    MR0,@_yn               ;MR0 = yn

   MMACF32	    MR3,MR2,MR1,MR0,MR1 || ;MR3 =q2 =yn*b1+xn*a1+q1,MR1=yn*b2
   MMOV32	    MR0,@_Q1               ;Restore saved off value of MR0

   MMOV32	    @_Q3,MR3               ;q3 = q2
   MADDF32	    MR0,MR1,MR0            ;MR0 =q0=yn*b2+xn*a2
   MMOV32	    @_Q1,MR0               ;q1 = q0

   MMOVD32	    MR0,@_Q3               ;MR0=q3 dummy move, q3=q2
   MNOP
   MMOVD32	    MR0,@_Q1               ;MR0=q1 dummy move, q1=q0
   MRCNDD
   MNOP
   MNOP
   MNOP

_Cla1Task8End:

