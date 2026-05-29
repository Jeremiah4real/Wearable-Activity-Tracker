/**
***************************************************************************
* @file         PedometerAlgorithm.c
* @author       ADI
* @version      V2.1.0
* @date         03-Jan-2022
* @brief        
/**
***************************************************************************
* @attention
***************************************************************************
*/
/******************************************************************************
*                                                                             *
* License Agreement                                                           *
*                                                                             *
* Copyright (c) 2017 Analog Devices Inc.                                      *
* All rights reserved.                                                        *
*                                                                             *
* This source code is intended for the recipient only under the guidelines of *
* the non-disclosure agreement with Analog Devices Inc.                       *
*                                                                             *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     *
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER         *
* DEALINGS IN THE SOFTWARE.                                                   *
*                                                                             *
*                                                                             *
* This software is intended for use with the Pedometer library                *
*                                                                             *
*                                                                             *
******************************************************************************/
#ifndef __PEDOMETER_ALGORITHM_H
#define __PEDOMETER_ALGORITHM_H

#ifdef __cplusplus
  extern "C" {
#endif
      #define PEDALGORITHMVERSIONNUMBER (0x00020000) // 2.0.0
      
      void InitAlgorithmParameters();
      int32_t StepAlgorithm(    int16_t X,
                                int16_t Y,
                                int16_t Z, 
                                int32_t count_steps);
   #ifdef __cplusplus
  }
  #endif

#endif // __PEDOMETER_ALGORITHM_H
