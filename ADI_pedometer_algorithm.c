/**
***************************************************************************
* @file         PedometerAlgorithm.c
* @author       ADI
* @version      V2.1.0
* @date         03-Jan-2022
* @brief
*
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
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <inttypes.h>
#include "ADI_pedometer_algorithm.h"

//------------------------ PEDOMETER CONSTANTS ---------------------------//

#define   _1_SECOND               70  // ODR = 50Hz, 50 samples = 1 second
#define   REGULATION_OFF_TIME     (_1_SECOND * 3) // 3 seconds
#define   THRESHOLD_ORDER         3
#define   FILTER_ORDER            2   //2
#define   WINDOW_SIZE             ((FILTER_ORDER <<2) +1)
#define   SENSITIVITY             ((1000*3)/4-20)            /* 400 = 0.1gee, for 2g range */
#define   INIT_OFFSET_VALUE       23000                   /* 1 gees, for 2g range*/
#define   INIT_VALUE_MIN          0                       /* 0 gees*/

//-------------------------PEDOMETER VARIABLES-------------------------//
#ifdef __cplusplus
  extern 'C' {
#endif

int32_t buffer_RawData[FILTER_ORDER];//--> Buffer to store the not filtering module data
int32_t buffer_filtered_window[WINDOW_SIZE];// --> Buffer to store the filtering module data
int32_t buffer_dynamic_threshold[THRESHOLD_ORDER]; // -->Buffer to store the Threshold data
int32_t count_steps;

// IndexWindowMin--> Shows where is the window minimum
// IndexWindowMax--> Shows where is the window max
// IndexThreshold--> Shows the current index in the Threshold Buffer
// IndexBuffer--> Shows the current index in the window
// flag_max_min_samplecounter--> Count the samples between max and min
//  to discard step if it would be very long.
// Regulation_mode-->Sets if the pedometer is in Regulation mode or not.
// StepToStepSamples--> This number is incremented every algorithm iteration
//    to count the time. If it is greater than an specific value the pedometer
//   leaves from the regulation mode. It is reset every detected step.
// possible_steps--> They are the counted steps in Regulation mode,
// if the pedometer counts 4 consecutive possible steps this steps will be
// real steps.

int8_t IndexWindowMin, IndexWindowMax;
int8_t IndexThreshold, IndexBuffer, IndexAverage;
int8_t StepToStepSamples, Regulation_mode;
int8_t flag_max_min_samplecounter, possible_steps;

// flag_max--> Maximum detected
// flag_threshold--> it indicates that the Threshold has been overcome
// flag_threshold_counter--> it indicates the number of times that the
//  Threshold has been overcome
uint8_t flag_max, flag_threshold,flag_threshold_counter;

// LastMax--> Last max value
// LastMin--> Last min value
// WindowMin--> window minimum value
// WindowMax--> window max value
// FilterMeanBuffer-->filtered mean buffer
// FilterModuleData--> filtered module data
// ModuleData--> Not filtered module data
// Difference--> Difference between the max and min
// BufferDinamicThreshold--> Threshold Buffer
// NewThreshold--> New Threshold
// old_threshold--> New Threshold
uint32_t LastMax, LastMin, WindowMin, WindowMax;
uint32_t FilterMeanBuffer, FilterModuleData, ModuleData;
uint32_t Difference, BufferDinamicThreshold, NewThreshold, old_threshold;

//-------------------------------------------------------------------//


void InitAlgorithmParameters() {
   int8_t i;
   flag_max = 0;
   flag_max_min_samplecounter = 0;
   count_steps = 0;
   IndexBuffer = 0;
   IndexAverage = 0;
   IndexWindowMin = 0;
   IndexWindowMax = 0;
   IndexThreshold = 0;
   FilterMeanBuffer = 0;
   FilterModuleData = 0;
   StepToStepSamples = 0;
   Regulation_mode = 0;
   possible_steps = 0;
   NewThreshold = 0;
   LastMax = 0;
   LastMin = 0;
   Difference = 0;
   ModuleData = 0;
   old_threshold = INIT_OFFSET_VALUE;
   BufferDinamicThreshold = INIT_OFFSET_VALUE * THRESHOLD_ORDER;//INIT_OFFSET_VALUE<<2
   flag_threshold = 0;
   flag_threshold_counter = 0;

   for (i = 0; i < THRESHOLD_ORDER; i++) {
     buffer_dynamic_threshold[i] = INIT_OFFSET_VALUE;
   }
   for (i = 0; i < FILTER_ORDER; i++) {
     buffer_RawData[i] = 0;
   }
   for (i = 0; i < WINDOW_SIZE; i++) {
     buffer_filtered_window[i] = 0;
   }
}


int32_t StepAlgorithm(int16_t X, int16_t Y, int16_t Z, int32_t count_steps) {

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //++++++++++++++++++++++++   SIGNAL FILTERING   ++++++++++++++++++++++++++
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      uint16_t Abs_X, Abs_Y, Abs_Z;
      int8_t i;
      Abs_X = abs(X);
      Abs_Y = abs(Y);
      Abs_Z = abs(Z);
      ModuleData = Abs_X + Abs_Y + Abs_Z;
#if 1
      // Eliminate the last value from the mean and add the new value
      FilterMeanBuffer = FilterMeanBuffer - buffer_RawData[IndexAverage] + ModuleData;
      FilterModuleData = FilterMeanBuffer / FILTER_ORDER;// FilterMeanBuffer>>2
      // stores the module in the not filtered buffer
      buffer_RawData[IndexAverage] = ModuleData;

      buffer_filtered_window[IndexBuffer] = FilterModuleData;

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //++++++++++++++++++++++++  MAX AND MIN DETECTION  ++++++++++++++++++++++++
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      WindowMax = buffer_filtered_window[0];
      IndexWindowMax = 0;
      for (i = 1; i < WINDOW_SIZE; i++) {

       if (buffer_filtered_window[i] > WindowMax) {
         WindowMax = buffer_filtered_window[i];
         IndexWindowMax = i;
        }
      }

      WindowMin = buffer_filtered_window[0];
      IndexWindowMin = 0;
      for (i = 1; i < WINDOW_SIZE; i++) {

         if (buffer_filtered_window[i] < WindowMin) {
         WindowMin = buffer_filtered_window[i];
         IndexWindowMin = i;
       }
       }
        //======================================================================
        //MAX SEARCHING
 if (flag_max == 0) {
   // Now, we try to know if the Window max is in the midle of the window,
   //  in this case I will mark it as a max
   if (IndexWindowMax == ((IndexBuffer + (WINDOW_SIZE>>1)) % WINDOW_SIZE)) {
     flag_max = 1;
     LastMax = WindowMax;
     flag_max_min_samplecounter = 0;
   }
 } else {
   // Now, we try to know if the Window min is in the midle of the window,
   //  in this case I will mark it as a min
   if (IndexWindowMin == ((IndexBuffer + (WINDOW_SIZE>>1)) % WINDOW_SIZE)) {
   LastMin = WindowMin;  // Updates the MIN
   Difference = LastMax - LastMin;  // Updates the Difference
   flag_max = 0;
   flag_max_min_samplecounter = 0;  // Reset the max-min sample counter

   /* The algorithm detects if the SENSITIVITYs are in of the Threshold level */

   if ((LastMax > (old_threshold+(SENSITIVITY>>1)))&&(LastMin < (old_threshold-(SENSITIVITY>>1)))) {
     flag_threshold = 1;  // Possible step (it will be analize later)
     flag_threshold_counter = 0;
   } else {
     flag_threshold_counter++;  // Number of times the SENSITIVITYs are out of the Threshold Level consecutively
   }
   /* SENSITIVITY detection */
   /* Each time the difference is higher than the fixed SENSITIVITY the algorithm updates the Threshold */

   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   //+++++++++++++++++++++++ THRESHOLD LEVEL UPDATE ++++++++++++++++++++++++++++
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   if (Difference > SENSITIVITY) {
     /* Threshold level is calculated with the 4 previous good differences */
     // The same method as the Filtering method
     NewThreshold = (LastMax + LastMin) >> 1;  // Calculates the New Threshold
     BufferDinamicThreshold = BufferDinamicThreshold - buffer_dynamic_threshold[IndexThreshold] + NewThreshold;
     old_threshold =BufferDinamicThreshold / THRESHOLD_ORDER; //BufferDinamicThreshold<<2
     buffer_dynamic_threshold[IndexThreshold] = NewThreshold;
     IndexThreshold++;
     if (IndexThreshold > THRESHOLD_ORDER - 1) {
       IndexThreshold = 0;
     }

     // Once the Threshold Level is calculated and if the SENSITIVITYs was in the
     // Threshold Level we certificate the step.

     //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
     //++++++++++++++++++++++++ STEP CERTIFICATION   +++++++++++++++++++++++++
     //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

     if (flag_threshold) {
       // DETECTED STEP
       //Regulation_mode=1;
       flag_threshold = 0;
       StepToStepSamples = 0; // Reset the time.
       flag_max_min_samplecounter = 0;

       if (Regulation_mode) {
         //=====Regulation mode======
         // the step is counted
         count_steps+=1;
       } else {
         //=====Non Regulation mode======
         possible_steps++;

         // if the possible steps number is 8,
         // the algorithm concludes that the person is walking.
         if (possible_steps == 1) {
           count_steps = count_steps + 2; // Possible steps are added to counted steps
           possible_steps = 0;
           Regulation_mode = 1;  // The pedometer enters the Regulation mode

         }
       }
     }
   }
   WindowMin = INIT_VALUE_MIN;
   // If the SENSITIVITYs are out of the Threshold Level 2 times consecutively with a good difference,
   // Is very probably that the acceleration profile isn't a step sequence
   // (Reset the possible steps to discard false steps)            //Edit by Jeremiah: reset possible steps in regulation mode only
   if (flag_threshold_counter > 1 && Regulation_mode) {
     flag_threshold_counter = 0;
     flag_max_min_samplecounter = 0;
     WindowMax = 0;  // maybe it is not necessary  reset them.
     WindowMin = 0;
     Regulation_mode = 0;
     possible_steps = 0;
     flag_threshold_counter = 0;
   }
   } else {
     // While the pedometer is searching a min if between MAX and MIN
     //  takes more than 25 samples I reset the step.
     flag_max_min_samplecounter++;
     if (flag_max_min_samplecounter == _1_SECOND) {
       // Reset Step
       flag_max_min_samplecounter = 0;
       WindowMax = 0;
       WindowMin = 0;
       flag_max = 0;
       possible_steps = 0;
     }
   }
 }
 // IndexBuffer is used as a circular buffer index
 IndexBuffer++;
 if (IndexBuffer > WINDOW_SIZE-1) {
   IndexBuffer = 0;
 }

  // IndexAverage is used as a circular buffer index to average
  IndexAverage++;
  if (IndexAverage > FILTER_ORDER-1) {
    IndexAverage = 0;
  }

  StepToStepSamples++;
  if (StepToStepSamples >= REGULATION_OFF_TIME)
  {
    // If the pedometer takes 2 seconds without counting a step it
    //  returns to the Non Regulation mode.
    StepToStepSamples = 0;
    possible_steps = 0;
    Regulation_mode = 0;
    flag_max_min_samplecounter = 0;
    if (Regulation_mode == 1) {
      Regulation_mode = 0;
      //old_threshold = INIT_OFFSET_VALUE;
      //BufferDinamicThreshold = INIT_OFFSET_VALUE*THRESHOLD_ORDER;//(INIT_OFFSET_VALUE)<<2
      IndexThreshold = 0;
      flag_threshold_counter = 0;
      for (char i = 0; i < THRESHOLD_ORDER; i++) {
        buffer_dynamic_threshold[i] = INIT_OFFSET_VALUE;
      }
    }
  }
    #ifdef __cplusplus
}
  #endif
#endif
  return count_steps;
}
//#include <stdio.h>
//#include <string.h>
//#include <stddef.h>
//#include <stdlib.h>
//#include <inttypes.h>
//#include "ADI_pedometer_algorithm.h"
//
////------------------------ PEDOMETER CONSTANTS ---------------------------//
//
//#define   _1_SECOND               100  // ODR = 50Hz, 50 samples = 1 second
//#define   REGULATION_OFF_TIME     (_1_SECOND * 3) // 3 seconds
//#define   THRESHOLD_ORDER         3
//#define   FILTER_ORDER            2   //2
//#define   WINDOW_SIZE             ((FILTER_ORDER <<2) +1)
//#define   SENSITIVITY             ((1000*3)/4-20)            /* 400 = 0.1gee, for 2g range */
//#define   INIT_OFFSET_VALUE       19000                   /* 1 gees, for 2g range*/
//#define   INIT_VALUE_MIN          0                       /* 0 gees*/
//
////-------------------------PEDOMETER VARIABLES-------------------------//
//#ifdef __cplusplus
//  extern 'C' {
//#endif
//
//int32_t buffer_RawData[FILTER_ORDER];//--> Buffer to store the not filtering module data
//int32_t buffer_filtered_window[WINDOW_SIZE];// --> Buffer to store the filtering module data
//int32_t buffer_dynamic_threshold[THRESHOLD_ORDER]; // -->Buffer to store the Threshold data
//int32_t count_steps;
//
//// IndexWindowMin--> Shows where is the window minimum
//// IndexWindowMax--> Shows where is the window max
//// IndexThreshold--> Shows the current index in the Threshold Buffer
//// IndexBuffer--> Shows the current index in the window
//// flag_max_min_samplecounter--> Count the samples between max and min
////  to discard step if it would be very long.
//// Regulation_mode-->Sets if the pedometer is in Regulation mode or not.
//// StepToStepSamples--> This number is incremented every algorithm iteration
////    to count the time. If it is greater than an specific value the pedometer
////   leaves from the regulation mode. It is reset every detected step.
//// possible_steps--> They are the counted steps in Regulation mode,
//// if the pedometer counts 4 consecutive possible steps this steps will be
//// real steps.
//
//int8_t IndexWindowMin, IndexWindowMax;
//int8_t IndexThreshold, IndexBuffer, IndexAverage;
//int8_t StepToStepSamples, Regulation_mode;
//int8_t flag_max_min_samplecounter, possible_steps;
//
//// flag_max--> Maximum detected
//// flag_threshold--> it indicates that the Threshold has been overcome
//// flag_threshold_counter--> it indicates the number of times that the
////  Threshold has been overcome
//uint8_t flag_max, flag_threshold,flag_threshold_counter;
//
//// LastMax--> Last max value
//// LastMin--> Last min value
//// WindowMin--> window minimum value
//// WindowMax--> window max value
//// FilterMeanBuffer-->filtered mean buffer
//// FilterModuleData--> filtered module data
//// ModuleData--> Not filtered module data
//// Difference--> Difference between the max and min
//// BufferDinamicThreshold--> Threshold Buffer
//// NewThreshold--> New Threshold
//// old_threshold--> New Threshold
//uint32_t LastMax, LastMin, WindowMin, WindowMax;
//uint32_t FilterMeanBuffer, FilterModuleData, ModuleData;
//uint32_t Difference, BufferDinamicThreshold, NewThreshold, old_threshold;
//
////-------------------------------------------------------------------//
//
//
//void InitAlgorithmParameters() {
//   int8_t i;
//   flag_max = 0;
//   flag_max_min_samplecounter = 0;
//   count_steps = 0;
//   IndexBuffer = 0;
//   IndexAverage = 0;
//   IndexWindowMin = 0;
//   IndexWindowMax = 0;
//   IndexThreshold = 0;
//   FilterMeanBuffer = 0;
//   FilterModuleData = 0;
//   StepToStepSamples = 0;
//   Regulation_mode = 0;
//   possible_steps = 0;
//   NewThreshold = 0;
//   LastMax = 0;
//   LastMin = 0;
//   Difference = 0;
//   ModuleData = 0;
//   old_threshold = INIT_OFFSET_VALUE;
//   BufferDinamicThreshold = INIT_OFFSET_VALUE * THRESHOLD_ORDER;//INIT_OFFSET_VALUE<<2
//   flag_threshold = 0;
//   flag_threshold_counter = 0;
//
//   for (i = 0; i < THRESHOLD_ORDER; i++) {
//     buffer_dynamic_threshold[i] = INIT_OFFSET_VALUE;
//   }
//   for (i = 0; i < FILTER_ORDER; i++) {
//     buffer_RawData[i] = 0;
//   }
//   for (i = 0; i < WINDOW_SIZE; i++) {
//     buffer_filtered_window[i] = 0;
//   }
//}
//
//
//int32_t StepAlgorithm(int16_t X, int16_t Y, int16_t Z, int32_t count_steps) {
//
//    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//    //++++++++++++++++++++++++   SIGNAL FILTERING   ++++++++++++++++++++++++++
//    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//      uint16_t Abs_X, Abs_Y, Abs_Z;
//      int8_t i;
//      Abs_X = abs(X);
//      Abs_Y = abs(Y);
//      Abs_Z = abs(Z);
//      ModuleData = Abs_X + Abs_Y + Abs_Z;
//#if 1
//      // Eliminate the last value from the mean and add the new value
//      FilterMeanBuffer = FilterMeanBuffer - buffer_RawData[IndexAverage] + ModuleData;
//      FilterModuleData = FilterMeanBuffer / FILTER_ORDER;// FilterMeanBuffer>>2
//      // stores the module in the not filtered buffer
//      buffer_RawData[IndexAverage] = ModuleData;
//
//      buffer_filtered_window[IndexBuffer] = FilterModuleData;
//
//    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//    //++++++++++++++++++++++++  MAX AND MIN DETECTION  ++++++++++++++++++++++++
//    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//      WindowMax = buffer_filtered_window[0];
//      IndexWindowMax = 0;
//      for (i = 1; i < WINDOW_SIZE; i++) {
//
//       if (buffer_filtered_window[i] > WindowMax) {
//         WindowMax = buffer_filtered_window[i];
//         IndexWindowMax = i;
//        }
//      }
//
//      WindowMin = buffer_filtered_window[0];
//      IndexWindowMin = 0;
//      for (i = 1; i < WINDOW_SIZE; i++) {
//
//         if (buffer_filtered_window[i] < WindowMin) {
//         WindowMin = buffer_filtered_window[i];
//         IndexWindowMin = i;
//       }
//       }
//        //======================================================================
//        //MAX SEARCHING
// if (flag_max == 0) {
//   // Now, we try to know if the Window max is in the midle of the window,
//   //  in this case I will mark it as a max
//   if (IndexWindowMax == ((IndexBuffer + (WINDOW_SIZE>>1)) % WINDOW_SIZE)) {
//     flag_max = 1;
//     LastMax = WindowMax;
//     flag_max_min_samplecounter = 0;
//   }
// } else {
//   // Now, we try to know if the Window min is in the midle of the window,
//   //  in this case I will mark it as a min
//   if (IndexWindowMin == ((IndexBuffer + (WINDOW_SIZE>>1)) % WINDOW_SIZE)) {
//   LastMin = WindowMin;  // Updates the MIN
//   Difference = LastMax - LastMin;  // Updates the Difference
//   flag_max = 0;
//   flag_max_min_samplecounter = 0;  // Reset the max-min sample counter
//
//   /* The algorithm detects if the SENSITIVITYs are in of the Threshold level */
//
//   if ((LastMax > (old_threshold+(SENSITIVITY>>1)))&&(LastMin < (old_threshold-(SENSITIVITY>>1)))) {
//     flag_threshold = 1;  // Possible step (it will be analize later)
//     flag_threshold_counter = 0;
//   } else {
//     flag_threshold_counter++;  // Number of times the SENSITIVITYs are out of the Threshold Level consecutively
//   }
//   /* SENSITIVITY detection */
//   /* Each time the difference is higher than the fixed SENSITIVITY the algorithm updates the Threshold */
//
//   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   //+++++++++++++++++++++++ THRESHOLD LEVEL UPDATE ++++++++++++++++++++++++++++
//   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   if (Difference > SENSITIVITY) {
//     /* Threshold level is calculated with the 4 previous good differences */
//     // The same method as the Filtering method
//     NewThreshold = (LastMax + LastMin) >> 1;  // Calculates the New Threshold
//     BufferDinamicThreshold = BufferDinamicThreshold - buffer_dynamic_threshold[IndexThreshold] + NewThreshold;
//     old_threshold =BufferDinamicThreshold / THRESHOLD_ORDER; //BufferDinamicThreshold<<2
//     buffer_dynamic_threshold[IndexThreshold] = NewThreshold;
//     IndexThreshold++;
//     if (IndexThreshold > THRESHOLD_ORDER - 1) {
//       IndexThreshold = 0;
//     }
//
//     // Once the Threshold Level is calculated and if the SENSITIVITYs was in the
//     // Threshold Level we certificate the step.
//
//     //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//     //++++++++++++++++++++++++ STEP CERTIFICATION   +++++++++++++++++++++++++
//     //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//     if (flag_threshold) {
//       // DETECTED STEP
//       flag_threshold = 0;
//       StepToStepSamples = 0; // Reset the time.
//       flag_max_min_samplecounter = 0;
//
//       if (Regulation_mode) {
//         //=====Regulation mode======
//         // the step is counted
//         count_steps++;
//       } else {
//         //=====Non Regulation mode======
//         possible_steps++;
//
//         // if the possible steps number is 8,
//         // the algorithm concludes that the person is walking.
//         if (possible_steps == 2) {
//           count_steps = count_steps + possible_steps; // Possible steps are added to counted steps
//           possible_steps = 0;
//           Regulation_mode = 1;  // The pedometer enters the Regulation mode
//
//         }
//       }
//     }
//   }
//   WindowMin = INIT_VALUE_MIN;
//   // If the SENSITIVITYs are out of the Threshold Level 2 times consecutively with a good difference,
//   // Is very probably that the acceleration profile isn't a step sequence
//   // (Reset the possible steps to discard false steps)
//   if (flag_threshold_counter > 1) {
//     flag_threshold_counter = 0;
//     flag_max_min_samplecounter = 0;
//     WindowMax = 0;  // maybe it is not necessary  reset them.
//     WindowMin = 0;
//     Regulation_mode = 0;
//     possible_steps = 0;
//     flag_threshold_counter = 0;
//   }
//   } else {
//     // While the pedometer is searching a min if between MAX and MIN
//     //  takes more than 25 samples I reset the step.
//     flag_max_min_samplecounter++;
//     if (flag_max_min_samplecounter == _1_SECOND) {
//       // Reset Step
//       flag_max_min_samplecounter = 0;
//       WindowMax = 0;
//       WindowMin = 0;
//       flag_max = 0;
//       possible_steps = 0;
//     }
//   }
// }
// // IndexBuffer is used as a circular buffer index
// IndexBuffer++;
// if (IndexBuffer > WINDOW_SIZE-1) {
//   IndexBuffer = 0;
// }
//
//  // IndexAverage is used as a circular buffer index to average
//  IndexAverage++;
//  if (IndexAverage > FILTER_ORDER-1) {
//    IndexAverage = 0;
//  }
//
//  StepToStepSamples++;
//  if (StepToStepSamples >= REGULATION_OFF_TIME)
//  {
//    // If the pedometer takes 2 seconds without counting a step it
//    //  returns to the Non Regulation mode.
//    StepToStepSamples = 0;
//    possible_steps = 0;
//    Regulation_mode = 0;
//    flag_max_min_samplecounter = 0;
//    if (Regulation_mode == 1) {
//      Regulation_mode = 0;
//      old_threshold = INIT_OFFSET_VALUE;
//      BufferDinamicThreshold = INIT_OFFSET_VALUE*THRESHOLD_ORDER;//(INIT_OFFSET_VALUE)<<2
//      IndexThreshold = 0;
//      flag_threshold_counter = 0;
//      for (char i = 0; i < THRESHOLD_ORDER; i++) {
//        buffer_dynamic_threshold[i] = INIT_OFFSET_VALUE;
//      }
//    }
//  }
//    #ifdef __cplusplus
//}
//  #endif
//#endif
//  return count_steps;
//}
//
