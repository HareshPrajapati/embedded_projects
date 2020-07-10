/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#ifndef __RTCOUNTERDRIVER_H
#define __RTCOUNTERDRIVER_H

#include <stdint.h>

typedef  int32_t (*rtcountcallback_ptr_t)(void *payload);

typedef struct rtcountStruct {
    rtcountcallback_ptr_t  callbackPtr;
    void* payload;
    struct rtcountStruct* volatile next;
    uint32_t absoluteTime;
} rtcountStruct_t;

// Functions called only once
void rtcount_initialize(void);


// Functions used regularly
void     rtcount_create(rtcountStruct_t *timer, int32_t timeout);
void     rtcount_delete(rtcountStruct_t * volatile timer);
void     rtcount_callNextCallback(void);
uint32_t rtcount_getTickCount(void);
void     rtcount_flushAll(void);

// Helper functions to implement a stopwatch like timer. 
//    Start will use the struct provided to create a timer with maximal duration
//    Stop will cancel this timer and return how long it has been running
void     rtcount_startTimer(rtcountStruct_t *timer);
uint32_t rtcount_stopTimer(rtcountStruct_t *timer);

#endif  //__RTCOUNTERDRIVER_H
