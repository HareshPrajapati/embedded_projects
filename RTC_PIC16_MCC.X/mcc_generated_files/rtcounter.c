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

#include <xc.h>
#include <stdio.h>
#include <stdbool.h>
#include "rtcounter.h"
#include "tmr0.h"


// Counter value is in 2 parts, high and low, this is the high, low is in the timer register
#ifdef RTCOUNTER_CONCATENATE_TIMER_TICKS
static uint16_t g_rtcounterH = 0;   // If you are interested in testing how this driver performs when 
                                    // the counter overflows we recommend you change this to 65500, which will make the 
                                    // counter start in range of the overflow, saving loads of time. Overflows are implemented
                                    // as a rebase of absolute times so you may want to take time to understand this first.
#else
static uint32_t g_rtcounterH = 0;
#endif
static rtcountStruct_t * volatile rtcoutListHead = NULL;
static volatile bool  isRunning = false;



// NOTE: assumes the callback completes before the next timer tick
// We only increment the overflow counter on every interrupt
void rtcount_isr(void) {
    g_rtcounterH++;
    PIR0bits.TMR0IF = 0;
}

// Must be called while GIE = 0
void rtcount_initialize(void) {
    TMR0_SetInterruptHandler(rtcount_isr);
}

// Return the 32-bit total tick count of the timer. This means concatenating the 
//    hardware timer value to the overflow counter
uint32_t rtcount_getTickCount(void)
{
    uint32_t tmp;
    PIE0bits.TMR0IE = 0;
    tmp = g_rtcounterH;
    tmp <<= 16;
    tmp |= TMR0_ReadTimer();
    PIE0bits.TMR0IE = 1;
    
    return tmp;
}

// Adjust the provided time forward by the current tick count
static inline uint32_t rtcount_makeAbsolute(uint32_t period) {
    uint32_t ac = period + rtcount_getTickCount();
    return ac;
}

// Has to be serialized as it modifies the linked list
// Returns true if the insert was at the head, false otherwise
static bool rtcount_sortedInsert(rtcountStruct_t *timer) {
    uint32_t timerAbs = timer->absoluteTime;

    uint8_t atHead = 1;
    rtcountStruct_t *insertPoint = rtcoutListHead;
    rtcountStruct_t *prevPoint = NULL;
    timer->next = NULL;

    // We ignore wrapping here
    while (insertPoint != NULL) {
        if (insertPoint->absoluteTime > timerAbs) {
            break; // found the spot
        }
        prevPoint = insertPoint;
        insertPoint = insertPoint->next;
        atHead = 0;
    }

    if (atHead == 1) // At the front of the list. 
    {
        // Make sure there are no timeouts nearby and clear interrupt flags
        timer->next = rtcoutListHead;
        rtcoutListHead = timer;
        return true;
    } else // middle of the list
    {
        timer->next = prevPoint->next;
    }
    prevPoint->next = timer;
    return false;
}

// Routine to rebase the timer to be in the lower half of the range
static inline void rebase(void)
{
    rtcountStruct_t *thisone = rtcoutListHead;
    PIE0bits.TMR0IE = 0;

#ifdef RTCOUNTER_CONCATENATE_TIMER_TICKS 
    // If we had an overflow we need a rebase
    if((int16_t) g_rtcounterH < 0 )
    {
        g_rtcounterH &= 0x7FFF;
#else
         // If we had an overflow we need a rebase
    if((int32_t) g_rtcounterH < 0 )
    {
        
        g_rtcounterH &= 0x7FFFFFFF;
#endif
        while(thisone)
        {
            thisone->absoluteTime &= 0x7FFFFFFF; // clear the MSB
            thisone = thisone->next;
        }
    }
    PIE0bits.TMR0IE = 1;
}

// Add a new timer to the list of serviced timers. The user supplies the memory for 
//    the timer and initializes it, this method simply updates absoluteTime and adds the timer to
//    the list of serviced timers. timeout is an integer number of timer ticks from right now
void rtcount_create(rtcountStruct_t *timer, int32_t timeout) {
    
    if(timeout < 0) timeout = 0;
    
    // rebase if needed
    rebase();

    timer->absoluteTime = rtcount_makeAbsolute((uint32_t)timeout);

    // We only have to start the timer at head if the insert was at the head
    rtcount_sortedInsert(timer);
}

// Rescheduling means adding time to previous ticks. We add it to the previous target and not 
//    to current time to ensure the cadence of the timer will be exact and no skidding is possible.
void rtcount_reschedule(rtcountStruct_t *timer, int32_t timeout) {
    
    // rebase if needed
    rebase();
    
    // Limit the reschedulre range to positive numbers, negative numbers are treated as now    
    if(timeout < 0) timeout = 0;

    timer->absoluteTime += (uint32_t)timeout;
    
    // We only have to start the timer at head if the insert was at the head
    rtcount_sortedInsert(timer);
}

// Remove a previously scheduled timer from the list of active timers. This does not 
//    modify any of the timer meta data, it simply removes it from the list of serviced timers
void rtcount_delete(rtcountStruct_t * volatile timer) {
    if (rtcoutListHead == NULL)
        return;

    // Special case, the head is the one we are deleting
    if (timer == rtcoutListHead) {
        rtcoutListHead = rtcoutListHead->next; // Delete the head
    } else { // More than one timer in the list, search the list.  
        rtcountStruct_t *findTimer = rtcoutListHead;
        rtcountStruct_t *prevTimer = NULL;
        while (findTimer != NULL) {
            if (findTimer == timer) {
                prevTimer->next = findTimer->next; // Delete not at the head
                break;
            }
            prevTimer = findTimer;
            findTimer = findTimer->next;
        }
    }
}

// This method will check if the timer at the head of the sorted list has expired and if it
//    has it will call the callback associated with this timer. 
void rtcount_callNextCallback(void) {
    int32_t   reschedule = 0;
    uint32_t  timerTmp;
    uint32_t  timeAtTheTip;

    // No timers in the list, return
    if (rtcoutListHead == NULL)
        return;
 
    // Serialize in case timer overflows right here
    timerTmp = rtcount_getTickCount();
    
    timeAtTheTip = rtcoutListHead->absoluteTime;
    
    if ((int32_t)(timerTmp - timeAtTheTip) > 0)
    {  
        // Remember the call details
        rtcountStruct_t* timer = rtcoutListHead;
        
        // Advance the list
        rtcoutListHead = rtcoutListHead->next;

        // Call the callback (no check is done for a NULL callback ptr, users need to check this 
        //     upon inserting the timer. We will be adding an assert here for checking the null  
        reschedule = timer->callbackPtr(timer->payload);

        if (reschedule) {
            rtcount_reschedule(timer, reschedule);
        }
    }
}

// Stop all timers
static void stopTimeouts(void)
{
    isRunning = false;
}

// These methods are for calculating the elapsed time in stopwatch mode.
// Start the Stopwatch timer to be maximal length
void rtcount_startTimer(rtcountStruct_t *timer)
{
    rtcount_create(timer, ((uint32_t)1<<31)-1);
}

// Cancel the stopwatch timer, return how long it ran. You can of course also
//    delete the timer yourself and read the result at your leisure
uint32_t rtcount_stopTimer(rtcountStruct_t *timer)
{
    rtcount_delete(timer);
    
    // This calculates the max range - remaining time which = elapsed time
    return (((uint32_t)1<<31)-1) - (timer->absoluteTime - rtcount_makeAbsolute(0));
}

// Stop all timers 
void rtcount_flushAll(void)
{
    stopTimeouts();
    rtcoutListHead = NULL;
}


// Helper function for debugging, prints the linked list
int rtcount_printList(void)
{
    rtcountStruct_t *basePoint = rtcoutListHead;
    while(basePoint != NULL)
    {
        printf("%4p:%lu -> ",basePoint,basePoint->absoluteTime);
        basePoint = basePoint->next;
    }
    printf("NULL\n");
    return 0;
}

