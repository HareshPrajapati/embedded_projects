/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.3
        Device            :  PIC16LF19156
        Driver Version    :  2.00
 */

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
 */

#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <pic16lf19156.h>



volatile uint32_t ticker = 0;
unsigned long previousMillis = 0;
#define TASK_MANAGER_MAX_TASK      3
typedef void (*TaskManagerCallBack)(void);
void callback_fun(void);

int getTriggerTaskID(void);

uint8_t totalTask = 0;
int calledTaskId = -1;

int set_random_range(float min, float max) {
    return ((int) (min + max * rand() / RAND_MAX));
}

void timer_callback() {
    ticker++;
}

uint32_t millis() {
    return ticker;
}

typedef struct {
    uint8_t task_id;
    uint8_t task_pri;
    uint32_t interval;
    uint32_t lastTick;
    TaskManagerCallBack _callback;
} TaskManager_t;

TaskManager_t TaskManager[TASK_MANAGER_MAX_TASK];

void TaskManagerInit(void) {
    totalTask = 0;
    memset(&TaskManager[0], NULL, sizeof (TaskManager));
}

int addTask(uint8_t TaskPri, uint32_t _interval, TaskManagerCallBack call) {
    if (totalTask < TASK_MANAGER_MAX_TASK) {
        TaskManager[totalTask].task_pri = TaskPri;
        TaskManager[totalTask].interval = _interval;
        TaskManager[totalTask]._callback = call;
        TaskManager[totalTask].task_id = totalTask;
        totalTask++;
        return TaskManager[totalTask].task_id;
    }
    return -1;
}

void TaskManagerHandle() {
    static uint32_t lastTaskTick = 0;
       unsigned long currentMillis = millis();
//     TaskManager[TASK_MANAGER_MAX_TASK].lastTick = 0;
    if (currentMillis - lastTaskTick >0) {                     //
        for (uint8_t i = 0; i < TASK_MANAGER_MAX_TASK; i++) {
             
            if ( millis() - TaskManager[i].lastTick > TaskManager[i].interval) {
                calledTaskId = i;
                
                if (TaskManager[i]._callback != NULL) {
                    TaskManager[i]._callback();
                    printf("called tasked id = %d and interval =  %u\r\n",TaskManager[i].task_id,TaskManager[i].interval);                
                    
                }
                calledTaskId = -1;
                 TaskManager[i].lastTick = millis();
               
            }
  
        }
        lastTaskTick = millis();
    }

}

int getTriggerTaskID(void) {
    
    return calledTaskId;
}

void callback_fun(void)
{
    switch(calledTaskId)
    {
        case 0:
            IO_RC3_Toggle();
            break;
        case 1:
            IO_RC2_Toggle();
            break;
        case 2:
            IO_RC1_Toggle();
            break;
            
    }
    
  

}

int comparator(const void* p, const void* q) {
    TaskManager_t *taskA = (TaskManager_t *) p;
    TaskManager_t *taskB = (TaskManager_t *) q;
    return (taskA->task_pri - taskB->task_pri);
}

/*
                         Main application
 */
void main(void) {
    // initialize the device
    SYSTEM_Initialize();
    TMR0_SetInterruptHandler(timer_callback);
    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    uint8_t i = 0;
    srand(rand());
    TaskManagerInit();

    for (i = 0; i < TASK_MANAGER_MAX_TASK; i++) {
         TaskManager[i].task_id = i;
        uint8_t k = set_random_range(1.0, TASK_MANAGER_MAX_TASK); 
        uint32_t j = set_random_range(1.0, 1000.0);
        addTask(k,j, callback_fun);
         printf(" task_priority = %d and id = %d and interval = %u \r \n", TaskManager[i].task_pri, TaskManager[i].task_id, TaskManager[i].interval);
    }

    printf(" after shorting task priority  in Ascending order........ \r\n");


    qsort(TaskManager, TASK_MANAGER_MAX_TASK, sizeof (TaskManager_t), comparator);

    for (i = 0; i < TASK_MANAGER_MAX_TASK; i++) {
        printf(" task_priority = %d and id = %d and interval = %u \r \n", TaskManager[i].task_pri, TaskManager[i].task_id, TaskManager[i].interval);
    }
//    TaskManager[1].task_id = 1;
//    TaskManager[2].task_id = 2;
//    TaskManager[3].task_id = 3;
//
//
//    TaskManager[1].task_pri = 1;
//    TaskManager[2].task_pri = 2;
//    TaskManager[3].task_pri = 3;
//
//
//
//    TaskManager[1].interval = 500;
//    TaskManager[2].interval = 1000;
//    TaskManager[3].interval = 1500;
//
//
//    TaskManager[1]._callback = callback_fun;
//    TaskManager[2]._callback = callback_fun;
//    TaskManager[3]._callback = callback_fun;
         
         
         while (1) {
        
              TaskManagerHandle();
       
     
    }

      


}
/**
 End of File
 */