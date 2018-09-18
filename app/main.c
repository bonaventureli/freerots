/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c                                                */
/*  DATE        :Mon, Oct 24, 2016                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*  version 0.1                                                                   */
/***********************************************************************/

#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"  
#include "semphr.h"
#include "event_groups.h"

void main(void);
#define STACK_SIZE 100

StaticTask_t IdleTaskBuffer;
StackType_t IdleStack[ STACK_SIZE ];

StaticTask_t TimerTaskBuffer;
StackType_t TimerStack[ STACK_SIZE ];

static void vLED_1_Task( void *pvParameters );      
static void vLED_2_Task( void *pvParameters ); 
static void vLED_3_Task( void *pvParameters ); 
unsigned int testcount1 = 0;
unsigned int testcount2 = 0;
unsigned int testcount3 = 0;
StaticTask_t TaskBuffer1;
StackType_t Stack1[ STACK_SIZE ];
StaticTask_t TaskBuffer2;
StackType_t Stack2[ STACK_SIZE ];
StaticTask_t TaskBuffer3;
StackType_t Stack3[ STACK_SIZE ];

void main(void)
{
	R_CLOCK_Init();                       /* Clock initialize    */

}
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    /* This will be called if a task overflows its stack.  pxCurrentTCB
    can be inspected to see which is the offending task. */
    xTaskCreateStatic( vLED_1_Task, ( signed portCHAR * ) "LED1", STACK_SIZE, NULL, tskIDLE_PRIORITY+1, Stack1,&TaskBuffer1 );
    xTaskCreateStatic( vLED_2_Task, ( signed portCHAR * ) "LED2", STACK_SIZE, NULL, tskIDLE_PRIORITY+3, Stack2,&TaskBuffer2 );
    xTaskCreateStatic( vLED_3_Task, ( signed portCHAR * ) "LED3", STACK_SIZE, NULL, tskIDLE_PRIORITY+2, Stack3,&TaskBuffer3 );      
    vTaskStartScheduler(); 
    for( ;; );
}

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
    *pulIdleTaskStackSize = STACK_SIZE;
    *ppxIdleTaskTCBBuffer = &IdleTaskBuffer;
    *ppxIdleTaskStackBuffer = IdleStack; 
}


void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
    *pulTimerTaskStackSize = STACK_SIZE;
    *ppxTimerTaskTCBBuffer = &TimerTaskBuffer;
    *ppxTimerTaskStackBuffer = TimerStack; 
}


static void  vLED_1_Task(void *pvParameters) 
{  
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;
    xLastWakeTime = xTaskGetTickCount ();
    for( ;; )
    {     
        //vTaskDelay(1000/portTICK_RATE_MS); 
        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        testcount1++;
        

    } 
    vTaskDelete( NULL );

}

  
static void  vLED_2_Task(void *pvParameters) 
{  
    
    for( ;; )
    {     
        vTaskDelay(500/portTICK_RATE_MS); 
        /* Wait until it is time to check all the other tasks again. */
        
        testcount2++;
        //break;
    } 
    vTaskDelete( NULL );

}

static void  vLED_3_Task(void *pvParameters) 
{  

    for( ;; )
    {     
        vTaskDelay(2000/portTICK_RATE_MS); 
        testcount3++;
        //break;
    } 
    vTaskDelete( NULL );

} 
