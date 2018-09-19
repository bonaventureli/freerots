/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c                                                */
/*  DATE        :Mon, Oct 24, 2016                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*  version 0.3                                                                   */
/***********************************************************************/

#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"  
#include "semphr.h"
#include "event_groups.h"

#include "semtest.h"
#include "QPeek.h"
#include "GenQTest.h"
#include "partest.h"
#include "comtest.h"
#include "serial.h"

#define mainFLASH_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCREATOR_TASK_PRIORITY           ( tskIDLE_PRIORITY + 2 )
#define mainINTEGER_TASK_PRIORITY           ( tskIDLE_PRIORITY )
#define mainGEN_QUEUE_TASK_PRIORITY			( tskIDLE_PRIORITY )
#define mainCOMTEST_PRIORITY				( tskIDLE_PRIORITY + 1 )

/* Passed into the check task just as a test that the parameter passing
mechanism is working correctly. */
#define mainCHECK_PARAMETER					( ( void * ) 0x12345678 )

/* The period between executions of the check task. */
#define mainNO_ERROR_DELAY		( ( TickType_t ) 3000 / portTICK_PERIOD_MS  )
#define mainERROR_DELAY			( ( TickType_t ) 500 / portTICK_PERIOD_MS )

/* There are no spare LEDs for the comtest tasks, so this is just set to an
invalid number. */
#define mainCOMTEST_LED			( 4 )

/* The baud rate used by the comtest task. */
#define mainBAUD_RATE			( 9600 )

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
#define LED0_MASK		( ( unsigned short ) 0x04 )
#define LED1_MASK		( ( unsigned short ) 0x08 )

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName );
static void prvCheckTask( void *pvParameters );
static void prvSetupHardware( void );
static volatile long lRegTestStatus = pdPASS;
void vTaskCode( void * pvParameters );
extern void vRegTest1( void *pvParameters );
extern void vRegTest2( void *pvParameters );
/*-----------------------------------------------------------*/
void vRegTest1( void *pvParameters )
{
	P8 |= (1<<5);//high level
}
void vRegTest2( void *pvParameters )
{
	P8 |= (1<<5);//high level
}
static void prvCheckTask( void *pvParameters )
{
TickType_t xDelayPeriod = mainNO_ERROR_DELAY, xLastWakeTime;
unsigned portBASE_TYPE uxLEDToUse = 0;

	/* Ensure parameter is passed in correctly. */
	if( pvParameters != mainCHECK_PARAMETER )
	{
		xDelayPeriod = mainERROR_DELAY;
	}
	
	/* Initialise xLastWakeTime before it is used.  After this point it is not
	written to directly. */
	xLastWakeTime = xTaskGetTickCount();
	
	/* Cycle for ever, delaying then checking all the other tasks are still
	operating without error. */
	for( ;; )
	{
		/* Wait until it is time to check all the other tasks again. */
		vTaskDelayUntil( &xLastWakeTime, xDelayPeriod );
		
		if( lRegTestStatus != pdPASS )
		{
			xDelayPeriod = mainERROR_DELAY;
		}
		
		if( xAreGenericQueueTasksStillRunning() != pdTRUE )
		{
			xDelayPeriod = mainERROR_DELAY;
		}

		if( xAreQueuePeekTasksStillRunning() != pdTRUE )
		{
			xDelayPeriod = mainERROR_DELAY;
		}

		if( xAreSemaphoreTasksStillRunning() != pdTRUE )
	    {
	    	xDelayPeriod = mainERROR_DELAY;
	    }

		if( xIsCreateTaskStillRunning() != pdTRUE )
	    {
	    	xDelayPeriod = mainERROR_DELAY;
	    }

		/* The Fx3 runs more tasks, so more checks are performed. */		
		#ifdef __IAR_V850ES_Fx3__
		{
			if( xAreComTestTasksStillRunning() != pdTRUE )
			{
				xDelayPeriod = mainERROR_DELAY;
			}
			
			if( xArePollingQueuesStillRunning() != pdTRUE )
			{
				xDelayPeriod = mainERROR_DELAY;
			}

			if( xAreBlockingQueuesStillRunning() != pdTRUE )
			{
				xDelayPeriod = mainERROR_DELAY;
			}
			
			if( xAreRecursiveMutexTasksStillRunning() != pdTRUE )
			{
				xDelayPeriod = mainERROR_DELAY;
			}		
			
			/* The application board has more LEDs and uses the flash tasks
			so the check task instead uses LED3 as LED3 is still spare. */
			uxLEDToUse = 3;
		}
		#endif

		/* Toggle the LED.  The toggle rate will depend on whether or not an
		error has been found in any tasks. */
		vParTestToggleLED( uxLEDToUse );
	}
}
/*-----------------------------------------------------------*/

 void prvSetupHardware( void )
{
	/* Setup the LED outputs. */
	vParTestInitialise();

	/* Any additional hardware configuration can be added here. */
}
/*-----------------------------------------------------------*/
void main(void)
{
	BaseType_t xReturned;
	TaskHandle_t xHandle = NULL;
	

	R_CLOCK_Init();                       /* Clock initialize    */
	prvSetupHardware();
	
	vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
  vStartGenericQueueTasks( mainGEN_QUEUE_TASK_PRIORITY );
  vStartQueuePeekTasks();
 
  xTaskCreate( prvCheckTask, "Check", configMINIMAL_STACK_SIZE, mainCHECK_PARAMETER, mainCHECK_TASK_PRIORITY, NULL );
	xTaskCreate( vRegTest1, "Reg1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vRegTest2, "Reg2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
	
	vAltStartComTestTasks( mainCOMTEST_PRIORITY, mainBAUD_RATE, mainCOMTEST_LED );
	vStartLEDFlashTasks( mainFLASH_PRIORITY );
	vCreateSuicidalTasks( mainCREATOR_TASK_PRIORITY );
	
	#if 1
  xReturned = xTaskCreate(
                    vTaskCode,       /* Function that implements the task. */
                    "NAME",          /* Text name for the task. */
                    STACK_SIZE,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    tskIDLE_PRIORITY,/* Priority at which the task is created. */
                    &xHandle );      /* Used to pass out the created task's handle. */
if( xReturned == pdPASS )
    {
        /* The task was created.  Use the task's handle to delete the task. */
        vTaskDelete( xHandle );
    }
	xTaskCreateStatic( vLED_1_Task, ( signed portCHAR * ) "LED1", STACK_SIZE, NULL, tskIDLE_PRIORITY+1, Stack1,&TaskBuffer1 );
	xTaskCreateStatic( vLED_2_Task, ( signed portCHAR * ) "LED2", STACK_SIZE, NULL, tskIDLE_PRIORITY+3, Stack2,&TaskBuffer2 );
	xTaskCreateStatic( vLED_3_Task, ( signed portCHAR * ) "LED3", STACK_SIZE, NULL, tskIDLE_PRIORITY+2, Stack3,&TaskBuffer3 );      
	#endif
	
	vTaskStartScheduler();
	for( ;; );
}


/* Task to be created. */
void vTaskCode( void * pvParameters )
{
    /* The parameter value is expected to be 1 as 1 is passed in the
    pvParameters value in the call to xTaskCreate() below. */
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );

    for( ;; )
    {
        /* Task code goes here. */
				P8 |= (1<<5);//high leve
    }
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    /* This will be called if a task overflows its stack.  pxCurrentTCB
    can be inspected to see which is the offending task. */
    //xTaskCreateStatic( vLED_1_Task, ( signed portCHAR * ) "LED1", STACK_SIZE, NULL, tskIDLE_PRIORITY+1, Stack1,&TaskBuffer1 );
    //xTaskCreateStatic( vLED_2_Task, ( signed portCHAR * ) "LED2", STACK_SIZE, NULL, tskIDLE_PRIORITY+3, Stack2,&TaskBuffer2 );
    //xTaskCreateStatic( vLED_3_Task, ( signed portCHAR * ) "LED3", STACK_SIZE, NULL, tskIDLE_PRIORITY+2, Stack3,&TaskBuffer3 );      
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
//    vTaskDelete( NULL );

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
   // vTaskDelete( NULL );

}

static void  vLED_3_Task(void *pvParameters) 
{  

    for( ;; )
    {     
        vTaskDelay(2000/portTICK_RATE_MS); 
        testcount3++;
        //break;
    } 
  //  vTaskDelete( NULL );

} 
