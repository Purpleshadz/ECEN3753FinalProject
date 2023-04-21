/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/

#include <gpio.h>
#include <capsense.h>
#include <em_emu.h>
#include <string.h>
#include <os_trace.h>

#include "sl_board_control.h"
#include "em_assert.h"
#include "glib.h"
#include "dmd.h"
#include "os.h"
#include "stdio.h"
#include "FIFO.h"

#define SLIDER_PERIOD 10
#define LCD_PERIOD 50
#define PHYSICS_PERIOD 25
#define LED1_PERIOD 5

bool below50 = true;
bool below100 = false;

OS_TMR sliderTimer;
OS_SEM sliderSem;
void  sliderTimerCallback (void  *p_tmr, void  *p_arg);
/***************************************************************************//**
 * @brief
 *   timer callback function. Posts a flag to the LED task to turn on the LED
 *    when the timer expires. Used to wait ~3 during held slider before turning
 *   on the LED1;
 ******************************************************************************/
void  sliderTimerCallback (void  *p_tmr, void  *p_arg)
{
    (void)&p_arg;
    (void)&p_tmr;
    RTOS_ERR err;
    OSSemPost(&sliderSem, OS_OPT_POST_ALL, &err);
    while (err.Code != RTOS_ERR_NONE) {}
}

OS_TMR physicsTimer;
OS_SEM physicsSem;
void  physicsTimerCallback (void  *p_tmr, void  *p_arg);
/***************************************************************************//**
 * @brief
 *   timer callback function. Posts a flag to the LED task to turn on the LED
 *    when the timer expires. Used to wait ~3 during held slider before turning
 *   on the LED1;
 ******************************************************************************/
void  physicsTimerCallback (void  *p_tmr, void  *p_arg)
{
    (void)&p_arg;
    (void)&p_tmr;
    RTOS_ERR err;
    OSSemPost(&physicsSem, OS_OPT_POST_ALL, &err);
    while (err.Code != RTOS_ERR_NONE) {}
}
#define  PHYSICS_TASK_PRIO            21u  /*   Task Priority.                 */
#define  PHYSICS_TASK_STK_SIZE       256u  /*   Stack size in CPU_STK.         */
OS_TCB   physicsTaskTCB;                            /*   Task Control Block.   */
CPU_STK  physicsTaskStk[PHYSICS_TASK_STK_SIZE]; /*   Stack.  */
void  physicsTask (void  *p_arg);
/***************************************************************************//**
 * @brief
 *   Creates the idle task.
 ******************************************************************************/
void  physicsTaskCreate (void)
{
    RTOS_ERR     err;

    OSTaskCreate(&physicsTaskTCB,                /* Pointer to the task's TCB.  */
                 "Physics Task.",                    /* Name to help debugging.     */
                 &physicsTask,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  PHYSICS_TASK_PRIO,             /* Task's priority.            */
                 &physicsTaskStk[0],             /* Pointer to base of stack.   */
                 (PHYSICS_TASK_STK_SIZE / 10u),  /* Stack limit, from base.     */
                  PHYSICS_TASK_STK_SIZE,         /* Stack size, in CPU_STK.     */
                  10u,                               /* Messages in task queue.     */
                  0u,                                /* Round-Robin time quanta.    */
                  DEF_NULL,                          /* External TCB data.          */
                  OS_OPT_TASK_STK_CHK,               /* Task options.               */
                 &err);
    if (err.Code != RTOS_ERR_NONE) {
        /* Handle error on task creation. */
    }
}
/***************************************************************************//**
 * @brief
 *   The idle task. Enters energy mode 1
 ******************************************************************************/
enum objectType {empty, player, satchel, shot};
struct physicsData {
    int objectType;
    int x;
    int y;
    int xVel;
    int yVel;
    int xAcc;
    int yAcc;
    int xForce;
    int yForce;
    int xMass;
    int yMass;
}
int playerStartX = 0;
int playerStartY = 0;
int rightForce = 10;
int leftForce = -10;
int satchelWeight = 10;
int shotWeight = 1;
int gravity = -10;
int playerWeight = 10;
int maxCapacity = 10;
struct physicsData[10];
void  physicsTask (void  *p_arg)
{
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR     err;
    OSTmrStart(&physicsTimer, &err);
    while (err.Code != RTOS_ERR_NONE) {}
    for (int i = 0; i < 10; i++) {
        physicsData[i].objectType = empty;
        physicsData[i].x = 0;
        physicsData[i].y = 0;
        physicsData[i].xVel = 0;
        physicsData[i].yVel = 0;
        physicsData[i].xAcc = 0;
        physicsData[i].yAcc = 0;
        physicsData[i].xForce = 0;
        physicsData[i].yForce = 0;
        physicsData[i].xMass = 0;
        physicsData[i].yMass = 0;
    }
    physicsData[0].objectType = player;
    physicsData[0].x = playerStartX;
    physicsData[0].y = playerStartY;
    int capacity = 10;
    bool charging = false;
   while (DEF_TRUE) {
       OSSemPend(&physicsSem, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
       while (err.Code != RTOS_ERR_NONE) {}
         for (int i = 0; i < 10; i++) {
                OSMutexPend(&physicsMutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
                while (err.Code != RTOS_ERR_NONE) {}
                if ((sliderState.farLeft || sliderState.left) && (sliderState.farRight || sliderState.right)) {
                    physicsData[i].xForce = 0;
                } else if (sliderState.farLeft == 1) {
                    physicsData[i].xForce = leftForce;
                } else if (sliderState.left == 1) {
                    physicsData[i].xForce = leftForce / 2;
                } else if (sliderState.right == 1) {
                    physicsData[i].xForce = rightForce / 2;
                } else if (sliderState.farRight == 1) {
                    physicsData[i].xForce = rightForce;
                } else {
                    physicsData[i].xForce = 0;
                }
                OSMutexPost(&physicsMutex, OS_OPT_POST_NONE, &err);
                while (err.Code != RTOS_ERR_NONE) {}
                // Button 0 is held down, charge capacitor
                if (buttonStates.button0 == 1) {
                    charging = true;
                } else if (buttonStates.button0 == 0 && charging == true) {
                    charging = false;
                    if (capacity > 0) {
                        for (int j = 0; j < 10; j++) {
                            if (physicsData[j].objectType == empty) {
                                physicsData[j].objectType = shot;
                                physicsData[j].x = physicsData[i].x;
                                physicsData[j].y = physicsData[i].y;
                                physicsData[j].xVel = physicsData[i].xVel;
                                physicsData[j].yVel = physicsData[i].yVel;
                                physicsData[j].xAcc = physicsData[i].xAcc;
                                physicsData[j].yAcc = physicsData[i].yAcc;
                                physicsData[j].xForce = physicsData[i].xForce;
                                physicsData[j].yForce = physicsData[i].yForce;
                                physicsData[j].xMass = shotWeight;
                                physicsData[j].yMass = shotWeight;
                                capacity = 0;;
                                break;
                            }
                        }
                    }
                } 
                if (charging == true && capacity < maxCapacity) {
                    capacity++;
                } else if (charging == true) {
                    capacity = maxCapacity;
                } 
                

                physicsData[i].xAcc = physicsData[i].xForce / physicsData[i].xMass;
                physicsData[i].yAcc = physicsData[i].yForce / physicsData[i].yMass;
                physicsData[i].xVel += physicsData[i].xAcc;
                physicsData[i].yVel += physicsData[i].yAcc;
                physicsData[i].x += physicsData[i].xVel;
                physicsData[i].y += physicsData[i].yVel;
         }

   }
   if (err.Code) {}
}
OS_TMR LCDTimer;
OS_SEM LCDSem;
void  LCDTimerCallback (void  *p_tmr, void  *p_arg);
/***************************************************************************//**
 * @brief
 *   timer callback function. Posts a flag to the LED task to turn on the LED
 *    when the timer expires. Used to wait ~3 during held slider before turning
 *   on the LED1;
 ******************************************************************************/
void  LCDTimerCallback (void  *p_tmr, void  *p_arg)
{
    (void)&p_arg;
    (void)&p_tmr;
    RTOS_ERR err;
    OSSemPost(&LCDSem, OS_OPT_POST_ALL, &err);
}

void LED0TimerCallback (void  *p_tmr, void  *p_arg);
/***************************************************************************//**
 * @brief
 *   timer callback function. Posts a flag to the LED task to turn on the LED
 *    when the timer expires. Used to wait ~3 during held slider before turning
 *   on the LED1;
 ******************************************************************************/
bool state = false;
OS_TMR LED0Timer;
void LED0TimerCallback (void  *p_tmr, void  *p_arg)
{
    (void)&p_arg;
    (void)&p_tmr;
    RTOS_ERR err;
    if (below100) {
        GPIO_PinOutSet(LED1_port, LED1_pin);
    } else if (below50) {
        if (state) {
            state = false;
            GPIO_PinOutSet(LED1_port, LED1_pin);
        } else {
            state = true;
            GPIO_PinOutClear(LED1_port, LED1_pin);
        }
    } else {
        GPIO_PinOutClear(LED1_port, LED1_pin);
    }

}

static GLIB_Context_t glibContext;

static void LCD_init()
{
  uint32_t status;
  /* Enable the memory lcd */
  status = sl_board_enable_display();
  EFM_ASSERT(status == SL_STATUS_OK);

  /* Initialize the DMD support for memory lcd display */
  status = DMD_init(0);
  EFM_ASSERT(status == DMD_OK);

  /* Initialize the glib context */
  status = GLIB_contextInit(&glibContext);
  EFM_ASSERT(status == GLIB_OK);

  glibContext.backgroundColor = White;
  glibContext.foregroundColor = Black;

  /* Fill lcd with background color */
  GLIB_clear(&glibContext);

  /* Use Normal font */
  GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNormal8x8);

  /* Draw text on the memory lcd display*/
  GLIB_drawStringOnLine(&glibContext,
                        "Welcome to...\n**Lab 7**!",
                        0,
                        GLIB_ALIGN_LEFT,
                        5,
                        5,
                        true);

  /* Draw text on the memory lcd display*/
  GLIB_drawStringOnLine(&glibContext,
                        "Review the lab\ninstructions!",
                        2,
                        GLIB_ALIGN_LEFT,
                        5,
                        5,
                        true);
  /* Post updates to display */
  DMD_updateDisplay();
}

#define  LED0_OUTPUT_PRIO            21u  /*   Task Priority.                 */
#define  LED0_OUTPUT_STK_SIZE       256u  /*   Stack size in CPU_STK.         */
OS_TCB   led0OutputTaskTCB;                            /*   Task Control Block.   */
CPU_STK  led0OutputTaskStk[LED0_OUTPUT_STK_SIZE]; /*   Stack.  */

void  led0OutputTask (void  *p_arg);
/***************************************************************************//**
 * @brief
 *   Creates the ledOutputTask
 ******************************************************************************/
void  LED0OutputTaskCreate (void)
{
    RTOS_ERR     err;

    OSTaskCreate(&led0OutputTaskTCB,                /* Pointer to the task's TCB.  */
                 "LED0 Output Task.",                    /* Name to help debugging.     */
                 &led0OutputTask,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  LED0_OUTPUT_PRIO,             /* Task's priority.            */
                 &led0OutputTaskStk[0],             /* Pointer to base of stack.   */
                 (LED0_OUTPUT_STK_SIZE / 10u),  /* Stack limit, from base.     */
                  LED0_OUTPUT_STK_SIZE,         /* Stack size, in CPU_STK.     */
                  10u,                               /* Messages in task queue.     */
                  0u,                                /* Round-Robin time quanta.    */
                  DEF_NULL,                          /* External TCB data.          */
                  OS_OPT_TASK_STK_CHK,               /* Task options.               */
                 &err);
    if (err.Code != RTOS_ERR_NONE) {
        /* Handle error on task creation. */
    }
}
/***************************************************************************//**
 * @brief
 *   Task that handles the LED output. It waits for a flag to be set and then
 *  turns on or off the LEDs as needed.
 ******************************************************************************/
void  led0OutputTask (void  *p_arg)
{
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR     err;
   OSTmrStart(&LED0Timer, &err);
    while (err.Code != RTOS_ERR_NONE) {}

    while (DEF_TRUE) {
    }
}

#define  LED1_OUTPUT_PRIO            21u  /*   Task Priority.                 */
#define  LED1_OUTPUT_STK_SIZE       256u  /*   Stack size in CPU_STK.         */
OS_TCB   led1OutputTaskTCB;                            /*   Task Control Block.   */
CPU_STK  led1OutputTaskStk[LED1_OUTPUT_STK_SIZE]; /*   Stack.  */

void  led1OutputTask (void  *p_arg);
/***************************************************************************//**
 * @brief
 *   Creates the ledOutputTask
 ******************************************************************************/
void  LED1OutputTaskCreate (void)
{
    RTOS_ERR     err;

    OSTaskCreate(&led1OutputTaskTCB,                /* Pointer to the task's TCB.  */
                 "LED1 Output Task.",                    /* Name to help debugging.     */
                 &led1OutputTask,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  LED1_OUTPUT_PRIO,             /* Task's priority.            */
                 &led1OutputTaskStk[0],             /* Pointer to base of stack.   */
                 (LED1_OUTPUT_STK_SIZE / 10u),  /* Stack limit, from base.     */
                  LED1_OUTPUT_STK_SIZE,         /* Stack size, in CPU_STK.     */
                  10u,                               /* Messages in task queue.     */
                  0u,                                /* Round-Robin time quanta.    */
                  DEF_NULL,                          /* External TCB data.          */
                  OS_OPT_TASK_STK_CHK,               /* Task options.               */
                 &err);
    if (err.Code != RTOS_ERR_NONE) {
        /* Handle error on task creation. */
    }
}
/***************************************************************************//**
 * @brief
 *   Task that handles the LED output. It waits for a flag to be set and then
 *  turns on or off the LEDs as needed.
 ******************************************************************************/
void  led1OutputTask (void  *p_arg)
{
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR     err;

    while (DEF_TRUE) {
    }
}

#define  LCD_DISPLAY_PRIO            21u  /*   Task Priority.                 */
#define  LCD_DISPLAY_STK_SIZE       256u  /*   Stack size in CPU_STK.         */
OS_TCB   LCDDisplayTaskTCB;                            /*   Task Control Block.   */
CPU_STK  LCDDisplayTaskStk[LCD_DISPLAY_STK_SIZE]; /*   Stack.  */
void  LCDDisplayTask (void  *p_arg);
/***************************************************************************//**
 * @brief
 *   Creates the LCDDisplayTask
 ******************************************************************************/
void  LCDTaskCreate (void)
{
    RTOS_ERR     err;

    OSTaskCreate(&LCDDisplayTaskTCB,                /* Pointer to the task's TCB.  */
                 "LCD Display Task.",                    /* Name to help debugging.     */
                 &LCDDisplayTask,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  LCD_DISPLAY_PRIO,             /* Task's priority.            */
                 &LCDDisplayTaskStk[0],             /* Pointer to base of stack.   */
                 (LCD_DISPLAY_STK_SIZE / 10u),  /* Stack limit, from base.     */
                  LCD_DISPLAY_STK_SIZE,         /* Stack size, in CPU_STK.     */
                  10u,                               /* Messages in task queue.     */
                  0u,                                /* Round-Robin time quanta.    */
                  DEF_NULL,                          /* External TCB data.          */
                  OS_OPT_TASK_STK_CHK,               /* Task options.               */
                 &err);
    if (err.Code != RTOS_ERR_NONE) {
        /* Handle error on task creation. */
    }
}
/***************************************************************************//**
 * @brief
 *   Task that handles the LCD output. It waits 100ms and then updates the LCD
 *    with the current speed and direction as recorded by the speed setpoint and vehicle 
 *  direction tasks.
 ******************************************************************************/
void  LCDDisplayTask (void  *p_arg)
{
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR     err;
   OSTmrStart(&LCDTimer, &err);
    while (err.Code != RTOS_ERR_NONE) {}
    while (DEF_TRUE) {

    }
}

/***************************************************************************//**
 * @brief
 *   Task that handles button interrupts. It is called by both button interrupts.
 *  It pushes the button state to the appropriate FIFO and posts to the speed setpoint semaphore.
 * @param button
 *  True if button 1 was pressed, false if button 0 was pressed.
 * @param state
 * True if the button was pressed, false if it was released.
 ******************************************************************************/
struct buttonStateStruct {
  bool button0State;
  bool button1State;
  bool change; // Marks if there has been an interrupt since last read
};

OS_MUTEX buttonStructMutex;
OS_SEM buttonSem;
struct buttonStateStruct buttonStates;
void GPIO_INTERRUPT_Handler() {
  RTOS_ERR err;
  OSSemPost(&buttonSem, OS_OPT_POST_ALL, &err);
  while (err.Code != RTOS_ERR_NONE) {}
}

#define  BUTTON_TASK_PRIO            21u  /*   Task Priority.                 */
#define  BUTTON_TASK_STK_SIZE       256u  /*   Stack size in CPU_STK.         */
OS_TCB   buttonTaskTCB;                            /*   Task Control Block.   */
CPU_STK  buttonTaskStk[BUTTON_TASK_STK_SIZE]; /*   Stack.  */
void  buttonTask (void  *p_arg);
/***************************************************************************//**
 * @brief
 *   Creates the idle task.
 ******************************************************************************/
void  buttonTaskCreate (void)
{
    RTOS_ERR     err;

    OSTaskCreate(&buttonTaskTCB,                /* Pointer to the task's TCB.  */
                 "Button Task.",                    /* Name to help debugging.     */
                 &buttonTask,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  BUTTON_TASK_PRIO,             /* Task's priority.            */
                 &buttonTaskStk[0],             /* Pointer to base of stack.   */
                 (BUTTON_TASK_STK_SIZE / 10u),  /* Stack limit, from base.     */
                  BUTTON_TASK_STK_SIZE,         /* Stack size, in CPU_STK.     */
                  10u,                               /* Messages in task queue.     */
                  0u,                                /* Round-Robin time quanta.    */
                  DEF_NULL,                          /* External TCB data.          */
                  OS_OPT_TASK_STK_CHK,               /* Task options.               */
                 &err);
    if (err.Code != RTOS_ERR_NONE) {
        /* Handle error on task creation. */
    }
}
/***************************************************************************//**
 * @brief
 *   The idle task. Enters energy mode 1
 ******************************************************************************/
void  buttonTask (void  *p_arg)
{
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR     err;

   while (DEF_TRUE) {
       OSSemPend(&buttonSem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
       while (err.Code != RTOS_ERR_NONE) {}
       OSMutexPend(&buttonStructMutex, OS_OPT_PEND_BLOCKING, 0, NULL, &err);
       while (err.Code != RTOS_ERR_NONE) {}
       if (GPIO_PinInGet(BUTTON1_port, BUTTON1_pin) && GPIO_PinInGet(BUTTON0_port, BUTTON0_pin)) {
           buttonStates.button0State = false;
           buttonStates.button1State = false;
           GPIO_PinOutClear(LED0_port, LED0_pin);
           GPIO_PinOutClear(LED1_port, LED1_pin);
       } else if (GPIO_PinInGet(BUTTON1_port, BUTTON1_pin)) {
           buttonStates.button1State = true;
           GPIO_PinOutSet(LED1_port, LED1_pin);
       } else if (GPIO_PinInGet(BUTTON0_port, BUTTON0_pin)){
           buttonStates.button0State = true;
           GPIO_PinOutSet(LED0_port, LED0_pin);
       } else {
           buttonStates.button0State = false;
          buttonStates.button1State = false;
          GPIO_PinOutClear(LED0_port, LED0_pin);
          GPIO_PinOutClear(LED1_port, LED1_pin);
       }
       buttonStates.change = 1;
       OSMutexPost(&buttonStructMutex, OS_OPT_POST_NONE, &err);
       while (err.Code != RTOS_ERR_NONE) {}
   }
   if (err.Code) {}
}

struct sliderState {
  bool farLeft;
  bool left;
  bool right;
  bool farRight;
};
struct sliderState sliderState;
OS_MUTEX sliderMutex;
#define  SLIDER_PRIO            20u  /*   Task Priority.                 */
#define  SLIDER_STK_SIZE       256u  /*   Stack size in CPU_STK.         */
OS_TCB   sliderTaskTCB;                            /*   Task Control Block.   */
CPU_STK  sliderTaskStk[SLIDER_STK_SIZE]; /*   Stack.  */
void  sliderTask (void  *p_arg);
/***************************************************************************//**
 * @brief
 *   Creates the vehicle direction task
 ******************************************************************************/
void sliderTaskCreate (void)
{
    RTOS_ERR     err;

    OSTaskCreate(&sliderTaskTCB,                /* Pointer to the task's TCB.  */
                 "Slider Task.",                    /* Name to help debugging.     */
                 &sliderTask,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  SLIDER_PRIO,             /* Task's priority.            */
                 &sliderTaskStk[0],             /* Pointer to base of stack.   */
                 (SLIDER_STK_SIZE / 10u),  /* Stack limit, from base.     */
                  SLIDER_STK_SIZE,         /* Stack size, in CPU_STK.     */
                  10u,                               /* Messages in task queue.     */
                  0u,                                /* Round-Robin time quanta.    */
                  DEF_NULL,                          /* External TCB data.          */
                  OS_OPT_TASK_STK_CHK,               /* Task options.               */
                 &err);
    if (err.Code != RTOS_ERR_NONE) {
        /* Handle error on task creation. */
    }
}
/***************************************************************************//**
 * @brief
 *   Vehicle direction task. Reads the capacitive touch sensor and sets the
 *    direction of the vehicle based on the button presses. Posts an event flag
 *   to the vehicle monitor task if the direction has changed.
 ******************************************************************************/
void  sliderTask (void  *p_arg)
{
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR     err;
   OSTmrStart(&sliderTimer, &err);
   while (err.Code != RTOS_ERR_NONE) {}
    while (DEF_TRUE) {
        OSSemPend(&sliderSem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        CAPSENSE_Sense();
        // Read the capacitive touch sensor
        bool chan0 = CAPSENSE_getPressed(0);
        bool chan1 = CAPSENSE_getPressed(1);
        bool chan2 = CAPSENSE_getPressed(2);
        bool chan3 = CAPSENSE_getPressed(3);
        // Map the capacitive touch sensor readings to the direction struct
        OSMutexPend(&sliderMutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        if (chan0 || chan1) {
            GPIO_PinOutSet(LED1_port, LED1_pin);
        } else {
            GPIO_PinOutClear(LED1_port, LED1_pin);
        }
        if (chan2 || chan3) {
            GPIO_PinOutSet(LED0_port, LED0_pin);
        } else {
            GPIO_PinOutSet(LED0_port, LED0_pin);
        }
        if (chan0) {
            sliderState.farLeft = 1;
        } else {
            sliderState.farLeft = 0;
        }
        if (chan1) {
            sliderState.left = 1;
        } else {
            sliderState.left = 0;
        }
        if (chan2) {
            sliderState.right = 1;
        } else {
            sliderState.right = 0;
        }
        if (chan3) {
            sliderState.farRight = 1;
        } else {
            sliderState.farRight = 0;
        }
        OSMutexPost(&sliderMutex, OS_OPT_POST_NONE, &err);
        while (err.Code != RTOS_ERR_NONE) {}
    }
}

#define  IDLE_TASK_PRIO            22u  /*   Task Priority.                 */
#define  IDLE_TASK_STK_SIZE       256u  /*   Stack size in CPU_STK.         */
OS_TCB   idleTaskTCB;                            /*   Task Control Block.   */
CPU_STK  idleTaskStk[IDLE_TASK_STK_SIZE]; /*   Stack.  */
void  idleTask (void  *p_arg);
/***************************************************************************//**
 * @brief
 *   Creates the idle task.
 ******************************************************************************/
void  idleTaskCreate (void)
{
    RTOS_ERR     err;

    OSTaskCreate(&idleTaskTCB,                /* Pointer to the task's TCB.  */
                 "IDLE Task.",                    /* Name to help debugging.     */
                 &idleTask,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  IDLE_TASK_PRIO,             /* Task's priority.            */
                 &idleTaskStk[0],             /* Pointer to base of stack.   */
                 (IDLE_TASK_STK_SIZE / 10u),  /* Stack limit, from base.     */
                  IDLE_TASK_STK_SIZE,         /* Stack size, in CPU_STK.     */
                  10u,                               /* Messages in task queue.     */
                  0u,                                /* Round-Robin time quanta.    */
                  DEF_NULL,                          /* External TCB data.          */
                  OS_OPT_TASK_STK_CHK,               /* Task options.               */
                 &err);
    if (err.Code != RTOS_ERR_NONE) {
        /* Handle error on task creation. */
    }
}
/***************************************************************************//**
 * @brief
 *   The idle task. Enters energy mode 1
 ******************************************************************************/
void  idleTask (void  *p_arg)
{
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR     err;

   while (DEF_TRUE) {
       EMU_EnterEM1();
   }
   if (err.Code) {

   }
}


void app_init(void)
{
  RTOS_ERR err;
  // Initialize GPIO
  gpio_open();

  // Initialize our capactive touch sensor driver!
  CAPSENSE_Init();

  // Initialize our LCD system
  LCD_init();
  // Mutex Creation
  OSMutexCreate(&buttonStructMutex, "button mutex", &err);
  while (err.Code != RTOS_ERR_NONE) {}
  OSMutexCreate(&sliderMutex, "Slider Muted", &err);
  while (err.Code != RTOS_ERR_NONE) {}
  // Timer Creation
  OSTmrCreate(&physicsTimer, "Physics Timer", 0, PHYSICS_PERIOD, OS_OPT_TMR_PERIODIC, &physicsTimerCallback, DEF_NULL, &err);
  while (err.Code != RTOS_ERR_NONE) {}
  OSTmrCreate(&LCDTimer, "LCD Timer", 0, LCD_PERIOD, OS_OPT_TMR_PERIODIC, &LCDTimerCallback, DEF_NULL, &err);
  while (err.Code != RTOS_ERR_NONE) {}
  OSTmrCreate(&sliderTimer, "Slider Timer", 0, SLIDER_PERIOD, OS_OPT_TMR_PERIODIC, &sliderTimerCallback, DEF_NULL, &err);
  while (err.Code != RTOS_ERR_NONE) {}
  OSTmrCreate(&LED0Timer, "Slider Timer", 0, LED1_PERIOD, OS_OPT_TMR_PERIODIC, &LED0TimerCallback, DEF_NULL, &err);
  while (err.Code != RTOS_ERR_NONE) {}
  // Semaphore Creation
  OSSemCreate(&buttonSem, "Button Semaphore", 0, &err);
  while (err.Code != RTOS_ERR_NONE) {}
  OSSemCreate(&sliderSem, "Slider Semaphore", 0, &err);
  while (err.Code != RTOS_ERR_NONE) {}
  OSSemCreate(&physicsSem, "LED0 Semaphore", 0, &err);
  while (err.Code != RTOS_ERR_NONE) {}
  OSSemCreate(&LCDSem, "LCD Semaphore", 0, &err);
  while (err.Code != RTOS_ERR_NONE) {}
  // Task Creation
  idleTaskCreate();
  sliderTaskCreate();
   LCDTaskCreate();
   LED0OutputTaskCreate();
   LED1OutputTaskCreate();
  buttonTaskCreate();
  physicsTaskCreate();

    // Start the OS
  OSStart(&err);
  while (err.Code != RTOS_ERR_NONE) {}

}
