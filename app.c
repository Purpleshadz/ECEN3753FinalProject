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

// #define TEST_MODE // Comment out to disable test mode

#define LED0_PERIOD 50
#define LED1_PERIOD 500
#define PHYSICS_VERSION 1
// Variables to allow for easy tuning of game
int gravity = -10;
int screenSize = 128;
struct physicsConstants physicsConstantsInit(void);
struct physicsConstants physicsConstantsInit(void) {
    struct physicsConstants val;
    if (PHYSICS_VERSION == 1) {
        val = { // Normal Version
            .physicsPeriod = 50,
            .sliderPierod = 25;
            .lcdPeriod = 100;
            .canyonSize = screenSize;
            .castleConst = {
                .castleHeight = screenSize * .75,
                .foundationHitsRequired = 3,
                .foundationDepth = 21
            }
            .satchelConst = {
                .limitingMethod = AlwaysOne,
                .satchelDisplayDiameter = 7,
                .throwPeriod = 10,
                .maxInFlight = 2,
                .maxInFlightPeriod = 10
                .satchelWeight = 1000;
            }
            .platformConst = {
                .maxPlatformForce = 5000,
                .platformMass = 100,
                .platformLength = 16,
                .maxPlatformBounceSpeed = 5000,
                .maxPlatformSpeed = 5000
            }
            .shieldConst = {
                .shieldEffectiveRange = 20,
                .shieldActivationEnergy = 30
            }
            .railGunConst = {
                .railgunAngle = 3*pi/4 * 100,
                .shotMass = 50,
                .shotRadius = 5
            }
            .generatorConst = {
                .energyCapacity = 50,
                .maxShotPower = 20
            }
        }
    } else if (PHYSICS_VERSION == 2) {
        val = { // Suggested Version
            .physicsPeriod = 50,
            .sliderPierod = 150;
            .lcdPeriod = 100;
            .canyonSize = 100000;
            .castleConst = {
                .castleHeight = 5000,
                .foundationHitsRequired = 2,
                .foundationDepth = 5000
            }
            .satchelConst = {
                .limitingMethod = AlwaysOne,
                .satchelDisplayDiameter = 10,
                .throwPeriod = 1000,
                .maxInFlight = 2,
                .maxInFlightPeriod = 500
                .satchelWeight = 1000;
            }
            .platformConst = {
                .maxPlatformForce = 20000000,
                .platformMass = 100,
                .platformLength = 10000,
                .maxPlatformBounceSpeed = 50000,
                .maxPlatformSpeed = 50000
            }
            .shieldConst = {
                .shieldEffectiveRange = 15000,
                .shieldActivationEnergy = 30000
            }
            .railGunConst = {
                .railgunAngle = 800,
                .shotMass = 50,
                .shotRadius = 5
            }
            .generatorConst = {
                .energyCapacity = 50000,
                .maxShotPower = 20000
            }
        }
    } else if (PHYSICS_VERSION == 3) { 
        val = { // Normal Version
            .physicsPeriod = 50,
            .sliderPierod = 25;
            .lcdPeriod = 100;
            .canyonSize = screenSize;
            .castleConst = {
                .castleHeight = screenSize * .75,
                .foundationHitsRequired = 3,
                .foundationDepth = 20
            }
            .satchelConst = {
                .limitingMethod = AlwaysOne,
                .satchelDisplayDiameter = 7,
                .throwPeriod = 10,
                .maxInFlight = 2,
                .maxInFlightPeriod = 10
                .satchelWeight = 1000;
            }
            .platformConst = {
                .maxPlatformForce = 5000,
                .platformMass = 100,
                .platformLength = 16,
                .maxPlatformBounceSpeed = 5000,
                .maxPlatformSpeed = 5000
            }
            .shieldConst = {
                .shieldEffectiveRange = 20,
                .shieldActivationEnergy = 30
            }
            .railGunConst = {
                .railgunAngle = 3*pi/4 * 100,
                .shotMass = 50,
                .shotRadius = 5
            }
            .generatorConst = {
                .energyCapacity = 50,
                .maxShotPower = 20
            }
        }
    }
    // Scale the physconsts to the screen size using ratio
    // Allows for any desired values but immediately scales them to the screen size
    int ratio = val.canyonSize / screenSize;
    val.canyonSize = screenSize;
    val.castleConst.castleHeight = physConsts.castleConst.castleHeight / ratio;
    val.castleConst.foundationDepth = physConsts.castleConst.foundationDepth / ratio;
    val.satchelConst.satchelDisplayDiameter = physConsts.satchelConst.satchelDisplayDiameter / ratio;
    val.platformConst.platformLength = physConsts.platformConst.platformLength / ratio;
    val.platformConst.maxPlatformForce = physConsts.platformConst.maxPlatformForce / ratio; // maybe
    val.platformConst.maxPlatformBounceSpeed = physConsts.platformConst.maxPlatformBounceSpeed / ratio;
    val.platformConst.maxPlatformSpeed = physConsts.platformConst.maxPlatformSpeed / ratio;
    val.shieldConst.shieldEffectiveRange = physConsts.shieldConst.shieldEffectiveRange / ratio;
    val.railGunConst.shotRadius = physConsts.railGunConst.shotRadius / ratio;
    return val;
}

struct castleConstants {
    int castleHeight;
    int foundationHitsRequired;
    int foundationDepth;
}
enum limitingMethod {AlwaysOne, MaxInFlight, PeriodicThrowTime};
struct satchelConstants {
    int limitingMethod;
    int satchelDisplayDiameter;
    int throwPeriod; // in amount of physics periods
    int maxInFlight;
    int maxInFlightPeriod; // in amount of physics periods
    int satchelWeight;
}
struct platformConstants {
    int maxPlatformForce;
    int platformMass;
    int platformLength;
    int maxPlatformBounceSpeed;
    int maxPlatformSpeed;
}
struct shieldConstants {
    int shieldEffectiveRange;
    int shieldActivationEnergy;
}
struct railGunConstants{
    int railgunAngle;
    int shotMass;
    int shotRadius;
}
struct generatorCosntants{
    int energyCapacity;
    int maxShotPower;    
}
struct physicsConstants {
    int physicsPeriod;
    int sliderPeriod;
    int lcdPeriod;
    int canyonSize;
    struct castleConstants castleConst;
    struct satchelConstants satchelConst;
    struct platformConstants platformConst;
    struct shieldConstants shieldConst;
    struct railGunConstants railGunConst;
    struct generatorCosntants generatorConst;
} physConsts;


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
    int mass;
    int x;
    int y;
    int xVel;
    int yVel;
    int xAcc;
    int yAcc;
    int xForce;
    int yForce;
} physDataArray[10];
enum states {menu, active, win, fail};
struct gameData {
    int state = 0; // use states enum
    int energy = physConsts.generatorConst.energyCapacity;
    int shotCharge = 0;
    int foundationDamage = 0;
    bool evacComplete = false;
    int satchelsThrown = 0;
    int shieldsActivated = 0;
    int usefulShields = 0;
    int shotsFired = 0;
} gameData;
void spawnSatchel(void);
void spawnSatchel(void) {
    // Decide random landing spot. Allow for beyond canyon size to give a chance to bounce off of wall
    int landingSpot = rand() % (physConsts.canyonSize * .2) - (physConsts.canyonSize * .1);
    landingSpot = landingSpot + physDataArray[0].x; // Landing spot is relative to player
    // Calculate random flight duration from 1000ms to 5000ms
    int flightDuration = rand() % 4000 + 1000;
    // Calculate X speed to arrive in flight duration time
    int xSpeed = landingSpot / flightDuration;
    // Calculate Y speed to arrive in flight duration time
    int ySpeed = (-physConsts.castleConst.castleHeight / flightDuration) - (gravity * flightDuration / 2);
    for (int i = 0; i < 10; i++) {
        if (physicsData[i].objectType == empty) {
            physicsData[i].objectType = satchel;
            physicsData[i].x = 0;
            physicsData[i].y = physConsts.castleConst.castleHeight;
            physicsData[i].xVel = xSpeed;
            physicsData[i].yVel = ySpeed;
            physicsData[i].xAcc = gravity;
            physicsData[i].yAcc = 0;
            physicsData[i].xForce = 0;
            physicsData[i].yForce = 0;
            physicsData[i].mass = physConsts.satchelConst.mass;
            break;
        }
    }
}
void clearPhysicsData(void);
void clearPhysicsData(int i) {
    // i is the index of the physicsData to clear
    physicsData[i].objectType = empty;
    physicsData[i].x = 0;
    physicsData[i].y = 0;
    physicsData[i].xVel = 0;
    physicsData[i].yVel = 0;
    physicsData[i].xAcc = 0;
    physicsData[i].yAcc = 0;
    physicsData[i].xForce = 0;
    physicsData[i].yForce = 0;
    physicsData[i].mass = 0;
}
void  physicsTask (void  *p_arg)
{
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR     err;
    OSTmrStart(&physicsTimer, &err);
    while (err.Code != RTOS_ERR_NONE) {}
    for (int i = 0; i < 10; i++) {
        physDataArray[i].objectType = empty;
        physDataArray[i].mass = 0;
        physDataArray[i].x = 0;
        physDataArray[i].y = 0;
        physDataArray[i].xVel = 0;
        physDataArray[i].yVel = 0;
        physDataArray[i].xAcc = 0;
        physDataArray[i].yAcc = 0;
        physDataArray[i].xForce = 0;
        physDataArray[i].yForce = 0;
    }
    physDataArray[0].objectType = player;
    physDataArray[0].x = physConsts.canyonSize / 2;
    physDataArray[0].y = 0;
    physDataArray[0].mass = physConsts.platformConst.mass;
    bool charging = false;


   while (DEF_TRUE) {
        while (gameData.state != active) {} // stop when game not running
       OSSemPend(&physicsSem, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
       while (err.Code != RTOS_ERR_NONE) {}
        OSMutexPend(&physicsStructMutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        physDataArray[i].xForce = 0;
        if ((sliderState.farLeft || sliderState.left) && (sliderState.farRight || sliderState.right)) {
            // do Nothing
        } else if (sliderState.left == 1) {
            physDataArray[i].xForce += -physConsts.platformConst.maxPlatformForce / 2;
        } else if (sliderState.right == 1) {
            physDataArray[i].xForce += physConsts.platformConst.maxPlatformForce;
        } else if (sliderState.farLeft == 1) {
            physDataArray[i].xForce += -physConsts.platformConst.maxPlatformForce;
        } else if (sliderState.farRight == 1) {
            physDataArray[i].xForce += physConsts.platformConst.maxPlatformForce / 2;
        } else {
            if (physDataArray[i].xVel > 0) {
                physDataArray[i].xForce += -physConsts.platformConst.maxPlatformForce;
            } else if (physDataArray[i].xVel < 0) {
                physDataArray[i].xForce += physConsts.platformConst.maxPlatformForce;
            }
        }
        OSMutexPend(&buttonStructMutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        if (buttonStates.button0 == 1) {
            charging = true;
        } else if (buttonStates.button0 == 0 && charging == true) {
            charging = false;
            if (gameData.shotCharge > 0) {
                for (int j = 0; j < 10; j++) {
                    if (physDataArray[j].objectType == empty) {
                        physDataArray[j].objectType = shot;
                        physDataArray[j].x = physDataArray[0].x;
                        physDataArray[j].y = physDataArray[0].y;
                        physDataArray[j].xVel = 0;
                        physDataArray[j].yVel = 0;
                        physDataArray[j].xAcc = 0;
                        physDataArray[j].yAcc = 0;
                        physDataArray[j].xForce = (gameData.shotCharge / physConsts.generatorConst.maxShotPower) * physConsts.generatorConst.maxShotPower * cos(physConsts.railGunConst.angle * 3.14159 / 180);
                        physDataArray[j].yForce = (gameData.shotCharge / physConsts.generatorConst.maxShotPower) * physConsts.generatorConst.maxShotPower * sin(physConsts.railGunConst.angle * 3.14159 / 180);
                        physDataArray[j].mass = physConsts.railGunConst.mass;
                        physDataArray[0].xForce += physDataArray[j].xForce;
                        gameData.shotCharge = 0;
                        gameData.shotsFired++;
                        break;
                    }
                }
            }
        } 
        
        if (buttonStates.button1 == 1 && physConsts.shieldConst.shieldActivationEnergy >= physConsts.shieldConst.shieldActivationEnergy && buttonStates.button1Change == 1) {
            gameData.energy -= physConsts.shieldConst.shieldActivationEnergy;
            gameData.shieldsActivated++;
            for (int i = 0; i < 10; i++) {
                if (physicsData[i].objectType == shot) {
                    int xDist = physicsData[i].x - physicsData[0].x;
                    int yDist = physicsData[i].y - physicsData[0].y;
                    int distance = sqrt(xDist * xDist + yDist * yDist);
                    if (distance =< physConsts.shieldConst.shieldEffectiveRange) {
                        clearPhysicsData(i);
                        gameData.usefulShields++;
                    }
                }
            }
        } else if (buttonStates.button1 == 1 && gameData.energy < physConsts.shieldConst.shieldActivationEnergy) {
            // Do something to indicate that the player can't use the shield
        }
        OSMutexPost(&buttonStructMutex, OS_OPT_POST_NONE, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        if (charging == true && gameData.energy > 0 && gameData.shotCharge < physConsts.generatorConst.maxShotPower) {
            gameData.shotCharge++;
            gameData.energy--;
        } else if (charging == true && gameData.energy == 0) {
            // Do nothing
        } else if (charging == false && gameData.energy < physConsts.generatorConst.energyCapacity) {
            gameData.energy++;
        }
        switch (physConsts.satchelConst.limitingMethod) {
        case AlwaysOne:
            // Check if there is a satchel in the array
            for (int i = 0; i < 10; i++) {
                if (physicsData[i].objectType == satchel) {
                    break;
                } else if (i == 9) {
                    // If there is no satchel in the array, create one
                    spawnSatchel();
                }
            }
            break;
        case MaxInFlight:
            // Check if # of satchels in flight is less than max
            int satchelCount = 0;
            int timer = 0;
            if (!(timer % physConsts.satchelConst.maxInFlightPeriod == 0)) {
                timer++;
            }
            for (int i = 0; i < 10; i++) {
                if (physicsData[i].objectType == satchel) {
                    satchelCount++;
                }
            }
            if (satchelCount < physConsts.satchelConst.maxInFlight && (timer % physConsts.satchelConst.maxInFlightPeriod)) { // If there are less than max, create a satchel
                spawnSatchel();
            }
            break;
        case PeriodicThrowTime:
            // Check if it is time to throw a satchel
            int timer = 0;
            if (timer % physConsts.satchelConst.throwPeriod == 0) {
                spawnSatchel();
            }
            timer++;
            break;
        default:
            while (1) {}
            // Shouldn't be here
            break;
        }   
        for (int i = 0; i < 10; i++) {
            if (physicsData[i].objectType == shot || physicsData[i].objectType == satchel) {
                physicsData[i].xAcc = physicsData[i].xForce / physicsData[i].xMass;
                physicsData[i].yAcc = physicsData[i].yForce / physicsData[i].yMass + gravity;
                physicsData[i].yVel += physicsData[i].yAcc * (physConsts.physicsPeriod / 1000);
                physicsData[i].xVel += physicsData[i].xAcc * (physConsts.physicsPeriod / 1000);
                physicsData[i].x += physicsData[i].xVel * (physConsts.physicsPeriod / 1000);
                physicsData[i].y += physicsData[i].yVel * (physConsts.physicsPeriod / 1000);
                physicsData[i].yForce = 0; // Forces aren't constant, so they need to be reset
                physicsData[i].xForce = 0; // Forces aren't constant, so they need to be reset
                // Check if the shot has hit the ground
                if (physicsData[i].objectType == satchel) { // Will allow satchel to fly above screen
                    if (physicsData[i].x > 0 && physicsData[i].x < 100 && physicsData[i].y < 10) { // Lose on hit
                        clearPhysicsData(i);
                        gameData.state = fail;
                    } else if ((physicsData[i].y + physConsts.satchelConst.satchelDisplayDiameter / 2) < 0) { // Destroy on ground
                        clearPhysicsData(i);
                    } else if ((physicsData[i].x + physConsts.satchelConst.satchelDisplayDiameter / 2) > physConsts.canyonSize){ // bounce off wall if hit
                        physicsData[i].xVel = -physicsData[i].xVel;
                        physicsData[i].x = physConsts.canyonSize - physicsData[i].x % physConsts.canyonSize;
                    }
                } else if (physicsData[i].objectType == shot) {
                    if (physicsData[i].x <= 0  && physicsData[i].y >= physConsts.castleConst.castleHeight && physicsData[i].y <= physConsts.canyonSize) { // Lose on hit
                        clearPhysicsData(i);
                        gameData.foundationDamage++;
                    } else if (physicsData[i].y <= 0) { // Destroy on ground
                        clearPhysicsData(i);
                    } else if (physicsData[i].x <= 0  && physicsData[i].y < physConsts.castleConst.castleHeight){ // Destroy of below castle
                        clearPhysicsData(i);
                    }
                }
            } else if (physicsData[i].objectType = player) {
                physicsData[i].xAcc = physicsData[i].xForce / physicsData[i].xMass;
                physicsData[i].xVel += physicsData[i].xAcc * (physConsts.physicsPeriod / 1000);
                if (physicsData[i].xVel > physConsts.platformConst.maxPlatformSpeed) {
                    physicsData[i].xVel = physConsts.platformConst.maxPlatformSpeed;
                } else if (physicsData[i].xVel < -physConsts.platformConst.maxPlatformSpeed) {
                    physicsData[i].xVel = -physConsts.platformConst.maxPlatformSpeed;
                }
                physicsData[i].x += physicsData[i].xVel * (physConsts.physicsPeriod / 1000);
                physicsData[i].xForce = 0; // Forces aren't constant, so they need to be reset
                // Check if player hit wall, if so bounce
                if (physicsData[i].x + physConsts.platformConst.platformLength / 2 < 0) {
                    physicsData[i].xVel = -physicsData[i].xVel;
                    physicsData[i].x = physicsData[i].x + 2((physicsData[i].x + physConsts.platformConst.platformLength) % physConsts.canyonSize);
                } else if (physicsData[i].x + physConsts.platformConst.platformLength / 2 > physConsts.canyonSize) {
                    physicsData[i].xVel = -physicsData[i].xVel;
                    physicsData[i].x = physicsData[i].x - 2((physicsData[i].x + physConsts.platformConst.platformLength) % physConsts.canyonSize);
                } 
            }
        }
        OSMutexPost(&physicsMutex, OS_OPT_POST_NONE, &err);
        while (err.Code != RTOS_ERR_NONE) {}
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
 *   
 ******************************************************************************/
void  LCDDisplayTask (void  *p_arg)
{
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR     err;
   OSTmrStart(&LED1Timer, &err);
   OSTmrStart(&LED0Timer, &err);
   OSTmrStart(&LCDTimer, &err);
   struct __GLIB_Rectangle_t platform;
   struct __GLIB_Rectangle_t rectangles[8];
   struct __GLIB_Rectangle_t battery[4];
   int cannonLength = physConsts.platformConst.platformWidth / 2;
    while (err.Code != RTOS_ERR_NONE) {}
    while (DEF_TRUE) {
        OSSemPend(&LCDUpdateSem, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        if (gameData.gameState == active) {
            GLIB_drawRectFilled(&glibContext, &platform);
            // Generate cliff
            GLIB_drawLineV(&glibContext, 0, 0, gameData.platformY + physConsts.castleConst.castleHeight);
            GLIB_drawLineV(&glibContext, 1, 0, gameData.platformY + physConsts.castleConst.castleHeight);
            // Generate right wall
            GLIB_drawLineV(&glibContext, screenSize - 1, 0, screenSize - 1);
            // Generate castle
            rectangles[0] = {
                // Left wall
                .xMin = 0,
                .xMax = physConsts.castleConst.foundationHitsRequired * 2,
                .yMin = physConsts.castleConst.castleHeight,
                .yMax = screenSize - 1
            };
            rectangles[1] = {
                // Ceiling
                .xMin = 0,
                .xMax = 20,
                .yMin = screenSize - 6,
                .yMax = screenSize - 1
            };
            rectangles[2] = {
                // Right wall
                .xMin = 15,
                .xMax = 20,
                .yMin = physConsts.castleConst.castleHeight,
                .yMax = screenSize - 1
            };
            rectangles[3] = {
                // Floor
                .xMin = 0,
                .xMax = 20,
                .yMin = physConsts.castleConst.castleHeight,
                .yMax = physConsts.castleConst.castleHeight + 5
            };
            rectangles[4] = {
                // Flag pole
                .xMin = 20,
                .xMax = 35,
                .yMin = screenSize - 3,
                .yMax = screenSize - 1
            };
            rectangles[5] = {
                // Flag
                .xMin = 25,
                .xMax = 35,
                .yMin = physConsts.castleConst.castleHeight,
                .yMax = screenSize - 1
            };
            // Generate Foundation
            rectangles[6] = {
                .xMin = 0,
                .xMax = (physConsts.castleConst.foundationHitsRequired - gameData.foundationDamage) * 2,
                .yMin = physConsts.castleConst.castleHeight - physConsts.castleConst.foundationDepth,
                .yMax = physConsts.castleConst.castleHeight
            };
            // Generate platform
            rectangles[7] = {
                .xMin = gameData.platformX - physConsts.platformConst.platformLength / 2,
                .xMax = gameData.platformX + physConsts.platformConst.platformLength / 2,
                .yMin = gameData.platformY,
                .yMax = gameData.platformY + 4 
            };
            // Draw castle
            for (int i = 0; i < 8; i++) {
                GLIB_drawRectFilled(&glibContext, &rectangles[i]);
            }
            // Draw Cannon 5 pixels thick
            for (int i = -2; i < 3; i++) {
                GLIB_drawLine(&glibContext, physicsData[i].x + i, 0, physicsData[i].x + cannonLength * cos(physConsts.railGunConst.angle * 3.14159 / 180) + i, cannonLength * sin(physConsts.railGunConst.angle * 3.14159 / 180));
            }
            // Draw Projectiles
            for (int i = 0; i < 10; i++) {
                if (physicsData[i].objectType == satchel) {
                    GLIB_drawCircleFilled(&glibContext, physicsData[i].x, physicsData[i].y, physConsts.satchelConst.satchelDisplayDiameter / 2);
                } else if (physicsData[i].objectType == shot) {
                    GLIB_drawCircleFilled(&glibContext, physicsData[i].x, physicsData[i].y, physConsts.railGunConst.shotRadius);
                }
            }
            // Draw Battery
            battery[0] = { // Remaining battery
                .xMin = screenSize - 17,
                .xMax = screenSize - 8,
                .yMin = screenSize - 32,
                .yMax = screenSize - 32 + (gameData.energy / physConsts.batteryConst.maxEnergy * 20)
            };
            battery[1] = { // Outer battery shell
                .xMin = screenSize - 19,
                .xMax = screenSize - 6,
                .yMin = screenSize - 24,
                .yMax = screenSize - 11
            };
            battery[2] = { // Inner battery shell
                .xMin = screenSize - 18,
                .xMax = screenSize - 7,
                .yMin = screenSize - 23,
                .yMax = screenSize - 12
            };
            battery[3] = { // Battery bump
                .xMin = screenSize - 14,
                .xMax = screenSize - 11,
                .yMin = screenSize - 7,
                .yMax = screenSize - 4
            };
        }
    }
}

void LED0TimerCallback (void  *p_tmr, void  *p_arg);
/***************************************************************************//**
 * @brief
 *   timer callback function. Posts a flag to the LED task to turn on the LED
 *    when the timer expires. Used to wait ~3 during held slider before turning
 *   on the LED1;
 ******************************************************************************/
OS_TMR LED0Timer;
int counter = 0;
void LED0TimerCallback (void  *p_tmr, void  *p_arg)
{
    (void)&p_arg;
    (void)&p_tmr;
    RTOS_ERR err;
    counter++;
    if


}

void LED1TimerCallback (void  *p_tmr, void  *p_arg);
/***************************************************************************//**
 * @brief
 *   timer callback function. Posts a flag to the LED task to turn on the LED
 *    when the timer expires. Used to wait ~3 during held slider before turning
 *   on the LED1;
 ******************************************************************************/
bool state = false;
OS_TMR LED1Timer;
int counter = 0;
int startBelow50 = 0;
int evacTime = 5; // seconds
void LED1TimerCallback (void  *p_tmr, void  *p_arg)
{
    (void)&p_arg;
    (void)&p_tmr;
    RTOS_ERR err;
    counter++;
    if (gameData.foundationDamage >= physConsts.castleConst.foundationHitsRequired * .5) {
        if (startBelow50 == 0) {
            startBelow50 = counter;
        }
        if (counter - startBelow50 > evacTime * 2) {
            gameData.evavComplete = true;
            GPIO_PinOutSet(LED1_port, LED1_pin);
        } else if (counter - startBelow50 > evacTime) {
            if (counter % 2 == 0) {
                GPIO_PinOutSet(LED1_port, LED1_pin);
            } else {
                GPIO_PinOutClear(LED1_port, LED1_pin);
            } 
        }

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
  bool button0Change; 
  bool button1Change;
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
#ifndef TEST_MODE
       OSSemPend(&buttonSem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
       while (err.Code != RTOS_ERR_NONE) {}
       OSMutexPend(&buttonStructMutex, OS_OPT_PEND_BLOCKING, 0, NULL, &err);
       while (err.Code != RTOS_ERR_NONE) {}
       if (GPIO_PinInGet(BUTTON1_port, BUTTON1_pin) != buttonStates.button1State) {
           buttonStates.button1Change = true;
       } else if (GPIO_PinInGet(BUTTON0_port, BUTTON0_pin) != buttonStates.button0State) {
           buttonStates.button0Change = true;
       }
       if (GPIO_PinInGet(BUTTON1_port, BUTTON1_pin) && GPIO_PinInGet(BUTTON0_port, BUTTON0_pin)) {
           buttonStates.button0State = false;
           buttonStates.button1State = false;
       } else if (GPIO_PinInGet(BUTTON1_port, BUTTON1_pin)) {
           buttonStates.button1State = true;
       } else if (GPIO_PinInGet(BUTTON0_port, BUTTON0_pin)){
           buttonStates.button0State = true;
           GPIO_PinOutSet(LED0_port, LED0_pin);
       } else {
           buttonStates.button0State = false;
          buttonStates.button1State = false;
       }
       OSMutexPost(&buttonStructMutex, OS_OPT_POST_NONE, &err);
       while (err.Code != RTOS_ERR_NONE) {}
#endif
#ifdef TEST_MODE
       if (GPIO_PinInGet(BUTTON1_port, BUTTON1_pin)) {
           GPIO_PinOutSet(LED1_port, LED1_pin);
       } else {
           GPIO_PinOutClear(LED1_port, LED1_pin);
       }
       if (GPIO_PinInGet(BUTTON0_port, BUTTON0_pin)) {
           GPIO_PinOutSet(LED0_port, LED0_pin);
       } else {
           GPIO_PinOutClear(LED0_port, LED0_pin);
       }
#endif
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
#ifndef TEST_MODE
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
#endif
#ifdef TEST_MODE
        bool chan0 = CAPSENSE_getPressed(0);
        bool chan1 = CAPSENSE_getPressed(1);
        bool chan2 = CAPSENSE_getPressed(2);
        bool chan3 = CAPSENSE_getPressed(3);
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
#endif
    }
}

#define  IDLE_TASK_PRIO            25u  /*   Task Priority.                 */
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

  // Initialize Physical constants
  physConsts = initPhysics();

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
  OSTmrCreate(&LED0Timer, "LED0 Timer", 0, LED0_PERIOD, OS_OPT_TMR_PERIODIC, &LED0TimerCallback, DEF_NULL, &err);
  while (err.Code != RTOS_ERR_NONE) {}
  OSTmrCreate(&LED1Timer, "LED1 Timer", 0, LED1_PERIOD, OS_OPT_TMR_PERIODIC, &LED1TimerCallback, DEF_NULL, &err);
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
  buttonTaskCreate();
#ifndef TEST_MODE
  physicsTaskCreate();
    LCDTaskCreate();
#endif

    // Start the OS
  OSStart(&err);
  while (err.Code != RTOS_ERR_NONE) {}

}
