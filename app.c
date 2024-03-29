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
#include <math.h>

#include "sl_board_control.h"
#include "em_assert.h"
#include "glib.h"
#include "dmd.h"
#include "os.h"
#include "stdio.h"
#include "FIFO.h"

// #define TEST_MODE // Comment out to disable test mode

#define PHYSICS_VERSION 1

struct castleConstants {
    int castleHeight; // cm
    int foundationHitsRequired;
    int foundationDepth; // cm
};
enum limitingMethod {AlwaysOne, MaxInFlight, PeriodicThrowTime};
struct satchelConstants {
    int limitingMethod;
    int satchelDisplayDiameter; // Pixels
    int throwPeriod; // in amount of physics periods
    int maxInFlight; // amount of satchels allowed in flight
    int maxInFlightPeriod; // in amount of physics periods
    int satchelWeight;
};
struct platformConstants {
    int maxPlatformForce; // Newtons
    int platformMass; // KG
    int platformLength; // cm
    int maxPlatformBounceSpeed; // cm/s
    int maxPlatformSpeed; // cm/s
};
struct shieldConstants {
    int shieldEffectiveRange; // cm
    int shieldActivationEnergy; // KJ
};
struct railGunConstants{
    int railgunAngle; // radians
    int shotMass; // kg
    int shotRadius; // pixels
};
struct generatorCosntants{
    int energyCapacity; // KJ
    float maxShotPower; // watts
};
struct physicsConstants {
    int physicsPeriod; // ms
    int sliderPeriod; // ms
    int lcdPeriod; // ms
    int canyonSize; // cm
    struct castleConstants castleConst;
    struct satchelConstants satchelConst;
    struct platformConstants platformConst;
    struct shieldConstants shieldConst;
    struct railGunConstants railGunConst;
    struct generatorCosntants generatorConst;
} physConsts;

OS_MUTEX buttonStructMutex;
OS_SEM buttonSem;
struct buttonStateStruct buttonStates;

struct sliderState {
  bool farLeft;
  bool left;
  bool right;
  bool farRight;
};
struct sliderState sliderState;
OS_MUTEX sliderMutex;
OS_SEM sliderSem;

OS_SEM LCDSem;

OS_SEM gameEndSem; // Used to block tasks when the game is over. Will only post if intending to restart.

struct buttonStateStruct {
  bool button0State;
  bool button1State;
  bool button0Change;
  bool button1Change;
};

// Variables to allow for easy tuning of game
int gravity = -10;
int screenSize = 127; // Actually 128 but 0 indexed
struct physicsConstants physicsConstantsInit(void);
struct physicsConstants physicsConstantsInit(void) {
    // struct physicsConstants val;
    if (PHYSICS_VERSION == 1) {
        struct physicsConstants val = { // Normal Version
            .physicsPeriod = 50,
            .sliderPeriod = 100,
            .lcdPeriod = 150,
            .canyonSize = screenSize,
            .castleConst = {
                .castleHeight = screenSize * .75,
                .foundationHitsRequired = 3,
                .foundationDepth = 21
            },
            .satchelConst = {
                .limitingMethod = AlwaysOne,
                .satchelDisplayDiameter = 7,
                .throwPeriod = 5,
                .maxInFlight = 2,
                .maxInFlightPeriod = 10,
                .satchelWeight = 1000,
            },
            .platformConst = {
                .maxPlatformForce = 5000,
                .platformMass = 100,
                .platformLength = 16,
                .maxPlatformBounceSpeed = 5000,
                .maxPlatformSpeed = 5000
            },
            .shieldConst = {
                .shieldEffectiveRange = 20,
                .shieldActivationEnergy = 30
            },
            .railGunConst = {
                .railgunAngle = 3*3.1415/4 * 100,
                .shotMass = 50,
                .shotRadius = 5
            },
            .generatorConst = {
                .energyCapacity = 50,
                .maxShotPower = 20
            }
        };
        // Scale the physconsts to the screen size using ratio
        // Allows for any desired values but immediately scales them to the screen size
        int ratio = val.canyonSize / screenSize;
        val.canyonSize = screenSize;
        val.castleConst.castleHeight = val.castleConst.castleHeight / ratio;
        val.castleConst.foundationDepth = val.castleConst.foundationDepth / ratio;
        val.satchelConst.satchelDisplayDiameter = val.satchelConst.satchelDisplayDiameter / ratio;
        val.platformConst.platformLength = val.platformConst.platformLength / ratio;
        val.platformConst.maxPlatformForce = val.platformConst.maxPlatformForce / ratio; // maybe
        val.platformConst.maxPlatformBounceSpeed = val.platformConst.maxPlatformBounceSpeed / ratio;
        val.platformConst.maxPlatformSpeed = val.platformConst.maxPlatformSpeed / ratio;
        val.shieldConst.shieldEffectiveRange = val.shieldConst.shieldEffectiveRange / ratio;
        val.railGunConst.shotRadius = val.railGunConst.shotRadius / ratio;
        return val;
    } else if (PHYSICS_VERSION == 2) {
        struct physicsConstants val = { // Suggested Version
            .physicsPeriod = 50,
            .sliderPeriod = 100,
            .lcdPeriod = 150,
            .canyonSize = 100000,
            .castleConst = {
                .castleHeight = 5000,
                .foundationHitsRequired = 2,
                .foundationDepth = 5000
            },
            .satchelConst = {
                .limitingMethod = AlwaysOne,
                .satchelDisplayDiameter = 10,
                .throwPeriod = 1000,
                .maxInFlight = 2,
                .maxInFlightPeriod = 500,
                .satchelWeight = 1000
            },
            .platformConst = {
                .maxPlatformForce = 20000000,
                .platformMass = 100,
                .platformLength = 10000,
                .maxPlatformBounceSpeed = 50000,
                .maxPlatformSpeed = 50000
            },
            .shieldConst = {
                .shieldEffectiveRange = 15000,
                .shieldActivationEnergy = 30000
            },
            .railGunConst = {
                .railgunAngle = 800,
                .shotMass = 50,
                .shotRadius = 5
            },
            .generatorConst = {
                .energyCapacity = 50000,
                .maxShotPower = 20000
            }
        };
        // Scale the physconsts to the screen size using ratio
        // Allows for any desired values but immediately scales them to the screen size
        int ratio = val.canyonSize / screenSize;
        val.canyonSize = screenSize;
        val.castleConst.castleHeight = val.castleConst.castleHeight / ratio;
        val.castleConst.foundationDepth = val.castleConst.foundationDepth / ratio;
        val.satchelConst.satchelDisplayDiameter = val.satchelConst.satchelDisplayDiameter / ratio;
        val.platformConst.platformLength = val.platformConst.platformLength / ratio;
        val.platformConst.maxPlatformForce = val.platformConst.maxPlatformForce / ratio; // maybe
        val.platformConst.maxPlatformBounceSpeed = val.platformConst.maxPlatformBounceSpeed / ratio;
        val.platformConst.maxPlatformSpeed = val.platformConst.maxPlatformSpeed / ratio;
        val.shieldConst.shieldEffectiveRange = val.shieldConst.shieldEffectiveRange / ratio;
        val.railGunConst.shotRadius = val.railGunConst.shotRadius / ratio;
        return val;
    } else if (PHYSICS_VERSION == 3) { 
        struct physicsConstants val = { // Normal Version
            .physicsPeriod = 50,
            .sliderPeriod = 100,
            .lcdPeriod = 150,
            .canyonSize = screenSize,
            .castleConst = {
                .castleHeight = screenSize * .75,
                .foundationHitsRequired = 3,
                .foundationDepth = 20
            },
            .satchelConst = {
                .limitingMethod = AlwaysOne,
                .satchelDisplayDiameter = 7,
                .throwPeriod = 10,
                .maxInFlight = 2,
                .maxInFlightPeriod = 10,
                .satchelWeight = 1000
            },
            .platformConst = {
                .maxPlatformForce = 5000,
                .platformMass = 100,
                .platformLength = 16,
                .maxPlatformBounceSpeed = 5000,
                .maxPlatformSpeed = 5000
            },
            .shieldConst = {
                .shieldEffectiveRange = 20,
                .shieldActivationEnergy = 30
            },
            .railGunConst = {
                .railgunAngle = 3*3.1415/4 * 100,
                .shotMass = 50,
                .shotRadius = 5
            },
            .generatorConst = {
                .energyCapacity = 50,
                .maxShotPower = 20
            },
        };
        // Scale the physconsts to the screen size using ratio
        // Allows for any desired values but immediately scales them to the screen size
        int ratio = val.canyonSize / screenSize;
        val.canyonSize = screenSize;
        val.castleConst.castleHeight = val.castleConst.castleHeight / ratio;
        val.castleConst.foundationDepth = val.castleConst.foundationDepth / ratio;
        val.satchelConst.satchelDisplayDiameter = val.satchelConst.satchelDisplayDiameter / ratio;
        val.platformConst.platformLength = val.platformConst.platformLength / ratio;
        val.platformConst.maxPlatformForce = val.platformConst.maxPlatformForce / ratio; // maybe
        val.platformConst.maxPlatformBounceSpeed = val.platformConst.maxPlatformBounceSpeed / ratio;
        val.platformConst.maxPlatformSpeed = val.platformConst.maxPlatformSpeed / ratio;
        val.shieldConst.shieldEffectiveRange = val.shieldConst.shieldEffectiveRange / ratio;
        val.railGunConst.shotRadius = val.railGunConst.shotRadius / ratio;
        return val;
    }
}


OS_SEM physicsSem;
#define  PHYSICS_TASK_PRIO            21u  /*   Task Priority.                 */
#define  PHYSICS_TASK_STK_SIZE       256u  /*   Stack size in CPU_STK.         */
OS_TCB   physicsTaskTCB;                            /*   Task Control Block.   */
CPU_STK  physicsTaskStk[PHYSICS_TASK_STK_SIZE]; /*   Stack.  */
void  physicsTask (void  *p_arg);
/***************************************************************************//**
 * @brief
 *   Creates the physics task.
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
OS_MUTEX physicsStructMutex;
enum objectType {empty, player, satchel, shot};
struct physicsData {
    int objectType;
    int mass;
    float x;
    float y;
    float xVel;
    float yVel;
    float xAcc;
    float yAcc;
    int xForce;
    int yForce;
} physDataArray[10];
enum states {menu, active, win, fail};
struct gameData {
    int state; // use states enum
    float energy;
    float shotCharge;
    int foundationDamage;
    bool evacComplete;
    bool shieldActive;
    int satchelsThrown;
    int shieldsActivated;
    int usefulShields;
    int shotsFired;
} gameData;
/***************************************************************************//**
 * @brief
 *   Spawns satchel. Called by physics task.
 ******************************************************************************/
void spawnSatchel(struct physicsData *phys);
void spawnSatchel(struct physicsData *phys) {
    // Decide random landing spot. Allow for beyond canyon size to give a chance to bounce off of wall
    int landingSpot = rand() % (int)(physConsts.canyonSize / 3);
    if (rand() % 2) { // Randomly make landing spot negative
        landingSpot = landingSpot * -1;
    }
    landingSpot = landingSpot + physDataArray[0].x; // Landing spot is relative to player
    // Calculate random flight duration from 1000ms to 5000ms
    float flightDuration = rand() % 4000 + 1000;
    flightDuration = flightDuration / 1000;
    // Calculate X speed to arrive in flight duration time
    float xSpeed = landingSpot / flightDuration;
    // Calculate Y speed to arrive in flight duration time
    float ySpeed = (-physConsts.castleConst.castleHeight / flightDuration) - (gravity * flightDuration / 2);

    phys->objectType = satchel;
    phys->x = 0;
    phys->y = physConsts.castleConst.castleHeight;
    phys->xVel = xSpeed;
    phys->yVel = ySpeed;
    phys->xAcc = 0;
    phys->yAcc = gravity;
    phys->xForce = 0;
    phys->yForce = 0;
    phys->mass = physConsts.satchelConst.satchelWeight;
}
/***************************************************************************//**
 * @brief
 *   Clears physics data. Called by physics task whenever an object is destroyed.
 ******************************************************************************/
void clearPhysicsData(struct physicsData *physData);
void clearPhysicsData(struct physicsData *physData) {
    physData->objectType = empty;
    physData->x = 0;
    physData->y = 0;
    physData->xVel = 0;
    physData->yVel = 0;
    physData->xAcc = 0;
    physData->yAcc = 0;
    physData->xForce = 0;
    physData->yForce = 0;
    physData->mass = 0;
}
/***************************************************************************//**
 * @brief
 *   Physics task. Handles all physics calculations.
 ******************************************************************************/
void  physicsTask (void  *p_arg)
{
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR     err;
    struct gameData localDat = {
        .state = menu, // use states enum
        .energy = physConsts.generatorConst.energyCapacity,
        .shotCharge = 0,
        .foundationDamage = 0,
        .evacComplete = false,
        .satchelsThrown = 0,
        .shieldsActivated = 0,
        .usefulShields = 0,
        .shotsFired = 0
    };
    gameData = localDat;
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
    physDataArray[0].mass = physConsts.platformConst.platformMass;
    bool charging = false;
    int timer = 0;
    gameData.state = active;
    struct physicsData localDataArray[10]; // Local copy of physics data to minimize mutex time

   while (DEF_TRUE) {
        while (gameData.state != active) {// stop when game not running. Used to block task
            OSSemPend(&gameEndSem, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
            while (err.Code != RTOS_ERR_NONE) {}
        }
        // Wait for physics period
        OSTimeDlyHMSM(0, 0, 0, physConsts.physicsPeriod, OS_OPT_TIME_DLY, &err);
       while (err.Code != RTOS_ERR_NONE) {}
        OSMutexPend(&physicsStructMutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        // Copy physics data to local array
        for (int i = 0; i < 10; i++) {
            localDataArray[i].mass = physDataArray[i].mass;
            localDataArray[i].objectType = physDataArray[i].objectType;
            localDataArray[i].x = physDataArray[i].x;
            localDataArray[i].y = physDataArray[i].y;
            localDataArray[i].xVel = physDataArray[i].xVel;
            localDataArray[i].yVel = physDataArray[i].yVel;
            localDataArray[i].xAcc = physDataArray[i].xAcc;
            localDataArray[i].yAcc = physDataArray[i].yAcc;
            localDataArray[i].xForce = physDataArray[i].xForce;
            localDataArray[i].yForce = physDataArray[i].yForce;
        }
        OSMutexPost(&physicsStructMutex, OS_OPT_POST_NONE, &err);
        OSMutexPend(&sliderMutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        // Use slider data to calculate platform force
        if ((sliderState.farLeft || sliderState.left) && (sliderState.farRight || sliderState.right)) {
            // do Nothing
        } else if (sliderState.left) {
            localDataArray[0].xForce += -physConsts.platformConst.maxPlatformForce / 2;
        } else if (sliderState.right) {
            localDataArray[0].xForce += physConsts.platformConst.maxPlatformForce;
        } else if (sliderState.farLeft) {
            localDataArray[0].xForce += -physConsts.platformConst.maxPlatformForce;
        } else if (sliderState.farRight) {
            localDataArray[0].xForce += physConsts.platformConst.maxPlatformForce / 2;
        } else { // no slider input, apply friction
            if (localDataArray[0].xVel > 0) {
                localDataArray[0].xForce += -physConsts.platformConst.maxPlatformForce;
            } else if (localDataArray[0].xVel < 0) {
                localDataArray[0].xForce += physConsts.platformConst.maxPlatformForce;
            }
        }
        OSMutexPost(&sliderMutex, OS_OPT_POST_NONE, &err);
        OSMutexPend(&buttonStructMutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        // Use button data to calculate shot charge
        if (buttonStates.button0State == 1) { // Start charging
            charging = true;
        } else if (buttonStates.button0State == 0 && charging == true) { // Fire shot
            charging = false;
            if (gameData.shotCharge > 0) {
                for (int j = 0; j < 10; j++) {
                    if (localDataArray[j].objectType == empty) {
                        localDataArray[j].objectType = shot;
                        localDataArray[j].x = localDataArray[0].x;
                        localDataArray[j].y = 1;
                        localDataArray[j].xVel = (gameData.shotCharge / physConsts.generatorConst.maxShotPower) * 100 * cos(physConsts.railGunConst.railgunAngle * 3.14159 / 180);
                        localDataArray[j].yVel = -(gameData.shotCharge / physConsts.generatorConst.maxShotPower) * 100 * sin(physConsts.railGunConst.railgunAngle * 3.14159 / 180);
                        localDataArray[j].xAcc = 0;
                        localDataArray[j].yAcc = 0;
                        localDataArray[j].mass = physConsts.railGunConst.shotMass;
                        localDataArray[0].xForce += 50000;
                        gameData.shotCharge = 0;
                        gameData.shotsFired++;
                        break;
                    }
                }
            }
        } 
        // Use button data to calculate shield activation. Destroy all satchels in range        
        if (buttonStates.button1State == 1 && gameData.energy >= physConsts.shieldConst.shieldActivationEnergy && buttonStates.button1Change == 1) {
            gameData.energy -= physConsts.shieldConst.shieldActivationEnergy;
            gameData.shieldsActivated++;
            gameData.shieldActive = true;
            for (int i = 1; i < 10; i++) {
                if (localDataArray[i].objectType == satchel) {
                    int xDist = localDataArray[i].x - localDataArray[0].x;
                    xDist = abs(xDist);
                    int yDist = localDataArray[i].y - localDataArray[0].y;
                    yDist = abs(yDist);
                    int distance = sqrt(xDist * xDist + yDist * yDist);
                    if (distance <= physConsts.shieldConst.shieldEffectiveRange) {
                        clearPhysicsData(&localDataArray[i]);
                        gameData.usefulShields++;
                    }
                }
            }
        }
        OSMutexPost(&buttonStructMutex, OS_OPT_POST_NONE, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        // If charging, add charge and reduce energy. Otherwise, charge energy
        if (charging == true && gameData.energy > 0 && gameData.shotCharge < physConsts.generatorConst.maxShotPower) {
            gameData.shotCharge += physConsts.generatorConst.maxShotPower * (float)physConsts.physicsPeriod / 1.5 / 1000;
            gameData.energy -= physConsts.generatorConst.maxShotPower * (float)physConsts.physicsPeriod / 1.5 / 1000;
        } else if (charging == true && gameData.energy == 0) {
            // Do nothing
        } else if (charging == false && gameData.energy >= physConsts.generatorConst.energyCapacity) {
            gameData.energy = physConsts.generatorConst.energyCapacity;
        } else if (charging == false && gameData.energy <= physConsts.generatorConst.energyCapacity) {
            gameData.energy += physConsts.generatorConst.maxShotPower * (float)physConsts.physicsPeriod / 1.5 / 1000;
        }
        // Check if satchel should be spawned
        switch (physConsts.satchelConst.limitingMethod) {
          case AlwaysOne:
              // Check if there is a satchel in the array
              for (int i = 0; i < 10; i++) {
                  if (localDataArray[i].objectType == satchel) {
                      break;
                  } else if (i == 9) {
                      // If there is no satchel in the array, create one
                      for (int i = 0; i < 10; i++) {
                        if (localDataArray[i].objectType == empty) {
                            spawnSatchel(&localDataArray[i]);
                            break;
                        }
                      }
                  }
              }
              break;
          case MaxInFlight:
              // Check if # of satchels in flight is less than max
              if(0 == 1) {}; // Allow for compilation due to label error
              int satchelCount = 0;
              timer++;
              for (int i = 0; i < 10; i++) {
                  if (localDataArray[i].objectType == satchel) {
                      satchelCount++;
                  }
              }
              if (satchelCount >= physConsts.satchelConst.maxInFlight) {
                  timer = 0;
              }
              if (satchelCount < physConsts.satchelConst.maxInFlight && !(timer % physConsts.satchelConst.maxInFlightPeriod)) { // If there are less than max, create a satchel
                    for (int i = 0; i < 10; i++) {
                        if (localDataArray[i].objectType == empty) {
                            spawnSatchel(&localDataArray[i]);
                            break;
                        }
                    }
              }
              break;
          case PeriodicThrowTime:
              // Check if it is time to throw a satchel
              if(0 == 1) {}; // Allow for compilation due to label error
              if ((timer % physConsts.satchelConst.throwPeriod) == 0) {
                    for (int i = 0; i < 10; i++) {
                        if (localDataArray[i].objectType == empty) {
                            spawnSatchel(&localDataArray[i]);
                            break;
                        }
                    }
              }
              timer++;
              break;
          default:
              while (1) {}
              // Shouldn't be here
              break;
        }   
        // Unique physics calculations for each object type
        for (int i = 0; i < 10; i++) {
            if (localDataArray[i].objectType == shot) { // shot physics
                localDataArray[i].xAcc = localDataArray[i].xForce / localDataArray[i].mass;
                localDataArray[i].yAcc = localDataArray[i].yForce / localDataArray[i].mass + gravity;
                localDataArray[i].yVel += localDataArray[i].yAcc * ((float)physConsts.physicsPeriod / 1000);
                localDataArray[i].xVel += localDataArray[i].xAcc * ((float)physConsts.physicsPeriod / 1000);
                localDataArray[i].x += localDataArray[i].xVel * ((float)physConsts.physicsPeriod / 1000);
                localDataArray[i].y += localDataArray[i].yVel * ((float)physConsts.physicsPeriod / 1000);
                localDataArray[i].yForce = 0; // Forces aren't constant, so they need to be reset
                localDataArray[i].xForce = 0; // Forces aren't constant, so they need to be reset
                // Check if the shot has hit castle
                if (localDataArray[i].x <= 0  && localDataArray[i].y >= physConsts.castleConst.castleHeight && physDataArray[i].y <= physConsts.canyonSize) { // hit
                    clearPhysicsData(&localDataArray[i]);
                    gameData.foundationDamage++;
                    if (gameData.foundationDamage >= physConsts.castleConst.foundationHitsRequired) {
                        gameData.state = win;
                    }
                } else if (localDataArray[i].y <= 0) { // Destroy on ground
                    clearPhysicsData(&localDataArray[i]);
                } else if (localDataArray[i].x <= 0  && localDataArray[i].y < physConsts.castleConst.castleHeight){ // Destroy of below castle
                    clearPhysicsData(&localDataArray[i]);
                }
            } else if (localDataArray[i].objectType == satchel) { // satchel physics
                localDataArray[i].xAcc = 0;
                localDataArray[i].yAcc = gravity;
                localDataArray[i].yVel += localDataArray[i].yAcc * ((float)physConsts.physicsPeriod / 1000);
                localDataArray[i].x += localDataArray[i].xVel * ((float)physConsts.physicsPeriod / 1000);
                localDataArray[i].y += localDataArray[i].yVel * ((float)physConsts.physicsPeriod / 1000);
                if ((localDataArray[i].y + physConsts.satchelConst.satchelDisplayDiameter / 2) <= 0 && localDataArray[i].x > localDataArray[0].x - physConsts.platformConst.platformLength / 2 && localDataArray[i].x < localDataArray[0].x + physConsts.platformConst.platformLength / 2) { // Lose on hit
                    clearPhysicsData(&localDataArray[i]);
                    gameData.state = fail;
                } else if ((localDataArray[i].y + physConsts.satchelConst.satchelDisplayDiameter / 2) < 0) { // Destroy on ground
                    clearPhysicsData(&localDataArray[i]);
                } else if ((localDataArray[i].x + physConsts.satchelConst.satchelDisplayDiameter / 2) > physConsts.canyonSize){ // bounce off wall if hit
                    localDataArray[i].xVel = -localDataArray[i].xVel;
                    localDataArray[i].x = physConsts.canyonSize - (int)localDataArray[i].x % physConsts.canyonSize;
                }
            } else if (localDataArray[i].objectType == player) { // player physics
                localDataArray[i].xAcc = localDataArray[i].xForce / localDataArray[i].mass; // F = ma
                localDataArray[i].xVel += localDataArray[i].xAcc * ((float)physConsts.physicsPeriod / 1000);
                if (localDataArray[i].xVel > physConsts.platformConst.maxPlatformSpeed) {
                    localDataArray[i].xVel = physConsts.platformConst.maxPlatformSpeed;
                } else if (localDataArray[i].xVel < -physConsts.platformConst.maxPlatformSpeed) {
                    localDataArray[i].xVel = -physConsts.platformConst.maxPlatformSpeed;
                }
                localDataArray[i].x += localDataArray[i].xVel * ((float)physConsts.physicsPeriod / 1000);
                localDataArray[i].xForce = 0; // Forces aren't constant, so they need to be reset
                // Check if player hit wall, if so bounce
                if (localDataArray[i].x + physConsts.platformConst.platformLength / 2 < 0) {
                    localDataArray[i].xVel = -localDataArray[i].xVel;
                    localDataArray[i].x = localDataArray[i].x + 2*((int)(localDataArray[i].x + physConsts.platformConst.platformLength) % physConsts.canyonSize);
                } else if (localDataArray[i].x + physConsts.platformConst.platformLength / 2 > physConsts.canyonSize) {
                    localDataArray[i].xVel = -localDataArray[i].xVel;
                    localDataArray[i].x = localDataArray[i].x - 2*((int)(localDataArray[i].x + physConsts.platformConst.platformLength) % physConsts.canyonSize);
                } 
            }
        }
        OSMutexPend(&physicsStructMutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        // Copy local data to global data
        for (int i = 0; i < 10; i++) {
            physDataArray[i].mass= localDataArray[i].mass;
            physDataArray[i].x = localDataArray[i].x;
            physDataArray[i].y = localDataArray[i].y;
            physDataArray[i].xVel = localDataArray[i].xVel;
            physDataArray[i].yVel = localDataArray[i].yVel;
            physDataArray[i].xAcc = localDataArray[i].xAcc;
            physDataArray[i].yAcc = localDataArray[i].yAcc;
            physDataArray[i].xForce = localDataArray[i].xForce;
            physDataArray[i].yForce = localDataArray[i].yForce;
            physDataArray[i].objectType = localDataArray[i].objectType;
        }
        OSMutexPost(&physicsStructMutex, OS_OPT_POST_NONE, &err);
        while (err.Code != RTOS_ERR_NONE) {}
   }
   if (err.Code) {}
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
 *   Task that displays the game on the LCD. 
 ******************************************************************************/
void  LCDDisplayTask (void  *p_arg)
{
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR     err;
   struct __GLIB_Rectangle_t rectangles[7];
   struct __GLIB_Rectangle_t battery[6];
   struct __GLIB_Rectangle_t platform;
   int cannonLength = physConsts.platformConst.platformLength;
    while (DEF_TRUE) {
        OSTimeDlyHMSM(0, 0, 0, physConsts.lcdPeriod, OS_OPT_TIME_DLY, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        if (gameData.state == active) {
            GLIB_clear(&glibContext);
            // Generate cliff
            GLIB_drawLineV(&glibContext, 0, screenSize - physConsts.castleConst.castleHeight - physConsts.castleConst.foundationDepth, screenSize);
            GLIB_drawLineV(&glibContext, 1, screenSize - physConsts.castleConst.castleHeight - physConsts.castleConst.foundationDepth, screenSize);
            // Generate right wall
            GLIB_drawLineV(&glibContext, screenSize, 0, screenSize);
            GLIB_drawLineV(&glibContext, screenSize - 1, 0, screenSize - 1);
            // Generate castle
            // Left wall
             rectangles[0].xMin = 0;
             rectangles[0].xMax = physConsts.castleConst.foundationHitsRequired * 2;
             rectangles[0].yMin = 0;
             rectangles[0].yMax = screenSize - physConsts.castleConst.castleHeight;
            // Ceiling
             rectangles[1].xMin = 0;
             rectangles[1].xMax = 20;
             rectangles[1].yMin = 0;
             rectangles[1].yMax = 5;
            // Right wall
             rectangles[2].xMin = 15;
             rectangles[2].xMax = 20;
             rectangles[2].yMin = 0;
             rectangles[2].yMax = screenSize - physConsts.castleConst.castleHeight;
            // Floor
             rectangles[3].xMin = 0;
             rectangles[3].xMax = 20;
             rectangles[3].yMin = screenSize - physConsts.castleConst.castleHeight - 5;
             rectangles[3].yMax = screenSize - physConsts.castleConst.castleHeight;
            // Flag pole
             rectangles[4].xMin = 20;
             rectangles[4].xMax = 35;
             rectangles[4].yMin = 0;
             rectangles[4].yMax = 2;
            // Flag
             rectangles[5].xMin = 25;
             rectangles[5].xMax = 35;
             rectangles[5].yMin = 0;
             rectangles[5].yMax = screenSize - physConsts.castleConst.castleHeight - 10;
            // Generate Foundation
             rectangles[6].xMin = 0;
             rectangles[6].xMax = (physConsts.castleConst.foundationHitsRequired - gameData.foundationDamage) * 2;
             rectangles[6].yMin = screenSize - physConsts.castleConst.castleHeight;
             rectangles[6].yMax = screenSize - physConsts.castleConst.castleHeight + physConsts.castleConst.foundationDepth;
            // Draw castle
            for (int i = 0; i < 7; i++) {
                GLIB_drawRectFilled(&glibContext, &rectangles[i]);
            }
            // Generate platform
              platform.xMin = physDataArray[0].x - physConsts.platformConst.platformLength / 2;
              platform.xMax = physDataArray[0].x + physConsts.platformConst.platformLength / 2;
              platform.yMin = screenSize - 4;
              platform.yMax = screenSize;
            GLIB_drawRectFilled(&glibContext, &platform);
            // Draw Cannon 3 pixels thick
            for (int i = -1; i < 3; i++) {
                GLIB_drawLine(&glibContext, physDataArray[0].x + (int32_t)cannonLength * cos(physConsts.railGunConst.railgunAngle * 3.14159 / 180) + i, screenSize - 4 + (int32_t)(cannonLength * sin(physConsts.railGunConst.railgunAngle * 3.14159 / 180)), physDataArray[0].x + i, screenSize - 4);
            }
            // Draw Projectiles
            for (int i = 0; i < 10; i++) {
                if (physDataArray[i].objectType == satchel) {
                    GLIB_drawCircleFilled(&glibContext, physDataArray[i].x, screenSize - physDataArray[i].y, physConsts.satchelConst.satchelDisplayDiameter / 2);
                } else if (physDataArray[i].objectType == shot) {
                    GLIB_drawCircleFilled(&glibContext, physDataArray[i].x, screenSize - physDataArray[i].y, physConsts.railGunConst.shotRadius);
                }
            }
            // Draw Battery
            // Remaining battery
             battery[0].xMin = screenSize - 13;
             battery[0].xMax = screenSize - 8;
             battery[0].yMin = 31 - (gameData.energy / physConsts.generatorConst.energyCapacity * 20);
             battery[0].yMax = 32;
             // Left Battery wall
             battery[1].xMin = screenSize - 16;
             battery[1].xMax = screenSize - 15;
             battery[1].yMin = 10;
             battery[1].yMax = 35;
            // Top Battery
             battery[2].xMin = screenSize - 16;
             battery[2].xMax = screenSize - 5;
             battery[2].yMin = 10;
             battery[2].yMax = 11;
            // Right Battery wall
            battery[3].xMin = screenSize - 6;
            battery[3].xMax = screenSize - 5;
            battery[3].yMin = 10;
            battery[3].yMax = 35;
            // Bottom Battery
            battery[4].xMin = screenSize - 16;
            battery[4].xMax = screenSize - 5;
            battery[4].yMin = 34;
            battery[4].yMax = 35;
            // Battery bump
             battery[5].xMin = screenSize - 13;
             battery[5].xMax = screenSize - 8;
             battery[5].yMin = 5;
             battery[5].yMax = 10;
             for (int i = 0; i < 6; i++) {
                GLIB_drawRectFilled(&glibContext, &battery[i]);
             }
             if (gameData.shieldActive) { // Draw shield
                GLIB_drawCircle(&glibContext, physDataArray[0].x, screenSize - 4, physConsts.shieldConst.shieldEffectiveRange);
                gameData.shieldActive = false;
             }
             DMD_updateDisplay();
        } else if (gameData.state == fail) {
            GLIB_clear(&glibContext);
            GLIB_drawStringOnLine(&glibContext,
                                    "Game Over",
                                    0,
                                    GLIB_ALIGN_LEFT,
                                    5,
                                    5,
                                    true);
            GLIB_drawStringOnLine(&glibContext,
                                    "You Lost",
                                    2,
                                    GLIB_ALIGN_LEFT,
                                    5,
                                    15,
                                    true);
             DMD_updateDisplay();
        } else if (gameData.state == win) {
            GLIB_clear(&glibContext);
            GLIB_drawStringOnLine(&glibContext,
                        "Game Over",
                        0,
                        GLIB_ALIGN_LEFT,
                        5,
                        5,
                        true);
            if (!gameData.evacComplete) {
                GLIB_drawStringOnLine(&glibContext,
                                        "You Lost",
                                        2,
                                        GLIB_ALIGN_LEFT,
                                        5,
                                        15,
                                        true);
                GLIB_drawStringOnLine(&glibContext,
                                        "The prisoners",
                                        4,
                                        GLIB_ALIGN_LEFT,
                                        5,
                                        25,
                                        true);
                GLIB_drawStringOnLine(&glibContext,
                                        "failed to evac",
                                        5,
                                        GLIB_ALIGN_LEFT,
                                        5,
                                        30,
                                        true);
            } else {
                GLIB_drawStringOnLine(&glibContext,
                                        "You Won",
                                        2,
                                        GLIB_ALIGN_LEFT,
                                        5,
                                        15,
                                        true);
                GLIB_drawStringOnLine(&glibContext,
                                        "The prisoners",
                                        4,
                                        GLIB_ALIGN_LEFT,
                                        5,
                                        25,
                                        true);
                GLIB_drawStringOnLine(&glibContext,
                                        "have escaped",
                                        5,
                                        GLIB_ALIGN_LEFT,
                                        5,
                                        30,
                                        true);
            }
             DMD_updateDisplay();
        }
    }
}

#define  LED0_PRIO            21u  /*   Task Priority.                 */
#define  LED0_STK_SIZE       256u  /*   Stack size in CPU_STK.         */
OS_TCB   LED0TaskTCB;                            /*   Task Control Block.   */
CPU_STK  LED0TaskStk[LED0_STK_SIZE]; /*   Stack.  */
void LED0Task (void  *p_arg);
/***************************************************************************//**
 * @brief
 *   Creates the LED0 task.
 ******************************************************************************/
void  LED0TaskCreate (void)
{
    RTOS_ERR     err;

    OSTaskCreate(&LED0TaskTCB,                /* Pointer to the task's TCB.  */
                 "lED 0 Task.",                    /* Name to help debugging.     */
                 &LED0Task,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  LED0_PRIO,             /* Task's priority.            */
                 &LED0TaskStk[0],             /* Pointer to base of stack.   */
                 (LED0_STK_SIZE / 10u),  /* Stack limit, from base.     */
                  LED0_STK_SIZE,         /* Stack size, in CPU_STK.     */
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
 *   LED0Task. This task is responsible for blinking LED0 based on the charge in the railgun.
 ******************************************************************************/
void LED0Task (void  *p_arg)
{
    (void)&p_arg;
    RTOS_ERR err;
    static int counter;
    while (DEF_TRUE) {
        OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_DLY, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        counter++;
        if (gameData.shotCharge == 0) {
            continue;
        }
        if (!(counter % (10 - (int)(10 *(gameData.shotCharge / physConsts.generatorConst.maxShotPower))))) {
            GPIO_PinOutSet(LED0_port, LED0_pin);
        } else {
            GPIO_PinOutClear(LED0_port, LED0_pin);
        }
    }
}

#define  LED1_PRIO            21u  /*   Task Priority.                 */
#define  LED1_STK_SIZE       256u  /*   Stack size in CPU_STK.         */
OS_TCB   LED1TaskTCB;                            /*   Task Control Block.   */
CPU_STK  LED1TaskStk[LED1_STK_SIZE]; /*   Stack.  */
void LED1Task (void  *p_arg);
/***************************************************************************//**
 * @brief
 *   LED1TaskCreate. Creates the LED1Task
 ******************************************************************************/
void  LED1TaskCreate (void)
{
    RTOS_ERR     err;

    OSTaskCreate(&LED1TaskTCB,                /* Pointer to the task's TCB.  */
                 "lED 1 Task.",                    /* Name to help debugging.     */
                 &LED1Task,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  LED1_PRIO,             /* Task's priority.            */
                 &LED1TaskStk[0],             /* Pointer to base of stack.   */
                 (LED1_STK_SIZE / 10u),  /* Stack limit, from base.     */
                  LED1_STK_SIZE,         /* Stack size, in CPU_STK.     */
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
 *   LED1Task. This task is responsible for blinking LED1 based on the evacuation status.
 ******************************************************************************/
int evacTime = 5; // seconds
void LED1Task (void  *p_arg)
{
    (void)&p_arg;
    RTOS_ERR err;
    int counter = 0;
    bool LEDState = false;
    gameData.evacComplete = 0;
    while (DEF_TRUE) {
        while (gameData.state != active) {// stop when game not running. Used to block task
            OSSemPend(&gameEndSem, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
            while (err.Code != RTOS_ERR_NONE) {}
        }
        OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_DLY, &err);
        if (gameData.foundationDamage >= physConsts.castleConst.foundationHitsRequired * .5) {
            counter++;
            // Turn led on and off with 1 second period 50% duty cycle
            if (counter % 10 && gameData.evacComplete == 0) {
                if (LEDState) {
                    GPIO_PinOutClear(LED1_port, LED1_pin);
                    LEDState = false;
                } else {
                    GPIO_PinOutSet(LED1_port, LED1_pin);
                    LEDState = true;
                }
            } else {
                GPIO_PinOutSet(LED1_port, LED1_pin);
            }
            if (counter == evacTime * 20) {
                gameData.evacComplete = 1;
            }
            
        }
    }

}

/***************************************************************************//**
 * @brief
 *   Task that handles button interrupts. It posts a semaphore to the button task
 ******************************************************************************/

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
 *   Creates the buttonTask
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
 *   ButtonTask. This task is responsible for handling button presses and monitoring changes in button state.
 *  It also handles the button debugging.
 ******************************************************************************/
void  buttonTask (void  *p_arg)
{
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR     err;

   while (DEF_TRUE) {
       OSSemPend(&buttonSem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
       while (err.Code != RTOS_ERR_NONE) {}
#ifndef TEST_MODE
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
    //    if (GPIO_PinInGet(BUTTON1_port, BUTTON1_pin)) {
    //        GPIO_PinOutSet(LED1_port, LED1_pin);
    //    } else if (!GPIO_PinInGet(BUTTON1_port, BUTTON1_pin)){
    //        GPIO_PinOutClear(LED1_port, LED1_pin);
    //    }
    //    if (GPIO_PinInGet(BUTTON0_port, BUTTON0_pin)) {
    //        GPIO_PinOutSet(LED0_port, LED0_pin);
    //    } else if (!GPIO_PinInGet(BUTTON0_port, BUTTON0_pin)) {
    //        GPIO_PinOutClear(LED0_port, LED0_pin);
    //   }
#endif
    }
   if (err.Code) {}
}

#define  SLIDER_PRIO            20u  /*   Task Priority.                 */
#define  SLIDER_STK_SIZE       256u  /*   Stack size in CPU_STK.         */
OS_TCB   sliderTaskTCB;                            /*   Task Control Block.   */
CPU_STK  sliderTaskStk[SLIDER_STK_SIZE]; /*   Stack.  */
void  sliderTask (void  *p_arg);
/***************************************************************************//**
 * @brief
 *   Creates the sliderTask
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
 *   SliderTask. This task is responsible for handling slider presses. It also handles the slider debugging.
 ******************************************************************************/
void  sliderTask (void  *p_arg)
{
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR     err;
   // Initialize slider state struct
    while (DEF_TRUE) {
#ifndef TEST_MODE
        OSTimeDlyHMSM(0, 0, 0, physConsts.sliderPeriod, OS_OPT_TIME_DLY, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        OSMutexPend(&sliderMutex, OS_OPT_PEND_BLOCKING, 0, NULL, &err);
        CAPSENSE_Sense();
        // Map the capacitive touch sensor readings to the direction struct
        if (CAPSENSE_getPressed(0)) {
            sliderState.farLeft = 1;
        } else {
            sliderState.farLeft = 0;
        }
        if (CAPSENSE_getPressed(1)) {
            sliderState.left = 1;
        } else {
            sliderState.left = 0;
        }
        if (CAPSENSE_getPressed(2)) {
            sliderState.right = 1;
        } else {
            sliderState.right = 0;
        }
        if (CAPSENSE_getPressed(3)) {
            sliderState.farRight = 1;
        } else {
            sliderState.farRight = 0;
        }
        OSMutexPost(&sliderMutex, OS_OPT_POST_NONE, &err);
#endif
#ifdef TEST_MODE
        OSTimeDlyHMSM(0, 0, 0, physConsts.sliderPeriod, OS_OPT_TIME_DLY, &err);
        while (err.Code != RTOS_ERR_NONE) {}
        CAPSENSE_Sense();
        if (CAPSENSE_getPressed(0) || CAPSENSE_getPressed(1)) {
            GPIO_PinOutSet(LED1_port, LED1_pin);
        } else {
            GPIO_PinOutClear(LED1_port, LED1_pin);
        }
        if (CAPSENSE_getPressed(2) || CAPSENSE_getPressed(3)) {
            GPIO_PinOutSet(LED0_port, LED0_pin);
        } else {
            GPIO_PinOutClear(LED0_port, LED0_pin);
        }
#endif
        while (err.Code != RTOS_ERR_NONE) {}
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

/***************************************************************************//**
 * @brief
 *   Main function.
 ******************************************************************************/
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
  physConsts = physicsConstantsInit();

  // Mutex Creation
  OSMutexCreate(&buttonStructMutex, "button mutex", &err);
  while (err.Code != RTOS_ERR_NONE) {}
  OSMutexCreate(&sliderMutex, "Slider Mutex", &err);
  while (err.Code != RTOS_ERR_NONE) {}
  OSMutexCreate(&physicsStructMutex, "Physics Mutex", &err);
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
  OSSemCreate(&gameEndSem, "Game End Semaphore", 0, &err);
  while (err.Code != RTOS_ERR_NONE) {}

  // Task Creation
  idleTaskCreate();
  sliderTaskCreate();
  buttonTaskCreate();
#ifndef TEST_MODE
  physicsTaskCreate();
    LCDTaskCreate();
    LED0TaskCreate();
    LED1TaskCreate();
#endif

    // Start the OS
  OSStart(&err);
  while (err.Code != RTOS_ERR_NONE) {}

}
