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

#include "em_timer.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include <stdio.h>

#define SPEED_MEASURE

//#define POSITION_CAPTURE

#define CHANNEL_A_PIN 0
#define CHANNEL_A_PORT gpioPortB
#define CHANNEL_B_PIN 1
#define CHANNEL_B_PORT gpioPortB

#define TIMER_POSITION TIMER0
#define TIMER_POSITION_ID 0
#define TIMER_POSITION_IRQN TIMER0_IRQn
#define TIMER_POSITION_CLK cmuClock_TIMER0
#define TIMER_POSITION_IRQHANDLER TIMER0_IRQHandler

#ifdef SPEED_MEASURE

//#define SPEED_CAPTURE
#define INACTIVITY_TIME_MS 500ULL

#define TIMER_SPEED_MSK 0xFFFF
#define TIMER_SPEED TIMER1
#define TIMER_SPEED_ID 1
#define TIMER_SPEED_IRQN TIMER1_IRQn
#define TIMER_SPEED_CLK cmuClock_TIMER1
#define TIMER_SPEED_IRQHANDLER TIMER1_IRQHandler

// Most recent measured period in microseconds
static volatile uint32_t measuredPeriod;

// Stored edge from previous interrupt
static volatile uint32_t lastCapturedEdge;

void TIMER_SPEED_IRQHANDLER(void)
{
  // Acknowledge the interrupt
  uint32_t flags = TIMER_IntGet(TIMER_SPEED);
  TIMER_IntClear(TIMER_SPEED, flags);

  // Read the capture value from the CC register
  uint32_t current_edge = TIMER_CaptureGet(TIMER_SPEED, 0);

  // Check if the timer overflowed
  if (flags & TIMER_IF_OF) {
    printf("Rotation stop: position %d\r\n", TIMER_CounterGet(TIMER_POSITION));
  }

#ifdef SPEED_CAPTURE
  // Check if a capture event occurred
  if (flags & TIMER_IF_CC0) {
      printf("period %d\r\n", TIMER_CaptureBufGet(TIMER_SPEED, 0));
  }
#endif
}

void timer_speed_init(void)
{
  // Initialize the timer
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  // Configure TIMER_SPEED Compare/Capture for output compare
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
  CMU_ClockEnable(TIMER_SPEED_CLK, true);

  timerInit.prescale = timerPrescale1;
  timerInit.enable = false;
  timerInit.oneShot = true;
  timerInit.fallAction = timerInputActionReloadStart;
  timerCCInit.edge = timerEdgeBoth;
  timerCCInit.mode = timerCCModeCapture;
  timerCCInit.filter = true;

  uint32_t freq = CMU_ClockFreqGet(TIMER_SPEED_CLK);
  uint64_t top = 0;
  uint32_t presc = 1;

  printf("Freq %d Hz\r\n", freq);

  while(1)
    {
      top = (INACTIVITY_TIME_MS * freq)/(1000 * presc);

      if (top == 0)
        EFM_ASSERT(0);

      if (top & ~TIMER_SPEED_MSK)
        {
          if (presc == 1024)
          // Not able to find a valid prescaler
            {
              printf("Bad prescaler\r\n");
              EFM_ASSERT(0);
              return;
            }

          presc <<= 1;
          continue;
        }
      timerInit.prescale = presc - 1;
      printf("Presc %d\r\n", presc);
      printf("Top 0x%llx\r\n", top);
      break;
    }

  TIMER_Init(TIMER_SPEED, &timerInit);
  TIMER_InitCC(TIMER_SPEED, 0, &timerCCInit);
  TIMER_TopSet(TIMER_SPEED, top);
  TIMER_CounterSet(TIMER_SPEED, 0);

  // Route input pin to timer
  GPIO->TIMERROUTE[TIMER_SPEED_ID].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
  GPIO->TIMERROUTE[TIMER_SPEED_ID].CC0ROUTE = (CHANNEL_A_PORT << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                        | (CHANNEL_A_PIN << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

  // Enable TIMER_SPEED interrupts
  TIMER_IntClear(TIMER_SPEED, 0xFFFFFFFF);
  NVIC_ClearPendingIRQ(TIMER_SPEED_IRQN);
#ifdef SPEED_CAPTURE
  TIMER_IntEnable(TIMER_SPEED, TIMER_IEN_OF | TIMER_IEN_CC0);
#endif
  TIMER_IntEnable(TIMER_SPEED, TIMER_IEN_OF);
  NVIC_EnableIRQ(TIMER1_IRQn);
}

#endif


void TIMER_POSITION_IRQHANDLER(void)
{
  uint32_t flags = 0;
  uint32_t cnt = 0;

  flags = TIMER_IntGet(TIMER_POSITION);
  flags &= TIMER_IntGetEnabled(TIMER_POSITION);

  TIMER_IntClear(TIMER_POSITION, flags);

  cnt = TIMER_CounterGet(TIMER_POSITION);

  printf("if 0x%x, cnt %d\n\r", flags, cnt);
}

void timer_pos_init(void)
{
  // Initialize timer with largest prescale setting
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  // Configure TIMER Compare/Capture for input capture of PRS channel 0
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;

  // Init CMUs for peripherals
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(TIMER_POSITION_CLK, true);

  printf("TIMER_Init\n\r");

  // Init GPIOs
  GPIO_PinModeSet(CHANNEL_A_PORT, CHANNEL_A_PIN, gpioModeInput, 1);
  GPIO_PinModeSet(CHANNEL_B_PORT, CHANNEL_B_PIN, gpioModeInput, 1);

  timerInit.enable = false;
  timerInit.mode = timerModeQDec;

  timerCCInit.edge = timerEdgeBoth;
  timerCCInit.mode = timerCCModeCapture;
  timerCCInit.prsInput = false;
  timerCCInit.filter = true;


  TIMER_Init(TIMER_POSITION, &timerInit);

  TIMER_InitCC(TIMER_POSITION, 0, &timerCCInit);
  TIMER_InitCC(TIMER_POSITION, 1, &timerCCInit);
  TIMER_TopSet(TIMER_POSITION, 0xFFFF);
  TIMER_CounterSet(TIMER_POSITION, 0);

  // Route TIMER CC0 output to PA6
  GPIO->TIMERROUTE[TIMER_POSITION_ID].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CC1PEN;
  GPIO->TIMERROUTE[TIMER_POSITION_ID].CC0ROUTE = (CHANNEL_A_PORT << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                    | (CHANNEL_A_PIN << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  GPIO->TIMERROUTE[TIMER_POSITION_ID].CC1ROUTE = (CHANNEL_B_PORT << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT)
                    | (CHANNEL_B_PIN << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);


  // Enable TIMER interrupts for Capture/Compare on channel 0
  TIMER_IntClear(TIMER_POSITION, 0xFFFFFFFF);
  NVIC_ClearPendingIRQ(TIMER_POSITION_IRQN);

#ifdef POSITION_CAPTURE
  TIMER_IntEnable(TIMER_POSITION, (TIMER_IEN_CC0 | TIMER_IEN_DIRCHG));
#else
  TIMER_IntEnable(TIMER_POSITION, (TIMER_IEN_DIRCHG));
#endif
  NVIC_EnableIRQ(TIMER_POSITION_IRQN);

  // Enable the TIMER
  TIMER_Enable(TIMER_POSITION, true);

  printf("initTimer: done.\n\r");
}


void app_init(void)
{
  timer_pos_init();
#ifdef SPEED_MEASURE
  timer_speed_init();
#endif
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
}
