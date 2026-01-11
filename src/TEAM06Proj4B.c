#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdbool.h>
#include <stdio.h>
#include "UART2.h"
#include <stdlib.h>


#define BTN_PIN   13
#define TRIG_PIN  4
#define TRIG_PORT GPIOA
#define ECHO_PIN  0
#define ECHO_PORT GPIOB


#define servo8_pin (8)  // PC8 -> TIM3_CH3 (left)
#define servo9_pin (9)  // PC9 -> TIM3_CH4 (right)
#define SERVO_PORT (GPIOC)
#define SERVO3_PIN (6)  // PC6 -> TIM3_CH1 (sensor servo)
#define SERVO3_PORT GPIOC


volatile int  ssd_digit = 0;
const unsigned char DIGMAP[4] = {3, 2, 1, 0};


volatile bool running       = false;
volatile int  endline_count = 0;
volatile int  was_all_line  = 0;
volatile int  lost_count    = 0;
volatile int  steer_mode    = 0;


const int LOST_TICKS_REQUIRED = 40; // ~20 ms @ 0.5 ms tick


volatile int center_lock = 0;
const int CENTER_LOCK_TICKS = 60;


volatile int right_trig_cnt = 0;
volatile int left_trig_cnt  = 0;
const int TRIGGER_HYST = 6;


// snap to center
volatile int snap_ticks    = 0;
const int   SNAP_TICKS     = 40;


// timer for display
volatile uint32_t time_tenths   = 0;  // 0.1 second units, max 9999
volatile int      tick_count_0_1s = 0;


// ultrasonic / servo
volatile uint32_t pulse_width = 0;
volatile float dist = 0.0f;
volatile bool parking = false;
volatile bool scanning = false;
volatile bool scan_request = false;


int angle = 0;




void delay_us(uint32_t us) {
   uint32_t start = TIM5->CNT;
   while ((TIM5->CNT - start) < us) {
       // busy wait
   }
}


void delay_ms(uint32_t ms) {
   delay_us(ms * 1000);
}




float measure_distance_cm(void) {
   uint32_t startLocal;


   // Make sure TRIG low
   TRIG_PORT->ODR &= ~(1 << TRIG_PIN);
   delay_us(2);


   // 10us trigger pulse
   TRIG_PORT->ODR |= (1 << TRIG_PIN);
   delay_us(10);
   TRIG_PORT->ODR &= ~(1 << TRIG_PIN);


   // Wait for ECHO high (with timeout ~30ms)
   startLocal = TIM5->CNT;
   while (!(ECHO_PORT->IDR & (1 << ECHO_PIN))) {
       if ((TIM5->CNT - startLocal) > 30000) {
           return -1.0f; // timeout
       }
   }
   uint32_t echoStart_local = TIM5->CNT;


   // Wait for ECHO low (with timeout)
   while (ECHO_PORT->IDR & (1 << ECHO_PIN)) {
       if ((TIM5->CNT - echoStart_local) > 30000) {
           return -1.0f; // timeout
       }
   }
   uint32_t echoEnd_local = TIM5->CNT;


   uint32_t pw      = echoEnd_local - echoStart_local; // microseconds
   float    dist_cm = pw / 58.3f;                      // HC-SR04 formula


   return dist_cm;
}




// ultrasonic head servo on PC6 / TIM3_CH1
void servo_angle_set(int angle_deg) {
   // -45..+45 deg mapped to about 1000..2000us around 1500
   pulse_width = 1500 - (int)(500 * (angle_deg / 45.0f));
   if (pulse_width < 1000) pulse_width = 1000;
   if (pulse_width > 2000) pulse_width = 2000;
   TIM3->CCR1 = pulse_width;
}


// drive servos on PC8/PC9
void servo_set_both_us(int left_us, int right_us) { //Servo control functions
    if (left_us < 1000)  left_us = 1000;
    if (left_us > 2000)  left_us = 2000;
    if (right_us < 1000) right_us = 1000;
    if (right_us > 2000) right_us = 2000;

    TIM3->CCR3 = (uint32_t)left_us;
    TIM3->CCR4 = (uint32_t)right_us;
}

void servo_both_stop(void) {
    servo_set_both_us(1500, 1500);
}

void servo_both_forward(void) {
    servo_set_both_us(1500 + 70, 1500 - 82);  
}

void servo_turn_left(void) {
    servo_set_both_us(1500 + 20, 1500 - 90);
}

void servo_turn_right(void) {
    servo_set_both_us(1500 + 80, 1500 - 20);
}

void servo_pivot_right(void) {
    servo_set_both_us(1500 + 60, 1500); // left forward, right stop
}

void servo_pivot_left(void) {
    servo_set_both_us(1500, 1500 - 60); // left stop, right forward
}



void EXTI15_10_IRQHandler(void) {
   if (EXTI->PR & (1 << BTN_PIN)) {
       EXTI->PR |= (1 << BTN_PIN);   // clear pending


       if (running) {
           running = false;
           servo_both_stop();
       } else {
           endline_count = 0;
           was_all_line  = 0;
           lost_count    = 0;
           steer_mode    = 0;
           center_lock   = 0;
           right_trig_cnt = left_trig_cnt = 0;
           snap_ticks = 0;


           time_tenths     = 0;
           tick_count_0_1s = 0;


           scan_request = false;
           scanning     = false;
           parking      = false;


           running = true;
           servo_both_forward();
       }
   }
}




void SysTick_Handler(void) {
// We only scan when requested and robot is not currently line-following
if (scan_request && !running) {
   scan_request = false;
   scanning     = true;


   // --- Look RIGHT first (-30 deg) ---
   servo_angle_set(-35);
   delay_ms(1000);
   float distR = measure_distance_cm();


   // --- Look LEFT (+30 deg) ---
   servo_angle_set(35);
   delay_ms(1000);
   float distL = measure_distance_cm();


   // Return the servo to center
   servo_angle_set(0);
   delay_ms(1000);


   bool parkLeft = false;
   bool parkRight = false;


   // If both fail → stop
   if (distL < 0 && distR < 0) {
       servo_both_stop();
   }
   // MORE SPACE **RIGHT**
   else if (distR < distL) {
       parkRight = true;
   }
   // MORE SPACE **LEFT**
   else {
       parkLeft = true;
   }


   // PARKING MANEUVERS
   if (parkRight) {
       servo_turn_right();
       delay_ms(1450);     // adjust timing
       servo_both_forward();
       delay_ms(2000);     // adjust timing
       // servo_both_stop();
   }
   else if (parkLeft) {
       servo_turn_left();
       delay_ms(1750);      // adjust timing
       servo_both_forward();
       delay_ms(2000);     // adjust timing
       // servo_both_stop();
   }


   scanning = false;
   }
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;

        // 0.5 ms ticks → update timer
        if (running) {
            if (++tick_count_0_1s >= 200) {  // 200 * 0.5ms = 100ms
                tick_count_0_1s = 0;
                if (time_tenths < 9999) {
                    time_tenths++;
                }
            }
        }

        // Read IR sensors (active-low → invert)
        unsigned int idr = GPIOC->IDR;
        int b0 = ((idr >> 0) & 1) ^ 1;
        int b1 = ((idr >> 1) & 1) ^ 1;
        int b2 = ((idr >> 2) & 1) ^ 1;
        int b3 = ((idr >> 3) & 1) ^ 1;

        int sum = b0 + b1 + b2 + b3;

        // NEW CHECK: detect real black vs white
        int on_black = (b0 || b1 || b2 || b3);

        // ----- PARKING -----
        if (!running && parking && sum == 4) {
            servo_both_stop();
            parking  = false;
            scanning = false;
            return;
        }

        // ----- SSD update -----
        int value = (int)time_tenths;
        SSD_update(DIGMAP[ssd_digit], value, 3);
        ssd_digit = (ssd_digit + 1) % 4;

        if (running) {

            // -------------------------------
            // FIXED 1111 ENDLINE / BAR LOGIC
            // Only treat 1111 as a bar if ON BLACK
            // -------------------------------
            if (sum == 4 && on_black) {

                if (!was_all_line) {
                    was_all_line = 1;
                    endline_count++;

                    if (endline_count >= 3) {
                        running = false;
                        servo_both_stop();
                        return;
                    }
                }

                // First bar: go straight
                steer_mode = 0;
                snap_ticks = 0;
                right_trig_cnt = left_trig_cnt = 0;
                center_lock = 0;
                servo_both_forward();
                lost_count = 0;
                return;
            } else {
                was_all_line = 0;
            }

            // Patterns
            int centered = (b3==0 && b2==1 && b1==1 && b0==0); // 0110
            int near_ctr = centered ||
                           (b3==0 && b2==1 && b1==0 && b0==0) || // 0100
                           (b3==0 && b2==0 && b1==1 && b0==0);   // 0010

            int slight_right = ((b3==1 && b2==1 && b1==0 && b0==0) || // 1100
                                (b3==0 && b2==0 && b1==0 && b0==1));  // 0001

            int slight_left  = ((b3==0 && b2==0 && b1==1 && b0==1) || // 0011
                                (b3==1 && b2==0 && b1==0 && b0==0));  // 1000

            int right_hard = (b3==1 && b2==1 && b1==1 && b0==0); // 1110
            int left_hard  = (b3==0 && b2==1 && b1==1 && b0==1); // 0111

            // persistence counters
            if (right_hard) right_trig_cnt++;
            else            right_trig_cnt = 0;

            if (left_hard)  left_trig_cnt++;
            else            left_trig_cnt  = 0;

            // --- LATCHED MODES ---
            if (steer_mode == 1) {
                if (centered) {
                    steer_mode = 3;
                    snap_ticks = SNAP_TICKS;
                    servo_pivot_left();
                } else servo_pivot_right();
                lost_count = 0;
            }

            else if (steer_mode == 2) {
                if (centered) {
                    steer_mode = 4;
                    snap_ticks = SNAP_TICKS;
                    servo_pivot_right();
                } else servo_pivot_left();
                lost_count = 0;
            }

            else if (steer_mode == 3) {
                if (snap_ticks-- > 0) servo_pivot_left();
                else {
                    steer_mode = 0;
                    servo_both_forward();
                }
                lost_count = 0;
            }

            else if (steer_mode == 4) {
                if (snap_ticks-- > 0) servo_pivot_right();
                else {
                    steer_mode = 0;
                    servo_both_forward();
                }
                lost_count = 0;
            }

            else {
                // --- NOT LATCHED ---
                if (right_trig_cnt >= TRIGGER_HYST) {
                    steer_mode = 1;
                    servo_pivot_right();
                    right_trig_cnt = 0;
                    lost_count = 0;
                }
                else if (left_trig_cnt >= TRIGGER_HYST) {
                    steer_mode = 2;
                    servo_pivot_left();
                    left_trig_cnt = 0;
                    lost_count = 0;
                }
                else if (near_ctr) {
                    servo_both_forward();
                    lost_count = 0;
                }
                else if (slight_right) {
                    servo_turn_right();
                    lost_count = 0;
                }
                else if (slight_left) {
                    servo_turn_left();
                    lost_count = 0;
                }
                else {
                    // LOST → stop and request ultrasonic
                    if (++lost_count >= LOST_TICKS_REQUIRED) {
                        servo_both_stop();
                        lost_count = LOST_TICKS_REQUIRED;

                        running   = false;
                        parking   = true;
                        scanning  = true;
                        scan_request = true;
                    }
                }
            }
        }
    }
}


int main(void) {
   SSD_init();


   angle = 0;


   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
   RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
   RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; 
   RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;         


   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;


   TRIG_PORT->MODER &= ~(3 << (TRIG_PIN * 2));
   TRIG_PORT->MODER |=  (1 << (TRIG_PIN * 2));    // TRIG output


   ECHO_PORT->MODER &= ~(3 << (ECHO_PIN * 2));    // ECHO input


   RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
   TIM5->PSC = 15;             // 16MHz / 16 = 1MHz (1us)
   TIM5->ARR = 0xFFFFFFFF;
   TIM5->EGR = TIM_EGR_UG;
   TIM5->CR1 = TIM_CR1_CEN;


   SysTick_Config(16000000);


   GPIOC->MODER &= ~(0x3 << (SERVO3_PIN * 2));
   GPIOC->MODER |=  (0x2 << (SERVO3_PIN * 2));
   GPIOC->AFR[0] &= ~(0xF << (SERVO3_PIN * 4));
   GPIOC->AFR[0] |=  (0x2 << (SERVO3_PIN * 4)); // AF2 = TIM3


   GPIOC->MODER &= ~(3 << (13 * 2));
   GPIOC->PUPDR &= ~(3 << (13 * 2));
   GPIOC->PUPDR |=  (1 << (13 * 2));


   for (int pin = 0; pin <= 3; ++pin) {
       GPIOC->MODER &= ~(3 << (pin * 2));
       GPIOC->PUPDR &= ~(3 << (pin * 2));
       GPIOC->PUPDR |=  (1 << (pin * 2));
   }


   GPIOC->MODER &= ~(3 << (servo8_pin * 2));
   GPIOC->MODER |=  (2 << (servo8_pin * 2));
   GPIOC->MODER &= ~(3 << (servo9_pin * 2));
   GPIOC->MODER |=  (2 << (servo9_pin * 2));


   GPIOC->AFR[1] &= ~((0xF << ((servo8_pin - 8) * 4)) | (0xF << ((servo9_pin - 8) * 4)));
   GPIOC->AFR[1] |=  (0x2 << ((servo8_pin - 8) * 4)) | (0x2 << ((servo9_pin - 8) * 4)); // AF2 = TIM3


   TIM3->PSC  = 15;        // 1 MHz
   TIM3->ARR  = 19999;     // 20ms


   // CH1 (PC6) PWM
   TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;
   TIM3->CCMR1 |=  (6 << TIM_CCMR1_OC1M_Pos);
   TIM3->CCMR1 |=  TIM_CCMR1_OC1PE;


   // CH3 & CH4 (PC8, PC9)
   TIM3->CCMR2 = 0;
   TIM3->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;


   TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3E | TIM_CCER_CC4E;


   // Center all servos
   TIM3->CCR1 = 1500;   // ultrasonic servo center
   TIM3->CCR3 = 1500;   // left neutral
   TIM3->CCR4 = 1500;   // right neutral


   TIM3->CR1 |= TIM_CR1_CEN;


   TIM2->PSC = 15;          // 1 MHz
   TIM2->ARR = 500 - 1;     // 0.5ms
   TIM2->DIER |= TIM_DIER_UIE;
   TIM2->SR &= ~TIM_SR_UIF;
   NVIC_SetPriority(TIM2_IRQn, 1);
   NVIC_EnableIRQ(TIM2_IRQn);
   TIM2->CR1 |= TIM_CR1_CEN;


   EXTI->IMR  |= (1 << BTN_PIN);
   EXTI->FTSR |= (1 << BTN_PIN);
   SYSCFG->EXTICR[3] &= ~(0xF << (1 * 4));
   SYSCFG->EXTICR[3] |=  (2 << (1 * 4)); // PC13
   EXTI->PR |= (1 << BTN_PIN);
   NVIC_SetPriority(EXTI15_10_IRQn, 0);
   NVIC_EnableIRQ(EXTI15_10_IRQn);


   servo_both_stop();


   while (1) {}
}