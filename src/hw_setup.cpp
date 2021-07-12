
#include "mbed.h"
#include "hw_setup.h"
#include "hw_config.h"
#include "structs.h"
#include "FastPWM.h"

void Init_PWM(GPIOStruct *gpio){

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;                        // enable the clock to GPIOC
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;                       // enable TIM2 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;                         // enable TIM1 clock

    GPIOC->MODER |= (1 << (2*2));                               // set pin c2,c3 to be general purpose output for LED
    GPIOC->MODER |= (1 << (3*2));                  

    gpio->enable = new DigitalOut(ENABLE_PIN);
    gpio->pwm_u = new FastPWM(PIN_U);
    gpio->pwm_v = new FastPWM(PIN_V);
    gpio->pwm_w = new FastPWM(PIN_W);
    
    
    
     //ISR Setup     
    
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);                         //Enable TIM1 IRQ

    TIM1->DIER |= TIM_DIER_UIE;                                 // enable update interrupt
    TIM1->CR1 = 0x40;                                           // CMS = 10, interrupt only when counting up
    TIM1->CR1 |= TIM_CR1_UDIS;
    TIM1->CR1 |= TIM_CR1_ARPE;                                  // autoreload on, 
    TIM1->RCR |= 0x001;                                         // update event once per up/down count of tim1 
    TIM1->EGR |= TIM_EGR_UG;
 
    //PWM Setup

    TIM1->PSC = 0x0;                                            // no prescaler, timer counts up in sync with the peripheral clock
    TIM1->ARR = PWM_ARR;                                          // set auto reload, 40 khz
    TIM1->CCER |= ~(TIM_CCER_CC1NP);                            // Interupt when low side is on.
    TIM1->CR1 |= TIM_CR1_CEN;                                   // enable TIM1
    
    }

void Init_ADC(void){
    // ADC Setup
     RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;                       // clock for ADC3 & 4 & 5
     RCC->APB2ENR |= RCC_AHB2ENR_ADC12EN;                        // clock for ADC1 & 2                     
     
     RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;                         // Enable clock for GPIOC
     RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;                        // Enable clock for GPIOA
     RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
         
     ADC12_COMMON->CCR |= 0x00000006;                           // Regular simultaneous mode only
     ADC345_COMMON->CCR |= 0x00000006;                          // Regular simultaneous mode only

     ADC1->CR &= ~ADC_CR_DEEPPWD; //clear Deep-power-down enable bit
     ADC2->CR &= ~ADC_CR_DEEPPWD;
     ADC3->CR &= ~ADC_CR_DEEPPWD;

     ADC1->CR |= ADC_CR_ADVREGEN; // enable ADC voltage regulator 
     ADC2->CR |= ADC_CR_ADVREGEN;
     ADC3->CR |= ADC_CR_ADVREGEN;

     //wait_us(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US); //wait for regulator start-up time

     ADC1->ISR |= ADC_ISR_ADRDY; //clear adrdy bits [sic, it is cleared by writing a '1' according to ref. manual]
     ADC2->ISR |= ADC_ISR_ADRDY;
     ADC3->ISR |= ADC_ISR_ADRDY;

     ADC1->CR |= ADC_CR_ADEN;
     ADC2->CR |= ADC_CR_ADEN;
     ADC3->CR |= ADC_CR_ADEN;



     ADC1->SQR3 = 0x0000006;                                    // use PC_0 as input- ADC1_IN6
     ADC2->SQR3 = 0x00000007;                                   // use PC_1 as input - ADC2_IN7
     ADC3->SQR3 = 0x0000000C;                                   // use PB_0, - ADC3_IN12
     GPIOC->MODER |= 0x0000000f;                                // Alternate function, PC_0, PC_1 are analog inputs 
     GPIOB->MODER |= 0x3;                                       // PB_0 as analog input
     
     ADC1->SMPR1 |= (3 << (6*3));                                  // 24.5 cycles on CH_6 
     ADC2->SMPR1 |= (3 << (7*3));                                  // 24.5 cycles on CH_7
     ADC3->SMPR2 |= (7 << (2*3));                                  // 640 cycles on CH_12 (Increased due to scaled-up voltage divider)

    

    }


void Init_All_HW(GPIOStruct *gpio){
    Init_PWM(gpio);
    Init_ADC();
    gpio->led1 = new DigitalOut(LED_1);
    gpio->led2 = new DigitalOut(LED_2);
    
    }