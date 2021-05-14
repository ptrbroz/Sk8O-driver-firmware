/// high-bandwidth 3-phase motor control, for robots
/// Written by benkatz, with much inspiration from Bayley Wang, Nick Kirkby, Shane Colton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com
/// Written for the STM32F446, but can be implemented on other STM32 MCU's with some further register-diddling
/// Version for the TI DRV8323 Everything Chip


#define REST_MODE           0
#define CALIBRATION_MODE    1
#define MOTOR_MODE          2
#define SETUP_MODE          4
#define ENCODER_MODE        5
#define INIT_TEMP_MODE      6

#define VERSION_NUM "1.10"


float __float_reg[64];                                                          // Floats stored in flash
int __int_reg[256];                                                             // Ints stored in flash.  Includes position sensor calibration lookup table

#include "mbed.h"
#include "PositionSensor.h"
#include "structs.h"
#include "foc.h" 
#include "calibration.h"
#include "hw_setup.h"
#include "math_ops.h" 
#include "current_controller_config.h"
#include "hw_config.h"
#include "motor_config.h"
#include "stm32g4xx.h"
//#include "stm32f4xx_flash.h"
//#include "FlashWriter.h"
#include "user_config.h"
//#include "PreferenceWriter.h"
#include "CAN_com.h"
#include "DRV.h"
#include "FlashAccess.h"

 
//PreferenceWriter prefs(6);

GPIOStruct gpio;
ControllerStruct controller;
ObserverStruct observer;
COMStruct com;

UnbufferedSerial pc(PA_2, PA_3);


CAN          can(PB_8, PB_9, 1000000);      // CAN Rx pin name, CAN Tx pin name
CANMessage   rxMsg;
CANMessage   txMsg;


SPI drv_spi(PA_7, PA_6, PA_5);
DigitalOut drv_cs(PA_4);
//DigitalOut drv_en_gate(PA_11);
DRV832x drv(&drv_spi, &drv_cs);

PositionSensorAM5147 spi(16384, 0.0, NPP);  

volatile int count_sm = 0; //state machine counter. Renamed from count to avoid conflict
volatile int state = REST_MODE;
volatile int state_change;

void onMsgReceived() {
    //msgAvailable = true;
    //printf("%d\n\r", rxMsg.id);
    can.read(rxMsg);  
    if(((int)rxMsg.id == CAN_ID)){
        controller.timeout = 0;
        if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) & (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFC))){
            state = MOTOR_MODE;
            state_change = 1;
            }
        else if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFD))){
            state = REST_MODE;
            state_change = 1;
            gpio.led1->write(0);; 
            }
        else if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFE))){
            spi.ZeroPosition();
            }
        else if(state == MOTOR_MODE){
            unpack_cmd(rxMsg, &controller);
            }
        pack_reply(&txMsg, controller.theta_mech, controller.dtheta_mech, controller.i_q_filt*KT_OUT);
        can.write(txMsg);
        }
    
}

void enter_menu_state(void){
    drv.disable_gd();
    reset_foc(&controller); 
    //gpio.enable->write(0);
    printf("\n\r\n\r\n\r");
    printf(" Commands:\n\r");
    wait_us(10);
    printf(" m - Motor Mode\n\r");
    wait_us(10);
    printf(" c - Calibrate Encoder\n\r");
    wait_us(10);
    printf(" s - Setup\n\r");
    wait_us(10);
    printf(" e - Display Encoder\n\r");
    wait_us(10);
    printf(" z - Set Zero Position\n\r");
    wait_us(10);
    printf(" esc - Exit to Menu\n\r");
    wait_us(10);
    state_change = 0;
    gpio.led1->write(0);
    }

void enter_setup_state(void){
    printf("\n\r\n\r Configuration Options \n\r\n\n");
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %-2s\n\r\n\r", "prefix", "parameter", "min", "max", "current value");
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "b", "Current Bandwidth (Hz)", "100", "2000", I_BW);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "i", "CAN ID", "0", "127", CAN_ID);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "m", "CAN Master ID", "0", "127", CAN_MASTER);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "l", "Current Limit (A)", "0.0", "40.0", I_MAX);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "f", "FW Current Limit (A)", "0.0", "33.0", I_FW_MAX);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %d\n\r", "t", "CAN Timeout (cycles)(0 = none)", "0", "100000", CAN_TIMEOUT);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "h", "Temp Cutoff (C) (0 = none)", "0", "150", TEMP_MAX);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "c", "Continuous Current (A)", "0", "40.0", I_MAX_CONT);
    wait_us(10);
    printf("\n\r To change a value, type 'prefix''value''ENTER'\n\r i.e. 'b1000''ENTER'\n\r\n\r");
    wait_us(10);
    state_change = 0;
    }
    
void enter_torque_mode(void){
    drv.enable_gd();
    //gpio.enable->write(1);
    controller.ovp_flag = 0;
    reset_foc(&controller);                                                     // Tesets integrators, and other control loop parameters
    wait_us(1000);
    controller.i_d_ref = 0;
    controller.i_q_ref = 0;                                                     // Current Setpoints
    gpio.led1->write(1);                                                     // Turn on status LED
    state_change = 0;
    printf("\n\r Entering Motor Mode \n\r");
    }
    
void calibrate(void){
    drv.enable_gd();
    //gpio.enable->write(1);
    gpio.led1->write(1);                                                    // Turn on status LED
    order_phases(&spi, &gpio, &controller);                             // Check phase ordering
    calibrate(&spi, &gpio, &controller);                                // Perform calibration procedure
    gpio.led1->write(0);;                                                     // Turn off status LED
    wait_us(50000);
    R_NOMINAL = 0;
    state = INIT_TEMP_MODE;
    //printf("\n\r Calibration complete.  Press 'esc' to return to menu\n\r");
    //drv.disable_gd();
    //state_change = 0;
     
    }
    
void print_encoder(void){
    printf(" Mechanical Angle:  %f    Electrical Angle:  %f    Raw:  %d\n\r", spi.GetMechPosition(), spi.GetElecPosition(), spi.GetRawPosition());
//    printf("%f \n\r", spi.GetMechVelocity());
    //printf("%d\n\r", spi.GetRawPosition());
    wait_us(1000);
    }

/// Current Sampling Interrupt ///
/// This runs at 40 kHz, regardless of of the mode the controller is in ///
//float testing[1000];
//float testing2[1000];
extern "C" void TIM1_UP_TIM10_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF ) {
        //gpio.led1->write(1);
        ///Sample current always ///
        //ADC1->CR2  |= 0x40000000;     
        ADC1->CR  |= ADC_CR_ADSTART;                                           // Begin sample and conversion
        //volatile int delay;   
        //for (delay = 0; delay < 55; delay++);
        
        spi.Sample(DT);                                                           // sample position sensor
        /*
        if(count_sm < 10){printf("%d\n\r", spi.GetRawPosition());}
        count_sm ++;
        */        
        controller.adc2_raw = ADC2->DR;                                         // Read ADC Data Registers
        controller.adc1_raw = ADC1->DR;
        controller.adc3_raw = ADC3->DR;
        controller.theta_elec = spi.GetElecPosition();
        controller.theta_mech = (1.0f/GR)*spi.GetMechPosition();
        controller.dtheta_mech = (1.0f/GR)*spi.GetMechVelocity();  
        controller.dtheta_elec = spi.GetElecVelocity();
        controller.v_bus = 0.95f*controller.v_bus + 0.05f*((float)controller.adc3_raw)*V_SCALE; //filter the dc link voltage measurement
        

        
        ///
        
        /// Check state machine state, and run the appropriate function ///
        switch(state){
            case REST_MODE:                                                     // Do nothing
                if(state_change){
                    enter_menu_state();
                    }
                update_observer(&controller, &observer);
                break;
            
            case CALIBRATION_MODE:                                              // Run encoder calibration procedure
                if(state_change){
                    calibrate();
                    }
                break;
            case INIT_TEMP_MODE:
                if(state_change){
                    enter_torque_mode();
                    count_sm = 0;
                    observer.resistance = 0.0f;
                    } 
                controller.i_d_ref = -10.0f;
                controller.i_q_ref = 0.0f;
                commutate(&controller, &observer, &gpio, controller.theta_elec); 

                if(count_sm > 200)
                {
                    float r_meas = controller.v_d*(DTC_MAX-DTC_MIN)/(controller.i_d*SQRT3);
                    //testing2[count_sm-100] = controller.i_d;
                    observer.resistance += .001f*r_meas;
                }
                if(count_sm > 1200)
                {
                    count_sm = 0;
                    state = REST_MODE;
                    state_change = 1;
                    gpio.led1->write(0);
                    observer.temperature = (double)(T_AMBIENT + ((observer.resistance/R_NOMINAL) - 1.0f)*254.5f);
                    printf("Winding Resistance:  %f\n\r", observer.resistance);
                    printf("Winding Temperature:  %f\n\r", observer.temperature);
                    
                    if(R_NOMINAL==0)
                    {
                        printf("Saving winding resistance\n\r");
                        R_NOMINAL = observer.resistance;
                        saveToFlash();
                        /*if (!prefs.ready()) prefs.open();
                        prefs.flush();                                                         // write offset and lookup table to flash
                        prefs.close();*/
                    }
                    //for(int i = 0; i<1000; i++){printf("%f \n\r", testing[i]);}
                }
                
                count_sm++; 
                break;
            case MOTOR_MODE:                                                   // Run torque control
                if(state_change){
                    enter_torque_mode();
                    count_sm = 0;
                    }
                else{
                /*
                if(controller.v_bus>28.0f){         //Turn of gate drive if bus voltage is too high, to prevent FETsplosion if the bus is cut during regen
                    gpio.
                    ->write(0);
                    controller.ovp_flag = 1;
                    state = REST_MODE;
                    state_change = 1;
                    printf("OVP Triggered!\n\r");
                    }
                    */  

                if((controller.timeout > CAN_TIMEOUT) && (CAN_TIMEOUT > 0)){
                    controller.i_d_ref = 0;
                    controller.i_q_ref = 0;
                    controller.kp = 0;
                    controller.kd = 0;
                    controller.t_ff = 0;
                    } 
    
                torque_control(&controller);
                update_observer(&controller, &observer);
                field_weaken(&controller);
                commutate(&controller, &observer, &gpio, controller.theta_elec);           // Run current loop
                controller.timeout++;

                if(controller.otw_flag)
                { 
                    state = REST_MODE;
                    state_change = 1;
                    gpio.led1->write(0);
                }
                
                count_sm++; 
                }   
                
                  
                break;
            
            case SETUP_MODE:
                if(state_change){
                    enter_setup_state();
                }
                break;
            case ENCODER_MODE:
                print_encoder();
                break;
                }                 
      }
      //gpio.led1->write(0);
  TIM1->SR = 0x0;                                                               // reset the status register
}


char cmd_val[8] = {0};
char cmd_id = 0;
char char_count = 0;

/// Manage state machine with commands from serial terminal or configurator gui ///
/// Called when data received over serial ///
void serial_interrupt(void){
    while(pc.readable()){
        char c;
        pc.read(&c, 1);
        if(c == 27){
                state = REST_MODE;
                state_change = 1;
                char_count = 0;
                cmd_id = 0;
                gpio.led1->write(0);;
                for(int i = 0; i<8; i++){cmd_val[i] = 0;}
                }
        if(state == REST_MODE){
            switch (c){
                case 'c':
                    state = CALIBRATION_MODE;
                    state_change = 1;
                    break;
                case 'm':
                    state = MOTOR_MODE;
                    state_change = 1;
                    break;
                case 'e':
                    state = ENCODER_MODE;
                    state_change = 1;
                    break;
                case 's':
                    state = SETUP_MODE;
                    state_change = 1;
                    break;
                case 'z':
                    spi.SetMechOffset(0);
                    spi.Sample(DT);
                    wait_us(20);
                    M_OFFSET = spi.GetMechPosition();
                    saveToFlash();
                    /*if (!prefs.ready()) prefs.open();
                        prefs.flush();                                                  // Write new prefs to flash
                        prefs.close(); 
                        prefs.load(); */
                    loadFromFlash();
                    spi.SetMechOffset(M_OFFSET);
                    printf("\n\r  Saved new zero position:  %.4f\n\r\n\r", M_OFFSET);
                    
                    break;
                }
                
                }
        else if(state == SETUP_MODE){
            if(c == 13){
                switch (cmd_id){
                    case 'b':
                        I_BW = fmaxf(fminf(atof(cmd_val), 2000.0f), 100.0f);
                        break;
                    case 'i':
                        CAN_ID = atoi(cmd_val);
                        break;
                    case 'm':
                        CAN_MASTER = atoi(cmd_val);
                        break;
                    case 'l':
                        I_MAX = fmaxf(fminf(atof(cmd_val), 40.0f), 0.0f);
                        break;
                    case 'f':
                        I_FW_MAX = fmaxf(fminf(atof(cmd_val), 33.0f), 0.0f);
                        break;
                    case 't':
                        CAN_TIMEOUT = atoi(cmd_val);
                        break;
                    case 'h':
                        TEMP_MAX = fmaxf(fminf(atof(cmd_val), 150.0f), 0.0f);
                        break;
                    case 'c':
                        I_MAX_CONT = fmaxf(fminf(atof(cmd_val), 40.0f), 0.0f);
                        break;
                    default:
                        printf("\n\r '%c' Not a valid command prefix\n\r\n\r", cmd_id);
                        break;
                    } 
                /*
                if (!prefs.ready()) prefs.open();
                prefs.flush();                                                  // Write new prefs to flash
                prefs.close();    
                prefs.load();
                */  
                saveToFlash();   
                loadFromFlash();                                         
                state_change = 1;
                char_count = 0;
                cmd_id = 0;
                for(int i = 0; i<8; i++){cmd_val[i] = 0;}
                }
            else{
                if(char_count == 0){cmd_id = c;}
                else{
                    cmd_val[char_count-1] = c;
                    
                }
                pc.write(&c, 1);
                char_count++;
                }
            }
        else if (state == ENCODER_MODE){
            switch (c){
                case 27:
                    state = REST_MODE;
                    state_change = 1;
                    break;
                    }
            }
        else if (state == MOTOR_MODE){
            switch (c){
                case 'd':
                    controller.i_q_ref = 0;
                    controller.i_d_ref = 0;
                }
            }
            
        }
    }
       
int main() {
    controller.v_bus = V_BUS;
    controller.mode = 0;
    Init_All_HW(&gpio);                                                         // Setup PWM, ADC, GPIO
    wait_us(100);
    
    gpio.enable->write(1);
    wait_us(100);
    drv.calibrate();
    wait_us(100);
    drv.write_DCR(0x0, DIS_GDF_DIS, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
    wait_us(100);
    drv.write_CSACR(0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x1, 0x1, 0x1, SEN_LVL_1_0);   // calibrate shunt amplifiers
    wait_us(100);
    zero_current(&controller.adc1_offset, &controller.adc2_offset); 
    wait_us(100);
    drv.write_CSACR(0x0, 0x1, 0x0, CSA_GAIN_40, 0x1, 0x0, 0x0, 0x0, SEN_LVL_1_0);
    wait_us(100);
    drv.write_OCPCR(TRETRY_50US, DEADTIME_50NS, OCP_NONE, OCP_DEG_8US, VDS_LVL_1_50); //VDS down to 1.5V from original 1.88. Still seems pretty high? (VDS monitor trip @ hundreads of amps current??)
    
    //drv.enable_gd();
    drv.disable_gd();
    //zero_current(&controller.adc1_offset, &controller.adc2_offset);             // Measure current sensor zero-offset
    //drv.enable_gd();

    wait_us(100);
    
    reset_foc(&controller);                                                     // Reset current controller
    reset_observer(&observer);                                                 // Reset observer
    //TIM1->CR1 |= TIM_CR1_UDIS; //enable interrupt
    
    wait_us(100);
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 2);                                             // commutation > communication
    NVIC_SetPriority(FDCAN2_IT0_IRQn, 3);
                                   // attach 'CAN receive-complete' interrupt handler    
    
    // If preferences haven't been user configured yet, set defaults 
    //prefs.load(); 
    loadFromFlash();                                                              // Read flash
    can.filter(CAN_ID , 0xFFF, CANStandard, 0);                                                         
    txMsg.id = CAN_MASTER;
    txMsg.len = 6;
    rxMsg.len = 8;
    can.attach(&onMsgReceived);  
    
    if(isnan(E_OFFSET)){E_OFFSET = 0.0f;}
    if(isnan(M_OFFSET)){M_OFFSET = 0.0f;}
    if(isnan(I_BW) || I_BW==-1){I_BW = 1000;}
    if(isnan(I_MAX) || I_MAX ==-1){I_MAX=40;}
    if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=12;}
    if(isnan(CAN_ID) || CAN_ID==-1){CAN_ID = 1;}
    if(isnan(CAN_MASTER) || CAN_MASTER==-1){CAN_MASTER = 0;}
    if(isnan(CAN_TIMEOUT) || CAN_TIMEOUT==-1){CAN_TIMEOUT = 1000;}
    if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}
    if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 125.0f;}
    if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
    spi.SetElecOffset(E_OFFSET);                                                // Set position sensor offset
    spi.SetMechOffset(M_OFFSET);
    spi.Sample(1.0f);
    if(spi.GetMechPosition() > PI){spi.SetMechOffset(M_OFFSET+2.0f*PI);}        // now zeroes to +- 30 degrees about nominal, independent of rollover point
    else if (spi.GetMechPosition() < -PI){spi.SetMechOffset(M_OFFSET-2.0f*PI);}
    
    int lut[128] = {0};
    memcpy(&lut, &ENCODER_LUT, sizeof(lut));
    spi.WriteLUT(lut);                                                          // Set potision sensor nonlinearity lookup table
    init_controller_params(&controller);
    

    pc.baud(921600);    
    wait_us(10000);                                                        // set serial baud rate
    printf("\n\r\n\r HobbyKing Cheetah\n\r\n\r");
    wait_us(10000);
    printf("\n\r Debug Info:\n\r");
    printf(" Firmware Version: %s\n\r", VERSION_NUM);
    printf(" ADC1 Offset: %d    ADC2 Offset: %d\n\r", controller.adc1_offset, controller.adc2_offset);
    printf(" Position Sensor Electrical Offset:   %.4f\n\r", E_OFFSET);
    printf(" Output Zero Position:  %.4f\n\r", M_OFFSET);
    printf(" CAN ID:  %d\n\r", CAN_ID);
    

    TIM1->CR1 ^= TIM_CR1_UDIS;

    
    pc.attach(&serial_interrupt);                                               // attach serial interrupt


    //int counter = 0;
    while(1) {
        //drv.print_faults();
        wait_us(100000);
        //printf("%.3f  %.3f\n\r" , observer.temperature, observer.q_in);
        if(controller.otw_flag){gpio.led1->write(!gpio.led1->read());}
             /*
        if(state == MOTOR_MODE)
        {
            if(controller.otw_flag){gpio.led1->write(!gpio.led1->read());}
            //printf("%f  %f\n\r", controller.dtheta_mech, controller.i_d_ref);
            //printf("%.3f  %.3f  %.3f\n\r", (float)observer.temperature, (float)observer.temperature2, observer.resistance);
            //printf("%.3f  %.3f  %.3f %.3f %.3f\n\r", controller.v_d, controller.v_q, controller.i_d_filt, controller.i_q_filt, controller.dtheta_elec);
            //printf("%.3f  %.3f  %.3f %.3f\n\r", controller.dtheta_elec, observer.resistance, observer.temperature, observer.temp_measured);
            //printf("%.3f  %.3f\n\r" , observer.temperature, observer.temp_measured);
        }
        
        */

    }
}
