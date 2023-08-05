/**
   usefule links:

    current sensor:
      digikey link   : https://www.digikey.in/en/products/detail/riedon/SSA-100/12149327?utm_adgroup=General&utm_source=google&utm_medium=cpc&utm_campaign=Dynamic%20Search_EN_Product&utm_term=&productid=
      datasheet link : https://riedon.com/media/pdf/SSA.pdf

    stm32-f411ceu6 (blackPill)
      Specs          : https://stm32-base.org/boards/STM32F411CEU6-WeAct-Black-Pill-V2.0.html
      Pin-out        : https://stm32world.com/wiki/Black_Pill
      IDE settings   : https://www.sgbotic.com/index.php?dispatch=pages.view&page_id=49


   Global variables use 6292 bytes (4%) of dynamic memory, leaving 124780 bytes for local variables. Maximum is 131072 bytes.

*/

#include<Servo.h>
#include <Wire.h>
#include "ADS1X15.h"


/*************************** <Start define> ***************************/


//generic macros
#define RESET                   (0)
#define SET                     (1)
#define NOT_AVAILABLE           (-1)


// current sensor macros
#define SSA_ADDR            (0x48)
#define MAX_CURRENT         (100.00F)
#define FLOAT_ROUND         (4)

#define ADS_GAIN_1          (4.096F)
#define MV_TO_V             (1000.00F)
#define ADC_VAL_15          (32767.00F)
#define MV_TO_AMP           (12.00F)


//delays
#define MAGNET_DELAY            (30000) // 30 sec.
#define PRESSURE_DELAY          (2000)  // 2 sec.
#define PUMP_DELAY              (5000)  // 0.1 sec.
#define PUMP_SWITCH_DELAY       (100)   // 0.1 sec.
#define GIMBAL_DELAY            (10)
#define CURRENT_DELAY           (1000)


// gimbal max limit
#define GIMBAL_BASE_MAX_LIMIT   (110)
#define GIMBAL_BASE_MIN_LIMIT   (0)
#define GIMBAL_MOUNT_MAX_LIMIT  (180)
#define GIMBAL_MOUNT_MIN_LIMIT  (0)


#define GIMBAL_INC              (1)
#define GIMBAL_DEC              (-1)



//Relay pins
#define PUMP                    (PB4)
#define BOTTOM_LED              (PB5)
#define TOP_LED                 (PB6)
#define MAGNET                  (PB7)
#define ARM                     (PB8)
#define CRAWLER                 (PB9)
#define LED                     (PC13)

//Gimabal pins
#define CAM_BASE                (PA8)
#define CAM_MOUNT               (PB15)

//buff tool pin
#define BUFF_TOOL               (PB13)

//Buffing tool speed
#define BUFFING_SPEED_SET       (80)
#define BUFFING_SPEED_RESET     (0)

//I2C2 pins
#define SCL2                (PB10)
#define SDA2                (PB3)

/*
  -------------------------
  |              on - off |
  | Pump   (PB4): 0 -  1  |
  | Magnet (PB7): 2 -  3  |
  | servo  (PB8): 4 -  5  |
  | b_led  (PB5): 6 -  7  |
  | t_led  (PB6): 8 -  9  |
   ------------------------
*/
#define PUMP_ON                (0)
#define PUMP_OFF               (1)
#define MAGNET_ON              (2)
#define MAGNET_OFF             (3)
#define BUFF_TOOL_ON           (4)
#define BUFF_TOOL_OFF          (5)
#define BOTTOM_LED_ON          (6)
#define BOTTOM_LED_OFF         (7)
#define TOP_LED_ON             (8)
#define TOP_LED_OFF            (9)


/*************************** <End define> ***************************/



ADS1115 SSA100(SSA_ADDR);



int read_gpio_pin = RESET; // to read the digital pin for feedback
double adc_val = RESET;           // get the ADC value from ADS1115 I@C module.
double currentValue = RESET;      // calculate current value



bool pump_flag = false;
unsigned long curr_pump_millis = RESET;
unsigned long prev_pump_millis = RESET;
//const long pump_interval = PUMP_DELAY;


bool magnet_auto_off_flag = false;
unsigned long curr_magnet_millis = RESET;
unsigned long prev_magnet_millis = RESET;
//const long magnet_interval = MAGNET_DELAY;


unsigned long curr_current_millis = RESET;
unsigned long prev_current_millis = RESET;


Servo buffTool;
Servo cam_base;
Servo cam_mount;

int mount = 90, base = 90;
