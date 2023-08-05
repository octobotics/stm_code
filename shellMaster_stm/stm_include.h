/**
   usefule links:

    current sensor:
      digikey link   : https://www.digikey.in/en/products/detail/riedon/SSA-100/12149327?utm_adgroup=General&utm_source=google&utm_medium=cpc&utm_campaign=Dynamic%20Search_EN_Product&utm_term=&productid=
      datasheet link : https://riedon.com/media/pdf/SSA.pdf

    stm32-f411ceu6 (blackPill)
      Specs          : https://stm32-base.org/boards/STM32F411CEU6-WeAct-Black-Pill-V2.0.html
      Pin-out        : https://stm32world.com/wiki/Black_Pill
      IDE settings   : https://www.sgbotic.com/index.php?dispatch=pages.view&page_id=49


*/



/*************************** <Start define> ***************************/


//generic macros
#define RESET                   (0)
#define SET                     (1)
#define NOT_AVAILABLE           (-1)


//Linear control pins
#define ACT_A1              (PA0) //PWM-1
#define ACT_A2              (PA1) //DIR-1

#define ACT_B1              (PB14) //PWM-2
#define ACT_B2              (PB15) //DIR-2

#define ACT_C1              (PB12) //PWM-3
#define ACT_C2              (PB13) //DIR-3

#define LED                 (PC13)

/*************************** <End define> ***************************/
