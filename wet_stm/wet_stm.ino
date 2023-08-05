/*
   Melodic:
    https://github.com/ros-drivers/rosserial/blob/a8c9219eb0aa88dd40bd72ef5bd188fe0bd0f44b/rosserial_python/src/rosserial_python/SerialClient.py

   Note:
      1. If internet is not available, the required file-content is present in "ros_include.h" file.
      2. Replace the following file at given location:
            /opt/ros/noetic/lib/python3/dist-packages/rosserial_python/SerialClient.py


     Hackmd:
     https://hackmd.io/hJOqWjruQsq7Xle3vMfS_w
*/

/*
  -------------------------
  |              on - off |
  | Pump   (PB4): 0 -  1  |
  | Magnet (PB7): 2 -  3  |
  | servo  (PB8): 4 -  5  |
  | b_led  (PB5): 6 -  7  |
  | t_led  (PB6): 8 -  9  |
   ------------------------

  SERVO:
   Grinder :  PA8
   Base    :  PB15
   Mount   :  PB14

   Relay pins:
      PB4 : pump
      PB5 : bottom_light
      PB6 : upper_led
      PB7 : magnet
      PB8 : arm
      PB9 : crawler

  Voltage Sensor:
      PA0 : Voltage sensor-1
      PA1 : Voltage sensor-2

  RX-TX pins:
     PA9  : Tx
     PA10 : Rx

  I2C pins:
     PB3  : SDA
     PB10 : SCL

  Sketch uses 44756 bytes (8%) of program storage space. Maximum is 524288 bytes.
  Global variables use 6620 bytes (5%) of dynamic memory, leaving 124452 bytes for local variables. Maximum is 131072 bytes.
*/
/*************************** <> ***************************/

#include "stm_include.h"
#include "ros_include.h"

/*************************** <function declaration> ***************************/

void callback_relay_toggle(const relay_control::Request &req, relay_control::Response &res);
void callback_servo_toggle(const Trigger::Request &req, Trigger::Response &res);

void setup_gpio();
void setup_servos();
void get_pressure_sensor();
void setup_currentSensor();
void start_pub_sub_serv();

void get_mpm3801();

void auto_magnet_off();
void readSend_pressureVal();
void readSend_currentVal();

/*
   create ros-handle
*/
ros::NodeHandle nh;

/*
   Publisher
*/
stm_client::arm_tool send_status;
ros::Publisher arm_tool_status("arm_tool_status", &send_status);

stm_client::atm_arr pressure_stat;
ros::Publisher pressure_status("pressure_status", &pressure_stat);

std_msgs::Float32 send_current_status;
ros::Publisher current_status("current_status", &send_current_status);

/*
   Server
*/
ros::ServiceServer<relay_control::Request, relay_control::Response> relay("relay_toggle_channel", &callback_relay_toggle); // Relay
ros::ServiceServer<Trigger::Request, Trigger::Response> servo_trig("servo_trigger_channel", &callback_servo_toggle);       // grinder

void setup()
{
  // nh.logwarn("Step-1");

  setup_gpio();
  setup_servos();
  /*
     Initialize Node handle.
  */
  nh.initNode();
  start_pub_sub_serv();

  get_pressure_sensor();
  setup_currentSensor();

}

void loop()
{
  auto_magnet_off();
  readSend_pressureVal();
  readSend_currentVal();

  nh.spinOnce();
  // delay(10);
}

void callback_relay_toggle(const relay_control::Request &req, relay_control::Response &res)
{
  switch (req.data)
  {
    // turn off everything
    case 1:
      // magnet off
      digitalWrite(MAGNET, LOW);
      magnet_auto_off_flag = false;
      prev_magnet_millis = millis();

      // top led off
      digitalWrite(TOP_LED, HIGH);

      // bottom led on
      digitalWrite(BOTTOM_LED, HIGH);

      // grinder off
      buffTool.write(BUFFING_SPEED_RESET);
      break;

    // Bottom light
    case 5:
      digitalWrite(BOTTOM_LED, HIGH - digitalRead(BOTTOM_LED));
      //analogWrite(BOTTOM_LED, 250);

      read_gpio_pin = digitalRead(BOTTOM_LED);

      if (read_gpio_pin)
      {
        send_status.data = BOTTOM_LED_ON;
      }
      else
      {
        send_status.data = BOTTOM_LED_OFF;
      }
      arm_tool_status.publish(&send_status);

      break;

    // Top light
    case 6:
      digitalWrite(TOP_LED, HIGH - digitalRead(TOP_LED));
      read_gpio_pin = digitalRead(TOP_LED);
      if (read_gpio_pin)
      {
        send_status.data = TOP_LED_ON;
      }
      else
      {
        send_status.data = TOP_LED_OFF;
      }
      arm_tool_status.publish(&send_status);
      break;

    // Magnet
    case 7:
      digitalWrite(MAGNET, HIGH - digitalRead(MAGNET));
      read_gpio_pin = digitalRead(MAGNET);
      if (read_gpio_pin)
      {
        send_status.data = MAGNET_ON;
        magnet_auto_off_flag = true;
        prev_magnet_millis = millis();
      }
      else
      {
        send_status.data = MAGNET_OFF;
        magnet_auto_off_flag = false;
        prev_magnet_millis = millis();
      }
      arm_tool_status.publish(&send_status);
      break;

    // Arm
    case 8:
      digitalWrite(ARM, HIGH - digitalRead(ARM));
      break;

    // Crawler
    case 9:
      digitalWrite(CRAWLER, HIGH - digitalRead(CRAWLER));
      break;
  }

  digitalWrite(LED, HIGH - digitalRead(LED));
  res.response = true;
}

void callback_servo_toggle(const Trigger::Request &req, Trigger::Response &res)
{
  if (servo_flag)
  {
    buffTool.write(BUFFING_SPEED_SET);
    send_status.data = BUFF_TOOL_ON;
  }
  else
  {
    buffTool.write(BUFFING_SPEED_RESET);
    send_status.data = BUFF_TOOL_OFF;
  }
  servo_flag = !servo_flag;

  arm_tool_status.publish(&send_status);

  res.success = true;
}

/*************************** <function definitions> ***************************/

/*
   Initialize the GPIO pins for all peripheral.
*/
void setup_gpio()
{

  // I2C
  Wire.setSDA(SDA2);
  Wire.setSCL(SCL2);
  Wire.begin();

  // relay
  pinMode(PUMP, OUTPUT);
  pinMode(BOTTOM_LED, OUTPUT);
  pinMode(TOP_LED, OUTPUT);
  pinMode(MAGNET, OUTPUT);
  pinMode(ARM, OUTPUT);
  pinMode(CRAWLER, OUTPUT);

  digitalWrite(PUMP, LOW);
  digitalWrite(BOTTOM_LED, LOW);
  digitalWrite(TOP_LED, LOW);
  digitalWrite(MAGNET, LOW);
  digitalWrite(ARM, HIGH);
  digitalWrite(CRAWLER, HIGH);

  // led
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
}

/*
   Initialize servo pins.
*/
void setup_servos()
{

  // servo
  buffTool.attach(BUFF_TOOL); // grinder
  buffTool.write(RESET);
}

void setup_currentSensor()
{
  SSA100.begin();
  SSA100.setGain(1);

}


/*
   get pressure sensors.
*/
void get_pressure_sensor()
{

  nh.loginfo("***Pressure sensor setup complete***");
}

/*
   Start all publisher subscriber services
*/
void start_pub_sub_serv()
{

  // Publisher
  nh.advertise(arm_tool_status);
  nh.advertise(pressure_status);
  nh.advertise(current_status);

  // Service
  nh.advertiseService(relay);      // Start relay server
  nh.advertiseService(servo_trig); // Start servo server
  nh.negotiateTopics();

}

/*
     Monitor the magnet status and turn it off once the defined time has passed.
*/
void auto_magnet_off()
{
  if (magnet_auto_off_flag == true)
  {
    curr_magnet_millis = millis();
    if (curr_magnet_millis - prev_magnet_millis > MAGNET_DELAY)
    {
      digitalWrite(MAGNET, LOW);
      send_status.data = MAGNET_OFF;
      arm_tool_status.publish(&send_status);
      magnet_auto_off_flag = false;
      prev_magnet_millis = curr_magnet_millis;
    }
  }
}

void readSend_pressureVal()
{
  curr_pressure_millis = millis();
  if (curr_pressure_millis - prev_pressure_millis > PRESSURE_DELAY)
  {
    //Electronics box

    get_mpm3801();

    pressure_status.publish(&pressure_stat);
    prev_pressure_millis = curr_pressure_millis;
  }
}


void get_mpm3801() {

  int rv = Wire.requestFrom(P_ELEC, 4);
  if (rv == 4)
  {
    for (uint8_t i = 0; i < 4; i ++)
    {
      data[i] = Wire.read();
    }
    uint16_t pressure = (data[0] << 8 | data[1] );
    pressure &= 0x3FFF;

    data[3] = data[3] >> 5;
    uint16_t temp = ((data[2] << 3) | (data[3]));
    temp &= 0x07FF;

    f_pressure = (MAX_PRESSURE * (pressure - LOWER_LIMIT)) / (13108.00F);
    f_temp = ((temp * 200.00) / 2047.00F) - 50.00F;

    pressure_stat.data[0] = f_pressure;

  }
}

void readSend_currentVal()
{
  /*
     Formula to calculate current:(gain:1)
          (adc_val * 4.096 * 1000.00) / (32767.00 * 12.00F)

     constant:
     (4.096 * 1000.00) / (32767.00 * 12.00F) = 0.0104169845678070416374197617521693573819188004191208634703614408
  */


  curr_current_millis = millis();
  if (curr_current_millis - prev_current_millis > CURRENT_DELAY)
  {

    adc_val = 0;
    for (int i = 0; i < 5; i++)
    {
      adc_val = adc_val + SSA100.readADC_Differential_0_1();
      delay(10);
    }
    adc_val = adc_val / 5.00F;


    //adc_val =  SSA100.readADC_Differential_0_1();
    currentValue = (adc_val * ADS_GAIN_1 * MV_TO_V) / (ADC_VAL_15 * MV_TO_AMP);
    //  currentValue = (adc_val * 4.096 * 1000.00) / (32767.00 * 12.00F);


    send_current_status.data = currentValue;
    current_status.publish(&send_current_status);

    prev_current_millis = curr_current_millis;
  }


}
