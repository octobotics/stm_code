/*
   Melodic:
        https://github.com/ros-drivers/rosserial/blob/a8c9219eb0aa88dd40bd72ef5bd188fe0bd0f44b/rosserial_python/src/rosserial_python/SerialClient.py

   Note:
      1. If internet is not available, the required file-content is present in "ros_include.h" file.
      2. Replace the following file at given location:
            /opt/ros/noetic/lib/python3/dist-packages/rosserial_python/SerialClient.py

    HackMD:
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

*/
/*************************** <> ***************************/

#include "stm_include.h"
#include "ros_include.h"

/*************************** <function declaration> ***************************/

void callback_relay_toggle(const relay_control::Request &req, relay_control::Response &res);
void callback_servo_toggle(const Trigger::Request &req, Trigger::Response &res);
void callback_camera(const stm_client::cam_array &cam_data);

void setup_I2C();
void setup_gpio();
void setup_servos();
void gimbal_setup_motion();
void setup_currentSensor();
void start_pub_sub_serv();

void auto_magnet_off();
void readSend_currentVal();
void pump_off_flagStat();

/*
   create ros-handle
*/
ros::NodeHandle nh;

/*
   Publisher
*/
stm_client::arm_tool send_status;
ros::Publisher arm_tool_status("arm_tool_status", &send_status);

std_msgs::Float32 send_current_status;
ros::Publisher current_status("current_status", &send_current_status);

/*
   Subscriber
*/
ros::Subscriber<stm_client::cam_array> camera_sub("/gimbal_twist", &callback_camera);

/*
   Server
*/
ros::ServiceServer<relay_control::Request, relay_control::Response> relay("relay_toggle_channel", &callback_relay_toggle); // Relay
ros::ServiceServer<Trigger::Request, Trigger::Response> servo_trig("servo_trigger_channel", &callback_servo_toggle);       // grinder

void setup()
{
  // nh.logwarn("Step-1");
  setup_I2C();
  setup_gpio();
  setup_servos();
  gimbal_setup_motion();

  /*
     Initialize Node handle.
  */
  nh.initNode();
  start_pub_sub_serv();

  setup_currentSensor();
}

void loop()
{
  pump_off_flagStat();
  auto_magnet_off();
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
      // pump off
      digitalWrite(PB4, LOW);
      pump_flag = false;
      curr_pump_millis = millis();

      // magnet off
      digitalWrite(MAGNET, LOW);
      magnet_auto_off_flag = false;
      prev_magnet_millis = millis();

      // top led off
      digitalWrite(TOP_LED, LOW);

      // bottom led on
      digitalWrite(BOTTOM_LED, HIGH);

      // grinder off
      buffTool.write(BUFFING_SPEED_RESET);
      break;

    case 2:
      // set gimbal pose before any
      cam_mount.write(0);
      cam_base.write(0);
      mount = 0;
      base = 0;
      break;

    // Pump
    case 4:
      // if (!nh.getParam("/", &pump_delay))

      digitalWrite(PB4, HIGH - digitalRead(PB4));
      delay(PUMP_SWITCH_DELAY);
      digitalWrite(PB4, HIGH - digitalRead(PB4));
      pump_flag = true;
      prev_pump_millis = millis();
      send_status.data = PUMP_ON;

      arm_tool_status.publish(&send_status);
      break;

    // Bottom light
    case 5:
      digitalWrite(BOTTOM_LED, HIGH - digitalRead(BOTTOM_LED));
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

void callback_camera(const stm_client::cam_array &cam_data)
{

  /*
     Gimbal Base
  */
  if (cam_data.data[1] == GIMBAL_INC)
  {
    if (base <= GIMBAL_BASE_MAX_LIMIT)
    {
      base++;
      cam_base.write(base);
      delay(GIMBAL_DELAY);
    }
  }
  else if (cam_data.data[1] == GIMBAL_DEC)
  {
    if (base >= GIMBAL_BASE_MIN_LIMIT)
    {
      base--;
      cam_base.write(base);
      delay(GIMBAL_DELAY);
    }
  }

  /*
     Gimbal Mount
  */
  if (cam_data.data[0] == GIMBAL_INC)
  {
    if (mount <= GIMBAL_MOUNT_MAX_LIMIT)
    {
      mount++;
      cam_mount.write(mount);
      delay(GIMBAL_DELAY);
    }
  }
  else if (cam_data.data[0] == GIMBAL_DEC)
  {
    if (mount >= GIMBAL_MOUNT_MIN_LIMIT)
    {
      mount--;
      cam_mount.write(mount);
      delay(GIMBAL_DELAY);
    }
  }
}

/*************************** <function definitions> ***************************/

void setup_I2C()
{

  // I2C
  Wire.setSDA(SDA2);
  Wire.setSCL(SCL2);
  Wire.begin();
}


/*
   Initialize the GPIO pins for all peripheral.
*/
void setup_gpio()
{

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

  cam_base.attach(CAM_BASE);   // Camera base
  cam_mount.attach(CAM_MOUNT); // Camera mount

  buffTool.write(BUFFING_SPEED_RESET); // grinder
}

/*
   Setup gimbal at every start of the robot.
*/
void gimbal_setup_motion()
{
  cam_base.write(0);
  cam_mount.write(0);
  delay(1000);
  cam_base.write(90);
  cam_mount.write(90);
}

void setup_currentSensor()
{
  SSA100.begin();
  SSA100.setGain(1);

}


/*
   Start all publisher subscriber services
*/
void start_pub_sub_serv()
{

  // Publisher
  nh.advertise(arm_tool_status);
  nh.advertise(current_status);

  // Subscriber
  nh.subscribe(camera_sub); // Start camera subscriber

  // Service
  nh.advertiseService(relay);      // Start relay server
  nh.advertiseService(servo_trig); // Start servo server
}

/*
     Send the off status of the flag
*/
void pump_off_flagStat()
{
  if (pump_flag == true)
  {
    curr_pump_millis = millis();
    if ((unsigned long)curr_pump_millis - prev_pump_millis > PUMP_DELAY)
    {
      send_status.data = 1;
      arm_tool_status.publish(&send_status);
      pump_flag = false;
      prev_pump_millis = curr_pump_millis;
    }
  }
}

/*
     Monitor the magnet status and turn it off once the defined time has passed.
*/
void auto_magnet_off()
{
  if (magnet_auto_off_flag == true)
  {
    curr_magnet_millis = millis();
    if ((unsigned long)curr_magnet_millis - prev_magnet_millis > MAGNET_DELAY)
    {
      digitalWrite(MAGNET, LOW);
      send_status.data = MAGNET_OFF;
      arm_tool_status.publish(&send_status);
      magnet_auto_off_flag = false;
      prev_magnet_millis = curr_magnet_millis;
    }
  }
}

/*
    Read current sensor value and send to the topic: "/current_status"
*/
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
