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
   Relay pins:
      PA0 : ACT_A1
      PA1 : ACT_A2
      PB14 : ACT_B1
      PB15 : ACT_B2
      PB12 : ACT_C1
      PB13 : ACT_C2


  RX-TX pins:
     PA9  : Tx
     PA10 : Rx

*/
/*************************** <> ***************************/

#include "stm_include.h"
#include "ros_include.h"

/*************************** <function declaration> ***************************/

void callback_linear(const std_msgs::Int8 &cam_data);

void setup_gpio();
void start_pub_sub_serv();


/*
   create ros-handle
*/
ros::NodeHandle nh;

/*
   Subscriber
*/
ros::Subscriber<std_msgs::Int8> lin_act_control("/stm_control/lin_act", &callback_linear);


void setup()
{
  // nh.logwarn("Step-1");
  setup_gpio();

  /*
     Initialize Node handle.
  */
  nh.initNode();
  start_pub_sub_serv();

}

void loop()
{
  nh.spinOnce();
  // delay(10);
}



/*************************** <function definitions> ***************************/


/*
   Initialize the GPIO pins for all peripheral.
*/
void setup_gpio()
{

  // relay
  pinMode(ACT_A1, OUTPUT);
  pinMode(ACT_A2, OUTPUT);
  pinMode(ACT_B1, OUTPUT);
  pinMode(ACT_B2, OUTPUT);
  pinMode(ACT_C1, OUTPUT);
  pinMode(ACT_C2, OUTPUT);

  digitalWrite(ACT_A1, LOW);
  digitalWrite(ACT_A2, LOW);
  digitalWrite(ACT_B1, LOW);
  digitalWrite(ACT_B2, LOW);
  digitalWrite(ACT_C1, LOW);
  digitalWrite(ACT_C2, LOW);

  // led
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
}


/*
   Start all publisher subscriber services
*/
void start_pub_sub_serv()
{
  // Subscriber
  nh.subscribe(lin_act_control); // Start camera subscriber

}


void callback_linear(const std_msgs::Int8 &msg)
{
  switch (msg.data)
  {
    //all 3 act C
    case 1:
      digitalWrite(ACT_A1, HIGH);
      digitalWrite(ACT_A2, LOW);

      digitalWrite(ACT_B1, HIGH);
      digitalWrite(ACT_B2, LOW);

      digitalWrite(ACT_C1, HIGH);
      digitalWrite(ACT_C2, LOW);

      nh.loginfo("all C");
      break;


    //all 3 act CC
    case 2:
      digitalWrite(ACT_A1, HIGH);
      digitalWrite(ACT_A2, HIGH);

      digitalWrite(ACT_B1, HIGH);
      digitalWrite(ACT_B2, HIGH);

      digitalWrite(ACT_C1, HIGH);
      digitalWrite(ACT_C2, HIGH);
      nh.loginfo("all CC");

      break;

    //act-1 C
    case 3:
      digitalWrite(ACT_A1, HIGH);
      digitalWrite(ACT_A2, LOW);

      break;

    //act-1 CC
    case 4:
      digitalWrite(ACT_A1, HIGH);
      digitalWrite(ACT_A2, HIGH);

      break;

    //act-2 C
    case 5:
      digitalWrite(ACT_B1, HIGH);
      digitalWrite(ACT_B2, LOW);
      break;


    //act-2 CC
    case 6:
      digitalWrite(ACT_B1, HIGH);
      digitalWrite(ACT_B2, HIGH);

      break;

    //act-3 C
    case 7:
      digitalWrite(ACT_C1, HIGH);
      digitalWrite(ACT_C2, LOW);

      break;

    //act-3 CC
    case 8:
      digitalWrite(ACT_C1, HIGH);
      digitalWrite(ACT_C2, HIGH);
      break;

    case 9:
      digitalWrite(ACT_A1, LOW);
      digitalWrite(ACT_A2, LOW);

      digitalWrite(ACT_B1, LOW);
      digitalWrite(ACT_B2, LOW);

      digitalWrite(ACT_C1, LOW);
      digitalWrite(ACT_C2, LOW);
      break;
  }
  //  delay(100);


  // digitalWrite(LED, HIGH - digitalRead(LED));

}
