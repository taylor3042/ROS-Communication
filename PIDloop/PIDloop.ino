#include <ros.h>
#include <std_msgs/Float64.h>

#include <PID_v1.h>



// Define encoder pins and motor control pins
#define MOTOR1_ENCODER_A 19
#define MOTOR1_ENCODER_B 42
#define MOTOR1_PWM 8
#define MOTOR1_DIR 44
#define MOTOR1_ENABLE 46

#define MOTOR2_ENCODER_A 18
#define MOTOR2_ENCODER_B 34
#define MOTOR2_PWM 45
#define MOTOR2_DIR 51
#define MOTOR2_ENABLE 53

#define MOTOR3_ENCODER_A 3
#define MOTOR3_ENCODER_B 52
#define MOTOR3_PWM 47
#define MOTOR3_DIR 49
#define MOTOR3_ENABLE 7

#define MOTOR4_ENCODER_A 2
#define MOTOR4_ENCODER_B 48
#define MOTOR4_PWM 10
#define MOTOR4_DIR 50
#define MOTOR4_ENABLE 9




double targetDistance = 0.0;
double difference = 0.0;
double previousTargetDistance = 0.0;

ros::NodeHandle nh;
// Define PID parameters
double Kp = 1.0;  // Proportional gain
double Ki = 0.1;  // Integral gain
double Kd = 0.05; // Derivative gain

//double setpoint = 1.0; // Desired position in meters, later changing

double output1, output2, output3, output4;
int count1 = 0;
int count2 = 0;
int count3 = 0;
int count4 = 0;

double position1 = 0.0;
double position2 = 0.0;
double position3 = 0.0;
double position4 = 0.0;


// Create PID objects
PID motor1PID(&position1, &output1, &targetDistance, Kp, Ki, Kd, DIRECT);
PID motor2PID(&position2, &output2, &targetDistance, Kp, Ki, Kd, DIRECT);
PID motor3PID(&position3, &output3, &targetDistance, Kp, Ki, Kd, DIRECT);
PID motor4PID(&position4, &output4, &targetDistance, Kp, Ki, Kd, DIRECT);

void distanceCallback(const std_msgs::Float64& msg) {
  previousTargetDistance = targetDistance;
  targetDistance = msg.data;
}

ros::Subscriber<std_msgs::Float64> distance_sub("target_distance", &distanceCallback);

void setup() {

  nh.initNode();
  nh.subscribe(distance_sub);
  // Initialize encoder pins as input
  pinMode(MOTOR1_ENCODER_A, INPUT);
  pinMode(MOTOR1_ENCODER_B, INPUT);
  pinMode(MOTOR2_ENCODER_A, INPUT);
  pinMode(MOTOR2_ENCODER_B, INPUT);
  pinMode(MOTOR3_ENCODER_A, INPUT);
  pinMode(MOTOR3_ENCODER_B, INPUT);
  pinMode(MOTOR4_ENCODER_A, INPUT);
  pinMode(MOTOR4_ENCODER_B, INPUT);
  
  // Initialize motor control pins
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR1_ENABLE, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR2_ENABLE, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR3_ENABLE, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT);
  pinMode(MOTOR4_DIR, OUTPUT);
  pinMode(MOTOR4_ENABLE, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCODER_A),Motor_Encoder1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENCODER_A),Motor_Encoder2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_ENCODER_A),Motor_Encoder3,CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR4_ENCODER_A),Motor_Encoder4,CHANGE);
  
  // Initialize the PID controllers
  motor1PID.SetMode(AUTOMATIC);
  motor2PID.SetMode(AUTOMATIC);
  motor3PID.SetMode(AUTOMATIC);
  motor4PID.SetMode(AUTOMATIC);
  
}

void loop() {

nh.spinOnce(); //handles messages coming in and out from ROS SDK
difference = targetDistance - position1;
if(abs(difference) < 0.6)
  {
    stopMotors();
  }else{
  
  // Compute PID output
  motor1PID.Compute();
  motor2PID.Compute();
  motor3PID.Compute();
  motor4PID.Compute();

  // Adjust motor speed based on PID output
  int motorSpeed1 = constrain(output1, 200, 255);
  int motorSpeed2 = constrain(output2, 200, 255);
  int motorSpeed3 = constrain(output3, 200, 255);
  int motorSpeed4 = constrain(output4, 200, 255);
  // Update motor speed
  analogWrite(MOTOR1_ENABLE, abs(motorSpeed1));
  analogWrite(MOTOR2_ENABLE, abs(motorSpeed2));
  analogWrite(MOTOR3_ENABLE, abs(motorSpeed3));
  analogWrite(MOTOR4_ENABLE, abs(motorSpeed4));


  // Adjust motor direction
  if (targetDistance > previousTargetDistance) {
    digitalWrite(MOTOR1_DIR, HIGH);
    digitalWrite(MOTOR1_PWM, LOW);
  } else {
    digitalWrite(MOTOR1_DIR, LOW);
    digitalWrite(MOTOR1_PWM, HIGH);
  }

  if (targetDistance > previousTargetDistance) {
    digitalWrite(MOTOR2_DIR, HIGH);
    digitalWrite(MOTOR2_PWM, LOW);
  } else {
    digitalWrite(MOTOR2_DIR, LOW);
    digitalWrite(MOTOR2_PWM, HIGH);
  }

if (targetDistance > previousTargetDistance) {
    digitalWrite(MOTOR3_DIR, LOW);
    digitalWrite(MOTOR3_PWM, HIGH);
  } else {
    digitalWrite(MOTOR3_DIR, HIGH);
    digitalWrite(MOTOR3_PWM, LOW);
  }

   if (targetDistance > previousTargetDistance) {
    digitalWrite(MOTOR4_DIR, LOW);
    digitalWrite(MOTOR4_PWM, HIGH);
  } else {
    digitalWrite(MOTOR4_DIR, HIGH);
    digitalWrite(MOTOR4_PWM, LOW);
  }
  
  }
}

// Function to read encoder position in meters
double readEncoderPosition(int counter, int ticksPerMeter) {
  double position = counter / ticksPerMeter;
  return position;
}



/*This is a void that interrupts the loop to add to each variable. I've tried simplifying this in 
 * many ways, but it won't accept an int method(), and it bogs down the processing when I add cases for each motor, making the counts innacurate.
 */
void Motor_Encoder1(){
  int b = digitalRead(MOTOR1_ENCODER_B);
    count1++;
    
    position1 = readEncoderPosition(count1,192.66);
    //encoder_msg.data = position1;
    //encoder_pub.publish(&encoder_msg);
  }

void Motor_Encoder2(){
  int b = digitalRead(MOTOR2_ENCODER_B);
    count2++;
    position2 = readEncoderPosition(count2,291.66);
  }

void Motor_Encoder3(){
  int b = digitalRead(MOTOR3_ENCODER_B);
    count3++;
    position3 = readEncoderPosition(count3,300.17);
  }
void Motor_Encoder4(){
  int b = digitalRead(MOTOR4_ENCODER_B);
    count4++;
    position4 = readEncoderPosition(count4,330.33);
  }

 void stopMotors()
 {
  analogWrite(MOTOR1_ENABLE, 0); // Set PWM to 0 to stop motor
  analogWrite(MOTOR2_ENABLE, 0); 
  analogWrite(MOTOR3_ENABLE, 0); // Set PWM to 0 to stop motor
  analogWrite(MOTOR4_ENABLE, 0); 
  }

  
