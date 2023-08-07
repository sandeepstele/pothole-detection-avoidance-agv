 //this is a simple code to control the AGV with the on-board joystick for testing purposes
//the VRX on the joystick steers the car left and right
//the upper section of the VRY throttles the car either forward or backwards, if the on-board LED of the arduino is ON, the car is in reverse mode, if it's not, the car is in forward mode
//the lowe section of the VRY is for braking the car

#include<PID_v1.h>
//macro expansions to print only when needed
#define  PRINT(x)   Serial.print(x)
#define  PRINTLN(x) Serial.println(x)


//macros for all the pins, each mechanism has 2 PWM inputs, all the EN pins are connected to 5V by the circuitry
#define PWM1_STEER 5
#define PWM2_STEER 6
#define PWM1_BRAKE 10
#define PWM2_BRAKE 9
#define PWM_THROTTLE 3

//feedback potentiometers for both mechanisms
#define ADC_BRAKE A1
#define ADC_STEER A0

//extreme value of the ADC of each mechanism, by default t1` hey should be 0 and 1023
#define MAX_STEER_ADC 546      //right
#define MIN_STEER_ADC 735      //left

#define MAX_BRAKE_ADC 575     //maximum braking
#define MIN_BRAKE_ADC 500     //no braking

//extreme value of each PWM, by default they should be 0 and 255
#define MAX_BRAKE_PWM 255
#define MIN_BRAKE_PWM 10

#define MAX_STEER_PWM 255
#define MIN_STEER_PWM 10

#define MIN_THROTTLE_PWM 62 //1.2v
#define MAX_THROTTLE_PWM 122 //2.5v

#define MIN_STEER_ANGLE -22.5 //degrees
#define MAX_STEER_ANGLE 22.5 //degrees

//PID gains
#define PID_STEER_KP 5
#define PID_STEER_KI 2
#define PID_STEER_KD 0.2

#define PID_BRAKE_KP 5
#define PID_BRAKE_KI 0.01
#define PID_BRAKE_KD 0.2

#define INTERVAL 50.0  //interval between each sample in ms

#define WHEEL_RADIUS 0.16

//global variables
bool ThrottleDir = true;
double TargetSteer, FeedbackSteer, OutputSteer;
double TargetBrake, FeedbackBrake , OutputBrake; 
bool SteerDir = true, BrakeDir = true;
unsigned long prev_t = millis();
unsigned long prev_int = millis();
unsigned long encoder_time = millis();
const unsigned long init_time = millis();
int TargetThrottle,Alpha, ThrottlePWM;
int SteerPWM, BrakePWM;
volatile double longitPosition;
double prevLongitPosition, longitSpeed;
int Throttle = 0, delta = 128, brake = 0;

char firstByte;

PID SteerPID(&FeedbackSteer , &OutputSteer , &TargetSteer , PID_STEER_KP , PID_STEER_KI , PID_STEER_KD , DIRECT);
PID BrakePID(&FeedbackBrake , &OutputBrake , &TargetBrake , PID_BRAKE_KP , PID_BRAKE_KI , PID_BRAKE_KD , DIRECT);

void setup() {
  //pin modes of all the I/Os
  //outputs
  pinMode(PWM1_BRAKE, OUTPUT);
  pinMode(PWM2_BRAKE, OUTPUT);
  pinMode(PWM1_STEER, OUTPUT);
  pinMode(PWM2_STEER, OUTPUT);
  pinMode(PWM_THROTTLE , OUTPUT);

  //INPUTS
  pinMode(ADC_BRAKE , INPUT);
  pinMode(ADC_STEER , INPUT);

  SteerPID.SetMode(AUTOMATIC);
  SteerPID.SetOutputLimits(-255 , 255);

  BrakePID.SetMode(AUTOMATIC);
  BrakePID.SetOutputLimits(-255 , 255);

  Serial.begin(57600);
}

void loop() {
  //feedbacks
  FeedbackSteer = analogRead(ADC_STEER);
  FeedbackBrake = analogRead(ADC_BRAKE);
 if (Serial.available() > 0) {
   while(Serial.available()<4);
   firstByte = Serial.read();
   if (firstByte == 'f') {
    Throttle = Serial.read();
    delta = Serial.read();
    brake = Serial.read();
    PRINTLN(delta);
   }
 }

  //both PIDs output a PWM signal that we are gonna map according to the minimum value that overpowers the static friction of the mechanism
  //and a Dir boolean according to the sign of that output to decide which input pin of the H-bridge should be the PWM and which should be GND

  //Steer PID

  TargetSteer = map(delta, 0, 255, MIN_STEER_ADC , MAX_STEER_ADC);
  SteerPID.Compute();
  SteerPWM = map(abs(OutputSteer) , 0 , 255 , MIN_STEER_PWM , MAX_STEER_PWM);
  SteerDir = ((OutputSteer / abs(OutputSteer)) + 1) / 2;


  // add an if statement to engage brakes
  TargetBrake = map(brake, 0, 100, MIN_BRAKE_ADC, MAX_BRAKE_ADC);
  //Brake PID
  BrakePID.Compute();
  BrakePWM = map(abs(OutputBrake) , 0 , 255 , MIN_BRAKE_PWM , MAX_BRAKE_PWM);
  BrakeDir = ((OutputBrake / abs(OutputBrake)) + 1) / 2;

  do {
    //steer
    analogWrite(PWM1_STEER ,  SteerDir * SteerPWM);
    analogWrite(PWM2_STEER , !SteerDir * SteerPWM);

    //brake
    analogWrite(PWM1_BRAKE ,  BrakeDir * BrakePWM);
    analogWrite(PWM2_BRAKE , !BrakeDir * BrakePWM);

    //Throttle
    analogWrite(PWM_THROTTLE , Throttle);
  }
  while (millis() - prev_t < INTERVAL);
}
