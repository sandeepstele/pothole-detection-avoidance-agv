 //this is a simple code to control the AGV with the on-board joystick for testing purposes
//the VRX on the joystick steers the car left and right
//the upper section of the VRY throttles the car either forward or backwards, if the on-board LED of the arduino is ON, the car is in reverse mode, if it's not, the car is in forward mode
//the lowe section of the VRY is for braking the car

#include<PID_v1.h>
//macro expansions to print only when needed
#define DEBUG 1
#if DEBUG == 1
#define  PRINT(x)   Serial.print(x)
#define  PRINTLN(x) Serial.println(x)
#else
#define PRINT(x)
#define PRINTLN(x)
#endif

//choose what to print for debugging 1 to print, 0 to not. 
#define PRINT_STEER 1
#define PRINT_BRAKE 1
#define PRINT_THROTTLE 1
#define PRINT_ENCODER 1

//macros for all the pins, each mechanism has 2 PWM inputs, all the EN pins are connected to 5V by the circuitry
#define PWM1_STEER 5
#define PWM2_STEER 6
#define PWM1_BRAKE 10
#define PWM2_BRAKE 9
#define PWM_THROTTLE 3

//feedback potentiometers for both mechanisms
#define ADC_BRAKE A1
#define ADC_STEER A0

//joystick pinouts
#define JOYSTICK_VRX A3
#define JOYSTICK_VRY A2
#define JOYSTICK_PB 2

//tachometer/positional encoder digital output
#define ENCODER_PIN 2

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
#define MAX_THROTTLE_PWM 90//2.5v

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
int Throttle,TargetThrottle,Alpha;
int SteerPWM, BrakePWM;
volatile double longitPosition;
double prevLongitPosition, longitSpeed;

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
  pinMode(JOYSTICK_VRX , INPUT);
  pinMode(JOYSTICK_VRY , INPUT);
  pinMode(JOYSTICK_PB , INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(JOYSTICK_PB), reverse , FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), incrementWheelPosition, FALLING);

  SteerPID.SetMode(AUTOMATIC);
  SteerPID.SetOutputLimits(-255 , 255);

  BrakePID.SetMode(AUTOMATIC);
  BrakePID.SetOutputLimits(-255 , 255);

  longitPosition = 0;
  prevLongitPosition = longitPosition;

#if DEBUG == 1
  Serial.begin(57600);
#endif

}

void loop() {
  //feedbacks
  FeedbackSteer = analogRead(ADC_STEER);
  FeedbackBrake = analogRead(ADC_BRAKE);

  //targets
   
  TargetSteer = map(analogRead(JOYSTICK_VRX) , 0 , 1023 , MIN_STEER_ADC , MAX_STEER_ADC);
  int VRY = analogRead(JOYSTICK_VRY);

  //upper side of VRY is throttle with no braking, lower side of VRY is braking with no throttle
  //there's a 20 ADC unit dead gap between both sections, as no joystick is exactly 512 in the middle and that would cause the car to either always have a brake or a throttle value

  //dead space (default)
  TargetThrottle = 0;
  TargetBrake = MIN_BRAKE_ADC;
  
  //braking
  if (VRY < 500) {
    // TargetBrake = map(VRY , 500 , 0 , MIN_BRAKE_ADC , MAX_BRAKE_ADC);
    TargetBrake = MIN_BRAKE_ADC;
    TargetThrottle = 0;
  }

  //throttle
  if (VRY > 520) {
    TargetThrottle = map(VRY , 520 , 1023 , MIN_THROTTLE_PWM , MAX_THROTTLE_PWM);
    TargetBrake = MIN_BRAKE_ADC;
  }
  
  //both PIDs output a PWM signal that we are gonna map according to the minimum value that overpowers the static friction of the mechanism
  //and a Dir boolean according to the sign of that output to decide which input pin of the H-bridge should be the PWM and which should be GND

  //Steer PID
  SteerPID.Compute();
  SteerPWM = map(abs(OutputSteer) , 0 , 255 , MIN_BRAKE_PWM , MAX_BRAKE_PWM);
  SteerDir = ((OutputSteer / abs(OutputSteer)) + 1) / 2;

  //Brake PID
  BrakePID.Compute();
  BrakePWM = map(abs(OutputBrake) , 0 , 255 , MIN_BRAKE_PWM , MAX_BRAKE_PWM);
  BrakeDir = ((OutputBrake / abs(OutputBrake)) + 1) / 2;

  //Exponential ramp for the throttle acceleration
  if(TargetThrottle > Throttle){
    Throttle = abs(TargetThrottle*(1 - (1-(TargetThrottle/Throttle))* exp(-Alpha/20)));
    Alpha++;
    }
  else{
    Throttle = TargetThrottle;
    Alpha = 0;
    }
  //debug Prints, set the macro DEBUG to 0 so the pre-processor removes them

 #if PRINT_STEER == 1
  //steer
  PRINT("Feedback Steer = ");
  PRINT(FeedbackSteer);
  PRINT("\t Target Steer = ");
  PRINT(TargetSteer);
  PRINT("\t Output Steer = ");
  PRINTLN(OutputSteer);
#endif

#if PRINT_BRAKE == 1
  //brake
  PRINT("Feedback Brake = ");
  PRINT(FeedbackBrake);
  PRINT("\t Target Brake = ");
  PRINT(TargetBrake);
  PRINT("\t Output Brake = ");
  PRINTLN(OutputBrake);
#endif
#if PRINT_THROTTLE == 1
  //Throttle
  PRINT("Target Throttle = ");
  PRINT(TargetThrottle);
  PRINT("\t Throttle = ");
  PRINTLN(Throttle);
  PRINTLN(TargetThrottle);
#endif

  // PRINTLN("--------------------------");
  do {
    //steer
//    analogWrite(PWM1_STEER ,  SteerDir * SteerPWM);
//    analogWrite(PWM2_STEER , !SteerDir * SteerPWM);
    analogWrite(PWM1_STEER ,  SteerDir * SteerPWM);
    analogWrite(PWM2_STEER , !SteerDir * SteerPWM);

    //brake
    analogWrite(PWM1_BRAKE ,  BrakeDir * BrakePWM);
    analogWrite(PWM2_BRAKE , !BrakeDir * BrakePWM);

    //Throttle
    analogWrite(PWM_THROTTLE , Throttle);
  }
  while (millis() - prev_t < INTERVAL);

  longitSpeed = (longitPosition - prevLongitPosition)*1000 / (millis() - prev_t);
  prev_t = millis();

#if PRINT_ENCODER == 1
  PRINT("prev_t = ");
  PRINT(prev_t);
  PRINT("\t Longitudinal Position = ");
  PRINT(longitPosition);
  PRINT("\t Longitudinal Speed = ");
  PRINTLN(longitSpeed);
#endif

  prevLongitPosition = longitPosition;
}

//interrupt to reverse the car
void reverse() {
  //anti aliasing
  if (millis() - prev_int < 1000)
    return;

  prev_int = millis();
  PRINTLN("---REVERSED---");
  ThrottleDir = !ThrottleDir;
  digitalWrite(LED_BUILTIN , !ThrottleDir);
  delay(50);
}

void incrementWheelPosition() {
  longitPosition += 6.28*WHEEL_RADIUS/25;
}
