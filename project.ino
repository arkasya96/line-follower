#include <QTRSensors.h>
  #define MIDDLE_SENSOR 4       //number of middle sensor used
#define NUM_SENSORS 6         //number of sensors used
#define TIMEOUT 2500          //waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN 2
// create an object for your type of sensor (RC or Analog)
#define leftMotorFwd 10   
#define rightMotorFwd 11

// in this example we have three sensors on analog inputs 0 - 2, a.k.a. digital pins 14 - 16
QTRSensorsRC qtr((unsigned char[]) {12,9,8,7,6,5}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
// QTRSensorsA qtr((char[]) {0, 1, 2}, 3);

void setup()
{
   pinMode(leftMotorFwd, OUTPUT); //Set the forward pin to an output
  pinMode(rightMotorFwd, OUTPUT); //Set the forward pin to an output
 // pinMode(EMITTER_PIN,OUTPUT);
  // optional: wait for some input from the user, such as  a button press
 
  // then start calibration phase and move the sensors over both
  // reflectance extremes they will encounter in your application:
  int i;

 // emittersOn();
  for (i = 0; i < 250; i++)  // make the calibration take about 5 seconds
  {
    //qtr.calibrate();
   manual_calibration();
    delay(20);
  }
  Serial.begin(9600);
  // optional: signal that the calibration phase is now over and wait for further
  // input from the user, such as a button press
}
int leftMotorSpeed = 100;
  int rightMotorSpeed = 100;
void loop(){
   // digitalWrite(EMITTER_PIN,HIGH);
  unsigned int sensors[6];
  
  // get calibrated sensor values returned in the sensors array, along with the line position
  // position will range from 0 to 2000, with 1000 corresponding to the line over the middle 
  // sensor

  
  int position = qtr.readLine(sensors);
  int error = position - 3000;
//emittersOn();
  Serial.println(position);
analogWrite(leftMotorFwd,leftMotorSpeed);
analogWrite(rightMotorFwd,rightMotorSpeed);

//  if (error < -500)  // the line is on the left
//    leftMotorSpeed = 0; 
//    else{
//      leftMotorSpeed = 100;// turn left
//    }
//  if (error > 500)  // the line is on the right
//    rightMotorSpeed = 0;
//    else{
//      rightMotorSpeed = 100;
//      // turn right
//    }
delay(500);
}

void manual_calibration() {
  
int i;
for (i = 0; i < 250; i++)
{
qtr.calibrate(QTR_EMITTERS_ON);
delay(20);
}

}
