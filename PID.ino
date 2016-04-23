#include <QTRSensors.h>

int maxvelder=230;
int maxveliz=230;
int basevelder=150;
int baseveliz=150;
int velder=0;
int veliz=0;
double Kd=analogRead(A0);
double Kp=analogRead(A1);
double Ki=analogRead(A2); //Kd>Kp

#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
#define der1 3
#define der2 4
#define derpwm 5
#define iz1 12
#define iz2 13
#define izpwm 11
#define motorPower 8



QTRSensorsRC qtrrc((unsigned char[]) {  14, 15, 16, 17, 18, 19} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  pinMode(der1, OUTPUT);
  pinMode(der2, OUTPUT);
  pinMode(derpwm, OUTPUT);
  pinMode(iz1, OUTPUT);
  pinMode(iz2, OUTPUT);
  pinMode(izpwm, OUTPUT);
  pinMode(motorPower, OUTPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  
  
  int i;
for (int i = 0; i < 100; i++)

  /* comment this part out for automatic calibration 
  if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
     turn_right();  
   else
     turn_left(); */ 
   qtrrc.calibrate();   
   delay(20);
wait();  
delay(2000);
    
    /* comment out for serial printing
    
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    */
  } 

int lastError = 0;;

void loop()
{
  unsigned int sensors[8];
  int position = qtrrc.readLine(sensors);
  int error = position - 1000;


  
  int u = Kp * error + Kd * (error - lastError) + Ki * (error + lastError)  ;
  lastError = error;

  if (error>0)
  {
  int velder = basevelder + u;
  int veliz = baseveliz - u;
  }
  if (error<0)
  {   
  int velder = basevelder - u;
  int veliz = baseveliz + u;
    }
  
  if (velder > maxvelder ) velder = maxvelder; 
  if (veliz > maxveliz ) veliz = maxveliz;
  if (velder < 0) velder = 0;
  if (veliz < 0) velder = 0; 
  
   {
  digitalWrite(motorPower, HIGH);
  digitalWrite(der1, HIGH);
  digitalWrite(der2, LOW);
  analogWrite(derpwm, velder);
  digitalWrite(motorPower, HIGH);
  digitalWrite(iz1, HIGH);
  digitalWrite(iz2, LOW);
  analogWrite(izpwm, veliz);
}
}
  
void wait(){
    digitalWrite(motorPower, LOW);
}

