#include <QTRSensors.h>

int maxvelder=230;
int maxveliz=230;
int basevelder=150;
int baseveliz=150;
int velder=0;
int veliz=0;


#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2



#define derpwm 5
#define izpwm 3
  double Kp; //Se corrigieron errores en la definición de las variables y la toma de estas mismas
  double Kd;
  double Ki;


QTRSensorsRC qtrrc((unsigned char[]) {  4, 7, 8, 9, 10, 11, 12, 13} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

void setup()
{

  pinMode(2, OUTPUT);  //la polaridad de los dos motores va a estar definida por dos mismos pines, ya que estos dos sólo van a trabajar en un sentido, con el objetivo de ahorrar pines
  pinMode(6, OUTPUT);
  pinMode(derpwm, OUTPUT);
  pinMode(izpwm, OUTPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  Kd=analogRead(A0); //se toman los datos en setup, en la versión anterior se tomaban fuera de éste por lo cual no funcionaba correctamente
  Kp=analogRead(A1);
  Ki=analogRead(A2); //Kd>Kp  
  
  int i;
for (int i = 0; i < 100; i++)

  /* comment this part out for automatic calibration 
  if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
     turn_right();  
   else
     turn_left(); */ 
   qtrrc.calibrate();   
   delay(20);
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

int lastError = 0;

void loop()
{
  unsigned int sensors[8];
  int position = qtrrc.readLine(sensors);
  int error = position - 3500; //cuando el sensor está en su posición ideal su valor promedio es de 3500, si se mueve hacia la izquierda baja hasta casi 0 y en el sentido contrario 
                               //éste valor sube hasta 7000, se deduce error=0 cuando el sensor está en la posición ideal, a partir de este valor se puede saber en que dirección gira
                               //el robot y aplicar la respectiva ley de control

  

  if (error>0)
  {
  int u = Kp * error + Kd * (error - lastError) + Ki * (error + lastError)  ;
  int velder = basevelder + u;
  int veliz = baseveliz - u;
  
  lastError = error;
  }
  if (error<0)
  {   
  error=error+3500;
  int u = Kp * error + Kd * (error - lastError) + Ki * (error + lastError)  ;  
  int velder = basevelder - u;
  int veliz = baseveliz + u;
  
  lastError = error;
    }
  
  if (velder > maxvelder ) velder = maxvelder; //algoritmo de saturación que evita una sobrecarga en la tarjeta
  if (veliz > maxveliz ) veliz = maxveliz;
  if (velder < 0) velder = 0;
  if (veliz < 0) velder = 0; 
  
   {
  digitalWrite(2,HIGH);//se establece la polaridad de los motores indicando que se muevan siempre hacia adelante
  digitalWrite(6,LOW);
  analogWrite(derpwm, velder);//se escriben los valores como pwm
  analogWrite(izpwm, veliz);
}
}
