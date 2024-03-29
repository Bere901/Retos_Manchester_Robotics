#include <ros.h>
#include <std_msgs/Float32.h>
#include <digitalWriteFast.h>
#include <SoftwareSerial.h>
float tiempo1 = 0;
float tiempo2 = 0;
float tiempoSegundos = 0;

const int variador = A0;  // Analog input pin that the potentiometer is attached to
const int EncA = 2;       // Entrada enconder A
const int EncB = 3;       // Entrada encoder B
const int In1 = 4;        //  Enable Puente H 1
const int In2 = 5;        //  Enable Puente H 2
const int EnA = 9;       //  PWM
int ciclo = 0;            // value output to the PWM (analog out)
int Vcc = 12;
int pot;
char opcion;
float conversion, posicion, posactual = 0, posanterior = 0, velocidad = 0;
float resolucion = 0.962;  //Definir resolución del encoderpulsos
int pulsos = 374;      //Número de pulsos a la salida del motorreductor
long contador = 0, contaux = 0, revoluciones;
volatile bool BSet = 0;
volatile bool ASet = 0;
float voltaje,duty;
int pwm = 0;
int pwm2 = 0;

float x; 
ros::NodeHandle  nh;

void messageCb( const std_msgs::Float32& msg){
  
  x=msg.data;
  x=x*100;
  Serial.println(x);
  

  
  // change the analog out value:
  if(x >= 0 ){
  voltaje = x * (Vcc/1023.0);  
  pwm = map(x, 0, 100, 0, 255); 
  //Serial.println(pwm);
  analogWrite(EnA,pwm);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  
  }
  else{
  voltaje = x * (Vcc/1023.0);  
  pwm = map(x, -1,-100, 0,255);  
  analogWrite(EnA,pwm);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  //pwm=(pwm-1000);
  //Serial.println(pwm2);
  }
  

}
std_msgs::Float32 test;
ros::Subscriber<std_msgs::Float32> sub("cmd_pwm", messageCb );
ros::Publisher p("my_topic", &test);

void setup ()
{
  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(p);
  pinMode(variador, INPUT);     //Pin declarado como entrada para leer el potenciómetro
  pinMode(EnA, OUTPUT);         //Salida de PWM
  pinMode(In1, OUTPUT);         //Pin declarado como salida para el motor
  pinMode(In2, OUTPUT);         //Pin declarado como salida para el motor
  pinMode(EncA, INPUT_PULLUP);          //Pin declarado como entrada, señal A del encoder de cuadratura
  pinMode(EncB, INPUT_PULLUP);          //Pin declarado como entrada, señal B del encoder de cuadratura
  attachInterrupt(0, Encoder, CHANGE);  //Leer señal A del encoder por interrupción, y asignar a Encoder
  tiempo1=millis();
}

void loop()
{
  tiempo2=millis();
  nh.spinOnce();
  if(tiempo2 > (tiempo1+1.5)){
    tiempo1 = millis();
    muestreo();
 
    tiempoSegundos = tiempo2;
    Serial.print("Ha transcurrido: ");
    Serial.println(tiempoSegundos);
    tiempo2=0;
    test.data = velocidad;
    p.publish( &test );
  }


  
  
  Serial.print ("Posicion:    ");
  Serial.print (revoluciones);
  Serial.print ("     Revoluciones  y      " );
  Serial.print (posicion);
  Serial.println ("    Grados" );
  Serial.print ("A una velocidad estimada de:    ");
  Serial.print (velocidad);
  Serial.println ("  Grados/segundos");
  Serial.println("");


}

//Función giro derecha del motor


//Función para realizar la lectura de las señales del encoder de cuadratura y
//definir el sentido de giro del motor
void Encoder()
{
  BSet = digitalReadFast(EncB);
  ASet = digitalReadFast(EncA);
  //Si ambas señales leidas son iguales, el motor gira en sentido antihorario
  //y se incrementa un contador para saber el número de lecturas
  if (BSet == ASet)
  {
    contador++;
    contaux++;
    posicion = contador * resolucion; //Convertir a grados
    posactual = contaux * resolucion;

    if (contador >= pulsos) //Contar por revoluciones
    {
      revoluciones++;
      contador = 0;
    }
  }
  //Si ambas señales leídas son distintas, el motor gira en sentido horario
  //y se decrementa un contador para saber el número de lecturas
  else
  {
    contador--;
    contaux--;
    posicion = contador * resolucion;
    posactual = contaux * resolucion; //Convertir a grados
    if (contador <= -pulsos) //Contar por revoluciones
    {
      revoluciones--;
      contador = 0;
    }
  }
}
//Función para estimar la velocidad por método de Euler
void muestreo()
{
  //Método de Euler para la estimación de la velocidad
  velocidad = ((posactual - posanterior) / 0.010); //Tiempo de muestreo = 0.015 segundos
  //Se pregunta por la velocidad, cuando hay una inversion de giro, para hacerla positiva
  if (velocidad < 0)
    velocidad = abs(velocidad);
    if(velocidad> 1000){
    velocidad= 1000;
    }
  //Actualizar la lectura de la posición anterior
  posanterior = posactual;

  if (x<0){
    velocidad=-velocidad;
    if(velocidad<-1000){
    velocidad=-1000;
    }
  }

  velocidad=velocidad/1000;
  

}
