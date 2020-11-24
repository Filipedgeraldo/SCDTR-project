#include <math.h>

const int analogInPin = A0; // Analog input pin that the potentiometer is attached to
const int analogOutPin = 3; // Analog output pin that the LED is attached to
int sensorValue = 0, outputValue = 0;
double vcc = 5, lux = 0, G = 0, u = 0, volt = 0;
double e_ant = 0, y_ant = 0, i_ant = 0, time = 0;

double Simulator(double U, double t, double vi){
  double lux_ = 0, tau = 0, R = 0, V = 0, vf = 0, theta = 0;
  lux_ = U*G;
  R = pow(10, -0.66*(log10(lux_)) + 1.903);
  tau = (pow(10,-6)*(R*10))/(R + 10);
  vf = (vcc*10)/(10+R);
  theta = 5000; 
  V = vf - (vf - vi)*exp((-1)*((micros() - t - theta)*pow(10,-6))/(tau*pow(10,3)));
  return VoltToLux(V);
}

double VoltToLux(double v){
  double R_ldr = 0;
  
  R_ldr = 10*(vcc - v)/v;
  return pow(10, (log10(R_ldr) - 1.903)/(-0.66));
}


double calibrateGain(){
  int j = 0;
  double gain = 0;
 
  analogWrite(analogOutPin, 255);
  delay(100);
  
  while(j != 400){
    sensorValue = analogRead(analogInPin);
    volt = (sensorValue*vcc)/1023;
    gain += VoltToLux(volt);
    j++;
    delay(1);
  }
  return gain/400;
}


void PID(double ref, double y){
  double u_ff = 0, u_fb = 0, e = 0;
  double k1 = 0.01, k2 = 0.0008; //0.01,0.01
  double p = 0, i = 0, dist = 0;

  if(y > ref) dist = y - ref;
  
  //feedforward term
  u_ff = (ref - dist)/G;

  
  //feedback term (implemented as a PID)
  e = ref - y;
  p = k1*e;
  i = i_ant + k2*(e + e_ant);
  u_fb = p + i;
  u = u_ff + u_fb;

  
  //anti-windup term
  if(u > 1)
  {
    i = 1 - u_ff - p;
    u = 1;
  }
  else if (u < 0){
    i = -u_ff - p;
    u = 0;
  }

  y_ant = y;
  i_ant = i;
  e_ant = e;
  
  outputValue = (u*255);
  analogWrite(analogOutPin, outputValue);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  TCCR1B = TCCR1B & B11111000 | B00000010;
  G = calibrateGain(); //calculates the gain when the system starts
  analogWrite(analogOutPin, 0);
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  double ref1 = 90.0, ref2 = 10.0, ref = 0, pred = 0, vi = 0;

  /*int i = 0, j = 0;
  
  for(j = 0; j < 11; j++)
  {
    i = 0;
    time = micros();
    sensorValue = analogRead(analogInPin);
    vi = (sensorValue*vcc)/1023;
    while(i < 600){
      analogWrite(analogOutPin, j*255/10);
      sensorValue = analogRead(analogInPin);
      volt = (sensorValue*vcc)/1023;
      lux = VoltToLux(volt);
      pred = Simulator(j/(double)10, time, vi);
      Serial.print(j*10);
      Serial.print("\t S: ");
      Serial.print(pred);
      Serial.print("\t R: ");
      Serial.print(lux);
      Serial.print("\n");
      i++;
    }

    i = 0;
    time = micros();
    sensorValue = analogRead(analogInPin);
    vi = (sensorValue*vcc)/1023;

    while(i < 600){
      analogWrite(analogOutPin, 0);
      sensorValue = analogRead(analogInPin);
      volt = (sensorValue*vcc)/1023;
      lux = VoltToLux(volt);
      pred = Simulator(0, time,vi);
      Serial.print(0);
      Serial.print("\t S: ");
      Serial.print(pred);
      Serial.print("\t R: ");
      Serial.print(lux);
      Serial.print("\n");
      i++;
    }
  }*/
  
  sensorValue = analogRead(analogInPin);
  volt = (sensorValue*vcc)/1023;
  lux = VoltToLux(volt);
  PID(ref, lux);
  Serial.println(lux);
}
