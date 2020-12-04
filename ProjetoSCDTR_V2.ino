#include <math.h>
#include <SPI.h>
#include <mcp2515.h>
#include <string.h>

const int analogInPin = A0; // Analog input pin that the potentiometer is attached to
const int analogOutPin = 3; // Analog output pin that the LED is attached to
int sensorValue = 0, outputValue = 0, N = 0, ID = 0;
unsigned long t_ = 0, samp_time = 0, samp_time_prev = 0;
double E = 0, V = 0, F = 0, lux = 0, lux_prev = 0, lux_2prev = 0;
double vcc = 5, G = 0, volt = 0, e_ant = 0, i_ant = 0, vi = 0, ref = 0, ref_prev = 0;
int values[2] = {0};
char states[3] = {0};

class gen_func{ //general functions

public:
  void Flicker();
  void Energy();
  void Visibility();
};

void Flicker::gen_func(){ //calculate flicker error
	F = F*(N-1)/N;
	if((lux - lux_prev)*(lux_prev - lux_2prev) < 0)
		F += (abs(lux - lux_prev) + abs(lux_prev - lux_2prev))/(2*0.01);
	lux_2prev = lux_prev;
	lux_prev = lux;
	F = F/N;
};

void Visibility::gen_func(){ //calculate visibility error
	V = V*(N-1)/N;
	if(ref - lux > 0)
		V += ref - lux;
	V = V/N;
};

void Energy::gen_func(){ //calculate spent energy
	E += u*(samp_time - samp_time_prev)*10^(-6);
};

class st_mach{ //state machine
  
private:
  char state, state_prev;

public:
  void readcommands();
  void run_state();
};

void st_mach::readcommands(){ //function to read commands from Serial
  char* token, aux;
  int k = 0, m = 0, i = 0, len = 0;
  String A;
  
  A = Serial.readString();
  len = A.length();
  char arr[len+1];
  strcpy(arr,A.c_str());
  token = strtok(arr, " ");
  while(token != NULL){
    if(strlen(token) == 1 && token[0] > 57){
      states[k] = token[0];
      k++;
    }
    else{
      values[m] = 0;
      for(i = strlen(token) - 1; i >= 0; --i){
        values[m] += (token[i] - 48)*pow(10, strlen(token) - 1 - i);
      }
      m++;
    }
    token = strtok(NULL, " ");
  }
  state = states[0];
};

void st_mach::run_state(){ //run the states read from the commands
  switch(state){
    case("g"): //get some specific value
      state = states[1];
      run_state();
      break;
    case("U"): //set/get Unoccupied Lower Bound for illuminance
      break;
    case("O"): //set/get Occupied Lower Bound for illuminance
      break;
    case("L"): //current illuminance
      break;
    case("o"): //set occupancy
      break;
    case("l"): //measured illuminance
      break;
    case("x"): //external illuminance
      break
    case("f"): //flicker
      break;
    case("v"): //visibility
      break;
    case("c"): //current energy cost
      break;
    case("e"): //energy consumption
      break;
    case("p"): //power consumption
      break;
    case("s"): //start/end stream
      break;
    case("r"): //restart the system/get reference value
      break;
    default:
      break;
  }
  state_prev = state;
}

class P_I{ //PI controller

private:
  float Kp, Ki;

public:
  void init(double, double);
  void calc_step(double,double,double);
};

void P_I::init(double p, double i){ //initiate controller gains
  Kp = p;
  Ki = i;
};

void P_I::calc_step(double ref, double y, double y_pred){ //controller implemented as a PI controller
  double u_ff = 0, u = 0, e = 0, p = 0, i = 0, dist = 0;
  
  if(y > ref) dist = y - ref; //measuring disturbance
  
  u_ff = (ref - dist)/G; //feedforward term
  
  e = y_pred - y;
  
  if(abs(e) < 0.04*ref) e = 0; //dead zone
  
  p = Kp*e; //feedback term (implemented as a PI)
  i = i_ant + Ki*(e + e_ant);  

  if(u_ff + p + i > 1) i = 1 - u_ff - p; //anti-windup term (saturate integral term)
  else if (u_ff + p + i < 0) i = - u_ff - p;
  
  u = u_ff + p + i;
  i_ant = i;
  e_ant = e;

  analogWrite(analogOutPin, u*255);
};

class Simulator{ //system simulator

private:
  double gain, t, vi;

public:
  void init(double, double, double);
  double Sim(double);
};

void Simulator::init(double g, double t_init, double v_init){
  gain = g;
  t = t_init;
  vi = v_init;
};

double Simulator::Sim(double U){
  double lux_ = 0, tau = 0, R = 0, delta_V = 0, vf = 0, theta = 0;
  lux_ = U*gain + 0.05;
  R = pow(10, -0.66*(log10(lux_)) + 1.903);
  tau = 0.0726/(pow(U*100, 0.434));
  vf = (vcc*10)/(10+R);
  theta = 1000;
  delta_V = vf - (vf - vi)*exp((-1)*((micros() - t - theta)*pow(10,-6))/(tau));
  return delta_V;
};

double VoltToLux(double v){
  double R_ldr = 0;
  
  R_ldr = 10*(vcc - v)/v;
  return pow(10, (log10(R_ldr) - 1.903)/(-0.66));
}

double calibrateGain(){
  int j = 0;
  double g = 0;
 
  analogWrite(analogOutPin, 255);
  delay(100);
  
  while(j != 100){
    sensorValue = analogRead(analogInPin);
    volt = (sensorValue*vcc)/1023;
    g += VoltToLux(volt);
    j++;
  }
  
  analogWrite(analogOutPin, 0);
  return g/100;
}

class Opt{ //optimization
	
public:
	double Evaluate_cost(double*);
	double NormSq(double*);
	double* Consensus_iterate();
	int isFeasible(double);
	
};

double Opt::Evaluate_cost(double* sol){ //evaluate cost of given solution
	double c = 0;
	for(int i = 0; i < 3; i++)
		c += cost[i]*sol[i] + y[i]*(sol[i] - d_av[i]) + rho*(pow(sol[i],2) - pow(d_av[i],2))/2; 
	
	return c;
}

double Opt::NormSq(double* vect){ //calculate squared norm of an array
	double res = 0;
	for(int i = 0; i < 3; i++)
		res += pow(vect[i],2);

	return res;
}

double* Opt::Consensus_iterate(){ //run an iteration of the consensus algorithm
	double d_u[3] = {0}, d_bL[3] = {0}, d_b1[3] = {0}, d_b0[3] = {0}, d_L1[3] = {0}, d_L0[3] = {0};
	double z = 0, cost_best = 100000, c = 0, m = 0;
	double* sol;
	
	for(int i = 0; i < 3; i++)
	{
		d_u[i] = z[i]/rho;
		d_bL[i] = z[i]/rho - (K[i]/NormSq(K))*(dist - ref + K[i]*z[i]/rho);
		d_b0[i] = z[i]/rho;
		d_b1[i] = z[i]/rho;
		m = NormSq(K) - K[ID]^2;
		d_L1[i] = z[i]/rho - K[i]*(dist - ref)/m + (1/(rho*m))*K[i]*(K[ID] + z[ID] - z[i]*K[i]);
		d_L0[i] = z[i]/rho - K[i]*(dist - ref)/m + (1/(rho*m))*K[i]*(K[ID] + z[ID] - z[i]*K[i]);
		
		if(i == ID)
		{
			d_b0[i] = 0;
			d_b1[i] = 1;
			d_L1[i] = 1;
			d_L0[i] = 0;
		}
	}
	
	if(isFeasible(d_u[ID])) //unconstrained
	{
		c = Evaluate_cost(d_u);
		if(c < cost_best)
			cost_best = c;
		return d_u;
	}
	else if(isFeasible(d_bL[ID])) //linear boundary
	{
		c = Evaluate_cost(d_bL);
		if(c < cost_best)
		{
			cost_best = c;
			sol = d_bL;
		}
	}
	else if(isFeasible(d_b0[ID])) //0 boundary
	{
		c = Evaluate_cost(d_b0);
		if(c < cost_best)
		{
			cost_best = c;
			sol = d_b0;
		}
	}
	else if(isFeasible(d_b1[ID])) //100 boundary
	{
		c = Evaluate_cost(d_b1);
		if(c < cost_best)
		{
			cost_best = c;
			sol = d_b1;
		}
	}
	else if(isFeasible(d_L0[ID])) //linear and 0 boundaries
	{
		c = Evaluate_cost(d_L0);
		if(c < cost_best)
		{
			cost_best = c;
			sol = d_L0;
		}
	}
	else if(isFeasible(d_L1[ID])) //linear and 100 boundaries
	{
		c = Evaluate_cost(d_L1);
		if(c < cost_best)
		{
			cost_best = c;
			sol = d_L1;
		}
	}
	return sol;
}

int Opt::isFeasible(double duty){ //check if node's duty cycle is feasible
	double tol = 0.001;
	if(duty < 0 + tol) return 0;
	else if(duty > 1 + tol) return 0;
	else if(duty*gain + dist < ref) return 0;
	
	return 1;
}

volatile bool flag;
ISR(TIMER1_COMPA_vect){
  flag = 1;
}

void setup() { // put your setup code here, to run once:
  Serial.begin(38400);
  EEPROM.write(0,ID);
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  mcp2515.setLoopbackMode();
  TCCR2B = (TCCR2B & B11111000) | B00000010; //increase PWM frequency (3921.16 Hz)
  cli(); //setup TIMER1 registers to assure sampling time of ~10 ms
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 155;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10)|(1 << CS12);
  TIMSK1 |= (1 << OCIE1A);
  sei();
  G = calibrateGain(); //calculates the gain when the system starts
}

void loop(){ // put your main code here, to run repeatedly:
  double ref = 0, aux = 0, pred = 0;
  P_I controller;
  Simulator S;
  st_mach R;
  gen_func Gen;
  
  while(Serial.available()){
    R.readcommands();
    Serial.print(states[0]);
    Serial.print("\t");
    Serial.print(states[1]);
    Serial.print("\t");
	Serial.print(states[2]);
    Serial.print("\t");
    Serial.print(values[0]);
    Serial.print("\t");
    Serial.print(values[1]);
    Serial.print("\n");
  }
/*

 
  if(flag == 0){
	N += 1;
	
	if(ref != ref_prev)
	{
		t_ = micros();
		sensorValue = analogRead(analogInPin);
		vi = (sensorValue*vcc)/1023;
		S.init(G, t_, vi);
		controller.init(0.01,0.002);
	}
	
    pred = VoltToLux(S.Sim(ref/G));
    sensorValue = analogRead(analogInPin);
    volt = (sensorValue*vcc)/1023;
    lux = VoltToLux(volt);
	samp_time = micros();
    controller.calc_step(ref, lux, pred);
	Gen.Visibility();
	Gen.Flicker();
	Gen.Energy();
	samp_time_prev = samp_time;
  }
*/
}
