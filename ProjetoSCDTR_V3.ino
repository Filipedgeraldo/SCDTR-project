#include "const.h"
#include "com.h"
#include <math.h>
#include <SPI.h>
#include <mcp2515.h>
#include <string.h>
#include <EEPROM.h>

const int analogInPin = A0; // Analog input pin that the potentiometer is attached to
const int analogOutPin = 3; // Analog output pin that the LED is attached to
const int n_nodes = 2;
int sensorValue = 0, outputValue = 0, N = 0, myID = 0, occ = 0, iter = 0;
bool ack [n_max];
int nodes[n_max] = {0};
bool check_acks_flag=0;
short config_flag=0, msg_iter = 1;
unsigned long t_ = 0, samp_time = 0, samp_time_prev = 0, elapsed_time = 0;
double E = 0, V = 0, F = 0, lux = 0, lux_prev = 0, lux_2prev = 0, lowref = 0, highref = 0, u = 0, cost = 1;
double volt = 0, vcc = 5, e_ant = 0, i_ant = 0, vi = 0, ref = 0, ref_prev = 0, dist = 0, rho = 1, tot = 0;
double y[3] = {0}, K[3] = {90,60,0}, d_av[3] = {0}, sol[3] = {0};

class gen_func{ //general functions

public:
  void Flicker();
  void Energy();
  void Visibility();
};

void gen_func::Flicker(){ //calculate flicker error
	F = F*(N-1)/N;
	if((lux - lux_prev)*(lux_prev - lux_2prev) < 0)
		F += (abs(lux - lux_prev) + abs(lux_prev - lux_2prev))/(2*0.01);
	lux_2prev = lux_prev;
	lux_prev = lux;
	F = F/N;
};

void gen_func::Visibility(){ //calculate visibility error
	V = V*(N-1)/N;
	if(ref - lux > 0)
		V += ref - lux;
	V = V/N;
};

void gen_func::Energy(){ //calculate spent energy
	E += u*(double)(samp_time - samp_time_prev)*pow(10,-6);
};

class st_mach{ //state machine

public:
  int values[2] = {0};
  char states[3] = {0};
  void readcommands();
  void run_state(char, int, int);
  double get_set_value(double);
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
      for(i = strlen(token) - 1; i >= 0; --i)
        values[m] += ((int)token[i] - 48)*pow(10, strlen(token) - 1 - i);
        
      m++;
    }
    token = strtok(NULL, " ");
    char *newline = strchr( token, '\n' );
    if ( newline )
      *newline = 0;
  }
};

void st_mach::run_state(char state, int node, int val){ //run the states read from the commands
  double res = 0;
  switch(state){
    case('g'): //get some specific value
      run_state(states[1],node,val);
      break;
    case('U'): //set/get Unoccupied Lower Bound for illuminance
	    res = get_set_value(lowref);
	    if(states[0] != 'g') lowref = res;
      break;
    case('O'): //set/get Occupied Lower Bound for illuminance
	    res = get_set_value(highref);
	    if(states[0] != 'g') highref = res;
      break;
    case('L'): //current illuminance
	    if(occ == 0) res = get_set_value(lowref);
	    else res = get_set_value(highref);
      break;
    case('o'): //set occupancy
	    res = get_set_value(occ);
	    if(states[0] != 'g') occ = res;
      break;
    case('l'): //measured illuminance
	    res = get_set_value(lux);
      break;
    case('d'): //duty-cycle
      res = get_set_value(u);
      break;
    case('x'): //external illuminance
	    res = get_set_value(dist);
      break;
    case('f'): //flicker
		if(states[2] != 'T') res = get_set_value(F);
		else{
			if(myID == 1){
				tot = F;
				for(int i = 1; i <= 2;i++)
					if(i != myID) send_message(i,myID,8,states[1],0);
			}
			else send_message(1,myID,8,states[1],F);
		}
      break;
    case('v'): //visibility
	    if(states[2] != 'T') res = get_set_value(V);
		else{
			if(myID == 1){
				tot = V;
				for(int i = 1; i <= 2;i++)
					if(i != myID) send_message(i,myID,8,states[1],0);
			}
			else send_message(1,myID,8,states[1],V);
		}
      break;
    case('c'): //current energy cost
	    res = get_set_value(cost);
	    if(states[0] != 'g') lowref = res;
      break;
    case('e'): //energy consumption
	    if(states[2] != 'T') res = get_set_value(E);
		else{
			if(myID == 1){
				tot = E;
				for(int i = 1; i <= 2;i++)
					if(i != myID) send_message(i,myID,8,states[1],0);
			}
			else send_message(1,myID,8,states[1],E);
		}
      break;
    case('p'): //power consumption
		if(states[2] != 'T') res = get_set_value(E*E);
		else{
			if(myID == 1){
				tot = E*E;
				for(int i = 1; i <= 2;i++)
					if(i != myID) send_message(i,myID,8,states[1],0);
			}
			else send_message(1,myID,8,states[1],E*E);
		}
      break;
	case('t'): //time of last restart
		res = get_set_value((double)(micros() - elapsed_time)*pow(10,-6));
      break;
    case('s'): //start/end stream
      break;
    case('r'): //restart the system/get reference value
      break;
    default:
	  Serial.print("err\n");
      break;
  }
  if(myID == 1){
	  if(states[0] == 'g' && node == 1)
	  {
		  Serial.print(states[1]);
		  Serial.print(" ");
		  Serial.print(values[0]);
		  Serial.print(" ");
		  Serial.print(res);
		  Serial.print("\n");
	  }
	  else if(states[0] != 'g') Serial.print("ack\n");
  }
};

double st_mach::get_set_value(double val)
{
	if(states[0] == 'g'){
		if(myID == 1 && values[0] == 1) return val;
		else if(myID == 1) send_message(values[0],1,11,states[1],0);
		else send_message(1,myID,11,states[1],val);
	}
	else{
		if(myID == 1 && values[0] == 1) return values[1];
		else if(myID == 1){
			send_message(values[0],myID,10,states[0],values[1]);
			return val;
		}
		else return values[1];
	}
  return 0;
};

class Opt{ //optimization

public:
  double Evaluate_cost(double[]);
  double NormSq(double[]);
  double Dot(double[], double[]);
  void Consensus_iterate();
  int isFeasible(double[]);
  
};

double Opt::Evaluate_cost(double vect[]){ //evaluate cost of given solution
  double c = cost*vect[myID - 1], aux = 0;
  for(int i = 0; i < 2; i++){
    aux = vect[i] - d_av[i];
    c +=  y[i]*aux + rho*(aux*aux)/2; 
  } 
  
  return c;
}

double Opt::NormSq(double vect[]){ //calculate squared norm of an array
  double res = 0;
  for(int i = 0; i < 2; i++)
    res += vect[i]*vect[i];

  return res;
}

double Opt::Dot(double a[], double b[]){
  double res = 0;
  for(int i = 0; i < 2; i++)
    res += a[i]*b[i];
    
  return res;
}

void Opt::Consensus_iterate(){ //run an iteration of the consensus algorithm
  double d_u[3] = {0}, d_bL[3] = {0}, d_b1[3] = {0}, d_b0[3] = {0}, d_L1[3] = {0}, d_L0[3] = {0};
  double z[3] = {0};
  double cost_best = 10000000, c = 0, m = NormSq(K) - K[myID-1]*K[myID-1], dot = 0;
  
  for(int i = 0; i < 2; i++)
  {
    d_av[i] = 0.5*d_av[i];
    y[i] += rho*(sol[i] - d_av[i]);
    if(i == myID-1)
      z[i] = rho*d_av[i] - cost - y[i];
    else z[i] = rho*d_av[i] - y[i];
  }

  dot = Dot(K,z);

  for(int i = 0; i < 2; i++){
    d_u[i] = z[i]/rho;
    d_bL[i] = z[i]/rho - (K[i]/NormSq(K))*(dist - ref + dot/rho);
    d_b0[i] = z[i]/rho;
    d_b1[i] = z[i]/rho;
    d_L1[i] = z[i]/rho - K[i]*(dist - ref + K[myID-1])/m + (1/(rho*m))*K[i]*(K[myID-1]*z[myID-1] - dot);
    d_L0[i] = z[i]/rho - K[i]*(dist - ref)/m + (1/(rho*m))*K[i]*(K[myID-1]*z[myID-1] - dot);
  }
  
  d_b0[myID-1] = 0;
  d_b1[myID-1] = 1;
  d_L0[myID-1] = 0;
  d_L1[myID-1] = 1;

  
  if(isFeasible(d_u)) //unconstrained
  {
    c = Evaluate_cost(d_u);
    if(c < cost_best)
    {
      cost_best = c;
      for(int i = 0; i < 2; i++)
        sol[i] = d_u[i];
    }
    return;
  }
  if(isFeasible(d_bL)) //linear boundary
  {
    c = Evaluate_cost(d_bL);
    if(c < cost_best)
    {
      cost_best = c;
      for(int i = 0; i < 2; i++)
        sol[i] = d_bL[i];
    }
  }
  if(isFeasible(d_b0)) //0 boundary
  {
    c = Evaluate_cost(d_b0);
    if(c < cost_best)
    {
      cost_best = c;
      for(int i = 0; i < 2; i++)
        sol[i] = d_b0[i];
    }
  }
  if(isFeasible(d_b1)) //100 boundary
  {
    c = Evaluate_cost(d_b1);
    if(c < cost_best)
    {
      cost_best = c;
      for(int i = 0; i < 2; i++)
        sol[i] = d_b1[i];
    }
  }
  if(isFeasible(d_L0)) //linear and 0 boundaries
  {
    c = Evaluate_cost(d_L0);
    if(c < cost_best)
    {
      cost_best = c;
      for(int i = 0; i < 2; i++)
        sol[i] = d_L0[i];
    }
  }
  if(isFeasible(d_L1)) //linear and 100 boundaries
  {
    c = Evaluate_cost(d_L1);
    if(c < cost_best)
    {
      cost_best = c;
      for(int i = 0; i < 2; i++)
        sol[i] = d_L1[i];
    }
  }
}

int Opt::isFeasible(double duty[]){ //check if node's duty cycle is feasible
  double tol = 1e-3;
  if(duty[myID-1] < - tol)return 0;
  else if(duty[myID-1] > 1 + tol) return 0;
  else if(Dot(K,duty) + dist + tol < ref) return 0;
  
  return 1;
}

class P_I{ //PI controller

private:
  float Kp, Ki;

public:
  void init(double, double);
  void calc_step(double,double);
};

void P_I::init(double p, double i){ //initiate controller gains
  Kp = p;
  Ki = i;
};

void P_I::calc_step(double y_med, double y_pred){ //controller implemented as a PI controller
  double u_ff = 0, e = 0, p = 0, i = 0;
  Opt optim;

  if(msg_iter == 1) optim.Consensus_iterate();

  if(iter < 100 && msg_iter == 1){
    u = sol[myID-1];
    for(int j = 0; j < 2; j++){
      d_av[j] = sol[j];
      for(int a = 1; a <= 2; a++)
        if(a != myID)
          send_message(a,myID,9,sol[j],j);
    }
    msg_iter = 0;
    iter++;
  }

  if(iter == 100){
    msg_iter = 0;
    iter++;
  }
  
  //if(u*K[myID-1] < y_med) dist = y_med - u*K[myID-1];
  
  
  /*
  if(y_med > ref) dist = y_med - ref; //measuring disturbance
  
  u_ff = (ref - dist)/G; //feedforward term
  
  e = y_pred - y_med;
  
  if(e < 0) e = 0; //if illuminance is bigger than the lower bound, we don't need the error

  //feedback term (implemented as a PI)
  p = Kp*e; 
  i = i_ant + Ki*(e + e_ant);  

  if(u_ff + p + i > 1) i = 1 - u_ff - p; //anti-windup term (saturate integral term)
  else if (u_ff + p + i < 0) i = - u_ff - p;
  
  u = u_ff + p + i;
  i_ant = i;
  e_ant = e;*/

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

void initialProcedure(){
  struct message m_read;
  bool has_data;
  unsigned int counter=0;
  bool done_acks=0;
  int state_config=0, node_config=0, total = 0;
  char command;
  float* slopes;
  st_mach R;    

  if( interrupt){
    m_read = read_message( &has_data);
    while (has_data){
      /*#ifdef DEBUG
        Serial.print( "\t \t Receiving: " );
        Serial.println( m_read.value1.value );
        Serial.print( "\t \t Receiving: " );
        Serial.println( m_read.value2.value );
        Serial.print( "\t \t to: " );
        Serial.print(m_read.id.to);
        Serial.print( "\t \t from: " );
        Serial.print(m_read.id.from);
        Serial.print( "\t \t code: " );
        Serial.println(m_read.id.code);
      #endif*/
      switch (m_read.id.code){
        //beguining of start up procedure messages
        case 0:
          if (myID==1){
            nodes[m_read.id.from]=1;
            config_flag=1;
          }
          else if (m_read.id.from==1){
            config_flag=1;
            // !!!!!!!! stops all streams of data !!!!!!!!
            // !!!!!!! put output at zero !!!!!!!
            send_message(m_read.id.from,myID, 1, 0, 0);
          }
          break;
        case 1:
          if(nodes[m_read.id.from] == 0){
            nodes[m_read.id.from]=1;
          }
          ack[m_read.id.from]=1; //put one in the vector
          break;
        case 2:
          Serial.print("Node with light on: ");
          Serial.print(m_read.value1.value);
          Serial.print("\n");
          break;
        case 3: 
          if(m_read.value1.value==0){
            analogWrite(analogOutPin, 0);  
            K[m_read.id.from - 1] = calibrateGain();       
          }
          else if(m_read.value1.value==1){
            analogWrite(analogOutPin, 255);
            K[myID - 1] = calibrateGain();
          }
          break;
        case 4:
          config_flag=2;
          send_message(1,myID,1,0,4);
          break;
        case 5:
          //not use 
          break;
        //end of start up procedure messages
        //beginning of hub function messages
        case 6:
          break;
        case 7:
          break;
        case 8:
		  total = 1;
		  if(myID == 1){
			  res += m_read.value2.value;
			  command = m_read.value1.value;;
		  }
		  else{
			  R.states[2] = 'T';
			  R.states[1] = m_read.value1.value;
			  R.states[0] = 'g';
			  R.run_state(R.states[0],myID,0);
		  }
          break;
        case 9:
          if(m_read.id.to == myID) d_av[(int)m_read.value2.value] += m_read.value1.value;
          break;
        case 10:
          R.states[0] = m_read.value1.value;
          R.values[0] = myID;
          R.values[1] = m_read.value2.value;
          R.run_state(m_read.value1.value, myID, m_read.value2.value);
          break;
        case 11:
          if(myID == 1){
            Serial.print((char)m_read.value1.value);
            Serial.print(" ");
            Serial.print(m_read.id.from);
            Serial.print(" ");
            Serial.print(m_read.value2.value);
            Serial.print("\n");
          }
          else {
            R.states[0] = 'g';
            R.states[1] = m_read.value1.value;
            R.run_state(m_read.value1.value, myID, m_read.value2.value);
          }
          break;
        case 12:
          iter = 0;
          for(int i = 0; i < 2; i++){
            y[i] = 0;
            d_av[i] = 0;
            sol[i] = 0;
          }
          break;
        case 13:
          break;
        case 14:
          Serial.println("Code 14");
          break;  
      }
      m_read = read_message(&has_data);
      if(m_read.id.code == 9 || m_read.id.code == 12) msg_iter = 1;
    }
	if(total == 1)
	{
		Serial.print(command);
		Serial.print(" ");
		Serial.print(res);
	}
  }

  //checking acks
  if (check_acks_flag==1){
    done_acks=check_acks(ack,nodes);
    if (done_acks==1){
      check_acks_flag=0;
      //reset ack vector
      for(int i=0;i<n_max;i++){
        ack[i]=0;
      }
    }
  }
  
  if(done_acks==1 && config_flag==1){
    done_acks=0;
    for(int i = 0; i < 2; i++){
      K[i] = config_setup(&check_acks_flag, ack, i, 0, n_max, nodes,&done_acks);
    }
    config_flag = 2;
  }

  //indicates its presence to node 1
  if (myID!=1 && config_flag==0){
    if (counter<=1000){
      counter+=1;
    }
    else{
      send_message(1,myID,0,0,0);
      counter=0;
    }
  }
}

volatile bool flag;
ISR(TIMER1_COMPA_vect){
  flag = 0;
}

void setup() { // put your setup code here, to run once:
  Serial.begin(500000);
  elapsed_time = micros();
  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING); //use interrupt at pin 2
  //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
  SPI.usingInterrupt(0);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  //getting id from eeprom
  myID=EEPROM.read(0);

  //setting filters and mask to receive only messages with my ID and generic ID
  #ifdef FILTERS
  mcp2515.setConfigMode();
  mcp2515.setFilterMask(MCP2515::MASK0, 0, mask);
  mcp2515.setFilter(MCP2515::RXF0, 0, filt1);
  mcp2515.setFilter(MCP2515::RXF1, 0, myID);
  mcp2515.setFilterMask(MCP2515::MASK1, 0, mask);
  mcp2515.setFilter(MCP2515::RXF2, 0, myID);
  mcp2515.setFilter(MCP2515::RXF3, 0, filt1);
  #endif

  #ifndef LOOPBACK
  mcp2515.setNormalMode();
  #endif
  #ifdef LOOPBACK
  mcp2515.setLoopbackMode();
  #endif

  /*#ifdef DEBUG
    Serial.print("myId: ");
    Serial.println(myID);
  #endif*/

  nodes[myID]=1;
  ack[myID]=1;

  if (myID==1){
    send_message(0,myID, 0, 0, 0);
    config_flag=1;
    check_acks_flag=1;    
  }
  
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
}

void loop(){ // put your main code here, to run repeatedly:
  double aux = 0, pred = 0;
  int com = 0;
  P_I controller;
  Simulator S;
  st_mach R;
  gen_func Gen;

  initialProcedure();

  if(myID == 1){
    while(Serial.available()){
      R.readcommands();
      R.run_state(R.states[0], R.values[0], R.values[1]);
      break;
    } 
  }
  
  if(occ == 0) ref = lowref;
  else ref = highref;


  if(flag == 0){
	  if(ref != 0) N += 1;
	  if(ref != ref_prev)
	  {
      iter = 0;
      for(int i = 1; i <= 2;i++){
        y[i-1] = 0;
        d_av[i-1] = 0;
        sol[i-1] = 0;
        if(i != myID)
          send_message(i,myID, 12, 0, 0);
      }
          
		  t_ = micros();
		  sensorValue = analogRead(analogInPin);
		  vi = (sensorValue*vcc)/1023;
		  S.init(K[myID-1], t_, vi);
		  controller.init(0.01,0.002);
	  }
    pred = VoltToLux(S.Sim(ref/K[myID-1]));
    sensorValue = analogRead(analogInPin);
    volt = (sensorValue*vcc)/1023;
    lux = VoltToLux(volt);
    samp_time = micros();
    controller.calc_step(lux, pred);
	
    if(N != 0){
	    Gen.Visibility();
	    Gen.Flicker();
	    Gen.Energy();
    }
    
	samp_time_prev = samp_time;
    ref_prev = ref;
    flag = 1;
  }
}
