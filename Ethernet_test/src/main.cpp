#include <mbed.h>
#include "EthernetInterface.h"
#include "TimEncoders/Nucleo_Encoder_16_bits.h"
#include <math.h>

// Creating Ethernet object
EthernetInterface eth;
// Static IP network variables
const int out_PORT = 8150;
const int PORT = 8151;
static const char* mbedIP       = "192.168.3.3";  //IP 
static const char* mbedMask     = "255.255.255.0";  // Mask
static const char* mbedGateway  = "192.168.3.1";    //Gateway
static const char* recvIP = "192.168.3.2";

DigitalOut Enable_m1(PE_2);
DigitalOut Dir_m1(PE_3);
AnalogOut Set_m1(PA_5);
AnalogIn ActCur_m1(PC_3);

DigitalOut Enable_m2(PG_0);
DigitalOut Dir_m2(PG_1);
AnalogOut Set_m2(PA_4);
AnalogIn ActCur_m2(PF_8);

DigitalIn motor_enable(PC_13);

Timer timer;
float t;
int elapsed_t = 0;
int prev_t = 0;
int geted_t = 0;
const int period = 2;// in ms

///encoder m1
Nucleo_Encoder_16_bits enc_m1(TIM1);
float enc_m1_pos;
int enc_m1_cpt = 1024;
float enc_m1_scale = 2*3.14/(enc_m1_cpt*4);

//encoder m2
Nucleo_Encoder_16_bits enc_m2(TIM3);
float enc_m2_pos;
int enc_m2_cpt = 1024;
float enc_m2_scale = 2*3.14/(enc_m2_cpt*4);

//encoder lin
Nucleo_Encoder_16_bits enc_lin(TIM4); 
int enc_lin_count = 0;
float enc_lin_pos;
int lin_enc_cpi = 360;
float lin_enc_scale = 25.4/(lin_enc_cpi*4);

float min_lin_enc;
float max_lin_enc;
float min_m1_enc;
float min_m2_enc;

int sign(float x){
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

float sat(float x, float x_min, float x_max){
  if (x > x_max ){
    return x_max;
  }
  if (x <= x_min ){
    return x_min;
  }
  return x;
}

void control(float u1, float u2){

  float u_max = 5;
  float u_min = -5;

  u1 = sat(u1, u_min, u_max);
  u2 = sat(u2, u_min, u_max);
  //printf("%f,%f\n",u1,u2);
  if (u1 > 0 ){
    Dir_m1.write(true);
  }
  else{
    Dir_m1.write(false);
  }

  if (u2 > 0 ){
    Dir_m2.write(true);
  }
  else{
    Dir_m2.write(false);
  }
  u1 = abs(u1)/5;
  u2 = abs(u2)/5;
  Set_m1.write(u1);
  Set_m2.write(u2);
}

void calibration(){
  if (enc_lin_pos < min_lin_enc){
    min_lin_enc = enc_lin_pos;
    min_m2_enc = enc_m2_pos;
  }
  if (enc_lin_pos > max_lin_enc){
    max_lin_enc = enc_lin_pos;
    min_m1_enc = enc_m1_pos;
  }
}

int main() {

  // Setting up communication channel
  eth.set_network(mbedIP, mbedMask, mbedGateway);// set device addres
  // Start network
  eth.connect();
  SocketAddress td_addr(recvIP, PORT);
  SocketAddress rec_addr(mbedIP, out_PORT);
  UDPSocket td_sock(&eth);
  td_sock.set_blocking(false);
  td_sock.set_timeout(period);
  int buffer[8]; // buffer to store output data 

    // put your setup code here, to run once:

  timer.start(); // Start global system timer 

  while(1) {
    if((timer.read_us() - elapsed_t)>period*1000-1){

      // update state
      elapsed_t = timer.read_us();
      t = elapsed_t/1E6;

      long int enc_m1_count = enc_m1.GetCounter();
      enc_m1_pos = enc_m1_scale*enc_m1_count;

      long int enc_m2_count = enc_m2.GetCounter();
      enc_m2_pos = enc_m2_scale*enc_m2_count;
      
      long int enc_lin_count = enc_lin.GetCounter();
      enc_lin_pos = lin_enc_scale*enc_lin_count;
      
      calibration();

      float x0 = (max_lin_enc - min_lin_enc)/2;
      float x = enc_lin_pos - min_lin_enc - x0;
      float q1 = enc_m1_pos - min_m1_enc;
      float q2 = enc_m2_pos - min_m2_enc;

      // send information
      buffer[0] = int(t*1E6); // elapsed time in micro seconds
      buffer[1] = prev_t - geted_t; // actual linear position 
      buffer[2] = x*1E3; // desired motor position 
      buffer[3] = q1*1E3; // actual motor position
      buffer[4] = q2*1E3; // actual linear acceleration
      buffer[5] = ActCur_m1.read(); // desired current
      buffer[6] = ActCur_m2.read(); // actual current
      buffer[7] = 0; // Measured force
      prev_t = t*1E6;

      td_sock.sendto(td_addr, &buffer, sizeof(buffer)); // send buffer to chosen client
      
      //request information
      int in_buffer[3];
      int n = td_sock.recvfrom(&rec_addr, &in_buffer, sizeof(in_buffer));
      geted_t = in_buffer[0];
      //printf("%d,%d\n",in_buffer[1],in_buffer[2]);
      if (n > 0){
        Enable_m1.write(motor_enable.read());
        Enable_m2.write(motor_enable.read());
        float u1 = float(in_buffer[1])/1000.;
        float u2 = float(in_buffer[2])/1000.;
        //printf("m1");
        //realize control
        control(u1, u2);
      }
      else{
        //printf("m2 ");
        control(0, 0);
        Enable_m1.write(false);
        Enable_m2.write(false);
      }
    }
  }
}