#include"mbed.h"
#include "bbcar.h"
#include "bbcar_rpc.h"

Ticker servo_ticker;
PwmOut pin5(D5), pin6(D6);
BBCar car(pin5, pin6, servo_ticker);

BufferedSerial pc(USBTX,USBRX); //tx,rx                                                                                                                                 
BufferedSerial xbee(D1,D0); //tx,rx   
BufferedSerial uart(D10,D9); //tx,rx                                                                                                                                  

DigitalInOut ping(D11);
Timer t;

Thread thread_1,thread_2;
int flag=1;


void task(Arguments *in, Reply *out){
int x = in->getArg<double>();

if(x==1)
xbee.write("Task 1 completed!\r\n", 19);

else
xbee.write("Task 3 completed!\r\n", 19);

}

RPCFunction rpcTask(&task, "task");

void mission(Arguments *in, Reply *out){
int x = in->getArg<double>();

if(x==0)
xbee.write("Mission start!\r\n",16);

else 
xbee.write("Mission complete!\r\n",17);

}

RPCFunction rpcMission(&mission, "mission");


void rpc_1(){
 char buf[256], outbuf[256];
   FILE *devin = fdopen(&uart, "r");
   FILE *devout = fdopen(&uart, "w");
   uart.set_baud(9600);

   while(1){
       memset(buf, 0, 256);
      for( int i = 0; ; i++ ) {
         char recv = fgetc(devin);
         if(recv == '\n') {
            printf("\r\n");
            break;
         }
         buf[i] = fputc(recv, devout);
      }
   RPC::call(buf, outbuf);

   }
}

void rpc_2(){
 char buf[256], outbuf[256];
   FILE *devin = fdopen(&xbee, "r");
   FILE *devout = fdopen(&xbee, "w");
   xbee.set_baud(9600);

   while(1){
       memset(buf, 0, 256);
      for( int i = 0; ; i++ ) {
         char recv = fgetc(devin);
         if(recv == '\n') {
            printf("\r\n");
            break;
         }
         buf[i] = fputc(recv, devout);
      }
   RPC::call(buf, outbuf);

   }
}

void avoid_blocking(){

for(int i=0;i<6;i++){
   car.turn(60,0.40);
   ThisThread::sleep_for(500ms);
}

for(int i=0;i<5;i++){
   car.turn(60,-0.30);
   ThisThread::sleep_for(500ms);
}

car.goStraight(40);
ThisThread::sleep_for(500ms);

for(int i=0;i<5;i++){
   car.turn(60,-0.50);
   ThisThread::sleep_for(500ms);
}

for(int i=0;i<5;i++){
   car.turn(60,-0.35);
   ThisThread::sleep_for(500ms);
}

car.turn(40,0.70);
ThisThread::sleep_for(3s);
xbee.write("Task 2 completed!\r\n",19);
 flag=0;
}

void task4(Arguments *in, Reply *out){

   car.goStraight(-100);
   ThisThread::sleep_for(1s);
   car.stop();

   car.turn(100,-0.01);
   ThisThread::sleep_for(2s);
   car.stop();

   car.turn(100,0.45);
    ThisThread::sleep_for(1500ms);

   car.goStraight(100);
   ThisThread::sleep_for(3s);
    car.stop();
   xbee.write("Task 4 completed!\r\n", 19);
   xbee.write("Mission completed!\r\n",20);

}

RPCFunction rpcTask4(&task4, "task4");


int main(){

pc.set_baud(9600);
thread_2.start(rpc_2);
thread_1.start(rpc_1);

   float val;
   
   while(1){

   
    ping.output();
      ping = 0; wait_us(200);
      ping = 1; wait_us(5);
      ping = 0; wait_us(5);

      ping.input();
      while(ping.read() == 0);
      t.start();
      while(ping.read() == 1);
      val = t.read();

      float distance =val*17700.4f-4;
      printf("Distance = %lf cm\r\n", distance);

      if(distance <25 && flag==1){
          avoid_blocking();
      }

      t.stop();
      t.reset();

    ThisThread::sleep_for(1s);

   }
}