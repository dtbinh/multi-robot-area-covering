// SLAVE2 start from the right hand side

#include "mbed.h"
#include "MRF24J40.h"
#include "m3pi.h"
#include "imu.h"
#include "zigbee.h"

#include <string>
#include <stdio.h>
#include <string.h>
#include <math.h>

// LEDs you can treat these as variables (led2 = 1 will turn led2 on!)
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

Serial pc(USBTX, USBRX);
m3pi slave;

InterruptIn mag(p5); //To detect reed switch output

float bat;
const float bat_thres = 3.50;
const float d_backward = 180.00;
const float d_right = 90.0;
const float d_left = 270.0;
float d_forward;
//float d_backward;

int dir = 1; 
int x = 7;
int y = 0;
int i = 0;
int back_count = 0;
int work_count = 0;
bool turn = false;

int stop_x = 2;
int stop_y = 2;

bool just_work = false; // just for testing

bool back_charge = false;
bool go_work = false;
bool zig = false;

bool work = false;
bool charge = false;

const float speed = 0.1;
float correction = 0.008; 
float d_desired; 

const int charge_x = 7;
const int charge_y = 0;

int final_x;
int final_y; 

const int row = 4;

float read_heading(){
    bool startupPassed;
    Euler e;
 
    // Initialize
    pc.baud(115200);
    wait_ms(200);
    startupPassed = initBNO055(); 
 
    e = getEulerAngles();
    printf("Heading: %7.2f\n", e.heading);
    //slave.cls();
    slave.locate(0,0);
    slave.printf("h: %.2f", e.heading);
    wait_us(100); 
    return e.heading;   
}
/*****************************************************************/
 

void flip(){
    if(zig){
      if(y>=0 && y<=row && turn == false){  
        if(dir==1){
            y++;
            if(y==row){
                turn = true;
            }    
        }else{
            y--;
            if(y==0){
                turn = true;
            }  
        }
    
        slave.locate(0,1);
        slave.printf("x=%d; y=%d;", x, y); 
        sprintf(txBuffer, "Robo2: x:%d, y:%d, dir:%d, desired: %.2f\n", x, y, dir, d_desired);
        rf_send(txBuffer, strlen(txBuffer) + 1);
        
      }else if(y>row){
        y = row;    
      }else if(y<row){
        y = 0;    
      }
      /*if(x == final_x && y == final_y){
        wait_ms(500);
        slave.stop();
        led1 = 1;
        led2 = 1;
        led3 = 1;
        led4 = 1;
        wait(3.0);
        zig = false;  
       }*/
    }
    if(back_charge){
        back_count++;
        //slave.cls();
        slave.locate(0,1);
        slave.printf("b: %d", back_count);
    }
    if(go_work){
        work_count++;
        slave.locate(0,1);
        slave.printf("w: %d", work_count);
    }
}
void turn_right(){
    slave.left_motor(0);
    slave.right_motor(speed);
    wait_ms(1900);     
}

void turn_left(){
    slave.left_motor(speed);
    slave.right_motor(0);
    wait_ms(1900);    
}

void go_straight(float desired){
    wait_us(300);
    float d = read_heading();
    slave.cls();
    slave.locate(0,0);
    slave.printf("h: %.2f", d);
    if((desired>=0 && desired<5.0) || (desired>355 && desired<=360)){
        if(d_forward>=0 && d_forward<5.0){
               if(d-d_forward<90 && d-d_forward>1.0){
                    slave.left_motor(speed);
                    slave.right_motor(speed - correction);   
                }else if(d-d_forward>270 && d<359.0){
                    slave.right_motor(speed);
                    slave.left_motor(speed - correction);     
                }else{
                    slave.forward(speed);    
                }     
        }else if(d_forward>355 && d_forward<=360){
                if(d-d_forward<-270 && d>1.0){
                    slave.left_motor(speed);
                    slave.right_motor(speed - correction);   
                }else if(d-d_forward<0 && d-d_forward>-90){
                    slave.right_motor(speed);
                    slave.left_motor(speed - correction);
                }else{
                    slave.forward(speed);
                }     
        }     
    }else{
        if(d-desired>0.5){
            slave.left_motor(speed);
            slave.right_motor(speed - correction);     
        }else if(d-desired<-0.5){
            slave.right_motor(speed);
            slave.left_motor(speed - correction);    
        }else{
            slave.forward(speed);    
        }    
    }    
}  

void calibrate_back_degree(float d){
    if(d-d_backward>0){
            while(read_heading()>d_backward+1.0){
                slave.left_motor(speed);
                slave.right_motor(0); 
                wait_ms(100);
                slave.stop();
                wait_us(300);   
            }
        }else if(d-d_backward<0){
            while(read_heading()<d_backward-1.0){
                slave.right_motor(speed);
                slave.left_motor(0); 
                wait_ms(100);
                slave.stop();
                wait_us(300);   
            }
        }     
}

void calibrate_forward_degree(float d){
    if(d_forward>=0 && d_forward<5.0){
           if(d-d_forward<90 && d-d_forward>1.0){
             while(read_heading()>1.0 && read_heading()<90){
                slave.left_motor(speed);
                slave.right_motor(0);
                wait_ms(100);
                slave.stop();
                wait_us(300);    
             }
            }else if(d-d_forward>270 && d<359){
                while(read_heading()<359 && read_heading()>270){
                    slave.right_motor(speed);
                    slave.left_motor(0);
                    wait_ms(100);
                    slave.stop();
                    wait_us(300);    
                }       
            }     
        }else if(d_forward>355 && d_forward<=360){
            if(d-d_forward<-270 && d>1.0){
                while(read_heading()>1.0 && read_heading()<90){
                    slave.left_motor(speed);
                    slave.right_motor(0);
                    wait_ms(100);
                    slave.stop();
                    wait_us(300);    
                }       
            }else if(d-d_forward<0 && d-d_forward>-90){
                while(read_heading()<359 && read_heading()>270){
                    slave.right_motor(speed);
                    slave.left_motor(0);
                    wait_ms(100);
                    slave.stop();
                    wait_us(300);    
                }  
            }     
        }     
}

void calibrate_go_right(float d){
    if(d-d_right>0){
            while(read_heading()>d_right+1.0){
                slave.left_motor(speed);
                slave.right_motor(0); 
                wait_ms(100);
                slave.stop();
                wait_us(300);   
            }
        }else if(d-d_right<0){
            while(read_heading()<d_right-1.0){
                slave.right_motor(speed);
                slave.left_motor(0); 
                wait_ms(100);
                slave.stop();
                wait_us(300);   
            }
        }     
}

void calibrate_go_left(float d){
    if(d-d_left>0){
            while(read_heading()>d_left+1.0){
                slave.left_motor(speed);
                slave.right_motor(0); 
                wait_ms(100);
                slave.stop();
                wait_us(300);   
            }
        }else if(d-d_left<0){
            while(read_heading()<d_left-1.0){
                slave.right_motor(speed);
                slave.left_motor(0); 
                wait_ms(100);
                slave.stop();
                wait_us(300);   
            }
        }     
}

void zigzag(){
     
    zig = true;    
    slave.locate(0,1);
    slave.printf("x=%d; y=%d", x, y);
    
    while(turn==false){
            bat = slave.battery();

            //slave.locate(0,0);
            //slave.printf("%.2f V", bat);
            /*if(bat <= bat_thres){
                zig = false;
                return;    
            }*/
            if(x==stop_x && y==stop_y && just_work == false){
                zig = false;
                return;    
            }
            go_straight(d_desired);
        }
        printf("direction=%d\n",dir);
        slave.forward(speed);
        wait_ms(800);
        
        if(x == final_x && y == final_y){
            zig = false;
            return; 
        }
        
        if(dir==0){ //turn right
            led4 = 1;
            turn_right();
            slave.forward(speed);
            wait_ms(405);
            turn_right();    
            led4 = 0;
            d_desired = d_forward;
        }
    
        if(dir==1){ //turn right
            led1 = 1;
            turn_left();
            slave.forward(speed);
            wait_ms(395);
            turn_left();    
            led1 = 0;
            d_desired = d_backward;
        }
    
        x--;
        dir = !dir;
        turn = false;
        
        slave.stop();
        wait_us(300);
        if(dir==0){
            float d_backward_act = read_heading();
            calibrate_back_degree(d_backward_act);
        }else if(dir == 1){
            float d_forward_act = read_heading();
            calibrate_forward_degree(d_forward_act);    
        }
        
    zig = false;
}

void back_to_charge(int current_x, int current_y){
    back_count = 0;
    back_charge = true;
    slave.cls();
    slave.locate(0,0);
    slave.printf("Back...");
    slave.locate(0,1);
    slave.printf("b: %d", back_count);
    
    if(dir==1){
        wait_ms(100);
        float d = read_heading();
        calibrate_forward_degree(d);

        while(back_count<row-current_y){
                slave.forward(speed);
            };
        wait_ms(500);
        slave.stop();    
        turn_right();
        slave.stop();
        wait_ms(100);         
    }else if(dir==0){
        wait_ms(100);
        float d = read_heading();
        calibrate_back_degree(d);

        while(back_count<row-current_y){
                slave.backward(speed);
            };
        wait_ms(500);
        slave.stop();
        turn_left();
        slave.stop();
        wait_ms(100);  
    }
    calibrate_go_right(read_heading()); 
     
    while(back_count<(charge_x-current_x)+(row-current_y)){
                 go_straight(d_right);
            }
    turn_right();
    slave.stop();
    wait_ms(100);
    float d_curr = read_heading();
    calibrate_forward_degree(d_curr);
    
    while(back_count<(charge_x-current_x)+(row-current_y)+row){
                go_straight(d_backward);
            }
    slave.forward(speed);
    wait_ms(1000);
    slave.stop();
    slave.cls();
    slave.locate(0,0);
    slave.printf("Charging..."); 
    x = charge_x;
    y = charge_y;
    
    back_charge = false;
    back_count = 0;
}

void charge_to_point(int p_x, int p_y){
    go_work = true;
    while(work_count == 0){
        slave.forward(speed);    
    }
    wait_ms(500);
    turn_left();
    slave.stop();
    wait_us(300);
    calibrate_go_left(read_heading());
    while(work_count<charge_x-p_x+1){
        go_straight(d_left);    
    }
    slave.stop();
    turn_right();
    slave.stop();
    wait_ms(100);
    float d_curr = read_heading();
    calibrate_forward_degree(d_curr);
    while(work_count<(charge_x-p_x)+p_y+1){
        go_straight(d_forward);   
    }      
    x = p_x;
    y = p_y;
    slave.stop();
    if(p_x%2==1){
        dir = 0;
        slave.forward(speed);
        wait_ms(500);
        slave.right_motor(speed);
        slave.left_motor(-speed);
        wait_ms(2000);
        slave.stop();
        wait_ms(100);
        float d_curr = read_heading();
        calibrate_back_degree(d_curr);    
    }
    wait(2.0);
    led1 = 1;
    led4 = 1;
    wait(1.0);
    led1 = 0;
    led4 = 0;
    go_work = false;
    work_count = 0;    
}

/********************************************************/
int main (void){

    led1 = 1;
    led2 = 1;
    led3 = 1;
    led4 = 1;
    
    slave.cls();
    slave.locate(0,0);
    slave.printf("Hello!");
    wait(2.0);
    slave.cls();
    led1 = 0;
    led2 = 0;
    led3 = 0;
    led4 = 0;
    
    read_heading();
    wait_us(500);
    d_forward = read_heading();
    d_desired = d_forward;
    
    slave.cls();
    slave.locate(0,0);
    slave.printf("h: %.2f", d_forward);
    slave.locate(0,1);
    slave.printf("No.2 Ready!");
    wait(1.0);
    slave.cls();
    
    mag.fall(&flip);
    
    //communication variables
    int des_x;
    int des_y;
    int robo1;
    int robo2;
    int robo;
    int final_x1;
    int final_y1;
    int final_x2;
    int final_y2;
    pc.baud(115200);
    
    mrf.SetChannel(12);  
    timer.start();
    
    while(1){
            rxLen = rf_receive(rxBuffer, 128);
            if(rxLen>0){    
                if(rxBuffer[0]=='R'){
                sscanf(rxBuffer, "Robo%d and Robo%d Work: %d,%d and %d,%d\r\n", &robo1, &robo2, &final_x1, &final_y1, &final_x2, &final_y2);
                
                if(robo2==2){
                    final_x = final_x2;
                    final_y = final_y2;
                    work = true;
                }else{
                    charge = true;    
                }
                break;
                }
            }
    }
    
    while(1) {
        bat = slave.battery();
        printf("thres: %.2f V\n", bat_thres);
        printf("battery: %.2f V\n", bat);
        
        if(x==final_x && y==final_y){
                    slave.stop();
                    led1 = 1;
                    led2 = 1;
                    led3 = 1;
                    led4 = 1;
                    wait(3.0);
                    break;    
        }
           
        if(work){
               
                while(1){
                    zigzag();
                    if((x==stop_x && y==stop_y) || (x==final_x && y==final_y)){
                        break;    
                    }
                }
                
                if(x==final_x && y==final_y){
                    slave.stop();
                    led1 = 1;
                    led2 = 1;
                    led3 = 1;
                    led4 = 1;
                    wait(3.0);
                    break;    
                }
                
                slave.stop();
                slave.locate(0,0);
                slave.printf("TIRED!");
                led2 = 1;
                led3 = 1; 
                sprintf(txBuffer, "Low1:%d,%d Des:%d,%d\r\n", x, y, final_x, final_y);
                rf_send(txBuffer, strlen(txBuffer) + 1);
                pc.printf("Sent: %s\r\n", txBuffer);
                wait_ms(100);
                while(1){
                    rxLen = rf_receive(rxBuffer, 128);
                    if(rxLen>0 && strcmp(rxBuffer,"Charge2")==0){
                        led2 = 0;
                        led3 = 0;
                        break;   
                    }    
                }
                //wait(3.0);
                wait_ms(500);
                back_to_charge(x, y);
                //slave.stop();
                work = false;
                charge = true;
        }
        
        if(charge){
            x = charge_x;
            y = charge_y;
            led1 = 1;
            led4 = 1;
            while(1){
                rxLen = rf_receive(rxBuffer, 128);  
                if(rxBuffer[0]=='R'){
                    sscanf(rxBuffer, "Robo%d Work: %d,%d\r\n", &robo, &des_x, &des_y);
                
                    if(robo==2){
                        led1 = 0;
                        led4 = 0;
                        wait_ms(500);
                        charge_to_point(des_x, des_y);
                        break;   
                    }
                } 
            }
            charge = false;
            work = true;
            
            just_work = true;
            zigzag();
            slave.stop();   
        }                        
    }
    slave.forward(speed);
    wait_ms(2000);
    slave.stop();
}
