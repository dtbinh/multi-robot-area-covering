//MASTER

#include "mbed.h"
#include "MRF24J40.h"
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// RF tranceiver to link with handheld.
MRF24J40 mrf(p11, p12, p13, p14, p21);


// Timer
Timer timer;

// Serial port for showing RX data.
Serial pc(USBTX, USBRX);

// Used for sending and receiving

char txBuffer[128];
char rxBuffer[128];
int  rxLen;
char Robo1[128];
char Robo2[128];
char Received[128];

//***************** Do not change these methods (please) *****************//

/**
* Receive data from the MRF24J40.
*
* @param data A pointer to a char array to hold the data
* @param maxLength The max amount of data to read.
*/
int rf_receive(char *data, uint8_t maxLength)
{
    uint8_t len = mrf.Receive((uint8_t *)data, maxLength);
    uint8_t header[8]= {1, 8, 0, 0xA1, 0xB2, 0xC3, 0xD4, 0x00};

    if(len > 10) {
        //Remove the header and footer of the message
        for(uint8_t i = 0; i < len-2; i++) {
            if(i<8) {
                //Make sure our header is valid first
                if(data[i] != header[i])
                    return 0;
            } else {
                data[i-8] = data[i];
            }
        }

        //pc.printf("Received: %s length:%d\r\n", data, ((int)len)-10);
    }
    return ((int)len)-10;
}

/**
* Send data to another MRF24J40.
*
* @param data The string to send
* @param maxLength The length of the data to send.
*                  If you are sending a null-terminated string you can pass strlen(data)+1
*/
void rf_send(char *data, uint8_t len)
{
    //We need to prepend the message with a valid ZigBee header
    uint8_t header[8]= {1, 8, 0, 0xA1, 0xB2, 0xC3, 0xD4, 0x00};
    uint8_t *send_buf = (uint8_t *) malloc( sizeof(uint8_t) * (len+8) );

    for(uint8_t i = 0; i < len+8; i++) {
        //prepend the 8-byte header
        send_buf[i] = (i<8) ? header[i] : data[i-8];
    }
    //pc.printf("Sent: %s\r\n", send_buf+8);

    mrf.Send(send_buf, len+8);
    free(send_buf);
}


//***************** You can start coding here *****************//
int main (void){
int xpos1=3;
int ypos1=0;
int xpos2=4;
int ypos2=0;
int xpos;
int ypos;
int robot;
int xre ;
int yre ;
int robo;
int des_x;
int des_y;
pc.baud(115200);
   
   
 
    timer.start();
    
    mrf.SetChannel(12); //setchannel
     //ask Robot1 and Robot2 to work
    sprintf(txBuffer,"Robo1 and Robo2 Work: %d,%d and %d,%d\r\n",xpos1,ypos1,xpos2,ypos2);
    rf_send(txBuffer,strlen(txBuffer)+1);
    pc.printf("Sent: %s\r\n", txBuffer);
    //sprintf(txBuffer,"Robo2 Work: %d,%d\r\n",xpos2,ypos2);
    //rf_send(txBuffer,strlen(txBuffer)+1);
    //pc.printf("Sent: %s\r\n", txBuffer);
    wait_ms(300);
while(true){
    rxLen = rf_receive(rxBuffer, 128);
    if(rxLen>0){
        pc.printf("%s", rxBuffer);
        if(rxBuffer[0] == 'L'){  //The form is like:  Low%n:%f,%f\r\n", &xre, &yre 
          sscanf(rxBuffer, "Low%d:%d,%d Des:%d,%d\r\n", &robo, &xre, &yre, &des_x, &des_y);
          xpos = xre;  //get x axis value
          ypos = yre;  //get y axis value
          robot = robo;  //which robot is out of charge
          //pc.printf("Rreceive: %s\r\n", rxBuffer);
          
          if(robot==1){ //if robot1 is out of charge
              //pc.printf("%d,%d",xre,yre); //x,y axis value
              wait(1);
              sprintf(txBuffer,"Robo3 Work: %d,%d Des: %d,%d\r\n",xre,yre,des_x,des_y);
              rf_send(txBuffer,strlen(txBuffer)+1);
              pc.printf("Sent: %s\r\n", txBuffer);
              wait(1.5);
              strcpy(txBuffer,"Charge1");
              rf_send(txBuffer,strlen(txBuffer)+1);
              pc.printf("Sent: %s\r\n", txBuffer);
              wait_ms(100);}
          else if(robot==2){
              sprintf(txBuffer,"Robo3 Work: %d,%d Des: %d,%d\r\n",xre,yre,des_x,des_y);
              rf_send(txBuffer,strlen(txBuffer)+1);
              pc.printf("Sent: %s\r\n", txBuffer);
              wait_ms(1.5);
              strcpy(txBuffer,"Charge2");
              rf_send(txBuffer,strlen(txBuffer)+1);
              pc.printf("Sent: %s\r\n", txBuffer);}
          }
          
        if(strcmp(rxBuffer,"Done")==0){  //Work Done will receive Done
          pc.printf("Work Done!"); 
            }
    }        
}                 
}