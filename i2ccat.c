#include<stdio.h>
#include<unistd.h>
#include<fcntl.h>
#include "bsc-slave.h"

#define SLV_ADDR  	 0x33
#define TX_BUF_SIZE      5

int main(void){

 char buffer[20];
 char tx_buffer[TX_BUF_SIZE];
 int fd;
 int length, tx_length = 2;
 int count, value_count, transfered;
 char *pointer;
 int output_end = 0;

 if((fd = open("/dev/i2c-slave", O_RDWR)) == -1){
   printf("could not open i2c-slave\n");
 }

  if( (ioctl(fd, I2C_SLAVE, SLV_ADDR) < 0) ){
    printf("failed setting slave adress!\n");
    return -1;
  }


 while(1){
                        //read out I2C RX buffer
   if((length = read(fd, buffer, 20)) == -1){
     printf("unable to read!\n");
   }

   for(count = 0; count < length; count++){
     printf("recieved value: %02d\n", buffer[count]);
   }

                       //fill TX buffer with arbitrary values
   for(count = 0; count < TX_BUF_SIZE; count++){
     tx_buffer[count] = count;
     tx_length = count + 1;
   }


   pointer = tx_buffer;
   while( tx_length > 0){             //send values
     transfered = write(fd, pointer, tx_length);
     for(count = 0 ; count < transfered; count++){
       printf("send value is %d\n", (int)*pointer++);
       tx_length--;
     }
   }

 }

 close(fd);
 return 0;
}
