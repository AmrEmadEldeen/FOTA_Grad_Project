
#include "driver/can.h"
#include "driver/gpio.h"
byte g_buffer[100] = {1,2,3,4,5,6,7,8,9, 10, 55, 33,0,15,4,7,9,10,11,12,13,14,15};
int g_index = 0;
int g_flag = 1;

void setup() {
    Serial.begin(500000); //kanet 500000
    
    can_general_config_t g_config =
     CAN_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4,CAN_MODE_NORMAL ); //el pins bta3et el tx w el rx 
    can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS(); //kanet 500kbits
    can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

    //    Install CAN driver
    if (can_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }

    //    Start CAN driver
    if (can_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return;
    }


}


 void loop() {

//Configure message to transmit
can_message_t message;
message.flags = CAN_MSG_FLAG_NONE;
message.identifier =0x110;
message.extd =0; //standard format not extended format   

message.data_length_code = 8;
for (int i = 0; i < 8; i++) {
 message.data[i] = g_buffer[g_index];
 g_index++; 
  }

if(g_flag < 5)
{
    // Queue message for transmission
    if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) { 
      printf("Message queued for transmission\n");
    } else {
      printf("Failed to queue message for transmission\n");     
    }
g_flag++;    
    
}


 delay(2000);

/*
can_message_t messagerx;
if (can_receive(&messagerx, pdMS_TO_TICKS(10000)) == ESP_OK) {
    printf("Message received\n");
} else {
    printf("Failed to receive message\n");
    return;
}

//Process received message
if (messagerx.extd) {
    printf("Message is in Extended Format\n");
} else {
    printf("Message is in Standard Format\n");
}
printf("ID is %d\n", messagerx.identifier);
if (!(messagerx.rtr)) {
    for (int i = 0; i < messagerx.data_length_code; i++) {
        printf("Data byte %d = %d\n", i, messagerx.data[i]);
    }
}
*/
}

//Note: if printf() doesn't work, replace it with Serial.println(), bs hya hatshtghl azon 3ady
