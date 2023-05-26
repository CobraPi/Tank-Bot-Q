#include <Arduino.h>
#include <Arduino_due_FreeRTOS.h>
#include <RF24.h>

// These are the different control modes
enum ControlMode {
  MANUAL_RADIO,
  WIFI,
  AUTONOMOUS,
};

// This is the structure of the received data packet
typedef struct {
  uint8_t radioId;  
  uint8_t adcX;
  uint8_t adcY;
  bool middleButton;
  bool buttonA;
  bool buttonB;
  bool buttonC;
  bool buttonD;
} rx_radio_data_t;

// This is the structure of the transmitted data packet
typedef struct {
  uint8_t radioId; 
  uint16_t heading;
  int8_t roll;
  int8_t pitch;
  unsigned long ultrasonicDistance;
} tx_radio_data_t;

const static uint8_t DESTINATION_RADIO_ID = 0x00;
const static uint8_t RADIO_ID = 0x01;

rx_radio_data_t rx_radio_data;
tx_radio_data_t tx_radio_data;
RF24 radio(7,8);
TaskHandle_t txData;

static void init_radio() {
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(RADIO_ID);
  radio.openReadingPipe(RADIO_ID, DESTINATION_RADIO_ID);
  radio.startListening(); 
}

// Receive and transmit RF data every 10 ms 
static void TaskRxTxRFData(void* pvParameters) {
  byte pipe;
  while(1) { 
    if(radio.available(&pipe)) {
      Serial.println("Got data"); 
      radio.read(&rx_radio_data, sizeof(rx_radio_data));
      radio.stopListening();
      radio.write(&tx_radio_data, sizeof(tx_radio_data));
      radio.startListening(); 
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  init_radio(); 
  portBASE_TYPE rx, tx;
  Serial.println("Radio initialized"); 
  tx = xTaskCreate(TaskRxTxRFData, NULL, configMINIMAL_STACK_SIZE, NULL, 1, &txData);
  if(tx != pdPASS) {
    Serial.println(F("Error creating tasks"));
  }
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while(1);
}

void loop() {
  // put your main code here, to run repeatedly:
}
