#include <Arduino.h>
#include <RF24.h>

#define PIN_R_CW   2
#define PIN_R_CCW  3
#define PIN_R_EN   9
#define PIN_L_CW   4
#define PIN_L_CCW  5
#define PIN_L_EN   10

// These are the different control modes
enum ControlMode {
  MANUAL_RADIO,
  WIFI,
  AUTONOMOUS,
};

// This is the structure of the received data packet
typedef struct {
  //uint8_t radioId;  
  int16_t adcX;
  int16_t adcY;
  bool buttonXy;
  bool buttonA;
  bool buttonB;
  bool buttonC;
  bool buttonD;
  bool buttonE;
  bool buttonF;
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
RF24 radio(11,12);

static void init_radio() {
  //rx_radio_data.radioId = RADIO_ID;
  while(!radio.begin()) Serial.println("Radio hardware not responding");  
  radio.setPALevel(RF24_PA_LOW);
  //radio.openWritingPipe(RADIO_ID);
  radio.openReadingPipe(RADIO_ID, DESTINATION_RADIO_ID);
  radio.startListening(); 
}

// Receive and transmit RF data every 10 ms 
static void TaskRxTxRFData() {
    byte pipe;
    if(radio.available(&pipe)) {
      radio.read(&rx_radio_data, sizeof(rx_radio_data_t));
      int16_t adcX = rx_radio_data.adcX;
      int16_t adcY = rx_radio_data.adcY; 
      Serial.print(adcX); Serial.print(" | ");
      Serial.println(adcY);  
    }
}

void set_l_pwm(int16_t pwm) {
  if(pwm < 0) {
    analogWrite(PIN_L_CW, abs(pwm));
    digitalWrite(PIN_L_CCW, LOW);
  } 
  else {
    analogWrite(PIN_L_CCW, pwm);
    digitalWrite(PIN_L_CW, LOW);
  }
}

void set_r_pwm(int16_t pwm) {
  if(pwm < 0) {
    analogWrite(PIN_R_CW, abs(pwm));
    digitalWrite(PIN_R_CCW, LOW);
  } 
  else {
    analogWrite(PIN_R_CCW, pwm);
    digitalWrite(PIN_R_CW, LOW);
  }

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  pinMode(13, OUTPUT);
  pinMode(PIN_L_CW, OUTPUT);
  pinMode(PIN_L_CCW, OUTPUT);
  pinMode(PIN_R_CW, OUTPUT);
  pinMode(PIN_R_CCW, OUTPUT); 
  pinMode(PIN_L_EN, OUTPUT);
  pinMode(PIN_R_EN, OUTPUT);
  digitalWrite(PIN_L_EN, HIGH);
  digitalWrite(PIN_R_EN, HIGH);
  init_radio(); 
  analogWriteResolution(12);
}

void loop() {
  // put your main code here, to run repeatedly:
  TaskRxTxRFData();
  if(rx_radio_data.buttonXy) 
    digitalWrite(13, HIGH);
  else 
    digitalWrite(13, LOW);
  int16_t adcX = rx_radio_data.adcX;
  int16_t adcY = rx_radio_data.adcY;
  int16_t lPwm = map(adcX + adcY, -500, 500, -4095, 4095);
  int16_t rPwm = map(adcX - adcY, -500, 500, -4095, 4095);
  set_l_pwm(lPwm);
  set_r_pwm(rPwm);
}
