#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";   // nrf communication address

struct data_pack {    //data structure for storing data for transmitting
  byte throttle;
  byte xval;
  byte yval;
  byte sw;
};

data_pack data;    // data structure object "data"

const int throttlePin = A0;
const int xpin = A2;
const int ypin = A1;
const int swpin = A4;

bool last_state = LOW;
bool current_button;
long last_pressed_time;
int debounce_delay = 100;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);  //00001
  radio.setAutoAck(false);   // auto acknowledge set to false
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
  pinMode(13, OUTPUT);
  pinMode(throttlePin, INPUT);
  pinMode(ypin, INPUT);
  pinMode(xpin, INPUT);
  pinMode(swpin, INPUT_PULLUP);
}

void loop() {
  delay(10);
  current_button = digitalRead(swpin);
  // if button pressed, mode is changed to high if prev state low or vice versa
  if (current_button == LOW) {                              // button pressed = LOW as pullup at input pin
    if (millis() - last_pressed_time > debounce_delay) {    //debounce for push button
      if (last_state == HIGH) {
        data.sw = 0;
        digitalWrite(13, LOW);
        last_state = LOW;
        last_pressed_time = millis();
      }
      else {
        data.sw = 1;
        digitalWrite(13, HIGH);
        last_state = HIGH;
        last_pressed_time = millis();
      }
    }
  }
  // mapping values to 0 to 255 for pwm
  data.throttle = map(analogRead(throttlePin), 0, 1023, 0, 255);
  data.xval = map(analogRead(xpin), 0, 1023, 0, 255);
  data.yval = map(analogRead(ypin), 0, 1023, 0, 255);
  float throttle_val = (float(data.throttle) / 255.0)*100.0;
  if (radio.write(&data, sizeof(data_pack))) {
    Serial.print("throttle: " + String(throttle_val) + "%");
    Serial.print(" xval: " + String(data.xval));
    Serial.print(" yval: " + String(data.yval));
    Serial.println(" SW: " + String(data.sw));
    Serial.println("");
    Serial.println("--------------------------------------------");
  }
}
