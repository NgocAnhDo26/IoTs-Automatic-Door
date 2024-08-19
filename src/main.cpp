// #include <Stepper.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>

// -------- Pins --------
#define PIR_PIN_1 26
#define PIR_PIN_2 27
#define LED_PIN 18
#define BUZZER_PIN 25
#define FLAME_PIN 1
#define LIGHT_SENSOR_PIN 1
#define EMERGENCY_BUTTON 1
#define SLIDE_SWITCH 1

const uint8_t NUMPAD_ROWS = 4;
const uint8_t NUMPAD_COLS = 3;
char keys[NUMPAD_ROWS][NUMPAD_COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};

uint8_t colPins[NUMPAD_COLS] = {16, 17, 5};
uint8_t rowPins[NUMPAD_ROWS] = {15, 2, 0, 4};

char doorPassword[4] = {'2', '2', '2', '2'};
char enteredPass[4] = {' ', ' ', ' ', ' '};

// -------- Boolean Flags --------
bool safetyMode = false;
bool enterPassComplete = false;
bool doorIsOpeningClosing = false;
bool openDoorCommand = false;
bool closeDoorCommand = false;
bool safetyLock = false;
bool lightAlwaysOn = false;

// -------- Functions declaration --------
void enterPassword(int &i);

void playDoorOpenSound();
void playAlertSound();

void wifiConnect();

void mqttConnect();
void callback(char *, byte *, unsigned int);

// Wifi
const char *ssid = "Wokwi-GUEST";
const char *password = "";
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// MQQT Server
const char *mqttServer = "broker.hivemq.com";
int port = 1883;

// Init devices
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, NUMPAD_ROWS, NUMPAD_COLS);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
  Serial.begin(115200);
  pinMode(PIR_PIN_1, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  lcd.init(); // initialize the lcd
  // myStepper.setSpeed(30);
}

// ------- Variables for 'loop' function --------
int i = 0;
int numberOfIncorrect = 0;
bool doorIsClosed = true;
long startLock;

void loop()
{
  int PIR1, PIR2;
  // if (!mqttClient.connected())
  //   mqttConnect();

  // mqttClient.loop();

  // // Publish data to MQTT Server
  // int temp = random(0, 100);
  // char buffer[50];
  // sprintf(buffer, "%d", temp);
  // mqttClient.publish("/12345/temp", buffer);

  // delay(5000);

  // bool status = digitalRead(PIR_pin_1);
  // LED Light
  if (!lightAlwaysOn)
  {
    bool lightSensor = analogRead(LIGHT_SENSOR_PIN);
    if (lightSensor > 50)
    {
      digitalWrite(LED_PIN, HIGH);
    }
    else
    {
      digitalWrite(LED_PIN, LOW);
    }
  }
  else
  {
    digitalWrite(LED_PIN, HIGH);
  }

  // Flame detection
  bool flameDetected = digitalRead(FLAME_PIN);
  if (flameDetected)
  {
    if (doorIsClosed)
    {
      // openDoor();
      doorIsClosed = false;
    }
  }
  else
  {
    if (!doorIsClosed)
    {
      PIR1 = digitalRead(PIR_PIN_1);
      PIR2 = digitalRead(PIR_PIN_2);

      if (!PIR1 && !PIR2)
      {
        // closeDoor();
        doorIsClosed = true;
      }
    }
  }

  // Nếu bật (safetyMode) và không đang mở/đang đóng cửa
  if (!safetyLock)
  {
    if (safetyMode)
    {
      if (numberOfIncorrect == 3)
      {
        playAlertSound();
        numberOfIncorrect = 0;
        // lock the door for 3 mins
        startLock = millis();
      }

      enterPassword(i);

      if (openDoorCommand)
      {
        // openDoor();
        playDoorOpenSound();

        // Check if there are people before closing
        PIR1 = digitalRead(PIR_PIN_1);
        PIR2 = digitalRead(PIR_PIN_2);
        if (!PIR1 && !PIR2)
        {
          // closeDoor();
        }
      }
    }
    // Chế độ bình thường
    else
    {
      PIR1 = digitalRead(PIR_PIN_1);
      PIR2 = digitalRead(PIR_PIN_2);

      if (doorIsClosed)
      {
        if (PIR1 || PIR2)
        {
          // openDoor();
          playDoorOpenSound();
        }
      }
      else
      {
        if (!PIR1 && !PIR2)
        {
          // closeDoor();
        }
      }

      doorIsClosed = !doorIsClosed;
    }
  }
  else if (millis() - startLock >= 3000)
  {
    safetyLock = false;
  }

  // if (lightStat)
  //   digitalWrite(led, HIGH);
  // else
  //   digitalWrite(led, LOW);
  // playAlertSound();

  // playDoorOpenSound();

  // while (!status)
  // {
  //   myStepper.step(stepsPerRevolution);
  // }
}

//  --------------- Wifi ---------------
void wifiConnect()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

// --------------------- MQQT SERVER ---------------------
void mqttConnect()
{
  while (!mqttClient.connected())
  {
    Serial.println("Attemping MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str()))
    {
      Serial.println("connected");

      // Subscribe all necessary topics
      mqttClient.subscribe("/12345/led");
    }
    else
    {
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

// MQTT Receiver
void callback(char *topic, byte *message, unsigned int length)
{
  Serial.println(topic);
  String strMsg;
  for (int i = 0; i < length; i++)
  {
    strMsg += (char)message[i];
  }
  Serial.println(strMsg);

  //***Code here to process the received package***
}

// --------------- Buzzer Sound ---------------
void playDoorOpenSound()
{
  tone(BUZZER_PIN, 784, 500);
  noTone(BUZZER_PIN);
}

void playAlertSound()
{
  for (int i = 0; i < 5; ++i)
  {
    tone(BUZZER_PIN, 700, 500);
    noTone(BUZZER_PIN);
    delay(500);
  }
}

// --------------- Numpad ---------------
void enterPassword(int &i)
{
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Enter password: ");
  lcd.setCursor(1, 0);

  char key = keypad.getKey();
  if (key)
  {
    lcd.setCursor(6 + i, 1);
    lcd.print("*");
    enteredPass[i++] = key;

    if (i == 4)
      enterPassComplete = true;
  }

  if (enterPassComplete)
  {
    if (enteredPass[0] == doorPassword[0] && enteredPass[1] == doorPassword[1] &&
        enteredPass[2] == doorPassword[2] && enteredPass[3] == doorPassword[3])
    {
      lcd.clear();
      lcd.print("    Correct!");
      delay(2000);

      // Open the door
      openDoorCommand = true;
      lcd.clear();
      lcd.noBacklight();
      i = 0;
    }
    else
    {
      lcd.clear();
      lcd.print("   Incorrect!");
      delay(2000);
      lcd.clear();
      i = 0;
    }
    enterPassComplete = false;
  }
}

// // Led
// void ledSetup(int pin)
// {
//   pinMode(pin, OUTPUT);
// }

// void ledHigh(int pin)
// {
//   digitalWrite(pin, HIGH);
// }

// void ledLow(int pin)
// {
//   digitalWrite(pin, LOW);
// }

// // Photoresister Sensor
// void ldrSetup(int pin)
// {
//   pinMode(pin, INPUT);
// }

// void ldrRead(int pin)
// {
//   analogRead(pin);
// }
// // Lcd I2C
// LiquidCrystal_I2C lcd(0x27, 16, 2);
// LiquidCrystal_I2C lcdSetUp()
// {
//   LiquidCrystal_I2C lcd(0x27, 16, 2);
//   lcd.init();
//   lcd.setCursor(0, 0);
//   return lcd;
// }
// template <typename content>
// void lcdPrint(LiquidCrystal_I2C lcd, int x, int y, content)
// {
//   lcd.setCursor(y, x);
//   lcd.print(content);
// }
// // Flame sensor / dht

// DHT dht(pin, DHT11);
// DHT dhtSetup(int pin)
// {
//   DHT dht(pin, DHT11);
//   dht.begin();
//   return dht;
// }
// float dhtRead(DHT dht)
// {
//   return dht.readTemperature();
// }