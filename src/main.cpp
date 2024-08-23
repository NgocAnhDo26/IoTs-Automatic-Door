#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Stepper.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// -------- Pins --------
#define PIR_PIN_1 34
#define PIR_PIN_2 35
#define LED_PIN 14
#define BUZZER_PIN 12
#define FLAME_PIN 13
#define LIGHT_SENSOR_PIN 27
#define EMERGENCY_BUTTON 18
#define ULTRA_SONIC_TRIG 23
#define ULTRA_SONIC_ECHO 19
#define SLIDE_SWITCH RX

const uint8_t NUMPAD_ROWS = 4;
const uint8_t NUMPAD_COLS = 3;
char keys[NUMPAD_ROWS][NUMPAD_COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};

uint8_t colPins[NUMPAD_COLS] = {16, 17, 5};
uint8_t rowPins[NUMPAD_ROWS] = {15, 2, 0, 4};

char doorPassword[4] = {'1', '2', '3', '4'};
char enteredPass[4] = {' ', ' ', ' ', ' '};

const int stepsPerRevolution = 200;

// -------- Boolean Flags --------
bool enterPassComplete = false;
bool openDoorKeyPad = false;
bool safetyLock = false;
bool fireDetected = false;
bool lightDetected = false;

bool safetyMode = true;
bool lightAlwaysOn = false;
bool doorAlwaysOpen = false;
bool openDoorFromWeb = false;

// -------- Topic --------
const char *t_doorIsClosed = "/22127010/flag/doorIsClosed";
const char *t_doorLight = "/22127303/flag/doorLight";
const char *t_peopleDetected = "/22127010/flag/peopleDetected";
const char *t_fireSensor = "/22127303/flag/fireSensor";
const char *t_restartRequest = "/22127010/flag/restartRequest";
const char *t_alert = "/22127010/flag/alert";

const char *t_safetyMode = "/22127010/flag/safetyMode";
const char *t_openDoorWeb = "/22127010/flag/openDoorWeb";
const char *t_alwaysOpen = "/22127010/flag/alwaysOpen";
const char *t_alwaysLightOn = "/22127010/flag/alwaysLightOn";
const char *t_doorPassword = "/22127303/flag/doorPassword";

// -------- Functions declaration --------
void enterPassword(int &i);

void playDoorOpenSound();
void playAlertSound();

void wifiConnect();

void mqttConnect();
void callback(char *, byte *, unsigned int);
void publishTopic(const char *topic, int message);
void publishTopic(const char *topic, String message);

void openDoor();
void closeDoor();
long ultraSonicDistance();

// Wifi
const char *ssid = "Wokwi-GUEST";
const char *password = "";
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// MQQT Server
const char *mqttServer = "test.mosquitto.org";
int port = 1883;

// Init devices
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, NUMPAD_ROWS, NUMPAD_COLS);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Stepper myStepper(stepsPerRevolution, 26, 25, 33, 32);
DHT dht(FLAME_PIN, DHT22);

void setup()
{
  Serial.begin(115200);
  pinMode(PIR_PIN_1, INPUT);
  pinMode(PIR_PIN_2, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FLAME_PIN, INPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(EMERGENCY_BUTTON, INPUT);
  pinMode(SLIDE_SWITCH, INPUT);
  pinMode(ULTRA_SONIC_TRIG, OUTPUT);
  pinMode(ULTRA_SONIC_ECHO, INPUT);

  lcd.init(); // initialize the lcd
  myStepper.setSpeed(30);

  wifiConnect();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive(90);

  mqttConnect();
  publishTopic(t_restartRequest, "true");
  delay(3000);
}

// ------- Variables for 'loop' function --------
int i = -1;
int numberOfIncorrect = 0;
bool doorIsClosed = true;
bool switchDoorAlwaysOpen = false;
bool fire = false;
int countFire = 0;
unsigned long startLock;
unsigned long fireDuration = 0;

void loop()
{
  int PIR1, PIR2;
  if (!mqttClient.connected())
    mqttConnect();

  mqttClient.loop();

  PIR1 = digitalRead(PIR_PIN_1);
  PIR2 = digitalRead(PIR_PIN_2);
  if (PIR1 || PIR2)
  {
    publishTopic(t_peopleDetected, "TRUE");
  }
  else
    publishTopic(t_peopleDetected, "FALSE");

  // LED Light
  if (!lightAlwaysOn)
  {
    bool lightSensor = digitalRead(LIGHT_SENSOR_PIN);
    if (lightSensor)
    {
      digitalWrite(LED_PIN, HIGH);
      publishTopic(t_doorLight, "ON");
    }
    else
    {
      digitalWrite(LED_PIN, LOW);
      publishTopic(t_doorLight, "OFF");
    }
  }
  else
  {
    digitalWrite(LED_PIN, HIGH);
    publishTopic(t_doorLight, "ON");
  }

  // Flame detection
  float temp = dht.readTemperature();
  if (temp > 30)
  {
    countFire++;
    if (countFire == 1)
      publishTopic(t_fireSensor, "FIRE!!!");
    fire = true;

    if (doorIsClosed)
      openDoor();
  }
  else
  {
    publishTopic(t_fireSensor, "SAFE");
    countFire = 0;
    fire = false;
  }

  // Slide switch door always open
  switchDoorAlwaysOpen = digitalRead(SLIDE_SWITCH);

  // Check for door always open mode
  if (!doorAlwaysOpen && !switchDoorAlwaysOpen)
  {
    if (!doorIsClosed && !fire)
    {
      PIR1 = digitalRead(PIR_PIN_1);
      PIR2 = digitalRead(PIR_PIN_2);
      if (!PIR1 && !PIR2)
      {
        closeDoor();
      }
    }

    if (openDoorFromWeb || digitalRead(EMERGENCY_BUTTON)) 
      openDoor();

    // Nếu bật (safetyMode) và không đang mở/đang đóng cửa
    if (!safetyLock)
    {
      if (safetyMode)
      {
        if (numberOfIncorrect == 3)
        {
          lcd.backlight();
          lcd.setCursor(0, 0);
          lcd.print("Wrong 3 times!");
          lcd.setCursor(0, 1);
          lcd.print("3 mins locked");

          publishTopic(t_alert, "true");
          playAlertSound();

          numberOfIncorrect = 0;
          // lock the door for 3 mins
          startLock = millis();
          safetyLock = true;
        }

        if (doorIsClosed)
          enterPassword(i);
        else
          lcd.noBacklight();

        if (openDoorKeyPad)
        {
          if (doorIsClosed)
          {
            openDoor();
            delay(1500);
          }

          // Check if there are people before closing
          PIR1 = digitalRead(PIR_PIN_1);
          PIR2 = digitalRead(PIR_PIN_2);
          if (!PIR1 && !PIR2)
          {
            publishTopic(t_peopleDetected, "FALSE");
            closeDoor();
            openDoorKeyPad = false;
          }
        }
      }
      // Chế độ bình thường
      else
      {
        lcd.clear();
        lcd.noBacklight();

        PIR1 = digitalRead(PIR_PIN_1);
        PIR2 = digitalRead(PIR_PIN_2);

        if (doorIsClosed)
        {
          if (PIR1 || PIR2)
          {
            publishTopic(t_peopleDetected, "TRUE");
            openDoor();
          }
        }
        else if (!PIR1 && !PIR2)
        {
          publishTopic(t_peopleDetected, "FALSE");
          closeDoor();
        }
      }
    }
    else if (millis() - startLock >= 3000)
    {
      safetyLock = false;
      lcd.clear();
      lcd.noBacklight();
      publishTopic(t_alert, "false");
    }
  }
  else
  {
    if (doorIsClosed)
      openDoor();
  }
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
      mqttClient.subscribe(t_safetyMode);
      mqttClient.subscribe(t_alwaysLightOn);
      mqttClient.subscribe(t_doorPassword);
      mqttClient.subscribe(t_openDoorWeb);
      mqttClient.subscribe(t_alwaysOpen);
    }
    else
    {
      Serial.println("try again in 3 seconds");
      delay(3000);
    }
  }
}

void publishTopic(const char *topic, int message)
{
  char buffer[50];
  sprintf(buffer, "%d", message);
  mqttClient.publish(topic, buffer);
}

void publishTopic(const char *topic, String message)
{
  char buffer[50];
  sprintf(buffer, "%s", message);
  mqttClient.publish(topic, buffer);
}

// MQTT Receiver
void callback(char *topic, byte *message, unsigned int length)
{
  String strMsg;

  if (String(topic) == String(t_doorPassword))
  {
    for (int k = 0; k < 4; k++)
    {
      doorPassword[k] = (char)message[k];
      Serial.print(doorPassword[k]);
    }
  }
  else
    for (int i = 0; i < length; i++)
    {
      strMsg += (char)message[i];
    }

  //***Code here to process the received package***
  if (String(topic) == String(t_safetyMode))
  {
    if (strMsg == "true")
      safetyMode = true;
    else
      safetyMode = false;
  }
  // mqttClient.subscribe(t_alwaysOpen);
  if (String(topic) == String(t_alwaysOpen)) {
    if (strMsg == "true")
      doorAlwaysOpen = true;
    else
      doorAlwaysOpen = false;
  }

  // mqttClient.subscribe(t_alwaysLightOn);
  if (String(topic) == String(t_alwaysLightOn))
  {
    if (strMsg == "true")
      lightAlwaysOn = true;
    else
      lightAlwaysOn = false;
  }

  // mqttClient.subscribe(t_openDoorWeb);
  if (String(topic) == String(t_openDoorWeb))
  {
    if (strMsg == "true")
      openDoorFromWeb = true;
    else
      openDoorFromWeb = false;
  }
}

// --------------- Buzzer Sound ---------------
void playDoorOpenSound()
{
  tone(BUZZER_PIN, 784, 500);
  noTone(BUZZER_PIN);
}

void playAlertSound()
{
  tone(BUZZER_PIN, 784, 500);
  noTone(BUZZER_PIN);
  delay(500);

  tone(BUZZER_PIN, 784, 500);
  noTone(BUZZER_PIN);
  delay(500);

  tone(BUZZER_PIN, 784, 500);
  noTone(BUZZER_PIN);
  delay(500);
}

// --------------- Numpad ---------------
void enterPassword(int &i)
{
  char key = keypad.getKey();
  if (key)
  {
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Enter password: ");

    if (i >= 0)
    {
      lcd.setCursor(6 + i, 1);
      lcd.print("*");
      enteredPass[i] = key;
    }

    if (++i == 4)
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
      lcd.noBacklight();

      // Open the door
      openDoorKeyPad = true;
      numberOfIncorrect = 0;
      lcd.clear();
      i = -1;
    }
    else
    {
      lcd.clear();
      lcd.print("   Incorrect!");
      delay(2000);
      lcd.clear();
      lcd.noBacklight();
      i = -1;
      numberOfIncorrect++;
    }

    enterPassComplete = false;
  }
}

// Door
void openDoor()
{
  playDoorOpenSound();
  while (true)
  {
    myStepper.step(stepsPerRevolution / 100);

    // Công tắc hành trình
    if (ultraSonicDistance() <= 10)
    {
      doorIsClosed = false;
      publishTopic(t_doorIsClosed, "OPENED");
      break;
    }

  }
}

void closeDoor()
{
  while (true)
  {
    myStepper.step(-stepsPerRevolution / 100);

    // Công tắc hành trình
    if (ultraSonicDistance() >= 50) {
      doorIsClosed = true;
      publishTopic(t_doorIsClosed, "CLOSED");
      break;
    }
  }
}

long ultraSonicDistance() {
  digitalWrite(ULTRA_SONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_SONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_SONIC_TRIG, LOW);

  long duration = pulseIn(ULTRA_SONIC_ECHO, HIGH);

  return duration * 0.034 / 2;
}