#include <Arduino.h>
#include "SBUS.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//OTA
const char* ssid = "Bommel";
const char* password = "HetWachtwoordIsFizbo";

// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial2);

// channel, fail safe, and lost frames data
uint16_t channels[16];
bool failSafe;
bool lostFrame;

#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)
#define SBUSMIN  180 // this is the 'minimum' pulse length count (out of 4096)
#define SBUSMAX  1811 // this is the 'maximum' pulse length count (out of 4096)

#define SbusChannelForward channels[1]
#define SbusChannelSteering channels[0]

#define SbusChannelLeftArm channels[5]
#define SbusChannelRightArm channels[6]
#define SbusChannelArmServoMode channels[15]
#define ServoChannelLeftArm 1
#define ServoChannelRightArm 0

#define SbusChannelHeadTurning channels[3]
#define ServoChannelHeadTurning 5

#define SbusChannelHeadUpDown channels[2]
#define ServoChannelHeadUpDownLower 4
#define ServoChannelHeadUpDownUpper 6

#define SbusChannelLeftEye channels[7]
#define SbusChannelRightEye channels[8]
#define ServoChannelLeftEye 3
#define ServoChannelRightEye 2

#define LEFT 1
#define RightMotorSpeedPin 19
#define RightMotorDirectionPin1 18
#define RightMotorDirectionPin2 5

#define RIGHT 0
#define LeftMotorSpeedPin 4
#define LeftMotorDirectionPin1 0
#define LeftMotorDirectionPin2 2

// setting PWM properties
#define motorPwmFreq 5000
#define motorPwmChannelRight 5
#define motorPwmChannelLeft 6
#define motorPwmResolution 8

#define FORWARD 1
#define BACKWARD 2

const char * udpAddress = "192.168.0.100";
const int udpPort = 60080;

WiFiUDP udp;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void MotorDirection(int motor, int direction);

void setupOta()
{
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

int counter = 0;

void setup() {
  Serial.begin(115200);
  //Serial2.begin(115200);

  setupOta();

  // begin the SBUS communication
  x8r.begin(16,17,true);

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  ledcSetup(motorPwmChannelRight, motorPwmFreq, motorPwmResolution);
  ledcAttachPin(RightMotorSpeedPin, motorPwmChannelRight);

  ledcSetup(motorPwmChannelLeft, motorPwmFreq, motorPwmResolution);
  ledcAttachPin(LeftMotorSpeedPin, motorPwmChannelLeft);

  pinMode(RightMotorSpeedPin, OUTPUT);
	pinMode(RightMotorDirectionPin1, OUTPUT);
	pinMode(RightMotorDirectionPin2, OUTPUT);

  pinMode(LeftMotorSpeedPin, OUTPUT);
	pinMode(LeftMotorDirectionPin1, OUTPUT);
	pinMode(LeftMotorDirectionPin2, OUTPUT);

  MotorDirection(LEFT, DISABLED);
  MotorDirection(RIGHT, DISABLED);


  udp.begin(4444);
}

void MotorDirection(int motor, int direction)
{
  if (motor == RIGHT && direction == FORWARD)
  {
    digitalWrite(RightMotorDirectionPin1, LOW);
	  digitalWrite(RightMotorDirectionPin2, HIGH);
  }
  else if (motor == RIGHT && direction == BACKWARD)
  {
    digitalWrite(RightMotorDirectionPin1, HIGH);
	  digitalWrite(RightMotorDirectionPin2, LOW);
  }
  else if (motor == LEFT && direction == FORWARD)
  {
    digitalWrite(LeftMotorDirectionPin1, HIGH);
	  digitalWrite(LeftMotorDirectionPin2, LOW);
  }
  else if (motor == LEFT && direction == BACKWARD)
  {
    digitalWrite(LeftMotorDirectionPin1, LOW);
	  digitalWrite(LeftMotorDirectionPin2, HIGH);
  }
  else if (motor == LEFT && direction == DISABLED)
  {
    digitalWrite(LeftMotorDirectionPin1, LOW);
	  digitalWrite(LeftMotorDirectionPin2, LOW);
  }
  else if (motor == RIGHT && direction == DISABLED)
  {
    digitalWrite(RightMotorDirectionPin1, LOW);
	  digitalWrite(RightMotorDirectionPin2, LOW);
  }
}

void DriveMotors(int forward, int steering)
{

int leftmotor = 0;
int rightmotor = 0;

  if (forward > -80)
  {
    leftmotor = forward + steering;
    rightmotor = forward - steering;
  }
  else
  {
    leftmotor = forward - steering;
    rightmotor = forward + steering;
  }

 if (leftmotor > 0)
      MotorDirection(LEFT, FORWARD);
    else
      MotorDirection(LEFT, BACKWARD);

    if (rightmotor > 0)
      MotorDirection(RIGHT, FORWARD);
    else
      MotorDirection(RIGHT, BACKWARD);

    ledcWrite(motorPwmChannelLeft, min(abs(leftmotor),256));
    ledcWrite(motorPwmChannelRight, min(abs(rightmotor),256));
}

void loop() {
  ArduinoOTA.handle();

  // look for a good SBUS packet from the receiver
  if(x8r.read(&channels[0], &failSafe, &lostFrame)){

    //Motor
    //{
    int valMotorForward = map(SbusChannelForward,SBUSMIN,SBUSMAX,-256, 256);
    int valMotorSteering = map(SbusChannelSteering,SBUSMIN,SBUSMAX,-256, 256);
    
    DriveMotors(valMotorForward, valMotorSteering);
    //}

    //Head turning
    int pwmvalHeadTurning = map(SbusChannelHeadTurning,SBUSMIN ,SBUSMAX,280, 480);
    pwm.setPWM(ServoChannelHeadTurning, 0, pwmvalHeadTurning);

    int pwmvalHeadUpDown = map(SbusChannelHeadUpDown,SBUSMIN ,SBUSMAX,170, 480);
    pwm.setPWM(ServoChannelHeadUpDownLower, 0, pwmvalHeadUpDown);
    pwm.setPWM(ServoChannelHeadUpDownUpper, 0, pwmvalHeadUpDown);


      int pwmvalLeftArm = map(SbusChannelLeftArm,SBUSMIN ,SBUSMAX,420, 170);
      pwm.setPWM(ServoChannelLeftArm, 0, pwmvalLeftArm);

      int pwmvalRightArm = map(SbusChannelRightArm,SBUSMIN,SBUSMAX,280, 480);
      pwm.setPWM(ServoChannelRightArm, 0, pwmvalRightArm);

      int pwmvalLeftEye = map(SbusChannelLeftEye,SBUSMIN ,SBUSMAX,234, 386);
      pwm.setPWM(ServoChannelLeftEye, 0, pwmvalLeftEye);

      int pwmvalRightEye = map(SbusChannelRightEye,SBUSMIN,SBUSMAX,348, 480);
      pwm.setPWM(ServoChannelRightEye, 0, pwmvalRightEye);

    //Arm movement
    if (SbusChannelArmServoMode > 1000)
    {
       if (counter++ > 100)
      {
        counter = 0;

        udp.beginPacket(udpAddress, udpPort);
        udp.printf("pwmvalLeftEye: %d", pwmvalLeftEye);
        udp.endPacket();
        
        udp.beginPacket(udpAddress, udpPort);
        udp.printf("pwmvalRightEye: %d", pwmvalRightEye);
        udp.endPacket();
      }

    }

    /*}
    else{
      pwm.setPWM(ServoChannelLeftArm, 0, 0);
      pwm.setPWM(ServoChannelRightArm, 0, 0);
    }*/
  }
}