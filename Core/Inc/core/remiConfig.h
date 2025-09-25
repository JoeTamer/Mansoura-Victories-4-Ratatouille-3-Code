#ifndef INC_CORE_REMICONFIG_H_
#define INC_CORE_REMICONFIG_H_

//// ---------- Motors ----------
//#define rightMotorIN1Port   GPIOB
//#define rightMotorIN1Pin    14
//#define rightMotorIN2Port   GPIOB
//#define rightMotorIN2Pin    15
//#define rightMotorENPort    GPIOA
//#define rightMotorENPin     8
//
//#define leftMotorIN1Port    GPIOA
//#define leftMotorIN1Pin     12
//#define leftMotorIN2Port    GPIOA
//#define leftMotorIN2Pin     11
//#define leftMotorENPort     GPIOB
//#define leftMotorENPin      8
//
//// ---------- Encoders ----------
//#define rightEncoderCH1Port GPIOB
//#define rightEncoderCH1Pin  12
//#define rightEncoderCH2Port GPIOB
//#define rightEncoderCH2Pin  13
//#define rightEncoderPPR     7
//
//#define leftEncoderCH1Port  GPIOB
//#define leftEncoderCH1Pin   4
//#define leftEncoderCH2Port  GPIOB
//#define leftEncoderCH2Pin   5
//#define leftEncoderPPR      7

// ---------- Motors ----------
#define rightMotorIN1Port   GPIOA
#define rightMotorIN1Pin    11
#define rightMotorIN2Port   GPIOA
#define rightMotorIN2Pin    12
#define rightMotorENPort    GPIOB
#define rightMotorENPin     8

#define leftMotorIN1Port    GPIOB
#define leftMotorIN1Pin     15
#define leftMotorIN2Port    GPIOB
#define leftMotorIN2Pin     14
#define leftMotorENPort     GPIOA
#define leftMotorENPin      8

// ---------- Encoders ----------
#define rightEncoderCH1Port GPIOB
#define rightEncoderCH1Pin  4
#define rightEncoderCH2Port GPIOB
#define rightEncoderCH2Pin  5
#define rightEncoderPPR     7

#define leftEncoderCH1Port  GPIOB
#define leftEncoderCH1Pin   12
#define leftEncoderCH2Port  GPIOB
#define leftEncoderCH2Pin   13
#define leftEncoderPPR      7

// ---------- MPU6050 ----------
#define mpuI2CPeripheral    I2C1
#define mpuI2CAlternatePins 0
#define mpuSpeedMode        SM
#define mpuAddress          0x68
#define mpuDlpfConfig       0x03
#define mpuGyroConfig       0x00

// ---------- Infrared ----------
#define irADCPeripheral     ADC1
#define irADCPort           GPIOA
#define irADCPin            0

// ---------- Extras ----------
#define dip0Port            GPIOB
#define dip0Pin             10
#define dip1Port            GPIOB
#define dip1Pin             11

#define searchLedPort       GPIOB
#define searchLedPin        0
#define runLedPort          GPIOB
#define runLedPin           1

#define buzzerPort          GPIOA
#define buzzerPin           7

#define leftLedPort         GPIOA
#define leftLedPin          6
#define frontLedPort        GPIOA
#define frontLedPin         5
#define rightLedPort        GPIOA
#define rightLedPin         4

#endif /* INC_CORE_REMICONFIG_H_ */
