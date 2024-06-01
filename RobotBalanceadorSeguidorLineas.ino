#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// Constantes de Motores
double MotorDerecha = 0.9;
double MotorIzquierda = 1.1;

// Configuración
double setpoint = 168.7;

// PID
double Kp = 18; // 21
double Kd = 0.53; // 0.4
double Ki = 150; // 100


// Motor pins
int IN1 = 11;
int IN2 = 10;
int IN3 = 9;
int IN4 = 6;

// Sensor pins
int infraPin1 = 4;
int infraPin2 = 7;
int valorInfra1 = 0;
int valorInfra2 = 0;

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688);

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-250, 250);
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(IN4, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(infraPin1, INPUT);
    pinMode(infraPin2, INPUT);

    analogWrite(IN4, LOW);
    analogWrite(IN3, LOW);
    analogWrite(IN2, LOW);
    analogWrite(IN1, LOW);
}

void loop() {

    // Lectura de infrarrojos.
    valorInfra1 = digitalRead(infraPin1);
    valorInfra2 = digitalRead(infraPin2);
    /*
    Serial.print("infra derecha: ");
    Serial.println(valorInfra1);
    Serial.print("infra izquierda: ");
    Serial.println(valorInfra2);
    */

    int state = (valorInfra1 << 1) | valorInfra2;

    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();
        /*
        Serial.print(input); 
        Serial.print(" =>"); 
        Serial.println(output);
        */

        if (input > 150 && input < 200) {
            if (output > 0)
                Forward();
            else if (output < 0)
                Reverse();

            int velocidad = abs(output);

            // Si está equilibrado y la velocidad es baja
            if (input >= 168 && input <= 170 && velocidad <= 40) {
                sigueLinea(state);
            }
        } else {
            Stop();
        }
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[1] * 180 / M_PI + 180;
    }


      // Cambio de valores PID, por medio del monitor serie.

      if (Serial.available() > 0) {
        char comando = Serial.read();
        double nuevoValor;
        switch (comando) {
            case 'p':
                nuevoValor = Serial.parseFloat();
                Kp = nuevoValor;
                pid.SetTunings(Kp, Ki, Kd);
                break;
            case 'i':
                nuevoValor = Serial.parseFloat();
                Ki = nuevoValor;
                pid.SetTunings(Kp, Ki, Kd);
                break;
            case 'd':
                nuevoValor = Serial.parseFloat();
                Kd = nuevoValor;
                pid.SetTunings(Kp, Ki, Kd);
                break;
        }
    }
}

void Forward() {

    analogWrite(IN4, output * MotorDerecha);
    analogWrite(IN3, 0);
    analogWrite(IN2, output * MotorIzquierda);
    analogWrite(IN1, 0);
}

void Reverse() {
 
    analogWrite(IN4, 0);
    analogWrite(IN3, -output * MotorDerecha);
    analogWrite(IN2, 0);
    analogWrite(IN1, -output * MotorIzquierda);
}

void Stop() {
    analogWrite(IN4, 0);
    analogWrite(IN3, 0);
    analogWrite(IN2, 0);
    analogWrite(IN1, 0);
}

void sigueLinea(int state) {
    switch (state) {
        case 0:
            Avanza();
            break;
        case 1:
            siguelineaI();
            break;
        case 2:
            siguelineaD();
            break;
        case 3:
            Stop();
            break;
        default:
            Serial.println("no deberia existir");
            break;
    }
}

void Avanza() {

    int speed = 40; // Ajusta la velocidad de avance aquí
    analogWrite(IN4, 0);
    analogWrite(IN3, speed);
    analogWrite(IN2, 0);
    analogWrite(IN1, speed);
    Serial.println("AVANZANDO");
}

void siguelineaD() {


    int turnSpeed = 40;
    analogWrite(IN4, -1*turnSpeed);
    analogWrite(IN3, 0);
    analogWrite(IN2, 0);
    analogWrite(IN1, turnSpeed);
    Serial.println("GirandoDER");

}

void siguelineaI() {

     int turnSpeed = 40;
    analogWrite(IN4, 0);    // retrocede izquierda
    analogWrite(IN3, turnSpeed);  // avanza derecha
    analogWrite(IN2, 55);  // retrocede izquierda
    analogWrite(IN1, 0);  // avanza izquierda
    Serial.println("GirandoIZQ");


}
