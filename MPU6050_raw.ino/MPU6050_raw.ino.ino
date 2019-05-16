
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az, a_ant = 0;
int16_t gx, gy, gz;
bool flag;
int suma, i = 0;


#define OUTPUT_READABLE_ACCELGYRO


//#define OUTPUT_BINARY_ACCELGYRO

int write_var = 10;
long A_new = 0;
unsigned long timeold,timeflag = millis();
unsigned long delta_t = 0;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  pinMode(10, OUTPUT);
  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // use the code below to change accel/gyro offset values
  /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    [-3513,-3512] --> [0,19]  [-1317,-1316] --> [-9,4]  [985,986] --> [16377,16385]
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
  */

  // configure Arduino LED pin for output

}

void loop() {
  //read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

#ifdef OUTPUT_BINARY_ACCELGYRO
  Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
  Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
  Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
  Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
  Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
  Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
#endif

  //First filter delete lsb;
  A_new = ((az / 2000) - 8);

  delta_t = (millis() - timeold);
  suma += (A_new);
  i ++;
  //    //analogWrite(10,25);   //EN el pin 10

  //Cada 50 ms promedie
  if ((delta_t) > 50)
  {
    //Second filter media movil
    A_new = suma / i;

    Serial.println(A_new);

    //Si se detecta un cambio de signo
    if (a_ant * A_new < 0)
    {
      timeflag = millis();    //Para que solo actue cuando detecte el primer cambio de signo
    }
    a_ant = A_new;
    timeold = millis();
    suma, i = 0;
  }

  //Si detecta el primer cambio de signo hacia arriba (aceleracion) e ignore la desaceleracion
  if ((A_new > 0) && ((millis() - timeflag) > 300))
  {
    //Un sentido
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
    //Solo valores positivos
    write_var = abs(A_new) * 20;   //Control Proporcional
    analogWrite(10, write_var);  //En el pin 10
  }

  //Si detecta el primer cambio de signo  hacia abajo (desaceleracion) e ignore la aceleracion
  else if ((A_new < 0) && ((millis() - timeflag) > 300))
  {
    //Otro Sentido
    digitalWrite(9, HIGH);
    digitalWrite(8, LOW);
    //Solo valores positivos
    write_var = abs(A_new) * 20;   //Control Proporcional
    analogWrite(10, write_var);  //En el pin 10
  }

}
