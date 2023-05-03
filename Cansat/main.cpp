#include "TinyGPSPlus.h"                  //Libreria para gps, TinyGPS++
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include "I2Cdev.h"
#include "MPU6050.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//Se declaran las constantes
#define SET 8
#define AUX 9
#define TXD 11
#define RXD 10
#define EN 12
#define VCC 13
#define GND GND

//Se declaran las constantes para el bmp
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

MPU6050 accelgyro(0x69);

Adafruit_BMP280 bmp;                            //Se prepara la clase bmp

TinyGPSPlus gps;                                //Se declara la clase gps
SoftwareSerial ss(4, 3);                        //Se declara el puerto del gps
SoftwareSerial APCport(RXD, TXD);               //definimos el puerto serie Software para comunicar con el modulo RF

const int hourZone=2;                   //Diferencia de hora retornada en UTC por el GPS y la hora española

#define OUTPUT_READABLE_ACCELGYRO

int16_t ax, ay, az;
int16_t gx, gy, gz;

int dataNum=0;

struct position{
    struct time{
        bool validTime=false;
        int hour, minute, second, centisecond;
    }time;
    bool validPosition=false;
    double latitude;
    double longitude;
    double altitude;
};                                      //Struct que almacena la información recibida del gps

struct BMPRead{
    double altitude;
    double pressure;
    double temperature;
};                                      //Struct que almacena la información leida por el BMP

//Declaración de funciones
position updateGPSInformation();
BMPRead updateBMPInformation();

//Declara las funciones que comprueban los sensores
void checkGPSState();       //Comprueba el estado del GPS
void checkBMPState();       //Comprueba el estado del BMP
void checkConfigureRadio(); //Comprueba la configuración de la radio
void readRFConfiguration(); //Comprueba y configura la radio
void checkGyroState();      //Comprueba y configura el giroscopio

void showInfo(BMPRead inBMP, position inGPS);   //Muestra todos los valores en recogidos

void setup(){
    Serial.begin(9600);
    ss.begin(9600);
    pinMode(2,INPUT);
    pinMode(SET,OUTPUT);
    pinMode(AUX,INPUT);
    pinMode(EN,OUTPUT);
    pinMode(VCC,OUTPUT);
    digitalWrite(SET,HIGH);
    digitalWrite(VCC,HIGH);
    digitalWrite(EN,HIGH);

    Serial.println("Preparando componentes:");
    checkGPSState();
    checkBMPState();
    checkConfigureRadio();
    checkGyroState();
    Serial.println("Todos los componentes listos.");
}

void loop(){
    /*Se leen los sensores y se almacena la información en bucle*/
    position actualPosition;
    BMPRead atmosphereInformation=updateBMPInformation();
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    while (ss.available() > 0)                                             //Actualiza la información cuando el GPS está listo
        if (gps.encode(ss.read()))
            actualPosition=updateGPSInformation();
    showInfo(atmosphereInformation, actualPosition);
    dataNum++;
    delay(500);
}

position updateGPSInformation(){
    /*Función que mide la posición y tiempo y devuelve un struct almacenando toda la información
    En caso de error se devuelve null en la variable                                            */
    position updatedGPS;
    if (gps.location.isValid()){
        updatedGPS.latitude=gps.location.lat();
        updatedGPS.longitude=gps.location.lng();
        updatedGPS.altitude=gps.altitude.meters();
        updatedGPS.validPosition=true;
    }
    if (gps.time.isValid()){
        updatedGPS.time.hour=gps.time.hour()+hourZone;
        updatedGPS.time.minute=gps.time.minute();
        updatedGPS.time.second=gps.time.second();
        updatedGPS.time.centisecond=gps.time.centisecond();
        updatedGPS.time.validTime=true;
    }
    return updatedGPS;
}

BMPRead updateBMPInformation(){             //Lee el BMP y devuelve un struct con la información
    BMPRead updatedBMP;
    updatedBMP.altitude=bmp.readAltitude(1013.25);
    updatedBMP.temperature=bmp.readTemperature();
    updatedBMP.pressure=bmp.readPressure();
    return updatedBMP;
}

void checkGPSState(){
    bool GPS=false;
    while(!GPS) {
        if (ss.available() > 0) {
            Serial.print(F("GPS\t\t\t"));
            GPS=true;
        }
        delay(10);
    }
    Serial.println(F("✓"));
}

void checkBMPState(){                       //Se busca el BMP y se configura
    
    unsigned status;
    Serial.print(F("BMP\t\t\t"));
    status = bmp.begin();
    delay(100);
    while (!status) {
        status = bmp.begin();
        delay(10);
    }
    //Se configura el modo del BMP
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500); 
    Serial.println(F("✓"));
}

void checkConfigureRadio(){
    digitalWrite(SET, LOW); // poner en modo configuracion
    delay(50);
    //Parametros de configuración
    APCport.print("WR 415370 3 9 3 0");
    APCport.write(0x0D);
    APCport.write(0x0A);
    delay(100);
    digitalWrite(SET, HIGH);
    Serial.println(F("RF\t\t\t✓"));
}

void readRFConfiguration(){
    Serial.println(F("Configuración del modulo de radio:\n"));
    digitalWrite(SET, LOW); // poner en modo configuracion
    delay(50); // pausa para estabilizar
    APCport.print("RD"); // peticion de datos
    APCport.write(0x0D); // fin de linea
    APCport.write(0x0A);
    delay(100); // pausa para estabilizar
    while (APCport.available()) {
    Serial.write(APCport.read());
    }
    digitalWrite(SET, HIGH); // volver al modo normal
}

void checkGyroState(){                                  //Se configura el giroscopio
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    accelgyro.initialize();
    Serial.print(F("Giroscopio\t\t"));
    while (!accelgyro.testConnection()) delay(10);
    Serial.println(F("✓"));
}

void showInfo(BMPRead inBMP, position inGPS){

    Serial.print(dataNum);
    Serial.print(F(":\t"));
    Serial.print(inGPS.latitude, 6);
    Serial.print(F(","));
    Serial.print(inGPS.longitude, 6);
    Serial.print(F("\t"));
    Serial.print(inGPS.altitude, 6);
    Serial.print(F(" m\t"));

    Serial.print(inGPS.time.hour);
    Serial.print(F(":"));
    Serial.print(inGPS.time.minute);
    Serial.print(F(":"));
    Serial.print(inGPS.time.second);
    Serial.print(F("."));
    Serial.print(inGPS.time.centisecond);
    Serial.print(F("\t\t"));

    Serial.print(F("Temperatura = "));
    Serial.print(inBMP.temperature);
    Serial.print(" *C\t\t");


    Serial.print(F("Presión= "));
    Serial.print(inBMP.pressure);
    Serial.print(" Pa\t\t");

    Serial.print(F("Altitud= "));
    Serial.print(inBMP.altitude);
    Serial.print(" m\t\t");

    

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print(" Accel X\t");
        Serial.print(ay); Serial.print(" Accel Y\t");
        Serial.print(az); Serial.print(" Accel Z\t");
        Serial.print(gx); Serial.print(" Gyro X\t");
        Serial.print(gy); Serial.print(" Gyro Y\t");
        Serial.print(gz);Serial.println(" Gyro Z\t");
    #endif
}
