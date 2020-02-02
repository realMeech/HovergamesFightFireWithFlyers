/*
   Autoren: Markus Brutzer, Michael Riffel

   Funktion:  Sketch wurde für ESP32 (DOIT Devkit V1) entwickelt!
              Läuft auf der Seite der Drohne, sendet Kamerabild, DHT22- und Ultraschalldaten an Empfänger

   Verbindungen: NRF24L01 per SPI anschließen, zudem: CE ->   D4
                                                      CSN ->  D2

                 MLX90640 per I2C anschließen

                 DHT22  ->  27 (OneWire)

                 die Vier Ultraschallsensoren folgendermaßen verbinden:
                 trigger(1) ->  D32, echo(1)  ->  D36
                 trigger(2) ->  D33, echo(2)  ->  D39
                 trigger(3) ->  D25, echo(3)  ->  D34
                 trigger(4) ->  D26, echo(4)  ->  D35


  Anmerkungen: Die Bibliothek "RF24.h" musste modifiziert werden, damit sie auf dem ESP32 kompiliert, Archiv im Anhang.
               Boardverwalter-Url für ESP32: https://dl.espressif.com/dl/package_esp32_dev_index.json
*/


//einbinden der benötigten Bibliotheken
#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include <SPI.h>
#include "RF24.h"
#include <SimpleDHT.h>

//Klassenobjekte
RF24 radio(4, 2);

//globale Variablen: Pindefinitionen
const int trigger_sens [4] = {32, 33, 25, 26};                 //define trigger pins for ultrasonic sensors: 2 front, 4 rear, 6 left, 17 right
const int echo_sens [4] = {36, 39, 34, 35};                    //define echo pins for ultrasonic sensors: 3 front, 5 rear, 7 left, 16 right
const int pinDHT22 = 27;

//Datenframes, die bestimmte Abschnitte markieren
const float endImage[8] = { -1, -1, -1, -1, -1, -1, -1, -1};        //End of Image

//Adressen der RF24-Module
const byte addresses[][6] = {"1Node", "2Node"};

//I2C-Adresse und Daten der Kamera
const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
#define TA_SHIFT 8 //Default shift for MLX90640 in open air
paramsMLX90640 mlx90640;

//This is, where an Image is stored
float mlx90640To[768];

//Struct with all the additional informations
struct uavData {
  uint16_t distance[4];
  uint8_t temperature = 123;
  uint8_t humidity;
  uint8_t errorCount;
};

uavData myData;

void setup()
{
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  if (isConnected() == false)   //if camera isn't detected at I2C Adress, freeze
  {
    while (1);
  }

  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);

  //Once params are extracted, we can release eeMLX90640 array

  MLX90640_SetRefreshRate(MLX90640_address, 0x03); //Set rate to 4Hz

  radio.begin();

  radio.setDataRate(RF24_1MBPS);           // Raise the data rate to reduce transmission distance and increase lossiness
  radio.setAutoAck(1);                     // Ensure autoACK is enabled
  radio.setRetries(2, 15);                 // Optionally, increase the delay between retries. Want the number of auto-retries as high as possible (15)
  radio.setCRCLength(RF24_CRC_16);         // Set CRC length to 16-bit to ensure quality of data

  radio.openWritingPipe(addresses[0]);

  for (int i = 0; i < 4; i++) {           //prepare ultrasonic sensors
    pinMode(trigger_sens[i], OUTPUT);
    pinMode(echo_sens[i], INPUT);
  }
}

void loop()
{
  for (byte x = 0 ; x < 2 ; x++)      //get data from Camera
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }

  for (int i = 0; i < 768; i += 8)        //split data int 32Byte-Frames and send them to the other Module
    radio.write(&mlx90640To[i], 32);

  radio.write(&endImage, 32);           //write endOfImage-condition to let the other module know, that the struct is the next Item to receive

  if (TemperatureMeasurement(&myData.temperature, &myData.humidity) != SimpleDHTErrSuccess) myData.errorCount++; //handle errors
  else myData.errorCount = 0;
  for (int i = 0; i < 4; i++) {                                                                               //get data from ultrasonic sensors
    myData.distance[i] = DistanceMeasurement(trigger_sens[i], echo_sens[i], myData.temperature);
  }

  radio.write(&myData, sizeof(myData));   //send the struct, receiver knows that transmission is complete.
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

/*Measures the distance to an object using HC-SR04 Ultrasonic sensors, range: 2-400cm
   param1 triggerpin of the sensor
   param2 echopin of the sensor
   param3 current ambient temperature
   return distance to object in cm
*/
int DistanceMeasurement (int triggerpin, int echopin , byte temperature) {
  digitalWrite(triggerpin, HIGH);
  delayMicroseconds(15);
  digitalWrite(triggerpin, LOW);                    //trigger the sensor
  long duration = pulseIn(echopin, HIGH, 50000);    //start the measurement between trigger and echo
  double speedOfSound = ((331.5 + (0.6 * temperature)) / 10000); //calculate current speed of sound
  int distance = (duration / 2) * speedOfSound;    //calculate distance to object in cm
  if (distance >= 400 || distance <= 2) distance = 0; //replace invalid values by 0
  return distance;
}


/*Measures the ambient temperature using DHT22 temperature sensor
   Maximum frequency of the function call: 0.5 Hz
   param1: pointer to temperature variable
   param2: pointer to humidity variable
   return: errorcode of the measurement
*/
int TemperatureMeasurement (byte* temperature, byte * humidity ) {
  SimpleDHT22 dht22(pinDHT22);
  byte temp_buf, hum_buf;
  int err = SimpleDHTErrSuccess;
  err = dht22.read(&temp_buf, &hum_buf, NULL);      //read out DHT22 values
  if (err == SimpleDHTErrSuccess) {                 //check for Error
    *temperature = temp_buf;                        //set temperature
    *humidity = hum_buf;                            //set humidity
  }
  return err;
}
