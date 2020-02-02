/*
   Autoren: Markus Brutzer, Michael Riffel

   Funktion:  Sketch wurde für ESP8266 (DOIT ESP Devkit V4) entwickelt!
              Läuft auf der Seite des Operators, empfängt Kamerabild, DHT22- und Ultraschalldaten von Drohne

   Verbindungen: NRF24L01 per SPI anschließen, zudem: CE ->   D4
                                                      CSN ->  D2
   Anmerkungen: Boardverwalter-Url für ESP8266: http://arduino.esp8266.com/stable/package_esp8266com_index.json
*/

//einbinden der benötigten Bibliotheken
#include <SPI.h>
#include "RF24.h"

//Klassenobjekte
RF24 radio(4, 2);

//Datenframes, die bestimmte Abschnitte markieren
const float endImage[8] = { -1, -1, -1, -1, -1, -1, -1, -1};        //End of Image

//Adressen der RF24-Module
const byte addresses[][6] = {"1Node", "2Node"};

//die einzelnen Frames des Bildes werden hier gespeichert
float mlx90640To[8];

//Struct with all the additional informations
struct uavData {
  uint16_t distance[4];
  uint8_t temperature;
  uint8_t humidity;
  uint8_t errorCount;
};

uavData myData;

void setup() {
  Serial.begin(115200);
  Serial.println("Ready to Receive Data from UAV");

  radio.begin();

  radio.setDataRate(RF24_1MBPS);           // Raise the data rate to reduce transmission distance and increase lossiness
  radio.setAutoAck(1);                     // Ensure autoACK is enabled
  radio.setRetries(2, 15);                 // Optionally, increase the delay between retries. Want the number of auto-retries as high as possible (15)
  radio.setCRCLength(RF24_CRC_16);         // Set CRC length to 16-bit to ensure quality of data

  radio.openReadingPipe(1, addresses[0]);

  // Start the radio listening for data
  radio.startListening();
}

void loop() {
  if (radio.available())                //when Data is available, read it into the array.
  {
    radio.read(&mlx90640To[0], 32);

    bool isEndOfImage = true;           //check for endOfImage
    for (int x = 0 ; x < 8 ; x++)
      isEndOfImage &= (mlx90640To[x] == endImage[x]);

    if (!isEndOfImage) {                  //if normal data, just print
      for (int x = 0 ; x < 8 ; x++)
      {
        Serial.print(mlx90640To[x], 2);
        Serial.print(",");
      }
    }
    else {                         //if end of image, read uavData. In this state of development, it is not used yet.
      Serial.println("");
      radio.read(&myData, sizeof(myData));
      /*
         TODO: do something with myData (Hand it to processing or so...
      */
    }
  }
}
