#include "BluetoothSerial.h"
#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;
TinyGPSPlus gps;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
#define DHTPIN 13
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

float temp;
float press;
float t_bmp;
float h_bmp;
bool bmp_ok;

const int oneWireBus = 4; //DS18B20 conectado ao GPIO4 
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

const int signalPin = 2;  // anemometro
unsigned long lastTime = 0;   
unsigned int frequency = 0;

#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

HardwareSerial gpsSerial(2);

/**************************************************************************************************************************************************/

float le_temperatura(){
  sensors.requestTemperatures(); 
  float temp = sensors.getTempCByIndex(0);
  return(temp);
}

/**************************************************************************************************************************************************/

unsigned long le_anemometro(){
  unsigned long currentTime = millis();
  unsigned long lastTime=currentTime;
  int lastSensorState;
  unsigned long pulseCount=0;
  while(currentTime-lastTime <= 1000){
    currentTime = millis();
    int sensorState = digitalRead(signalPin);
    if (sensorState == HIGH){
      if (lastSensorState == LOW){
        pulseCount++;
      }
    }
    lastSensorState = sensorState;
  }
  return(pulseCount);
}

/**************************************************************************************************************************************************/

float le_pressao(){
  float pressao=0.0;
  if(bmp_ok==true) {
    float t_bmp=bmp.readTemperature();
    pressao=bmp.readPressure();
  }
  return(pressao);
}

/**************************************************************************************************************************************************/

float le_umidade(){
  dht.read(DHTPIN);
  float t_dht11 = dht.readTemperature();
  float u_dht11 = dht.readHumidity();
  return(u_dht11);
}

//******************************************************************************************

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  sensors.begin();
  dht.begin();
  pinMode(signalPin, INPUT); // Configura o pino como entrada
  bmp_ok=false;
  if (bmp.begin()) {
	  Serial.println("valid BMP085/BMP180 sensor");
	  bmp_ok=true;
  }
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

/**************************************************************************************************************************************************/

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
    {
      Serial.print('*');
      SerialBT.print('*');
    } 

    Serial.print(' ');
      
    SerialBT.print(' ');


  }
  else
  {
    Serial.print(val, prec);      Serial.print('*');
    Serial.print(' ');
    SerialBT.print(val, prec);      SerialBT.print('*');
    SerialBT.print(' ');


    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
    {
      Serial.print(' ');
      SerialBT.print(' ');
    }
  }
  smartDelay(0);
}

/**************************************************************************************************************************************************/

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}

/**************************************************************************************************************************************************/

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
    SerialBT.print(F("********** "));
  }
  else
  {
    char sz[32];
    char tz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    sprintf(tz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
    Serial.print(tz);
    SerialBT.print(sz);
    SerialBT.print(tz);



  }
}

/**************************************************************************************************************************************************/

void le_gps(){
  printDateTime(gps.date, gps.time);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  Serial.print(" ");
}

/**************************************************************************************************************************************************/

float le_direcao(){
  return(0);
}

/**************************************************************************************************************************************************/

float le_radiacao(){
  return(0);
}

/**************************************************************************************************************************************************/

void loop() {
  le_gps();

  
  float temperatura=le_temperatura();
  float umidade=le_umidade();
  float pressao=le_pressao();
  unsigned long velocidade=le_anemometro();
  float direcao=le_direcao();
  float radiacao=le_radiacao();

  Serial.print(temperatura);
  Serial.print(" ");
  SerialBT.print(temperatura);
  SerialBT.print(" ");

  Serial.print(umidade);
  Serial.print(" ");
  SerialBT.print(umidade);
  SerialBT.print(" ");


  Serial.print(pressao);
  Serial.print(" ");
  SerialBT.print(pressao);
  SerialBT.print(" ");


  Serial.print(radiacao);
  Serial.print(" ");
  SerialBT.print(radiacao);
  SerialBT.print(" ");


  Serial.print (velocidade);
  Serial.print(" ");
  SerialBT.print (velocidade);
  SerialBT.print(" ");

  Serial.println(direcao);
  SerialBT.println(direcao);
  delay(2000);

}
