/***************************************************************************
  This is a library for the EnviromentNode FTTech
  +--------------------+--------------+--------+-------+
  |     Sensor Name    | Manufacturer |  Model | Click |
  +--------------------+--------------+--------+-------+
  | Rain Gauge         |       -      | WS3000 |   1   |
  +--------------------+--------------+--------+-------+
  | Wind Set           | Campbell     | 034B   |   2   |
  +--------------------+--------------+--------+-------+
  | Particulate Matter | AlphaSense   | OPC-N2 |   2   |
  +--------------------+--------------+--------+-------+
  | Lipo Battery Level |       -      | A8     |   -   |
  +--------------------+--------------+--------+-------+
  | Weather Click      | MikroE       | 1978   |   3   |
  +--------------------+--------------+--------+-------+
  | Solar Radiatio     | Apogee       | SQ-110 |   1   |
  +--------------------+--------------+--------+-------+

  These sensors uses diferent comunication, like I2C, SPI, Serial and 
  simpler ones like basic Analog and Digital signals.

  After each reading, the data is sent using XBee Mesh on Click 4 

  Please report if you find any issue when using this code so when can
  keep improving it

  CHANGES HISTORY:
  - 3. * Improved interruption for the Rain Gauge
       * Removed tempoTotal from RainGauge as time measurements aren't
          good while using sleeping methods
  - 4. * Redone the code to work with the latests libraries:
            - FTTech SAMD51 Clicks 1.3.7
            - FTTech SAMD51 XBee 1.5.0
       * But you will need to use this specific
            - FTTech SAMD Boards 1.0.4

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 ***************************************************************************/
#include "FTTech_SAMD51Clicks.h"
#include <Adafruit_SleepyDog.h>
#include "itoa.h"
// XBEE 2
#include <FTTech_Xbee.h>
// CAMPBELL WATHER SET SENSOR
#include "WS_034B.h"
// WEATHER CLICK
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// ALPHASENSE OPC-N2 PARTICULATE MATTER SENSORK
#include "opcn2.h"

/* ***************************************************************************
 * XBEE 2
 * ***************************************************************************
 * Used in the first and only 20-pin position with UFL antena's conector  
 */
  uint8_t kyCmdValue[] = "WaspmoteLinkKey!";
  uint8_t keyLength = 16;
  int BAUDRATE = 115200;
  int address_HB = 0x0013A200;
  int address_LB = 0x40B5F379;
//int address_HB = 0x00000000;
//int address_LB = 0x0000FFFF;
//  XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x0000ffff);
//  ZBTxStatusResponse txStatus = ZBTxStatusResponse();
/* ***************************************************************************
 * CAMPBELL WATHER SET SENSOR
 * ***************************************************************************
 * To connect the WeatherSet with the baord, use +3.3V and GND to power the WD
 * and GND as the reference to the WS
 * 
 * WD GND/VCC/Reading
 * WS GND/Reading
 * 
 * Marrom    - GND
 * Preto     - A2  - NB(A4)
 * Vermelho  - 3.3V
 * 
 * Branco    - GND
 * Verde     - A3  - NB(12)
 */ 
  #define wSpeedPin 12
  #define wDirecPin A4
  WS_034B objWindSet(wSpeedPin, wDirecPin);
  float storeddata[5];
  float velocityToSend;
  uint8_t control = 0;
  unsigned long timeInitWS = 0;
  unsigned long timeOfReadingsWS = 10 * 1000; // It will read the sensor in  a while loop for this time (ms)
/* ***************************************************************************
 * ALPHASENSE OPC-N2 PARTICULATE MATTER SENSOR
 * ***************************************************************************
 * To connect the OPC-N2, use the default SPI pins.
 * 
 * Vermelho - MOSI  (24)
 * Branco   - MISO  (22)
 * Marrom   - SCK   (23)
 * Verde    - CS    (0) - NB(44)
 * Amarelo  - 5V
 * Preto    - GND
 */
  #define CS 44
  char infoString[61];
  OPCN2 objOpcn2(CS); 
/* ***************************************************************************
 * WAETHER CLICK
 * ***************************************************************************
 * Used on Click3 slot. It uses I2C
 */ 
  #define SEALEVELPRESSURE_HPA (1019)
  Adafruit_BME280 bme;
/* ***************************************************************************
 * RAIN GAUGE WS3000
 * ***************************************************************************
 * To connect the WS3000 with the baord, use GND and DigitalPin. Remmember
 * to add an pull up resistor with 10k.
 * WD GND/VCC/Reading
 * WS GND/Reading
 * 
 * Laranja   - NB (51) + 10k pullup + 10nF cap signal to gnd
 * Verde     - GND 
 * 
 * 
 */ 
  uint8_t rainGaugePin = 10;
  volatile int rainFallCount = 0;
  volatile float rainMM = 0;
  uint16_t risingTime = 500; //micro secs
/* ***************************************************************************
 * APOGEE QUANTUM SENSORE SQ-110 Serial Numbers range 0-25369
 * ***************************************************************************
 * This sensor uses 2 outputs, one positive and another ngative.
 * WARNING: The negative can't be read. Beaware of the voltage devider 
 * to read it.
 * 
 * Vermelho - Positive - NB(A0)
 * Black    - Negative - NB(A1)
 * Branco   - GND
 */
  #define APPosPin A0
  #define APNegPin A1
/* ***************************************************************************/

#define DEBUG false
#define DEBUG_BAUDRATE 9600
#define BatteryPin A8
int fatorCorrecao = 75;
uint16_t SLEEP_TIME = 3 * 60; // time in seconds
FTTech_Xbee xbee; // Create object with the serial parameter
uint8_t payload[100] = {0}; // Creates payload, and defines its maximum size

void setup() {
  Serial.begin(DEBUG_BAUDRATE);

  if(DEBUG){
    while(!Serial) delay(10); 
    // wait for Arduino Serial Monitor (native USB boards)
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  FTClicks.begin();
  pinMode(43,OUTPUT);
  digitalWrite(43,HIGH);
  pinMode(49,OUTPUT);
  digitalWrite(49,HIGH);
  pinMode(50,OUTPUT);
  digitalWrite(50,HIGH);
  pinMode(52,OUTPUT);
  digitalWrite(52,HIGH);
  delay(5000);

  Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  Serial.println("CS-034B - OPCN2 - SQ110 - WeatherClick - WS3000");

  //////////////////////////////////////////////////
  // Init XBee
  Serial.println("+++++++++++++++++++++++++++++++++++++++");
  Serial.println("Creating XBee object and Serial 4 ");
  digitalWrite(A9, HIGH); //Xbee Reset
  xbee.begin(BAUDRATE);
  
  //////////////////////////////////////////////////
  // FTClicks.XBeeSetEncryptionMode(false, false); 
  // FTClicks.XBeeSetEncryptionKey(kyCmdValue, keyLength, false); 
  // FTClicks.XBeeWriteEEPROM(false);
  //////////////////////////////////////////////////

  //////////////////////////////////////////////////
  //Init Weather Click
  Serial.println("+++++++++++++++++++++++++++++++++++++++");
  Serial.println("Init  Weather Click");
  delay(2000);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }else
  {
    Serial.println("Found BME280 sensor");
  }
  //////////////////////////////////////////////////

  //////////////////////////////////////////////////
  //Init OPCN2 Click
  Serial.println("+++++++++++++++++++++++++++++++++++++++");
  Serial.println("Init  OPC-N2");
  objOpcn2.on();
  if(objOpcn2.ping())
  {
    Serial.println("OPC-N2: Pinged Successfully");
    
    if(objOpcn2.read_information_string(infoString))
    {
      Serial.println("OPC-N2: Information string extracted");
      Serial.print("\t->");Serial.println(infoString);
      if(objOpcn2.read_configuration_variables())
      {
      // Read and print the configuration variables    
        Serial.println("\nConfiguration Variables");
        Serial.print("\tGSC:\t"); Serial.println(objOpcn2.gain);
        Serial.print("\tSFR:\t"); Serial.println(objOpcn2.sample);
        Serial.print("\tLaser DAC:\t"); Serial.println(objOpcn2.laserDAC);
        Serial.print("\tFan DAC:\t"); Serial.println(objOpcn2.fanDAC);
        Serial.print("\tToF-SFR:\t"); Serial.println(objOpcn2.timeFlight);
      }
    }else
    {
      Serial.println("OPC-N2: Couldn't extract Information String");
    }

    if(objOpcn2.read_firmware_version())
    {
      Serial.println("OPC-N2: read_firmware_version OK");
      Serial.print("\t->");Serial.print(objOpcn2.firm_ver.major);Serial.print(".");Serial.println(objOpcn2.firm_ver.minor);
    }else
    {
      Serial.println("OPC-N2: Couldn't extract firmware_version");
    }
  }else
  {
    Serial.println("OPC-N2: Could't ping device. Check connections");
  }
  objOpcn2.off();

  //////////////////////////////////////////////////
  // Init WS3000
  // make the raingauge's pin an input:
  pinMode(rainGaugePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainGaugePin), onBucketTip, RISING);
  //////////////////////////////////////////////////

  //FTClicks.OFF(1);//has to be on because of the pull up in the rain gauge
  digitalWrite(49,LOW);
  digitalWrite(50,LOW);
  //FTClicks.OFF(4);//has to be on because of the xbee

  Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  if(!DEBUG) USBDevice.detach();

}

void loop() {
  //Without some delay here it loses the first package due to boot time in the xbee
  delay(2500); 

  Watchdog.enable(8000);
  BatteryMeasument();
  Watchdog.disable();

  // CLICK 2
  Serial.println("READING CLICK 2");
  digitalWrite(49,HIGH);
  Serial.println("WeatherSetSensor"); 
  Watchdog.enable(16000);
  WeatherSetSensor();
  delay(2000);        // a little bit more time to the ParticulateMatterSensor;
  Watchdog.reset();
  ParticulateMatterSensor();
  digitalWrite(49,LOW);
  Watchdog.disable();

  // CLICK 3
  Serial.println("READING CLICK 3");
  digitalWrite(50,HIGH);
  Watchdog.enable(16000);
  bme.begin(0x76);
  Serial.println("Step 2");
  delay(1000); // a little bit more time to the WeatherClickSensor;
  WeatherClickSensor();  
  digitalWrite(50,LOW);  
  Watchdog.disable();

  // CLICK 1
  Serial.println("READING CLICK 1");
  Watchdog.enable(8000);
  QuantunSensor();
  WeatherRainGauge();
  //FTClicks.OFF(1); //always on to receive rain
  Watchdog.disable();

  digitalWrite(52,LOW);
  ResetRainGauge();

  if(DEBUG)
  {
    Serial.println("-- >> GOING TO SLEEP << --");
    Serial.flush();
    USBDevice.detach();
   }
  digitalWrite(LED_BUILTIN, LOW);
  uint32_t sleepMS = FTClicks.sleepForSeconds(SLEEP_TIME);
  digitalWrite(LED_BUILTIN, HIGH);
  if(DEBUG)
  {
    USBDevice.attach();
    while(!Serial){};
   }

  digitalWrite(52,HIGH);;

  Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

}

void onBucketTip() {
  noInterrupts();
  unsigned long lastPulseTime = micros();
  while(( micros() - lastPulseTime) < 2*risingTime ){};
  
  rainFallCount++;
  rainMM += 0.2794; // calculate precipitation (mm)
  
  interrupts();  
}

void ResetRainGauge(void)
{
  rainFallCount = 0;
  rainMM = 0;
}

void WeatherRainGauge(void)
{  
  char sensor[] = "WS3000";  
  xbee.messageClearPayload();
  xbee.message(sensor, 0, rainFallCount, rainMM);
  xbee.getOutPayload(payload);
  xbee.sendPayload(address_HB, address_LB);
}

void QuantunSensor(void)
{
  analogReadResolution(12);
  uint32_t APpositve = analogRead(APPosPin);
  uint32_t APnegative = analogRead(APNegPin);
  float APPosValue = APpositve*3.3/(pow(2, 12)-1);
  float APNegValue = APnegative*3.3/(pow(2, 12)-1);
  analogReadResolution(10);

  if(DEBUG)
  {
    Serial.print("Positive: "); Serial.print(APpositve); Serial.print("\tVolts: "); Serial.println(APpositve*3.3/(pow(2, 12)-1));
    Serial.print("Negative: "); Serial.print(APnegative);  Serial.print("\tVolts: "); Serial.println(APnegative*3.3/(pow(2, 12)-1));
    Serial.print("Diff: "); Serial.println(APPosValue - APNegValue); 
  }

  char sensor[] = "SQ100";
  xbee.messageClearPayload();
  xbee.message(sensor, APPosValue, APNegValue);
  xbee.getOutPayload(payload);
  xbee.sendPayload(address_HB, address_LB);
}

void BatteryMeasument()
{
  float bat = analogRead(LIPO_BATTERY) + fatorCorrecao;
  float voltage = FTClicks.readBattery(); //bat*3.3/1023 * (330000 + 1000000)/(330000);
  if(DEBUG)
  {
    Serial.print("Bateria Analog: ");  Serial.println(bat);
    Serial.print("Bateria: ");  Serial.print(voltage); Serial.println("[V]");
  }

  char sensor[] = "BAT";
  xbee.messageClearPayload();
  xbee.message(sensor, bat, voltage);
  xbee.getOutPayload(payload);
  xbee.sendPayload(address_HB, address_LB);
}

void WeatherSetSensor()
{
    Serial.println("before readWindSet"); 
    objWindSet.readWindSet();

  /*IMPLEMENTANDO O FILTRO DE JHONATAN*/

  // Se a leitura for diferente de 0
  if(objWindSet.freqKmph > 0){

    // Caso não exista nenhuma leitura anterior
    if(control == 0){
      for(uint8_t i = 0; i<=4; i++){
        storeddata[i] = 0;
      }
      // Atualiza o vetor
      storeddata[0] = objWindSet.freqKmph; // Menor valor lido
      storeddata[1] = objWindSet.freqKmph; // Maior valor lido
      storeddata[4] = objWindSet.freqKmph; // Último valor lido
      velocityToSend = objWindSet.freqKmph; // Velocidade que será enviada ao banco de dados
    }

    // Caso exista apenas 1 leitura anterior
    else if(control == 1){
      if(objWindSet.freqKmph < storeddata[0]) storeddata[0] = objWindSet.freqKmph; // Menor valor lido
      if(objWindSet.freqKmph > storeddata[1]) storeddata[1] = objWindSet.freqKmph; // Maior valor lido
      storeddata[3] = storeddata[4];
      storeddata[4] = ((2*objWindSet.freqKmph)+((storeddata[0]+storeddata[1])/2))/3;

      velocityToSend = storeddata[4];
    }

    // Caso já existam 2 leituras anteriores
    else if(control >= 2){
      // Atualiza o vetor
      storeddata[2] = storeddata[3];
      storeddata[3] = storeddata[4];

      if((storeddata[3] - storeddata[2]) > 0) storeddata[4] = (storeddata[0] + 2*storeddata[1] + 3*storeddata[4])/6;
      if((storeddata[3] - storeddata[2]) == 0) storeddata[4] = (storeddata[0] + storeddata[1] + 3*storeddata[4])/5;
      if((storeddata[3] - storeddata[2]) < 0) storeddata[4] = (2*storeddata[0] + storeddata[1] + 3*storeddata[4])/6;

      //Atualizo o maior e menor valor do vetor
      if(storeddata[4] < storeddata[0]) storeddata[0] = storeddata[4];
      if(storeddata[4] > storeddata[1]) storeddata[1] = storeddata[4];

      velocityToSend = storeddata[4];
    }

    // Atualiza a cada 3 horas
    if(control <= 40) ++control;
    else control = 0;

  }
    else velocityToSend = objWindSet.freqKmph;

    if(DEBUG)
    {
      Serial.print("WSpeed [Hz]: ");
      Serial.print(objWindSet.freq);
      Serial.print("\tWSpeed [Km/h]: ");
      Serial.print(velocityToSend);
      Serial.print("\tWD Degree:");
      Serial.println(objWindSet.windDirecDegrees);
    } else delay(500);

    // Envia a payload pelo Xbee
    char sensor[] = "034B";
    xbee.messageClearPayload();
    xbee.message(sensor, velocityToSend, objWindSet.windDirecDegrees);
    xbee.getOutPayload(payload);
    xbee.sendPayload(address_HB, address_LB);

}

void WeatherClickSensor()
{
  float temp = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float humidity = bme.readHumidity();
  if(DEBUG)
  {
    Serial.print("Temperature = ");
    Serial.print(temp);
    Serial.println("*C");

    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println("hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(altitude);
    Serial.println("m");

    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println("%");
  }

  char sensor[] = "CLK_WTH";
  xbee.messageClearPayload();
  xbee.message(sensor, temp, pressure, altitude, humidity);
  xbee.getOutPayload(payload);
  xbee.sendPayload(address_HB, address_LB);
} 


void ParticulateMatterSensor(){
    delay(2000);
    objOpcn2.on();

    objOpcn2.read_PMs(3000, 5000);
  
    if(DEBUG)
    {
      Serial.print(F("\nSampling Period (s):\t")); Serial.println(objOpcn2.periodCount);
      Serial.print(F("PM 1: "));
      Serial.print(objOpcn2.PM1);
      Serial.println(F(" ug/m3"));
      Serial.print(F("PM 2.5: "));
      Serial.print(objOpcn2.PM2_5);
      Serial.println(F(" ug/m3"));
      Serial.print(F("PM 10: "));
      Serial.print(objOpcn2.PM10);
      Serial.println(F(" ug/m3"));
    }

    char sensor[] = "OPCN2";
    xbee.messageClearPayload();
    xbee.message(sensor, objOpcn2.PM1, objOpcn2.PM2_5, objOpcn2.PM10, objOpcn2.periodCount);
    xbee.getOutPayload(payload);
    xbee.sendPayload(address_HB, address_LB);
}
