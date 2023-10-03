#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_INA219.h> 
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
// --------------------------------------
WiFiUDP udp;
IPAddress raspberryPiIP(255, 255, 255, 255);  // Replace with the Raspberry Pi's IP address (192, 168, 4, 255)
const int udpPort = 1234;


#define SAMPLE_TIME 2000

Adafruit_INA219 ina219_0 (0x68);
TinyGPSPlus gps;
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
Adafruit_AHTX0 aht;
Adafruit_Sensor *aht_humidity, *aht_temp;

float current = 0;
float power = 0;
float lat = 0;
float lon = 0;
float temperatura_bmp = 0;
float pressao_bmp = 0;
float altitude_bmp = 0;
double ozonio = 0;
double carbono = 0;
float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
sensors_event_t humidity;
sensors_event_t temp;

//Definição dos parâmetros do sensor de O3
#define RL_o3 10     // Resistência ao lado do DOUT_LED
#define APin_o3 34   // Pino analógico utilizado
float curve_o3[2] = {-0.32372, 0.648};  // Curva do gráfico em log do MQ131 para O3 (a, b)

//Definição dos parâmetros do sensor de CO2
#define RL_co2 10     // Resistência ao lado do DOUT_LED
#define APin_co2 33   // Pino analógico utilizado

float curve_co2[2] = {-0.32372, 0.648};  // Curva do gráfico em log do MQ135 para CO2 (a, b)

float R0_co2 = 0;
float R0_o3 = 0;

// -------------- Configuracoes WiFi -----------------------
//Definindo as informações da rede Wi-Fi
const char* ssid = "Embauba"; //Define o nome do ponto de acesso
const char* pass = "satelitaos2"; //Define a senha




// ------------------ Calibracao ----------------------------
void calibracao_co2(){
  int cont;
  float val=0;
  // Calcula o valor de RS no cenário de ar limpo 50 vezes e pega a média
  for (cont=0;cont<50;cont++) {
                val += ((float)RL_co2 * (4095-analogRead(APin_co2)) / analogRead(APin_co2));
                delay(500);
  }
  val = val/50;                                                                                        
  R0_co2 = val;
}

void calibracao_o3(){
  int cont;
  float val=0;
  // Calcula o valor de RS no cenário de ar limpo 50 vezes e pega a média
  for (cont=0;cont<50;cont++) {
                val += ((float)RL_o3 * (4095-analogRead(APin_o3)) / analogRead(APin_o3));
                delay(500);
  }
  val = val/50;                                                                                        
  R0_o3 = val;
}
// ----------------------------------------------------------

// ------------------------ Leituras ------------------------

void readINA(){
    current = ina219_0.getCurrent_mA(); /* comando para chamar a corrente */
    power = ina219_0.getPower_mW(); /*comando para chamar a potência */

    Serial.print("Corrente: "); 
    Serial.print(current); 
    Serial.println(" mA"); /*printa a corrente */
    Serial.print("Potência: "); 
    Serial.print(power); 
    Serial.println(" mW"); /* printa a potência */
}

void readGPS()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid()){
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }
}

void readBMP(){
    temperatura_bmp = bmp.readTemperature();
    pressao_bmp = bmp.readPressure();
    altitude_bmp = bmp.readAltitude(1013.25);
    Serial.print(F("Temperatura = "));
    Serial.print(temperatura_bmp);
    Serial.println(" *C");
    //Imprimindo os valores de Pressão
    Serial.print(F("Pressão = "));
    Serial.print(pressao_bmp);
    Serial.println(" Pa");
    //Imprimindo os valores de Altitude Aproximada
    Serial.print(F("Altitude Aprox = "));
    Serial.print(altitude_bmp); /* Ajustar a pressão de nível do mar de acordo com o local!*/
    Serial.println(" m");

}

void readMQ131(){
    double ADCread=0;
    double RS, RSR0, Y, X;

    //5 Leituras e tira a media
    for (int count=0;count<5;count++) {
                    ADCread += analogRead(APin_o3);
                    delay(50);
    }
    ADCread = ADCread/5;

    //Calcula RS
    RS = (float)RL_o3 * (4095-ADCread) / ADCread;

    //Calcula RS/R0
    RSR0 = RS/R0_o3;

    //Tira o Log de RSR0 para utilizar na curva log-log (Y)
    Y = log10(RSR0);

    //Calcula o X
    X = (Y - curve_o3[1])/curve_o3[0];

    ozonio =  pow10(X);
    Serial.print("Ozonio: ");
    Serial.println(ozonio);
}

void readMQ135(){
    double ADCread=0;
    double RS, RSR0, Y, X;

    //5 Leituras e tira a media
    for (int count=0;count<5;count++) {
                    ADCread += analogRead(APin_co2);
                    delay(50);
    }
    ADCread = ADCread/5;

    //Calcula RS
    RS = (float)RL_co2 * (4095-ADCread) / ADCread;

    //Calcula RS/R0
    RSR0 = RS/R0_co2;

    //Tira o Log de RSR0 para utilizar na curva log-log (Y)
    Y = log10(RSR0);

    //Calcula o X
    X = (Y - curve_co2[1])/curve_co2[0];

    //Retorna 10^X = PPM
    carbono = pow10(X);
    Serial.print("Carbono: ");
    Serial.println(carbono);
}

void readMPU(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
  /* Print out the values */
  Serial.print("AccelX:");
  Serial.print(accX);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(accY);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.println(accZ);
  Serial.print("GyroX:");
  Serial.print(gyroX);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(gyroY);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.println(gyroZ);
}

void readAHT(){
  // Serial.print("AHT temperatura: ");
  // Serial.print(aht_temp);
  // Serial.print("\nAHT Umidade: ");
  // Serial.print(aht_humidity);
  aht_humidity->getEvent(&humidity);
  aht_temp->getEvent(&temp);
  Serial.print("AHT temperatura: ");
  Serial.println(temp.temperature);
  Serial.print("Umidade: ");
  Serial.println(humidity.relative_humidity);
}

void saveSD(){
  Serial.println("Salvando no SD...");
}

void espToRasp(){
  // StaticJsonDocument<240> jsonBuffer; //Cada par de valores utiliza aproximadamente 16 bytes
  //                                     //Cada par nome-vetor utiliza aproximadamente 16*(1+N) bytes, em que N é o comprimento do vetor 
  // //Criando um objeto JsonObject para armazenar os valores dos sensores
  // JsonObject sensores = jsonBuffer.to<JsonObject>();
  DynamicJsonDocument sensores(256);

  //Adicionando os valores dos sensores ao JsonObject
  sensores["sat"] = "Embauba";
  sensores["cur"] = current;
  sensores["pot"] = power;
  sensores["temp"] = temperatura_bmp;
  sensores["press"] = pressao_bmp;
  sensores["alt"] = altitude_bmp;
  sensores["gX"] = gyroX;
  sensores["gY"] = gyroY;
  sensores["gZ"] = gyroZ;
  sensores["aX"] = accX;
  sensores["aY"] = accY;
  sensores["aZ"] = accZ;
  sensores["o3"] = ozonio;
  sensores["co2"] = carbono;

  //Convertendo o JsonDocument em uma string JSON
  // String jsonString;
  // serializeJson(jsonBuffer, jsonString);
  // udp.beginPacket(raspberryPiIP, udpPort);
  // udp.print(jsonString);
  // udp.endPacket();
  // // Imprimindo a string JSON no monitor serial
  // Serial.println(jsonString);

  String jsonStr;
  serializeJson(sensores, jsonStr);
  // Send the JSON packet to the Raspberry Pi's IP address
  udp.beginPacket(raspberryPiIP, udpPort);
  udp.print(jsonStr);
  udp.endPacket();
  Serial.println(jsonStr);


}


void setup (){
    Serial.begin(115200);
    Serial.println("Hello, world!");

    // Inicialização do INA
    if (! ina219_0.begin()) 
	{ 
		while (1) {
            Serial.println("Falha ao encontrar o INA219"); 
            delay(10); 
	    } 
    }
    Serial.println("INA inicializado...");

    // Inicializa o GPS
    Serial2.begin(115200);
    Serial.println("GPS inicializado..."); 

    // Inicializa o BMP
    if (!bmp.begin(0x76)) { /*Definindo o endereço I2C como 0x76. Mudar, se necessário, para (0x77)*/
    //Imprime mensagem de erro no caso de endereço invalido ou não localizado. Modifique o valor 
    Serial.println(F(" Não foi possível encontrar um sensor BMP280 válido, verifique a fiação ou "
                      "tente outro endereço!"));
    while (1) delay(10);
    }
    Serial.println("BMP inicializado...");

    //  ----------------------- Calibrando sensores ------------------------------------------
    Serial.println("Calibrando os sensores...");
    calibracao_co2();
    calibracao_o3();
    Serial.println("Sensores calibrados...");

    // Inicializando o MPU
    if (!mpu.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
      }
    }
    Serial.println("MPU inicializado...");

    //setupt motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    //  Inicializando AHT
//      if (!aht.begin()) {
//       Serial.println("Failed to find AHT10/AHT20 chip");
//       while (1) {
//       delay(10);
//       }
//       // aht_temp = aht.getTemperatureSensor();
//       // Serial.print("AHT temperatura: ");
//       // aht_temp->printSensorDetails();
//       // Serial.print("\nAHT Umidade: ");
//       // aht_humidity = aht.getHumiditySensor();
//       // aht_temp->printSensorDetails();
// } 
//     Serial.println("AHT inicializado...");

  // Inicializando o WiFi
  WiFi.softAP(ssid, pass); //Inicia o ponto de acesso
  Serial.print("Se conectando a: "); //Imprime mensagem sobre o nome do ponto de acesso
  Serial.println(ssid);
  Serial.print("Endereço de IP: "); //Imprime o endereço de IP
  Serial.println(WiFi.softAPIP()); //Endereço de IP
  udp.begin(udpPort); // Choose a port number

  
  Serial.println("Todos os sensores inicializados. Codigo rodando...");   
}

void loop() {

    readINA();
    readGPS();
    readBMP();
    readMQ131();
    readMQ135();
    readMPU();
    // readAHT();
    saveSD();
    espToRasp();
    delay(SAMPLE_TIME);
}