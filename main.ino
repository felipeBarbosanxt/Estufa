// Conexão com o Blynk
#define BLYNK_TEMPLATE_ID "TMPL27qJEmJdg"
#define BLYNK_TEMPLATE_NAME "Felipe Barbosa dos Santos"
#define BLYNK_AUTH_TOKEN "QfilIM9eMwz5iAKfArgc7Xc9p8HzIrNk"
#define BLYNK_PRINT Serial

#include <BlynkSimpleEsp32.h> // Biblioteca para conectar o ESP32 ao Blynk
#include <WiFi.h>            // Biblioteca para gerenciar conexão WiFi
#include <DHT.h>             // Biblioteca para o sensor de temperatura e umidade DHT11
#include <Wire.h>            // Biblioteca para comunicação I2C
#include <BH1750.h>          // Biblioteca para o sensor de luminosidade BH1750

// Constantes
#define DHTPIN 32
#define DHTTYPE DHT11
#define SOILMOISTUREPIN  33
#define RELE_FAN 4
#define RELE_LIGHT 5
#define limitTemperature 25
#define limitLight 50

// Informações para conectar o ESP32 à rede WiFi e ao Blynk
char auth[] = BLYNK_AUTH_TOKEN; // Token do Blynk
char ssid[] = "...";            // Nome da rede WiFi
char pass[] = "505283fb";       // Senha da rede WiFi

// Definindo objetos
DHT dht(DHTPIN, DHTTYPE);
BH1750 bh1750;
BlynkTimer timer;

void sendSensorData() {
    // Leituras dos sensores
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    int soilMoisture = analogRead(SOILMOISTUREPIN);
    float light = bh1750.readLightLevel(); // Leitura da luminosidade em lux

    int percentSoilMoisture = map(soilMoisture, 1490, 1100, 0, 100);
    percentSoilMoisture = constrain(percentSoilMoisture, 0, 100);

    int percentLight = map(light, 0, 190, 0, 100);
    percentLight = constrain(percentLight, 0, 100);

    // Verifica funcionamento do DHT11
    if (isnan(temperature) || isnan(humidity)) {  
      Serial.println("Falha ao ler sensor DHT");
      return; // Encerra a função se houver erro no DHT
    }

    activeFan(temperature);  // Chamando a função que ativa as ventoinhas
    activeLight(light);      // Chamando a função que ativa os LEDs

    // Enviando dados para o Blynk
    Blynk.virtualWrite(V3, percentSoilMoisture); // Umidade do solo
    Blynk.virtualWrite(V1, humidity);           // Umidade do ar
    Blynk.virtualWrite(V0, temperature);        // Temperatura
    Blynk.virtualWrite(V2, percentLight);       // Luminosidade em lux


    // Exibindo dados no monitor serial
    Serial.println("Temperatura: ");
    Serial.println(temperature);
    Serial.println("Umidade do ar: ");
    Serial.println(humidity);
    Serial.println("Umidade do solo bruta: ");
    Serial.println(soilMoisture);
    Serial.println("Umidade do solo: ");
    Serial.println(percentSoilMoisture);
    Serial.println("Luminosidade bruta (lux): ");
    Serial.println(light);
    Serial.println("Porcentagem luminosidade: ");
    Serial.println(percentLight);
    Serial.println("--------------------");
}

void activeFan(float temperature) {
  if (temperature > limitTemperature) {
    digitalWrite(RELE_FAN, LOW); // Liga as ventoinhas
    Serial.println("Ventoinhas ligadas");
  }else {
    digitalWrite(RELE_FAN, HIGH);
  }
}

void activeLight(float light) {
  if (light < limitLight) {
    digitalWrite(RELE_LIGHT, LOW);
    Serial.println("LEDs ligados");
  } else {
    digitalWrite(RELE_LIGHT, HIGH);
  }
}

void setup() {
  Serial.begin(115200); // Inicialização do monitor serial
  pinMode(RELE_FAN, OUTPUT);
  pinMode(RELE_LIGHT, OUTPUT);
  pinMode(SOILMOISTUREPIN, INPUT); // Configuração do pino do sensor de umidade do solo como entrada

  dht.begin();     // Inicializando o sensor DHT
  Wire.begin(21, 22); // Inicializando o barramento I2C nos pinos personalizados (SCL, SDA)

  // Inicializando o sensor BH1750
  if (!bh1750.begin(BH1750::CONTINUOUS_LOW_RES_MODE)) {
    Serial.println("Falha ao inicializar o BH1750. Verifique as conexões.");
    while (1); // Para o programa se der erro 
  } else {
    Serial.println("BH1750 inicializado com sucesso.");
  }

    delay(100); // Atraso para garantir que todos os sensores estejam prontos    
    Blynk.begin(auth, ssid, pass);   // Conexão ao Blynk
    timer.setInterval(4000L, sendSensorData); // Configuração do timer para leituras a cada 5 segundos
}

void loop() {
  Blynk.run(); // Gerenciando a comunicação com o Blynk
  timer.run(); // Executando as funções agendadas pelo timer
}