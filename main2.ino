#include <WiFi.h>            // Biblioteca para gerenciar conexão WiFi
#include <DHT.h>             // Biblioteca para o sensor de temperatura e umidade DHT11
#include <Wire.h>            // Biblioteca para comunicação I2C
#include <BH1750.h>          // Biblioteca para o sensor de luminosidade BH1750
#include <PubSubClient.h>    // Biblioteca para MQTT

// Constantes
#define DHTPIN 32
#define DHTTYPE DHT11
#define SOILMOISTUREPIN  33
#define RELE_FAN 4
#define RELE_LIGHT 5
#define LIMIT_TEMPERATURE 25
#define LIMIT_LIGHT 50

// Informações para conectar o ESP32 à rede WiFi
const char* ssid = "...";      // Nome da rede WiFi
const char* pass = "505283fb"; // Senha da rede WiFi

// Informações do MQTT Mosquitto
const char* mqtt_server = "192.168.0.146"; // Endereço IP do broker Mosquitto
WiFiClient espClient;
PubSubClient client(espClient);

// Definindo objetos
DHT dht(DHTPIN, DHTTYPE);
BH1750 bh1750;



// Função para reconectar ao MQTT em caso de desconexão
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    if (client.connect("ESP32_Client")) {
      Serial.println("Conectado ao MQTT!");
      // Inscreve-se em tópicos caso necessário
      // client.subscribe("sensor/#"); // Exemplo de inscrição em todos os tópicos de sensor
    } else {
      Serial.print("Falha, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos...");
      delay(5000);  // Espera antes de tentar novamente
    }
  }
}

// Função para inicializar a conexão Wi-Fi
void initWiFi() {
  WiFi.begin(ssid, pass);  // Conexão à rede WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
}

// Função para inicializar os pinos dos dispositivos
void initPins() {
  pinMode(RELE_FAN, OUTPUT);
  pinMode(RELE_LIGHT, OUTPUT);
  pinMode(SOILMOISTUREPIN, INPUT);
}

// Função para inicializar os sensores
void initSensors() {
  // Inicializando o sensor DHT
  dht.begin();
  if (isnan(dht.readTemperature()) || isnan(dht.readHumidity())) {
    Serial.println("Falha ao inicializar o sensor DHT. Verifique as conexões.");
    while (1); // Para o programa se der erro
  } else {
    Serial.println("Sensor DHT inicializado com sucesso.");
  }

  // Inicializando o barramento I2C
  Wire.begin(21, 22); // Inicializando os pinos personalizados para SCL, SDA
  delay(100);  // Espera para garantir que o barramento I2C tenha tempo de iniciar

  // Inicializando o sensor BH1750
  if (!bh1750.begin(BH1750::CONTINUOUS_LOW_RES_MODE)) {
    Serial.println("Falha ao inicializar o BH1750. Verifique as conexões.");
    while (1); // Para o programa se der erro
  } else {
    Serial.println("BH1750 inicializado com sucesso.");
  }

  // Verificação do sensor de umidade do solo
  int soilMoisture = analogRead(SOILMOISTUREPIN);
  if (soilMoisture < 0 || soilMoisture > 1023) {
    Serial.println("Falha ao ler sensor de umidade do solo. Verifique as conexões.");
    while (1); // Para o programa se der erro
  } else {
    Serial.println("Sensor de umidade do solo lido com sucesso.");
  }
}


// Função para ler a temperatura do sensor DHT
float getTemperature() {
  return dht.readTemperature();
}

// Função para ler a umidade do sensor DHT
float getHumidity() {
  return dht.readHumidity();
}

// Função para ler a umidade do solo
int getSoilMoisture() {
  return analogRead(SOILMOISTUREPIN);
}

// Função para ler a luminosidade do sensor BH1750
float getLightLevel() {
  return bh1750.readLightLevel();
}

// Função para controlar as ventoinhas com base na temperatura
void controlFan(float temperature) {
  if (temperature > LIMIT_TEMPERATURE) {
    digitalWrite(RELE_FAN, LOW); // Liga as ventoinhas
    Serial.println("Ventoinhas ligadas");
  } else {
    digitalWrite(RELE_FAN, HIGH);
    Serial.println("Ventoinhas desligadas");
  }
}

// Função para controlar os LEDs com base na luminosidade
void controlLight(float light) {
  if (light < LIMIT_LIGHT) {
    digitalWrite(RELE_LIGHT, LOW); // Liga os LEDs
    Serial.println("LEDs ligados");
  } else {
    digitalWrite(RELE_LIGHT, HIGH);
  }
}

// Função para publicar os dados dos sensores no MQTT
void publishSensorData(float temperature, float humidity, int soilMoisture, float light) {
  // Publica os dados nos tópicos respectivos do MQTT
  char temperatureStr[8];
  dtostrf(temperature, 1, 2, temperatureStr);  // Converte para string
  client.publish("sensor/temperature", temperatureStr);

  char humidityStr[8];
  dtostrf(humidity, 1, 2, humidityStr);  // Converte para string
  client.publish("sensor/humidity", humidityStr);

  char soilMoistureStr[8];
  itoa(soilMoisture, soilMoistureStr, 10);  // Converte para string
  client.publish("sensor/soilMoisture", soilMoistureStr);

  char lightStr[8];
  dtostrf(light, 1, 2, lightStr);  // Converte para string
  client.publish("sensor/light", lightStr);
}

// Função para exibir os dados no monitor serial
void displaySensorData(float temperature, float humidity, int soilMoisture, float light) {

  int percentSoilMoisture = map(soilMoisture, 1490, 1100, 0, 100);
  percentSoilMoisture = constrain(percentSoilMoisture, 0, 100);

  int percentLight = map(light, 0, 190, 0, 100);
  percentLight = constrain(percentLight, 0, 100);

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

void setup() {
  // Inicialização do monitor serial
  Serial.begin(115200); 

  // Inicialização dos sensores e dispositivos
  initPins();
  initWiFi();
  initSensors();
  
  // Inicializando MQTT
  client.setServer(mqtt_server, 1883); // Configura o broker MQTT
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT(); // Reconecta caso a conexão MQTT seja perdida
  }
  client.loop(); // Mantém a comunicação com o broker MQTT

  // Leitura dos sensores
  float temperature = getTemperature();
  float humidity = getHumidity();
  int soilMoisture = getSoilMoisture();
  float light = getLightLevel();

  // Ativa os dispositivos com base nas leituras
  controlFan(temperature);
  controlLight(light);

  // Publica os dados no MQTT
  publishSensorData(temperature, humidity, soilMoisture, light);

  // Exibe os dados no monitor serial
  displaySensorData(temperature, humidity, soilMoisture, light);

  delay(5000); // Espera 5 segundos antes de fazer a próxima leitura
}
