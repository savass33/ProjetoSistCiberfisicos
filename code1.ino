#include <WiFi.h>
#include <esp_now.h>
#include <DHT.h>
#include <PubSubClient.h>

// ========= CONFIGURAÇÕES ========= //
#define ID "ESPSAVASUNIFOR"
#define DHTPIN 15
#define DHTTYPE DHT11
#define LED_PIN 2
#define INTERVALO_ENVIO_MS 5000
#define INTERVALO_MQTT_MS 10000

const char *WIFI_SSID = "iPhone de Savas";
const char *WIFI_PASS = "savasneto";
const char *MQTT_BROKER = "io.adafruit.com";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "Savas33";
const char *MQTT_PASS = "chave-mqtt";

// ========= ESTRUTURAS ========= //
typedef struct {
  char id[30];
  int temperatura;
  int umidade;
} DadosESP;

// ========= VARIÁVEIS ========= //
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
DadosESP dadosRecebidos;
unsigned long ultimoEnvioESPNow = 0;
unsigned long ultimoEnvioMQTT = 0;
bool modoWiFiAtivo = false;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
DHT dht(DHTPIN, DHTTYPE);

// ========= FUNÇÕES AUXILIARES ========= //
void piscarLED(int tempo = 200) {
  digitalWrite(LED_PIN, HIGH);
  delay(tempo);
  digitalWrite(LED_PIN, LOW);
}

void debugSerial(const DadosESP &dados) {
  Serial.println("📥 Dados Recebidos:");
  Serial.println("ID: " + String(dados.id));
  Serial.println("🌡 Temperatura: " + String(dados.temperatura));
  Serial.println("💧 Umidade: " + String(dados.umidade));
  Serial.println("------------------");
}

// ========= CONEXÃO WI-FI ========= //
void conectarWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("📶 Conectando ao Wi-Fi...");
  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 10) {
    delay(500);
    Serial.print(".");
    tentativas++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Wi-Fi conectado! IP: " + WiFi.localIP().toString());
    modoWiFiAtivo = true;
  } else {
    Serial.println("\n❌ Falha ao conectar Wi-Fi");
    modoWiFiAtivo = false;
  }
}

// ========= CONEXÃO MQTT ========= //
void conectarMQTT() {
  if (!modoWiFiAtivo) return;

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);

  if (!mqttClient.connected()) {
    Serial.print("🔌 Conectando ao MQTT...");
    if (mqttClient.connect("ESP32_" ID, MQTT_USER, MQTT_PASS)) {
      Serial.println("✅ MQTT conectado!");
    } else {
      Serial.println("❌ Falha no MQTT (" + String(mqttClient.state()) + ")");
    }
  }
}

// ========= GESTÃO DE MODO ========= //
void reconfigurarESPNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Falha ao reiniciar ESP-NOW");
    return;
  }

  esp_now_unregister_recv_cb();
  esp_now_unregister_send_cb();

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  esp_now_del_peer(broadcastAddress); // evita erro de peer duplicado

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("⚠ Falha ao adicionar peer ESP-NOW");
  }
}

void alternarModo(bool usarWiFi) {
  if (usarWiFi) {
    esp_now_deinit();
    WiFi.disconnect(true, true);
    delay(100);
    conectarWiFi();
    conectarMQTT();
  } else {
    if (mqttClient.connected()) mqttClient.disconnect();
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.mode(WIFI_STA);
    reconfigurarESPNow();
  }
}

// ========= ENVIO ========= //
void enviarDadosESPNow(const DadosESP &dados) {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&dados, sizeof(dados));
  if (result == ESP_OK) {
    piscarLED();
  } else {
    Serial.println("❌ Erro ao enviar ESP-NOW: " + String(result));
  }
}

void publicarDadosMQTT(const DadosESP &dados) {
  alternarModo(true);

  if (modoWiFiAtivo && mqttClient.connected()) {
    String baseFeed = String(MQTT_USER) + "/feeds/g-projetofinal.";

    mqttClient.publish((baseFeed + "idequipe").c_str(), dados.id);
    mqttClient.publish((baseFeed + "temperature").c_str(), String(dados.temperatura).c_str());
    mqttClient.publish((baseFeed + "umidade").c_str(), String(dados.umidade).c_str());

    mqttClient.loop();
    delay(100);
    Serial.println("📤 Dados publicados no Adafruit IO!");
  } else {
    Serial.println("⚠ MQTT não disponível");
  }

  alternarModo(false);
}

// ========= CALLBACKS ESP-NOW ========= //
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *dadosRecebidosBytes, int len) {
  memcpy(&dadosRecebidos, dadosRecebidosBytes, sizeof(dadosRecebidos));
  debugSerial(dadosRecebidos);

  if (String(dadosRecebidos.id) != ID) {
    enviarDadosESPNow(dadosRecebidos);
  }
}

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.printf("📤 Pacote ESP-NOW enviado: %s\n",
                status == ESP_NOW_SEND_SUCCESS ? "✅ Sucesso" : "❌ Falha");
}

// ========= SETUP ========= //
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  dht.begin();

  WiFi.mode(WIFI_STA);
  reconfigurarESPNow();

  Serial.println("🚀 Dispositivo inicializado!");
}

// ========= LOOP ========= //
void loop() {
  unsigned long agora = millis();

  if (agora - ultimoEnvioESPNow >= INTERVALO_ENVIO_MS) {
    DadosESP dadosLocais;
    strlcpy(dadosLocais.id, ID, sizeof(dadosLocais.id));
    dadosLocais.temperatura = (int)dht.readTemperature();
    dadosLocais.umidade = (int)dht.readHumidity();

    enviarDadosESPNow(dadosLocais);
    ultimoEnvioESPNow = agora;
  }

  if (agora - ultimoEnvioMQTT >= INTERVALO_MQTT_MS) {
    if (dadosRecebidos.id[0] != '\0') {
      publicarDadosMQTT(dadosRecebidos);
    }
    ultimoEnvioMQTT = agora;
  }

  delay(100);
}