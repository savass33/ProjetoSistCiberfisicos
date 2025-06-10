#include <WiFi.h>
#include <esp_now.h>
#include <DHT.h>
#include <PubSubClient.h>

// ========= CONFIGURAÇÕES ========= //
#define ID "JOAOPEDRO"           // Identificação do dispositivo
#define DHTPIN 15                // Pino do sensor DHT (GPIO15)
#define DHTTYPE DHT11            // Tipo do sensor (DHT11)
#define LED_PIN 2                // Pino do LED (GPIO2)
#define INTERVALO_ENVIO_MS 5000  // Intervalo de envio ESP-NOW (5s)
#define INTERVALO_MQTT_MS 5000   // Intervalo de envio MQTT (10s)

// 🔑 Credenciais Wi-Fi e MQTT
const char *WIFI_SSID = "S24 Ultra de Joao Pedro";
const char *WIFI_PASS = "joaopedro0085";
const char *MQTT_BROKER = "io.adafruit.com";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "JPMendes";
const char *MQTT_PASS = "chave-mqtt";

// ========= ESTRUTURAS DE DADOS ========= //
typedef struct {
  char id[30];      // Identificação do remetente
  int temperatura;  // Dado 01: Temperatura (int)
  int umidade;      // Dado 02: Umidade (int)
} DadosESP;

// ========= VARIÁVEIS GLOBAIS ========= //
uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };  // Broadcast ESP-NOW
DadosESP dadosRecebidos;                                              // Armazena dados recebidos
DadosESP ultimoDadoRecebido;                                          // Armazena o último dado válido recebido
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

  Serial.print("📶 Conectando ao Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

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

  if (!mqttClient.connected()) {
    Serial.print("🔌 Conectando ao MQTT...");
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);

    if (mqttClient.connect("ESP32_" ID, MQTT_USER, MQTT_PASS)) {
      Serial.println("✅ MQTT conectado!");
    } else {
      Serial.println("❌ Falha no MQTT (" + String(mqttClient.state()) + ")");
    }
  }
}

// ========= GERENCIAMENTO DE MODOS ========= //
void alternarModo(bool usarWiFi) {
  if (usarWiFi) {
    // 🔄 Ativa Wi-Fi e desativa ESP-NOW temporariamente
    esp_now_deinit();
    WiFi.mode(WIFI_STA);
    conectarWiFi();
    conectarMQTT();
  } else {
    // 🔄 Volta para ESP-NOW e desliga Wi-Fi
    if (mqttClient.connected()) mqttClient.disconnect();
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) Serial.println("⚠ Erro ao reiniciar ESP-NOW");
    // Reconfigura peers ESP-NOW
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }
}

// ========= ENVIO DE DADOS ========= //
void enviarDadosESPNow(const DadosESP &dados) {
  esp_now_send(broadcastAddress, (uint8_t *)&dados, sizeof(dados));
  piscarLED();
}

void publicarDadosMQTT(const DadosESP &dados) {
  alternarModo(true);  // Ativa Wi-Fi temporariamente

  if (modoWiFiAtivo && mqttClient.connected()) {
    String baseFeed = String(MQTT_USER) + "/feeds/espnow.";

    mqttClient.publish((baseFeed + "equipe").c_str(), dados.id);
    mqttClient.publish((baseFeed + "temperature").c_str(), String(dados.temperatura).c_str());
    mqttClient.publish((baseFeed + "umidade").c_str(), String(dados.umidade).c_str());

    mqttClient.loop();
    delay(100);  // Pequeno delay para garantir envio
    Serial.println("📤 Dados publicados no Adafruit IO!");
  } else {
    Serial.println("⚠ MQTT não disponível para publicação");
  }

  alternarModo(false);  // Volta para ESP-NOW
}

// ========= CALLBACKS ESP-NOW ========= //
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *dadosRecebidosBytes, int len) {
  DadosESP recebidoTemp;
  memcpy(&recebidoTemp, dadosRecebidosBytes, sizeof(recebidoTemp));

  // Verifica se os dados são de outro dispositivo
  if (String(recebidoTemp.id) != ID) {
    debugSerial(recebidoTemp);

    // Armazena os dados recebidos para envio MQTT
    memcpy(&ultimoDadoRecebido, &recebidoTemp, sizeof(recebidoTemp));

    // Repassa os dados
    enviarDadosESPNow(recebidoTemp);
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

  // Inicia em modo ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Falha ao iniciar ESP-NOW");
    while (1)
      ;
  }

  // Configura peer de broadcast
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Falha ao adicionar peer");
    while (1)
      ;
  }

  // Registra callbacks
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  Serial.println("🚀 Dispositivo inicializado!");
}

// ========= LOOP PRINCIPAL ========= //
void loop() {
  unsigned long agora = millis();

  // Envia dados locais via ESP-NOW periodicamente
  if (agora - ultimoEnvioESPNow >= INTERVALO_ENVIO_MS) {
    DadosESP dadosLocais;
    strlcpy(dadosLocais.id, ID, sizeof(dadosLocais.id));
    float temp = dht.readTemperature();
    float umi = dht.readHumidity();

    dadosLocais.temperatura = isnan(temp) ? -1 : (int)temp;
    dadosLocais.umidade = isnan(umi) ? -1 : (int)umi;


    enviarDadosESPNow(dadosLocais);
    ultimoEnvioESPNow = agora;
  }

  // Publica no MQTT em intervalos maiores
  if (agora - ultimoEnvioMQTT >= INTERVALO_MQTT_MS) {
    // Publica tanto os dados locais quanto os recebidos
    DadosESP dadosParaPublicar;

    // Preferência para dados recebidos de outros ESPs
    if (ultimoDadoRecebido.id[0] != '\0') {
      memcpy(&dadosParaPublicar, &ultimoDadoRecebido, sizeof(dadosParaPublicar));
    } else {
      // Se não houver dados recebidos, publica os dados locais
      strlcpy(dadosParaPublicar.id, ID, sizeof(dadosParaPublicar.id));
      dadosParaPublicar.temperatura = (int)dht.readTemperature();
      dadosParaPublicar.umidade = (int)dht.readHumidity();
    }

    publicarDadosMQTT(dadosParaPublicar);
    ultimoEnvioMQTT = agora;
  }

  delay(100);
}