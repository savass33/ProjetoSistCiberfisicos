# 🌐 Projeto de Comunicação ESP-NOW + MQTT com ESP32

Este projeto demonstra a comunicação entre dois dispositivos ESP32 utilizando **ESP-NOW** para troca de dados local e **MQTT (Adafruit IO)** para publicação em nuvem. Os dispositivos também fazem leitura de temperatura e umidade com o sensor **DHT11**.

---

## 📡 Tecnologias Utilizadas

- ESP32
- ESP-NOW (comunicação local)
- Wi-Fi (modo STA)
- MQTT (com Adafruit IO)
- Sensor DHT11
- Plataforma Arduino

---

## ⚙️ Funcionamento

### Comunicação ESP-NOW
- Cada ESP32 envia periodicamente dados do sensor para outros dispositivos ESP32 via **ESP-NOW**.
- Os dados transmitidos incluem:
  - ID do dispositivo
  - Temperatura (°C)
  - Umidade (%)
- Os dispositivos retransmitem os dados recebidos (exceto os próprios) para ampliar o alcance da rede.

### Publicação MQTT (Adafruit IO)
- A cada intervalo definido, os dados mais recentes são enviados para a plataforma Adafruit IO usando **MQTT**.
- O ESP32 alterna dinamicamente entre os modos ESP-NOW e Wi-Fi para realizar a publicação sem interromper a coleta de dados.

---

## 📑 Estrutura dos Dados

```cpp
typedef struct {
  char id[30];      // Identificação do dispositivo
  int temperatura;  // Temperatura em °C
  int umidade;      // Umidade relativa em %
} DadosESP;
