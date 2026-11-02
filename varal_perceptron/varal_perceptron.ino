/*
 * Varal automático com Perceptron embarcado - ESP32
 * ---------------------------------------------------
 * Sensor DHT11 -> Perceptron (2 entradas) -> Servo -> POST HTTP (JSON) para PHP
 *
 * Bibliotecas (Gerenciador de Bibliotecas):
 *  - DHT sensor library (Adafruit) + Adafruit Unified Sensor
 *  - ESP32Servo
 *  - ArduinoJson (v7.x)
 *  WiFi.h e HTTPClient.h já vêm com o core do ESP32.
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <ESP32Servo.h>

// ===================== CONFIGURAÇÕES =====================
const char* WIFI_SSID     = "Becks";
const char* WIFI_PASSWORD = "12345678";
const char* SERVER_URL    = "http://192.168.0.100/varal/salvar.php";  // seu script PHP

#define DHT_PIN    4
#define DHT_TYPE   DHT11
#define SERVO_PIN  13

//angulos do servo motor
const int ANGULO_ESTENDIDO = 0;     // varal fora
const int ANGULO_RECOLHIDO = 180;   // varal no abrigo

const unsigned long INTERVALO_LEITURA_MS = 10000;  // lê e decide a cada 10 s
const unsigned long INTERVALO_ENVIO_MS   = 30000;  // envia ao servidor a cada 30 s

// ===================== PERCEPTRON =====================
// Entradas normalizadas: x1 = umidade/100, x2 = temperatura/40
// z = w1*x1 + w2*x2 + bias  ->  saída = 1 (chuva) se z > 0
float W_UMIDADE = 10.0f;   // umidade alta aumenta a chance de chuva
float W_TEMP    = -3.0f;   // temperatura alta reduz a chance de chuva
float BIAS      = -6.0f;   // limiar de decisão

// ===================== OBJETOS / ESTADO =====================
DHT dht(DHT_PIN, DHT_TYPE);
Servo servoVaral;

bool varalRecolhido = false;          // estado atual do servo
unsigned long ultimaLeitura = 0;
unsigned long ultimoEnvio   = 0;

float ultimaTemp = NAN, ultimaUmid = NAN;
float ultimaProb = 0.0f;
int   ultimaPredicao = 0;

// ===================== 1. LEITURA DO SENSOR =====================
bool lerSensor(float &temperatura, float &umidade) {
  umidade     = dht.readHumidity();
  temperatura = dht.readTemperature();
  if (isnan(umidade) || isnan(temperatura)) {
    Serial.println("[DHT] Falha na leitura do sensor.");
    return false;
  }
  return true;
}

// ===================== 2. PERCEPTRON =====================
float sigmoide(float z) {
  return 1.0f / (1.0f + expf(-z));
}

// Retorna 1 (chovendo/úmido -> recolher) ou 0 (seco -> estender).
// 'probabilidade' recebe a saída sigmoide (0..1), útil para log.
int preverPerceptron(float temperatura, float umidade, float &probabilidade) {
  float x1 = umidade / 100.0f;
  float x2 = temperatura / 40.0f;

  float z = (W_UMIDADE * x1) + (W_TEMP * x2) + BIAS;  // soma ponderada + viés

  probabilidade = sigmoide(z);          // sigmoide só informativa
  return (z > 0.0f) ? 1 : 0;            // função degrau (decisão)
}

// (Opcional) Treinamento simples pela regra do Perceptron.
// Chame uma vez no setup() se quiser ajustar os pesos com seus próprios dados.
// Cada amostra: {umidade %, temperatura °C, rótulo (1 = chuva, 0 = seco)}
void treinarPerceptron(const float dados[][3], int n, int epocas, float taxa) {
  for (int e = 0; e < epocas; e++) {
    for (int i = 0; i < n; i++) {
      float x1 = dados[i][0] / 100.0f;
      float x2 = dados[i][1] / 40.0f;
      int alvo = (int)dados[i][2];

      float z = W_UMIDADE * x1 + W_TEMP * x2 + BIAS;
      int saida = (z > 0.0f) ? 1 : 0;
      int erro = alvo - saida;

      W_UMIDADE += taxa * erro * x1;
      W_TEMP    += taxa * erro * x2;
      BIAS      += taxa * erro;
    }
  }
  Serial.printf("[TREINO] W_UMIDADE=%.3f W_TEMP=%.3f BIAS=%.3f\n", W_UMIDADE, W_TEMP, BIAS);
}

// ===================== 3. CONTROLE DO SERVO =====================
// Só move o servo se o estado mudar (evita vibração e desgaste).
void controlarVaral(int predicao) {
  bool deveRecolher = (predicao == 1);
  if (deveRecolher == varalRecolhido) return;

  servoVaral.write(deveRecolher ? ANGULO_RECOLHIDO : ANGULO_ESTENDIDO);
  varalRecolhido = deveRecolher;
  Serial.println(deveRecolher ? "[SERVO] Varal RECOLHIDO (180°)" : "[SERVO] Varal ESTENDIDO (0°)");
}

// ===================== 4. ENVIO HTTP =====================
bool enviarDados(float temperatura, float umidade, int predicao) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[HTTP] Sem Wi-Fi, reconectando...");
    WiFi.reconnect();
    return false;
  }

  JsonDocument doc;
  doc["temperatura"] = temperatura;
  doc["umidade"]     = umidade;
  doc["predicao"]    = (predicao == 1) ? "Chovendo/Recolhido" : "Seco/Estendido";

  String payload;
  serializeJson(doc, payload);

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  int codigo = http.POST(payload);

  if (codigo > 0) {
    Serial.printf("[HTTP] Código %d | Resposta: %s\n", codigo, http.getString().c_str());
  } else {
    Serial.printf("[HTTP] Erro: %s\n", http.errorToString(codigo).c_str());
  }
  http.end();
  return (codigo >= 200 && codigo < 300);
}

// ===================== WI-FI =====================
void conectarWiFi() {
  Serial.printf("[WIFI] Conectando a %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long inicio = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicio < 20000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WIFI] Conectado! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WIFI] Falha ao conectar (tentará novamente no envio).");
  }
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  dht.begin();

  servoVaral.setPeriodHertz(50);
  servoVaral.attach(SERVO_PIN, 500, 2400);
  servoVaral.write(ANGULO_ESTENDIDO);  // começa estendido

  conectarWiFi();

  /* Exemplo de treino opcional:
  const float dados[][3] = {
    {45, 30, 0}, {55, 28, 0}, {65, 26, 0}, {70, 24, 0},
    {85, 24, 1}, {90, 22, 1}, {95, 20, 1}, {88, 26, 1}
  };
  treinarPerceptron(dados, 8, 200, 0.1f);
  */
  ultimaLeitura = millis() - INTERVALO_LEITURA_MS;
}

void loop() {
  unsigned long agora = millis();

  // Leitura + predição + servo
  if (agora - ultimaLeitura >= INTERVALO_LEITURA_MS) {
    ultimaLeitura = agora;

    float t, u;
    if (lerSensor(t, u)) {
      ultimaTemp = t;
      ultimaUmid = u;
      ultimaPredicao = preverPerceptron(t, u, ultimaProb);

      Serial.printf("[IA] T=%.1f°C U=%.1f%% | P(chuva)=%.2f -> %s\n",
                    t, u, ultimaProb, ultimaPredicao ? "RECOLHER" : "ESTENDER");

      controlarVaral(ultimaPredicao);
    }
  }

  // Envio periódico ao servidor PHP
  if (!isnan(ultimaTemp) && agora - ultimoEnvio >= INTERVALO_ENVIO_MS) {
    ultimoEnvio = agora;
    enviarDados(ultimaTemp, ultimaUmid, ultimaPredicao);
  }
}
