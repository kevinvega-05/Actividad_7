/*
 * =====================================================================
 *  Enjambre de 3 carritos con algoritmo de hormigas (ACO) en ESP32
 *  UMNG - Mecatrónica
 * =====================================================================
 *  - Cada ESP32 corre su propia colonia de hormigas sobre el laberinto
 *    (maze.h) buscando la ruta más corta de A a la meta.
 *  - El carrito 1 crea la red WiFi (modo AP) y reenvía los mensajes
 *    entre los carritos 2, 3 y el PC con la simulación PyBullet.
 *  - Cada vez que un carrito termina una iteración, "deposita" feromona
 *    en su mejor ruta y comparte ese depósito por UDP: los demás lo suman
 *    a su propia matriz de feromonas (estigmergia distribuida).
 *  - Cada carrito recorre virtualmente su mejor ruta celda por celda y
 *    reporta su posición; el gemelo digital en PyBullet la replica.
 *
 *  Protocolo UDP (texto, campos separados por coma), puerto UDP_PORT:
 *    HELLO,<id>                               registro ante el AP (0 = PC)
 *    DEP,<id>,<iter>,<pasos>,<c0>-<c1>-...    depósito de feromona en una ruta
 *    ST,<id>,<iter>,<celda>,<mejor>,<conv>,<origen>   estado del carrito
 *    PH,<id>,<hex>                            mapa de feromonas (solo AP)
 *  Celdas indexadas como fila*COLS + columna.
 *
 *  Placa: "ESP32 Dev Module" (core arduino-esp32 2.x o 3.x)
 * =====================================================================
 */
#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"
#include "maze.h"

#define N_CELDAS (ROWS * COLS)
#define START_CELL (START_R * COLS + START_C)
#define GOAL_CELL  (GOAL_R * COLS + GOAL_C)
#define ES_AP (CAR_ID == 1)

// Direcciones: N, E, S, O
const int8_t DR[4] = {-1, 0, 1, 0};
const int8_t DC[4] = {0, 1, 0, -1};

// --------------------------- Estado ACO -------------------------------
float tau[N_CELDAS][4];              // feromona en cada arista dirigida
uint16_t mejorRuta[N_CELDAS];        // mejor ruta conocida (celdas)
int mejorLen = 0;                    // nº de celdas (0 = aún no hay)
int mejorOrigen = 0;                 // qué carrito la encontró
uint32_t iteracion = 0;
int itersSinMejora = 0;
bool convergido = false;

// --------------------------- Movimiento virtual -----------------------
uint16_t rutaViaje[N_CELDAS];        // copia de la ruta que se recorre
int lenViaje = 0;
int posViaje = 0;
int celdaActual = START_CELL;
int pausaMeta = 0;

// --------------------------- Red --------------------------------------
WiFiUDP udp;
const IPAddress AP_IP(AP_IP_BYTES);
char bufRx[1500];
char bufTx[1500];

#define MAX_PEERS 6
IPAddress peerIp[MAX_PEERS];
uint16_t peerPort[MAX_PEERS];
uint32_t peerVisto[MAX_PEERS];
int nPeers = 0;

uint32_t tIter = 0, tMov = 0, tHello = 0, tPh = 0, tLed = 0;

// Prototipos (para que compile también fuera del IDE de Arduino, p. ej. PlatformIO)
void enviarDeposito(const uint16_t *ruta, int len);
void enviarEstado();
void enviar(const char *msg);

// =====================================================================
//  Utilidades del laberinto
// =====================================================================
bool esLibre(int r, int c) {
  return r >= 0 && r < ROWS && c >= 0 && c < COLS && MAZE[r][c] == 0;
}

int vecino(int celda, int d) {
  int r = celda / COLS + DR[d];
  int c = celda % COLS + DC[d];
  return esLibre(r, c) ? r * COLS + c : -1;
}

int direccionEntre(int a, int b) {
  for (int d = 0; d < 4; d++)
    if (vecino(a, d) == b) return d;
  return -1;
}

float heuristica(int celda) {   // eta: más alta cuanto más cerca de la meta
  int r = celda / COLS, c = celda % COLS;
  return 1.0f / (1.0f + abs(r - GOAL_R) + abs(c - GOAL_C));
}

bool rutaValida(const uint16_t *ruta, int len) {
  if (len < 2 || ruta[0] != START_CELL || ruta[len - 1] != GOAL_CELL) return false;
  for (int i = 0; i < len - 1; i++)
    if (ruta[i] >= N_CELDAS || direccionEntre(ruta[i], ruta[i + 1]) < 0) return false;
  return true;
}

// =====================================================================
//  ACO
// =====================================================================
void iniciarFeromonas() {
  for (int i = 0; i < N_CELDAS; i++)
    for (int d = 0; d < 4; d++) tau[i][d] = TAU_INICIAL;
}

float aleatorio01() { return (float)(esp_random() & 0xFFFFFF) / 16777216.0f; }

// Una hormiga camina de A a la meta. Usa lista tabú y retrocede si
// queda atrapada en un callejón sin salida. Devuelve nº de celdas (0 = falla).
int caminarHormiga(uint16_t *ruta) {
  static bool visitado[N_CELDAS];
  memset(visitado, 0, sizeof(visitado));
  int len = 0;
  int actual = START_CELL;
  ruta[len++] = actual;
  visitado[actual] = true;

  while (actual != GOAL_CELL) {
    float peso[4], suma = 0;
    for (int d = 0; d < 4; d++) {
      int n = vecino(actual, d);
      if (n < 0 || visitado[n]) { peso[d] = 0; continue; }
      peso[d] = powf(tau[actual][d], ALPHA) * powf(heuristica(n), BETA);
      suma += peso[d];
    }
    if (suma <= 0) {                 // callejón: retrocede
      len--;
      if (len <= 0) return 0;
      actual = ruta[len - 1];
      continue;
    }
    float x = aleatorio01() * suma;  // ruleta
    int elegido = -1;
    for (int d = 0; d < 4; d++) {
      if (peso[d] <= 0) continue;
      elegido = d;
      x -= peso[d];
      if (x <= 0) break;
    }
    actual = vecino(actual, elegido);
    visitado[actual] = true;
    ruta[len++] = actual;
    if (len >= N_CELDAS) return 0;
  }
  return len;
}

void evaporar() {
  for (int i = 0; i < N_CELDAS; i++)
    for (int d = 0; d < 4; d++) {
      tau[i][d] *= (1.0f - RHO);
      if (tau[i][d] < TAU_MIN) tau[i][d] = TAU_MIN;
    }
}

void depositar(const uint16_t *ruta, int len, float q) {
  float cantidad = q / (float)(len - 1);
  for (int i = 0; i < len - 1; i++) {
    int d = direccionEntre(ruta[i], ruta[i + 1]);
    if (d < 0) continue;
    tau[ruta[i]][d] += cantidad;
    if (tau[ruta[i]][d] > TAU_MAX) tau[ruta[i]][d] = TAU_MAX;
  }
}

// Si la ruta es mejor que la conocida, la adopta. Devuelve true si mejoró.
bool considerarRuta(const uint16_t *ruta, int len, int origen) {
  if (mejorLen != 0 && len >= mejorLen) return false;
  memcpy(mejorRuta, ruta, len * sizeof(uint16_t));
  mejorLen = len;
  mejorOrigen = origen;
  itersSinMejora = 0;
  convergido = false;
  Serial.printf("[ACO] Nueva mejor ruta: %d pasos (encontrada por carrito %d)\n",
                len - 1, origen);
  return true;
}

void iteracionACO() {
  static uint16_t ruta[N_CELDAS];
  static uint16_t mejorIter[N_CELDAS];
  int lenMejorIter = 0;

  evaporar();
  for (int a = 0; a < N_ANTS; a++) {
    int len = caminarHormiga(ruta);
    if (len > 0) depositar(ruta, len, Q_HORMIGA);   // rastro local de cada hormiga
    if (len > 0 && (lenMejorIter == 0 || len < lenMejorIter)) {
      memcpy(mejorIter, ruta, len * sizeof(uint16_t));
      lenMejorIter = len;
    }
  }

  iteracion++;
  if (lenMejorIter == 0) return;

  depositar(mejorIter, lenMejorIter, Q_DEPOSITO);  // refuerzo de la mejor (se comparte)
  enviarDeposito(mejorIter, lenMejorIter);

  if (!considerarRuta(mejorIter, lenMejorIter, CAR_ID)) {
    if (++itersSinMejora >= ITERS_CONVERGE && !convergido) {
      convergido = true;
      Serial.printf("[ACO] Convergido en %d pasos (iter %lu)\n",
                    mejorLen - 1, (unsigned long)iteracion);
    }
  }
}

// =====================================================================
//  Red
// =====================================================================
void enviarA(IPAddress ip, uint16_t puerto, const char *msg) {
  udp.beginPacket(ip, puerto);
  udp.write((const uint8_t *)msg, strlen(msg));
  udp.endPacket();
}

// Carrito 1: manda a todos los registrados (menos al que lo originó).
// Carritos 2 y 3: mandan al AP, que reenvía.
void difundir(const char *msg, IPAddress exIp, uint16_t exPort) {
#if ES_AP
  for (int i = 0; i < nPeers; i++) {
    if (peerIp[i] == exIp && peerPort[i] == exPort) continue;
    enviarA(peerIp[i], peerPort[i], msg);
  }
#else
  enviarA(AP_IP, UDP_PORT, msg);
#endif
}

void enviar(const char *msg) { difundir(msg, IPAddress(0, 0, 0, 0), 0); }

void registrarPeer(IPAddress ip, uint16_t puerto) {
  for (int i = 0; i < nPeers; i++)
    if (peerIp[i] == ip && peerPort[i] == puerto) { peerVisto[i] = millis(); return; }
  if (nPeers >= MAX_PEERS) return;
  peerIp[nPeers] = ip;
  peerPort[nPeers] = puerto;
  peerVisto[nPeers] = millis();
  nPeers++;
  Serial.printf("[RED] Nodo registrado: %s:%u (total %d)\n",
                ip.toString().c_str(), puerto, nPeers);
}

void limpiarPeers() {
  for (int i = 0; i < nPeers; i++) {
    if (millis() - peerVisto[i] > TIMEOUT_PEER_MS) {
      Serial.printf("[RED] Nodo %s sin respuesta, se elimina\n",
                    peerIp[i].toString().c_str());
      peerIp[i] = peerIp[nPeers - 1];
      peerPort[i] = peerPort[nPeers - 1];
      peerVisto[i] = peerVisto[nPeers - 1];
      nPeers--;
      i--;
    }
  }
}

void enviarDeposito(const uint16_t *ruta, int len) {
  int n = snprintf(bufTx, sizeof(bufTx), "DEP,%d,%lu,%d,", CAR_ID,
                   (unsigned long)iteracion, len - 1);
  for (int i = 0; i < len && n < (int)sizeof(bufTx) - 6; i++)
    n += snprintf(bufTx + n, sizeof(bufTx) - n, i ? "-%u" : "%u", ruta[i]);
  enviar(bufTx);
}

void enviarEstado() {
  snprintf(bufTx, sizeof(bufTx), "ST,%d,%lu,%d,%d,%d,%d", CAR_ID,
           (unsigned long)iteracion, celdaActual,
           mejorLen ? mejorLen - 1 : -1, convergido ? 1 : 0, mejorOrigen);
  enviar(bufTx);
}

void enviarFeromonas() {           // 1 byte (2 hex) por celda: máximo de sus 4 aristas
  int n = snprintf(bufTx, sizeof(bufTx), "PH,%d,", CAR_ID);
  for (int i = 0; i < N_CELDAS; i++) {
    float m = 0;
    if (MAZE[i / COLS][i % COLS] == 0)
      for (int d = 0; d < 4; d++) if (tau[i][d] > m) m = tau[i][d];
    float x = (m - TAU_MIN) / (TAU_MAX - TAU_MIN);
    if (x < 0) x = 0;
    if (x > 1) x = 1;
    n += snprintf(bufTx + n, sizeof(bufTx) - n, "%02X", (int)(x * 255));
  }
  enviar(bufTx);
}

// Procesa un DEP de otro carrito: suma su feromona y adopta su ruta si es mejor
void procesarDeposito(char *resto) {
  // resto = "<id>,<iter>,<pasos>,<celdas>"
  char *ctx;
  char *sId = strtok_r(resto, ",", &ctx);
  strtok_r(NULL, ",", &ctx);       // iter (no se usa)
  strtok_r(NULL, ",", &ctx);       // pasos (se recalcula)
  char *sRuta = strtok_r(NULL, ",", &ctx);
  if (!sId || !sRuta) return;
  int origen = atoi(sId);
  if (origen == CAR_ID) return;

  static uint16_t ruta[N_CELDAS];
  int len = 0;
  char *ctx2;
  for (char *t = strtok_r(sRuta, "-", &ctx2); t && len < N_CELDAS;
       t = strtok_r(NULL, "-", &ctx2))
    ruta[len++] = (uint16_t)atoi(t);

  if (!rutaValida(ruta, len)) return;
  depositar(ruta, len, Q_DEPOSITO);
  considerarRuta(ruta, len, origen);
  digitalWrite(LED_PIN, HIGH);     // parpadeo: llegó feromona
  tLed = millis();
}

void recibir() {
  int tam;
  while ((tam = udp.parsePacket()) > 0) {
    int n = udp.read(bufRx, sizeof(bufRx) - 1);
    if (n <= 0) continue;
    bufRx[n] = 0;
#if ES_AP
    IPAddress ip = udp.remoteIP();
    uint16_t puerto = udp.remotePort();
    registrarPeer(ip, puerto);
    if (strncmp(bufRx, "HELLO", 5) == 0) continue;   // el HELLO no se reenvía
    static char copia[1500];
    strcpy(copia, bufRx);
    difundir(copia, ip, puerto);                     // relevo a los demás
#endif
    if (strncmp(bufRx, "DEP,", 4) == 0) procesarDeposito(bufRx + 4);
    // ST y PH de otros nodos no los necesita el carrito (solo el PC)
  }
}

// =====================================================================
//  Movimiento virtual del carrito sobre su mejor ruta
// =====================================================================
void avanzar() {
  if (lenViaje == 0 || posViaje >= lenViaje - 1) {
    // En A esperando ruta, o llegó a la meta: pausa y vuelve a A con la mejor actual
    if (lenViaje != 0 && pausaMeta++ < 3) { enviarEstado(); return; }
    pausaMeta = 0;
    if (mejorLen > 0) {
      memcpy(rutaViaje, mejorRuta, mejorLen * sizeof(uint16_t));
      lenViaje = mejorLen;
    }
    posViaje = 0;
    celdaActual = START_CELL;
  } else {
    posViaje++;
    celdaActual = rutaViaje[posViaje];
  }
  enviarEstado();
}

// =====================================================================
//  WiFi
// =====================================================================
void iniciarWiFi() {
#if ES_AP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CANAL, 0, 4);
  Serial.printf("[RED] AP '%s' creado. IP %s\n", WIFI_SSID,
                WiFi.softAPIP().toString().c_str());
#else
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[RED] Conectando a '%s'", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? " OK" : " (sigo intentando)");
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("[RED] IP %s\n", WiFi.localIP().toString().c_str());
#endif
  udp.begin(UDP_PORT);
}

void vigilarWiFi() {
#if !ES_AP
  static uint32_t tReintento = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - tReintento > 5000) {
    tReintento = millis();
    Serial.println("[RED] Reconectando...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
#endif
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(LED_PIN, OUTPUT);
  Serial.printf("\n=== Carrito %d  |  ACO enjambre  |  laberinto %dx%d ===\n",
                CAR_ID, ROWS, COLS);
  iniciarFeromonas();
  iniciarWiFi();
}

void loop() {
  uint32_t ahora = millis();
  vigilarWiFi();
  recibir();

  if (ahora - tIter >= PERIODO_ITER_MS) { tIter = ahora; iteracionACO(); }
  if (ahora - tMov >= PERIODO_MOV_MS && ahora > (CAR_ID - 1) * RETARDO_ARRANQUE_MS) {
    tMov = ahora;
    avanzar();
  }

#if ES_AP
  if (ahora - tPh >= PERIODO_PH_MS) { tPh = ahora; enviarFeromonas(); limpiarPeers(); }
#else
  if (ahora - tHello >= PERIODO_HELLO_MS) {
    tHello = ahora;
    snprintf(bufTx, sizeof(bufTx), "HELLO,%d", CAR_ID);
    enviarA(AP_IP, UDP_PORT, bufTx);
  }
#endif

  // LED: fijo si convergió, parpadeo corto al recibir feromona
  if (convergido) digitalWrite(LED_PIN, HIGH);
  else if (ahora - tLed > 60) digitalWrite(LED_PIN, LOW);
}
