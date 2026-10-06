#pragma once
// =====================================================================
//  CONFIGURACIÓN DEL CARRITO  — cambia SOLO CAR_ID antes de cargar
//  cada placa:  carrito 1 -> CAR_ID 1 (crea la red WiFi, modo AP)
//               carrito 2 -> CAR_ID 2
//               carrito 3 -> CAR_ID 3
// =====================================================================
#define CAR_ID 1

// ---------------- Red (la crea el carrito 1 en modo AP) ----------------
#define WIFI_SSID   "ACO_SWARM"
#define WIFI_PASS   "hormigas123"     // mínimo 8 caracteres
#define WIFI_CANAL  6
#define UDP_PORT    4210
#define AP_IP_BYTES 192, 168, 4, 1

// ---------------- Parámetros del ACO (MAX-MIN Ant System) --------------
#define N_ANTS          4       // hormigas por iteración en cada ESP32
#define ALPHA           1.0f    // peso de la feromona
#define BETA            1.0f    // peso de la heurística (cercanía a la meta)
#define RHO             0.10f   // evaporación por iteración
#define Q_HORMIGA       1.0f    // cada hormiga deposita Q_HORMIGA / longitud (local)
#define Q_DEPOSITO      5.0f    // la mejor de la iteración deposita Q / longitud (y se comparte)
#define TAU_INICIAL     1.0f
#define TAU_MIN         0.05f
#define TAU_MAX         10.0f
#define ITERS_CONVERGE  25      // iteraciones sin mejorar => "convergido"

// ---------------- Tiempos (ms) -----------------------------------------
#define PERIODO_ITER_MS    800   // cada cuánto corre una iteración ACO
#define PERIODO_MOV_MS     350   // cada cuánto el carrito avanza una celda
#define PERIODO_HELLO_MS   2000  // registro ante el AP
#define PERIODO_PH_MS      1000  // (solo AP) envía mapa de feromonas
#define TIMEOUT_PEER_MS    10000 // (solo AP) olvida nodos que no hablan

#define RETARDO_ARRANQUE_MS 1200 // desfase entre carritos al empezar a moverse

#define LED_PIN 2                // LED azul de la placa DevKit
