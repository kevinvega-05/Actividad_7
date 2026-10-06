"""
Réplica en Python de la lógica ACO del firmware (aco_carrito.ino).
La usa emulador_esp32.py para probar todo sin tener los ESP32 a mano.
Los parámetros y el protocolo son los mismos que en config.h.
"""
import random

import laberinto as L

N_ANTS = 4
ALPHA = 1.0
BETA = 1.0
RHO = 0.10
Q_HORMIGA = 1.0
Q_DEPOSITO = 5.0
TAU_INICIAL = 1.0
TAU_MIN = 0.05
TAU_MAX = 10.0
ITERS_CONVERGE = 25

N_CELDAS = L.FILAS * L.COLUMNAS
START = L.idx(*L.INICIO)
GOAL = L.idx(*L.META)


def heuristica(celda):
    r, c = L.rc(celda)
    return 1.0 / (1 + abs(r - L.META[0]) + abs(c - L.META[1]))


def direccion_entre(a, b):
    for d in range(4):
        if L.vecino(a, d) == b:
            return d
    return -1


class NodoACO:
    """Un ESP32 virtual. `enviar(msg)` es la función que pone el mensaje en la red."""

    def __init__(self, car_id, enviar):
        self.id = car_id
        self.enviar = enviar
        self.tau = [[TAU_INICIAL] * 4 for _ in range(N_CELDAS)]
        self.mejor = []
        self.mejor_origen = 0
        self.iter = 0
        self.sin_mejora = 0
        self.convergido = False
        # movimiento virtual
        self.viaje = []
        self.pos = 0
        self.celda = START
        self.pausa = 0

    # ---------------- ACO ----------------
    def caminar_hormiga(self):
        visitado = {START}
        ruta = [START]
        actual = START
        while actual != GOAL:
            pesos = []
            for d in range(4):
                n = L.vecino(actual, d)
                if n < 0 or n in visitado:
                    pesos.append(0.0)
                else:
                    pesos.append(self.tau[actual][d] ** ALPHA * heuristica(n) ** BETA)
            s = sum(pesos)
            if s <= 0:
                ruta.pop()
                if not ruta:
                    return None
                actual = ruta[-1]
                continue
            d = random.choices(range(4), weights=pesos)[0]
            actual = L.vecino(actual, d)
            visitado.add(actual)
            ruta.append(actual)
        return ruta

    def evaporar(self):
        for fila in self.tau:
            for d in range(4):
                fila[d] = max(TAU_MIN, fila[d] * (1 - RHO))

    def depositar(self, ruta, q_total=Q_DEPOSITO):
        q = q_total / (len(ruta) - 1)
        for a, b in zip(ruta, ruta[1:]):
            d = direccion_entre(a, b)
            if d >= 0:
                self.tau[a][d] = min(TAU_MAX, self.tau[a][d] + q)

    def considerar(self, ruta, origen):
        if self.mejor and len(ruta) >= len(self.mejor):
            return False
        self.mejor = list(ruta)
        self.mejor_origen = origen
        self.sin_mejora = 0
        self.convergido = False
        return True

    def iteracion(self):
        mejor_it = None
        self.evaporar()
        for _ in range(N_ANTS):
            r = self.caminar_hormiga()
            if r:
                self.depositar(r, Q_HORMIGA)
            if r and (mejor_it is None or len(r) < len(mejor_it)):
                mejor_it = r
        self.iter += 1
        if not mejor_it:
            return
        self.depositar(mejor_it)
        self.enviar(f"DEP,{self.id},{self.iter},{len(mejor_it) - 1},"
                    + "-".join(map(str, mejor_it)))
        if not self.considerar(mejor_it, self.id):
            self.sin_mejora += 1
            if self.sin_mejora >= ITERS_CONVERGE:
                self.convergido = True

    # ---------------- Red ----------------
    def recibir(self, msg):
        if not msg.startswith("DEP,"):
            return
        partes = msg.split(",")
        if len(partes) < 5 or int(partes[1]) == self.id:
            return
        ruta = [int(x) for x in partes[4].split("-")]
        if not L.ruta_valida(ruta):
            return
        self.depositar(ruta)
        self.considerar(ruta, int(partes[1]))

    def estado(self):
        mejor = len(self.mejor) - 1 if self.mejor else -1
        return (f"ST,{self.id},{self.iter},{self.celda},{mejor},"
                f"{int(self.convergido)},{self.mejor_origen}")

    def feromonas(self):
        hexs = []
        for i in range(N_CELDAS):
            r, c = L.rc(i)
            m = max(self.tau[i]) if L.GRID[r][c] == 0 else 0.0
            x = min(1.0, max(0.0, (m - TAU_MIN) / (TAU_MAX - TAU_MIN)))
            hexs.append(f"{int(x * 255):02X}")
        return f"PH,{self.id}," + "".join(hexs)

    # ---------------- Movimiento ----------------
    def avanzar(self):
        if not self.viaje or self.pos >= len(self.viaje) - 1:
            if self.viaje and self.pausa < 3:
                self.pausa += 1
                self.enviar(self.estado())
                return
            self.pausa = 0
            if self.mejor:
                self.viaje = list(self.mejor)
            self.pos = 0
            self.celda = START
        else:
            self.pos += 1
            self.celda = self.viaje[self.pos]
        self.enviar(self.estado())
