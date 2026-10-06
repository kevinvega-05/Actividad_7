"""
Cliente de red de la simulación: se registra ante el AP (carrito 1) y
recibe todos los mensajes del enjambre. Guarda el último estado conocido.
"""
import socket
import threading
import time

import laberinto as L

N_CELDAS = L.FILAS * L.COLUMNAS


class RedACO:
    def __init__(self, ap_ip="192.168.4.1", ap_puerto=4210, puerto_local=4210):
        self.ap = (ap_ip, ap_puerto)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", puerto_local))
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()
        self.carros = {}                  # id -> dict con el último ST
        self.feromonas = [0.0] * N_CELDAS  # 0..1 por celda
        self.mejor_ruta = []              # mejor ruta vista en la red
        self.mejor_origen = 0
        self.depositos = 0
        self.eventos = []                 # últimos mensajes para el visor
        self.ultimo_rx = 0.0
        threading.Thread(target=self._hello, daemon=True).start()
        threading.Thread(target=self._escuchar, daemon=True).start()

    def _hello(self):
        while True:
            try:
                self.sock.sendto(b"HELLO,0", self.ap)
            except OSError:
                pass                       # aún no hay red: se reintenta
            time.sleep(2.0)

    def _evento(self, txt):
        self.eventos.append(f"{time.strftime('%H:%M:%S')}  {txt}")
        del self.eventos[:-12]

    def _escuchar(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except (socket.timeout, OSError):
                continue
            try:
                self._procesar(data.decode(errors="ignore").strip())
            except (ValueError, IndexError):
                pass                       # mensaje corrupto: se ignora

    def _procesar(self, msg):
        p = msg.split(",")
        with self.lock:
            self.ultimo_rx = time.time()
            if p[0] == "ST" and len(p) >= 7:
                cid = int(p[1])
                self.carros[cid] = {
                    "iter": int(p[2]), "celda": int(p[3]), "mejor": int(p[4]),
                    "convergido": p[5] == "1", "origen": int(p[6]),
                    "visto": time.time(),
                }
            elif p[0] == "DEP" and len(p) >= 5:
                self.depositos += 1
                ruta = [int(x) for x in p[4].split("-")]
                if L.ruta_valida(ruta) and (not self.mejor_ruta
                                            or len(ruta) < len(self.mejor_ruta)):
                    self.mejor_ruta = ruta
                    self.mejor_origen = int(p[1])
                    self._evento(f"Carrito {p[1]} encontró ruta de {len(ruta) - 1} pasos")
            elif p[0] == "PH" and len(p) >= 3:
                h = p[2]
                if len(h) == 2 * N_CELDAS:
                    self.feromonas = [int(h[i:i + 2], 16) / 255.0
                                      for i in range(0, len(h), 2)]

    def foto(self):
        """Copia segura del estado para dibujar."""
        with self.lock:
            return {
                "carros": {k: dict(v) for k, v in self.carros.items()},
                "feromonas": list(self.feromonas),
                "mejor_ruta": list(self.mejor_ruta),
                "mejor_origen": self.mejor_origen,
                "depositos": self.depositos,
                "eventos": list(self.eventos),
                "conectado": time.time() - self.ultimo_rx < 3.0,
            }
