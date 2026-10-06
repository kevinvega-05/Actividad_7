"""
Emulador de los 3 ESP32 (para probar sin hardware).

Se comporta en la red EXACTAMENTE como el carrito 1 en modo AP:
escucha en UDP, registra a quien le manda HELLO (la simulación) y le
reenvía todos los mensajes DEP / ST / PH de los 3 carritos virtuales.

Uso:
    python emulador_esp32.py --puerto 4211
y en otra terminal:
    python simulacion.py --ap 127.0.0.1 --ap-puerto 4211
"""
import argparse
import socket
import threading
import time

from aco_core import NodoACO

PERIODO_ITER = 0.80
PERIODO_MOV = 0.35
PERIODO_PH = 1.0
TIMEOUT_PEER = 10.0
RETARDO_ARRANQUE = 1.2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto", type=int, default=4211)
    ap.add_argument("--velocidad", type=float, default=1.0,
                    help="multiplica la velocidad del tiempo (2 = el doble de rápido)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.puerto))
    peers = {}                      # (ip, puerto) -> último visto
    lock = threading.Lock()
    nodos = []

    def difundir(msg, origen):
        # a los otros carritos virtuales (como lo haría la red)
        for n in nodos:
            if n is not origen:
                n.recibir(msg)
        # a los nodos externos registrados (PC con la simulación)
        with lock:
            for p in list(peers):
                sock.sendto(msg.encode(), p)

    for i in (1, 2, 3):
        nodo = NodoACO(i, None)
        nodo.enviar = (lambda n: (lambda m: difundir(m, n)))(nodo)
        nodos.append(nodo)

    def escuchar():
        while True:
            data, addr = sock.recvfrom(2048)
            msg = data.decode(errors="ignore")
            with lock:
                if addr not in peers:
                    print(f"[RED] Nodo registrado: {addr[0]}:{addr[1]}")
                peers[addr] = time.time()

    threading.Thread(target=escuchar, daemon=True).start()
    print(f"Emulador de 3 ESP32 (AP virtual) escuchando en UDP {args.puerto}")

    k = 1.0 / args.velocidad
    t_iter = t_mov = t_ph = 0.0
    t0 = time.time()
    mejor_imp = None
    while True:
        ahora = time.time()
        if ahora - t_iter >= PERIODO_ITER * k:
            t_iter = ahora
            for n in nodos:
                n.iteracion()
            m = nodos[0].mejor
            if m and len(m) != mejor_imp:
                mejor_imp = len(m)
                print(f"[ACO] iter {nodos[0].iter}: mejor ruta {len(m) - 1} pasos "
                      f"(carrito {nodos[0].mejor_origen})")
        if ahora - t_mov >= PERIODO_MOV * k:
            t_mov = ahora
            for i, n in enumerate(nodos):        # desfase como en el firmware
                if ahora - t0 >= i * RETARDO_ARRANQUE * k:
                    n.avanzar()
        if ahora - t_ph >= PERIODO_PH * k:
            t_ph = ahora
            difundir(nodos[0].feromonas(), nodos[0])
            with lock:
                for p, t in list(peers.items()):
                    if ahora - t > TIMEOUT_PEER:
                        print(f"[RED] Nodo {p} sin respuesta, se elimina")
                        del peers[p]
        time.sleep(0.01)


if __name__ == "__main__":
    main()
