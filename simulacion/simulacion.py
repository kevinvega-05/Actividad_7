"""
Gemelo digital en PyBullet del enjambre ACO.

- Construye el laberinto de laberinto.py (paredes, inicio A, meta con bandera).
- Pinta el piso según la feromona que reporta el carrito 1 (azul -> rojo).
- Marca con esferas doradas la mejor ruta encontrada por el enjambre.
- Mueve 3 carros virtuales a la celda que reporta cada ESP32.

Modos de visualización:
    --modo web  (por defecto, ideal en Docker): abre http://localhost:8080
    --modo gui  ventana nativa de PyBullet (necesita pantalla / X11)

Ejemplos:
    python simulacion.py                                  # con los ESP32 reales
    python simulacion.py --ap 127.0.0.1 --ap-puerto 4211  # con el emulador
"""
import argparse
import io
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pybullet as p
import pybullet_data

import laberinto as L
from red_aco import RedACO

CELDA = 1.0          # metros por celda
ALTO_PARED = 0.6
COLORES_CARRO = {1: (0.90, 0.15, 0.15, 1), 2: (0.15, 0.45, 0.95, 1), 3: (0.10, 0.75, 0.30, 1)}
OFFSET_CARRO = {1: (-0.12, 0.0), 2: (0.10, -0.14), 3: (0.10, 0.14)}  # para que no se encimen
VEL_CARRO = 3.0      # m/s de la animación
ESCALA_CARRO = 1.6   # tamaño visual de los carritos


def centro(celda):
    r, c = L.rc(celda)
    # x a la derecha = columnas, y hacia arriba = filas invertidas
    return (c * CELDA, (L.FILAS - 1 - r) * CELDA)


def color_feromona(x):
    """0 -> gris azulado, 1 -> rojo intenso (mapa de calor)."""
    x = max(0.0, min(1.0, x))
    stops = [(0.0, (0.82, 0.85, 0.90)), (0.35, (0.30, 0.55, 0.95)),
             (0.65, (1.00, 0.75, 0.15)), (1.0, (0.95, 0.20, 0.10))]
    for (x0, c0), (x1, c1) in zip(stops, stops[1:]):
        if x <= x1:
            t = (x - x0) / (x1 - x0)
            return tuple(a + (b - a) * t for a, b in zip(c0, c1)) + (1,)
    return stops[-1][1] + (1,)


def caja(medias, pos, color, masa=0):
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=medias, rgbaColor=color)
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=medias) if masa >= 0 else -1
    return p.createMultiBody(masa, col, vis, pos)


def crear_carrito(color, pos):
    """Carrito sencillo (carrocería + cabina + 4 ruedas) mirando hacia +x."""
    llanta = (0.08, 0.08, 0.08, 1)
    vidrio = (0.75, 0.88, 1.0, 1)
    cuerpo_q = p.getQuaternionFromEuler([math.pi / 2, 0, 0])   # cilindro acostado
    tipos = [p.GEOM_BOX, p.GEOM_BOX, p.GEOM_BOX] + [p.GEOM_CYLINDER] * 4
    medias = [[0.20, 0.11, 0.05], [0.10, 0.09, 0.045], [0.02, 0.085, 0.035]] + [[0, 0, 0]] * 4
    radios = [0, 0, 0] + [0.055] * 4
    largos = [0, 0, 0] + [0.04] * 4
    colores = [color, color, vidrio] + [llanta] * 4
    pos_rel = [[0, 0, 0.09], [-0.03, 0, 0.18], [0.075, 0, 0.18],
               [0.12, 0.12, 0.055], [0.12, -0.12, 0.055],
               [-0.12, 0.12, 0.055], [-0.12, -0.12, 0.055]]
    orient = [[0, 0, 0, 1]] * 3 + [cuerpo_q] * 4
    k = ESCALA_CARRO
    medias = [[v * k for v in m] for m in medias]
    radios = [v * k for v in radios]
    largos = [v * k for v in largos]
    pos_rel = [[v * k for v in q] for q in pos_rel]
    vis = p.createVisualShapeArray(shapeTypes=tipos, halfExtents=medias, radii=radios,
                                   lengths=largos, rgbaColors=colores,
                                   visualFramePositions=pos_rel,
                                   visualFrameOrientations=orient)
    return p.createMultiBody(0, -1, vis, pos)


class Escena:
    def __init__(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf", [0, 0, -0.02])
        self.baldosas = {}
        self.color_baldosa = {}
        for r in range(L.FILAS):
            for c in range(L.COLUMNAS):
                i = L.idx(r, c)
                x, y = centro(i)
                if L.GRID[r][c]:
                    caja([CELDA / 2, CELDA / 2, ALTO_PARED / 2], [x, y, ALTO_PARED / 2],
                         (0.20, 0.22, 0.28, 1))
                else:
                    self.baldosas[i] = caja([CELDA * 0.47, CELDA * 0.47, 0.01],
                                            [x, y, 0.0], color_feromona(0))
        self._marcador_inicio()
        self._bandera_meta()
        self._borde()
        # esferas para la mejor ruta (pool)
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.07, rgbaColor=(1, 0.84, 0, 1))
        self.puntos = [p.createMultiBody(0, -1, vis, [0, 0, -5]) for _ in range(L.FILAS * L.COLUMNAS)]
        self.ruta_dibujada = None
        # carros
        self.carros = {}
        ini = L.idx(*L.INICIO)
        for cid in (1, 2, 3):
            x, y = centro(ini)
            ox, oy = OFFSET_CARRO[cid]
            body = crear_carrito(COLORES_CARRO[cid], [x + ox, y + oy, 0.0])
            self.carros[cid] = {"body": body, "x": x + ox, "y": y + oy, "yaw": -math.pi / 2}

    def _marcador_inicio(self):
        x, y = centro(L.idx(*L.INICIO))
        caja([0.3, 0.3, 0.012], [x, y, 0.005], (0.1, 0.8, 0.3, 1))
        caja([0.04, 0.04, 0.35], [x - 0.42, y + 0.42, 0.35], (0.1, 0.8, 0.3, 1))

    def _bandera_meta(self):
        x, y = centro(L.idx(*L.META))
        # baldosa de meta a cuadros
        n = 4
        lado = 0.94 / n
        for i in range(n):
            for j in range(n):
                col = (0.05, 0.05, 0.05, 1) if (i + j) % 2 else (0.97, 0.97, 0.97, 1)
                caja([lado / 2, lado / 2, 0.013],
                     [x - 0.47 + lado * (i + 0.5), y - 0.47 + lado * (j + 0.5), 0.004], col)
        yb = y + 0.45                                                          # arco de llegada
        caja([0.03, 0.03, 0.6], [x - 0.42, yb, 0.6], (0.85, 0.45, 0.20, 1))
        caja([0.03, 0.03, 0.6], [x + 0.42, yb, 0.6], (0.85, 0.45, 0.20, 1))
        caja([0.42, 0.02, 0.015], [x, yb, 0.35], (0.9, 0.1, 0.1, 1))            # cinta roja
        n, lado = 6, 0.84 / 6                                                  # bandera
        for i in range(n):
            for j in range(2):
                col = (0.05, 0.05, 0.05, 1) if (i + j) % 2 else (0.97, 0.97, 0.97, 1)
                caja([lado / 2, 0.015, lado / 2],
                     [x - 0.42 + lado * (i + 0.5), yb, 1.15 - lado * (j + 0.5)], col)

    def _borde(self):
        ancho, alto = L.COLUMNAS * CELDA, L.FILAS * CELDA
        cx, cy = ancho / 2 - CELDA / 2, alto / 2 - CELDA / 2
        g, h, col = 0.05, ALTO_PARED / 2, (0.12, 0.13, 0.17, 1)
        caja([ancho / 2 + g, g, h], [cx, -CELDA / 2 - g, h], col)
        caja([ancho / 2 + g, g, h], [cx, alto - CELDA / 2 + g, h], col)
        caja([g, alto / 2, h], [-CELDA / 2 - g, cy, h], col)
        caja([g, alto / 2, h], [ancho - CELDA / 2 + g, cy, h], col)

    def actualizar(self, foto, dt):
        # piso según feromona (solo si cambió notablemente)
        for i, b in self.baldosas.items():
            v = round(foto["feromonas"][i], 2)
            if self.color_baldosa.get(i) != v:
                self.color_baldosa[i] = v
                p.changeVisualShape(b, -1, rgbaColor=color_feromona(v))
        # mejor ruta
        ruta = foto["mejor_ruta"]
        if ruta != self.ruta_dibujada:
            self.ruta_dibujada = ruta
            for k, esf in enumerate(self.puntos):
                if k < len(ruta):
                    x, y = centro(ruta[k])
                    p.resetBasePositionAndOrientation(esf, [x, y, 0.09], [0, 0, 0, 1])
                else:
                    p.resetBasePositionAndOrientation(esf, [0, 0, -5], [0, 0, 0, 1])
        # carros: se acercan suavemente a la celda reportada
        for cid, car in self.carros.items():
            st = foto["carros"].get(cid)
            if not st:
                continue
            tx, ty = centro(st["celda"])
            tx += OFFSET_CARRO[cid][0]
            ty += OFFSET_CARRO[cid][1]
            dx, dy = tx - car["x"], ty - car["y"]
            dist = math.hypot(dx, dy)
            if dist > 2.5 * CELDA:          # salto (volvió a A): teletransporte
                car["x"], car["y"] = tx, ty
            elif dist > 1e-3:
                paso = min(dist, VEL_CARRO * dt)
                car["x"] += dx / dist * paso
                car["y"] += dy / dist * paso
                if dist > 0.05:
                    obj = math.atan2(dy, dx)
                    dyaw = (obj - car["yaw"] + math.pi) % (2 * math.pi) - math.pi
                    car["yaw"] += max(-8 * dt, min(8 * dt, dyaw))
            p.resetBasePositionAndOrientation(
                car["body"], [car["x"], car["y"], 0.0],
                p.getQuaternionFromEuler([0, 0, car["yaw"]]))


# ------------------------------------------------------------------
#  Visor web (para Docker / sin pantalla)
# ------------------------------------------------------------------
HTML = """<!doctype html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Enjambre ACO</title><style>
:root{--bg:#0f1218;--panel:#181d26;--txt:#e8ecf2;--sub:#8b95a7;--line:#262d3a}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--txt);
font:15px/1.45 system-ui,Segoe UI,Roboto,sans-serif}
header{padding:16px 20px;border-bottom:1px solid var(--line);display:flex;gap:12px;align-items:center;flex-wrap:wrap}
h1{font-size:18px;margin:0;font-weight:650}.pill{font-size:12px;padding:3px 10px;border-radius:99px;background:#2a3140;color:var(--sub)}
.pill.ok{background:#12391f;color:#58d68d}main{display:grid;grid-template-columns:minmax(0,1fr) 340px;gap:16px;padding:16px}
@media(max-width:900px){main{grid-template-columns:1fr}}
.vista{background:#000;border-radius:10px;overflow:hidden}.vista img{width:100%;display:block}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px;margin-bottom:12px}
.panel h2{font-size:12px;letter-spacing:.06em;text-transform:uppercase;color:var(--sub);margin:0 0 10px}
.carro{display:flex;align-items:center;gap:10px;padding:7px 0;border-top:1px solid var(--line)}
.carro:first-of-type{border-top:0}.dot{width:12px;height:12px;border-radius:3px;flex:none}
.carro b{min-width:74px}.carro span{color:var(--sub);font-size:13px}
.big{font-size:30px;font-weight:700}.ev{font:12px/1.6 ui-monospace,Consolas,monospace;color:var(--sub);white-space:pre-wrap}
.ley{height:10px;border-radius:5px;background:linear-gradient(90deg,#d1d9e6,#4d8cf2,#ffbf26,#f2331a);margin:6px 0 4px}
.ley+div{display:flex;justify-content:space-between;font-size:12px;color:var(--sub)}
</style></head><body>
<header><h1>Enjambre de 3 carritos · ACO en ESP32 · gemelo digital PyBullet</h1>
<span id="red" class="pill">sin datos</span></header>
<main><div class="vista"><img id="f" src="/frame.jpg" alt="Simulación"></div>
<div><div class="panel"><h2>Mejor ruta del enjambre</h2><div class="big" id="mejor">—</div>
<div id="orig" style="color:var(--sub)"></div></div>
<div class="panel"><h2>Carritos</h2><div id="carros"></div></div>
<div class="panel"><h2>Feromona en el piso</h2><div class="ley"></div><div><span>baja</span><span>alta</span></div></div>
<div class="panel"><h2>Eventos</h2><div class="ev" id="ev"></div></div></div></main>
<script>
const COL={1:'#e62626',2:'#2673f2',3:'#1abf4d'};
function frame(){const i=new Image();i.onload=()=>{document.getElementById('f').src=i.src;setTimeout(frame,80)};
i.onerror=()=>setTimeout(frame,500);i.src='/frame.jpg?t='+Date.now()}frame();
async function estado(){try{const s=await (await fetch('/estado.json')).json();
const r=document.getElementById('red');r.textContent=s.conectado?'recibiendo datos de la red':'esperando a los ESP32…';
r.className='pill'+(s.conectado?' ok':'');
document.getElementById('mejor').textContent=s.mejor_pasos>0?s.mejor_pasos+' pasos':'—';
document.getElementById('orig').textContent=s.mejor_origen?('encontrada por el carrito '+s.mejor_origen+' · '+s.depositos+' depósitos de feromona'):'';
document.getElementById('carros').innerHTML=[1,2,3].map(i=>{const c=s.carros[i];
return `<div class="carro"><div class="dot" style="background:${COL[i]}"></div><b>Carrito ${i}</b><span>${c?
`iter ${c.iter} · mejor ${c.mejor>0?c.mejor:'—'} · ${c.convergido?'convergido':'buscando'}`:'sin conexión'}</span></div>`}).join('');
document.getElementById('ev').textContent=s.eventos.slice().reverse().join('\\n')||'—';}catch(e){}
setTimeout(estado,500)}estado();
</script></body></html>"""


class Visor:
    def __init__(self, red, puerto):
        self.red = red
        self.jpeg = b""
        self.lock = threading.Lock()
        visor = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _enviar(self, cuerpo, tipo):
                self.send_response(200)
                self.send_header("Content-Type", tipo)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(cuerpo)))
                self.end_headers()
                self.wfile.write(cuerpo)

            def do_GET(self):
                ruta = self.path.split("?")[0]
                if ruta == "/frame.jpg":
                    with visor.lock:
                        self._enviar(visor.jpeg, "image/jpeg")
                elif ruta == "/estado.json":
                    f = visor.red.foto()
                    f["mejor_pasos"] = len(f["mejor_ruta"]) - 1 if f["mejor_ruta"] else -1
                    del f["feromonas"]
                    self._enviar(json.dumps(f).encode(), "application/json")
                else:
                    self._enviar(HTML.encode(), "text/html; charset=utf-8")

        srv = ThreadingHTTPServer(("0.0.0.0", puerto), H)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"Visor web listo: abre http://localhost:{puerto}")

        cx = (L.COLUMNAS - 1) * CELDA / 2
        cy = (L.FILAS - 1) * CELDA / 2
        dist = 1.25 * max(L.FILAS, L.COLUMNAS) * CELDA
        self.view = p.computeViewMatrixFromYawPitchRoll([cx, cy - 0.5, 0], dist, 0, -65, 0, 2)
        self.proj = p.computeProjectionMatrixFOV(50, 4 / 3, 0.1, 50)

    def capturar(self, ancho=800, alto=600):
        from PIL import Image
        _, _, rgb, _, _ = p.getCameraImage(ancho, alto, self.view, self.proj,
                                           renderer=p.ER_TINY_RENDERER,
                                           shadow=1, lightDirection=[3, -4, 8])
        img = Image.frombytes("RGBA", (ancho, alto), bytes(bytearray(rgb))).convert("RGB")
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=82)
        with self.lock:
            self.jpeg = buf.getvalue()


def main():
    ap = argparse.ArgumentParser(description="Gemelo digital PyBullet del enjambre ACO")
    ap.add_argument("--modo", choices=["web", "gui"], default="web")
    ap.add_argument("--ap", default="192.168.4.1", help="IP del carrito 1 (AP)")
    ap.add_argument("--ap-puerto", type=int, default=4210)
    ap.add_argument("--puerto-local", type=int, default=4210)
    ap.add_argument("--puerto-web", type=int, default=8080)
    ap.add_argument("--fps", type=float, default=12, help="cuadros por segundo del visor web")
    args = ap.parse_args()

    p.connect(p.GUI if args.modo == "gui" else p.DIRECT)
    if args.modo == "gui":
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(1.1 * max(L.FILAS, L.COLUMNAS), 0, -65,
                                     [(L.COLUMNAS - 1) / 2, (L.FILAS - 1) / 2 - 0.5, 0])

    escena = Escena()
    red = RedACO(args.ap, args.ap_puerto, args.puerto_local)
    print(f"Escuchando el enjambre en UDP {args.puerto_local}; registrando ante {args.ap}:{args.ap_puerto}")
    visor = Visor(red, args.puerto_web) if args.modo == "web" else None

    t_prev = time.time()
    t_frame = 0.0
    while True:
        ahora = time.time()
        dt = min(0.1, ahora - t_prev)
        t_prev = ahora
        escena.actualizar(red.foto(), dt)
        if visor and ahora - t_frame >= 1.0 / args.fps:
            t_frame = ahora
            visor.capturar()
        time.sleep(1 / 60)


if __name__ == "__main__":
    main()
