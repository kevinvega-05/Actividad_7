"""
Laberinto / almacén compartido por los ESP32 y la simulación PyBullet.

Esta es la FUENTE ÚNICA del mapa. Si lo cambias, regenera el archivo del
firmware con:

    python laberinto.py --exportar-h ../firmware/aco_carrito/maze.h

Leyenda:  S = punto A (inicio)   G = meta   # = pared   . = libre
"""
import sys

MAPA = """
S..#..........
.#.#.######.#.
.#...#....#.#.
.#####.##.#.#.
.......#..#.#.
.#####.#.##.#.
.#...#.#....#.
.#.#.#.####.#.
...#.......#..
##.#######.#.#
...#.....#.#..
.###.###.#.##.
.....#.....#..
####...#.#...G
"""

FILAS_TXT = [f for f in MAPA.strip().splitlines()]
FILAS = len(FILAS_TXT)
COLUMNAS = len(FILAS_TXT[0])

# 1 = pared, 0 = libre
GRID = [[1 if ch == "#" else 0 for ch in fila] for fila in FILAS_TXT]


def _buscar(ch):
    for r, fila in enumerate(FILAS_TXT):
        c = fila.find(ch)
        if c >= 0:
            return r, c
    raise ValueError(f"El mapa no tiene '{ch}'")


INICIO = _buscar("S")
META = _buscar("G")

# Direcciones en el MISMO orden que el firmware: N, E, S, O
DR = (-1, 0, 1, 0)
DC = (0, 1, 0, -1)


def idx(r, c):
    return r * COLUMNAS + c


def rc(i):
    return divmod(i, COLUMNAS)


def libre(r, c):
    return 0 <= r < FILAS and 0 <= c < COLUMNAS and GRID[r][c] == 0


def vecino(celda, d):
    r, c = rc(celda)
    r, c = r + DR[d], c + DC[d]
    return idx(r, c) if libre(r, c) else -1


def ruta_valida(ruta):
    """Empieza en A, termina en la meta y cada paso es a una celda vecina libre."""
    if len(ruta) < 2 or ruta[0] != idx(*INICIO) or ruta[-1] != idx(*META):
        return False
    for a, b in zip(ruta, ruta[1:]):
        if b not in [vecino(a, d) for d in range(4)]:
            return False
    return True


def exportar_h(ruta_archivo):
    lineas = [
        "// ARCHIVO GENERADO por simulacion/laberinto.py --exportar-h",
        "// No lo edites a mano: cambia el mapa en laberinto.py y regenera.",
        "#pragma once",
        "#include <stdint.h>",
        "",
        f"#define ROWS {FILAS}",
        f"#define COLS {COLUMNAS}",
        f"#define START_R {INICIO[0]}",
        f"#define START_C {INICIO[1]}",
        f"#define GOAL_R {META[0]}",
        f"#define GOAL_C {META[1]}",
        "",
        "// 1 = pared, 0 = libre",
        "const uint8_t MAZE[ROWS][COLS] = {",
    ]
    for r, fila in enumerate(GRID):
        coma = "," if r < FILAS - 1 else ""
        lineas.append("  {" + ",".join(str(v) for v in fila) + "}" + coma
                      + f"  // {FILAS_TXT[r]}")
    lineas.append("};")
    with open(ruta_archivo, "w", encoding="utf-8") as f:
        f.write("\n".join(lineas) + "\n")
    print(f"maze.h escrito en {ruta_archivo}")


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--exportar-h":
        exportar_h(sys.argv[2])
    else:
        print("\n".join(FILAS_TXT))
        print(f"{FILAS}x{COLUMNAS}  inicio={INICIO}  meta={META}")
