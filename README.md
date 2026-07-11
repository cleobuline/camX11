# MacCam-6 (X11)

A cellular-automata machine in the spirit of the CAM-6, the hardware
described in Tommaso Toffoli and Norman Margolus's *Cellular Automata
Machines* (MIT Press, 1987). Rules are written in a small Forth dialect,
compiled to lookup tables, and run in real time on a toroidal grid.

This is the portable X11 build. The engine is plain C with no
dependencies; only the interface needs Xlib. It builds and runs on
Linux and on macOS (via XQuartz), and the engine itself compiles on any
machine with a C compiler — with no display at all, for headless use.

```
                                                              
    Brian's Brain running: white = firing, cyan = refractory  
    (drop a screenshot here — screenshot.png)                 
                                                              
```

## Build

```sh
# Debian / Ubuntu
sudo apt install libx11-dev

# Fedora / RHEL
sudo dnf install libX11-devel

# Arch
sudo pacman -S libx11

# macOS: install XQuartz (https://www.xquartz.org), then use its Terminal

make
./camx11 --dir regles
```

On a headless machine (a server, or CI), check the engine without
opening a window:

```sh
make selftest
```

## Running

```
./camx11 [--size 128|256|512|1024] [--rule file] [--dir folder]
```

- `--size` grid side in cells (default 256). Each cell is drawn as a
  square block; the window scales with nearest-neighbour, so a cell
  always stays a crisp square.
- `--rule file` load a single rule at startup.
- `--dir folder` folder scanned for rules (default `regles`). Files
  ending in `.rule`, `.forth`, `.fth` or `.f` are picked up.

The simulation always starts **paused**, and loading a rule never
clears the grid or starts the run — you stay in control of the
workshop. Press space to run, `r` to seed at random.

## Keyboard

The status bar at the top of the window shows the current state; press
`?` for the on-screen help. Layout-independent — works on AZERTY,
QWERTY, and the rest.

| key | action | | key | action |
|-----|--------|-|-----|--------|
| space | play / pause | | `f` | toggle the FHP gas |
| `s` | single step | | `h` | hydrodynamic view (gas) |
| `b` | step **backward** (reversible rules) | | `[` `]` | coarse-graining radius |
| `r` | seed at random | | `w` | steady wind |
| `c` | clear everything | | `x` | open right edge |
| `0` `1` `2` `3` | drawing plane | | `i` | isotropic spray |
| `!` `@` `#` `$` | show / hide a plane | | `d` `D` | seeding density |
| `p o a e y` | pencil, circle, square, eraser, spray | | `v` `V` | gas viscosity |
| `+` `-` | brush size | | `n` `N` | next / previous rule |
| `<` `>` | frames per second | | `l` | reload current rule |
| `?` | help | | `ESC` `q` | quit |

The mouse draws on the grid.

**Live reload:** while a rule is loaded, editing its file and saving it
recompiles it automatically — keep your editor open next to the window.
This is the Forth way of working: change, watch, change again, with no
dialog boxes in between.

## Writing rules

A rule is Forth source that ends in `MAKE-TABLE`. The simplest ones are
a single expression over the neighbourhood; Conway's Life, the default
rule, reads:

```forth
: LIFE
  NORTH SOUTH + EAST + WEST + N.EAST + N.WEST + S.EAST + S.WEST +
  CENTER IF DUP 2 = SWAP 3 = OR ELSE 3 = THEN ;
MAKE-TABLE LIFE
```

Neighbourhoods are selected in the source:

- **`N/MOORE`** (default) — the eight neighbours of plane 0, plus
  `CENTER`.
- **`N/VONN`** — von Neumann, reading the *primed* neighbours of plane 1
  (`NORTH'`, `SOUTH'`, …, `CENTER'`), as in Table 7.2 of the book.
- **`N/MARG`** — the Margolus 2×2 block neighbourhood (`CENTER`, `CW`,
  `CCW`, `OPP`). Block rules that are bijective can be run backward.
- **`N/HEX`** — a pseudo-hexagonal grid (Chapter 16).
- **`N/CUSTOM … END-CUSTOM`** — an arbitrary neighbourhood: up to twelve
  neighbours declared as `dx dy plane`, each optionally named, on either
  plane. For example:

  ```forth
  N/CUSTOM
    0 0 0 CENTER   0 1 0 NORTH   1 0 0 EAST
   -1 0 1 WEST-P1  1 1 0 NE
  END-CUSTOM
  : R CENTER NORTH XOR EAST XOR WEST-P1 XOR NE XOR ;
  MAKE-TABLE R
  ```

Results are shipped to planes with `>PLN0` and `>PLN1`. Comments use
`( … )` or the line comment `\`.

The `regles/` folder holds a set of ready-to-run rules, most of them
from the book: Life, Critters, Brian's Brain, the BBM billiard-ball
gas, HPP and FHP lattice gases, diffusion, annealing, and others.

## What this adds to the book

The core follows *Cellular Automata Machines* faithfully, with a few
additions of its own:

- **Arbitrary neighbourhoods** with named neighbours, on either plane.
- **Time reversal** for bijective Margolus rules (`b` steps backward).
- **The FHP lattice gas** of Chapter 16 as a self-contained subsystem,
  with wind, open boundaries, adjustable viscosity, and a
  **coarse-grained view**: the raw gas is noisy — an acoustic wave sits
  at a signal-to-noise ratio of about 0.4 and is invisible cell by cell
  — so the hydrodynamic view averages over a square whose radius you set
  with `[` and `]`, and the wave emerges. That averaging is not a
  cosmetic filter: it is the step from statistical to fluid mechanics.

## Origin

I wrote a first version of this in HyperCard on a 68000 Macintosh around
1990, and lost the source. This is the second attempt, thirty-five years
later. There is also a native macOS (Cocoa) version; this X11 build
exists because most people working on cellular automata are not on a
Mac.

## Licence

TODO — pick one (GPL or MIT) and add a LICENSE file. Without it, nobody
can legally reuse the code.
