# Pendulum Control Board V2: single ESP32

Board for the single-ESP32 rig (`cart/cart_single` v11.2). Replaces the V1 two-ESP32
board in `../Pendulum`. Saved in KiCad 9 format, which KiCad 10 opens and upgrades.

## What's on it

| Ref | Part | Connects to |
|---|---|---|
| U1 | NodeMCU-32S socket | pinout = header of `cart/cart_single/cart_single.ino` |
| U2 | 4-ch level shifter module | encoders 5 V → 3.3 V, LVn↔HVn straight through |
| J1 | 5 V screw terminal | off-board HW-536 buck (24 V never on this board) |
| J2 | 1x6 header | Pi by jumper wires, in the order of the R1-R7 stack: 1 TXD (Pi pin 8), 2 RXD (10), 3 START (36), 4 STOP (38), 5 READY (33), 6 GND. Never jumper Pi pin 2 or 4 (5 V) |
| J3 | 2x5 IDC | Feedback encoder cable (same pinout as V1 J10 / 10pinBoxAdapter) |
| J4/J5 | 1x5 | pendulum / cart encoder test points |
| J6 | 1x8 | 1 START, 3 STOP (NC), 5 LIMIT-L, 7 LIMIT-R, even pins GND |
| J7 | 1x8 | IBT-2 in its own pin order: RPWM LPWM R_EN L_EN R_IS L_IS VCC GND |

Pi lines have 330 Ω series resistors. START/STOP/READY have 10k pull-downs (keep R6 on
strapping pin GPIO5). Switch inputs have a 10k pull-up, 1k series and 100n filter. RPWM/LPWM have
10k pull-downs so the motor stays off while the ESP32 boots.

## Changes from V1 to check

- **Level shifter:** V1 crossed the channels (LV1/LV2 to cart pins, HV1/HV2 to pendulum lines).
  V2 is straight through. If the encoders come out swapped, swap the cables or the labels.
  Don't re-cross the board.
- **Encoder pull-ups and filter caps removed (2026-11-09):** R9–R12 (4.7k) and C6–C9 (1 nF) are gone; the breadboard
  runs without them and the level-shifter module has its own pull-ups. If counts go missing with the motor running,
  add 1 nF from each encoder line to GND (not 100 nF: V1's 4.7k × 100n = 0.47 ms lost edges).
- **IBT-2 enables:** R_EN/L_EN are tied to +5 V.
- **Pi power pins:** the Pi's 3V3/5V pins are not connected. The board shares GND only.
- **USB and 5 V:** the ESP32 is powered from the 5 V terminal. Before you leave USB plugged in
  with the 5 V terminal live, check the DevKit has a diode on VBUS.

## Status

- ERC: clean (only "library not in config" warnings from an unconfigured kicad-cli).
- Schematic ↔ PCB parity: clean. The netlist was checked net by net against the design.
- PCB: routed with Freerouting 2.4.1 on 2026-10-27: 0 unconnected items, 0 clearance
  violations. Net classes: Power (GND, +5V, +3V3) 1.0 mm, Default 0.4 mm, 0.508 mm
  clearance (V1's board minimum). Ground is carried by the F.Cu/B.Cu pours only; the
  autorouted GND traces were removed, and DRC confirms every GND pad still connects.
  Same 147.5 × 120.5 mm outline and M3 hole positions as V1.
- Known DRC warnings: 26 silkscreen overlaps (cosmetic). 2 starved thermals: R22 pad 2
  touches a small F.Cu pour island, and J5 pin 1 has one spoke. Both are connected through
  the B.Cu pour.
- `Gerber/` and `*_gerbers.zip` are from **before** the GND traces were removed. Re-export
  before ordering.
- The custom footprints in `Skylar.pretty` were recovered from the V1 board, because the original
  `Skylar.pretty` folder isn't in the repo.
