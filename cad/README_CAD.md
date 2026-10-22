# C.A.R.L. CAD
Vehicle frame: X left/right, Y forward, Z up (ground Z=0), millimetres.

## Files
- stl/            printable parts, each in a sensible print orientation (29 parts, Rev G)
- step/           same parts as STEP for editing in Fusion/FreeCAD/SolidWorks
- CARL_assembly.step               whole car (printed parts detailed, other parts simplified blocks)
- CARL_printed_parts_assembled.stl all printed parts in assembled position
- carl_cad.py     parametric source. Edit the parameters at the top, run `python3 carl_cad.py` (needs `pip install cadquery`)

## Parameters to confirm after delivery
- SHAFT_D 6.7 (motor shaft 6.6 measured) -> set from the fit-test plate result
- WHEEL_BORE 12.0 -> the hole you drill in each Robocraze wheel; measure the wheel hub and change this
- UNO Q hole pattern uses the UNO R3 positions -> verify against the real board
- ToF board: the two M2 holes in the bumper (14) are at +/-9.5 mm, an assumption: measure your VL53L0X board and change carl_cad.py
- Gimbal: pan servo MG996R sits through the upper-deck slot, tilt servo MG996R sits outside the left upright (its spline passes through the upright and screws to the cradle's left flange)
- Not modelled: IMU and USB camera mounts (tape or hot-glue), switch/fuse cutouts, battery tray

## Rev G parts (13-18)
13 steering-servo pillars L/R, 14 front bumper with ToF window, 15 pan plate, 16 tilt uprights L/R, 17 tilt cradle (ball head screws into its centre hole), 18 pivot bush (print 2)

## Hardware these parts assume
- M3 screws (10-12 mm) + nuts for decks, straps, outriggers, skirts; M3 grub screws (6 mm) x4 for rear hubs
- 4x aluminium standoffs 45 mm + M3 screws, silicone dampers
- Kingpins: 2x M3 x 65 bolt (or cut M3 rod) + nyloc nuts
- Front axles: 2x M6 x 65 bolt + 2 nyloc nuts + 4 washers (wheels spin on printed bushes)
- Gimbal and servo screws: M3 x 10 self-tapping x ~12, one M3 x 30 pivot bolt + nyloc nut, M3 x 10 x 4 under the pan plate, M2 x 6 x 2 for the ToF board
- Ackermann: knuckle arms are angled toward the rear axle centre; tie rod bolts to arm tips, servo horn pin to the centre hole

## Print notes (Bambu A1, PETG)
Decks 4 mm: 3 walls, 25% infill, flat. Straps/saddles: 4 walls, 40% infill. Hub adapters: 5 walls, 100% infill.
