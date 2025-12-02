# RP2350B VGA Attempt

`attempt1` contains the application-layer code that wraps the untouched Cornell
`Source_VGA_Graphics` library. The demo initializes the VGA pipeline on an
RP2350B, fills the frame buffer with a checkerboard test pattern, and overlays
basic geometry/text so that timing, alignment, and color decoding are easy to
inspect on a monitor.

## Build Notes
- The project is configured for PlatformIO (`board = rpipico2`, Pico SDK).
- Application sources live in `src/attempt1`, while `lib/Source_VGA_Graphics`
  simply re-exports the upstream Cornell code without modification.
- Run `platformio run -t upload` to flash, or `platformio run -t build` for a
  dry compile.

## Hardware Wiring
- **Resistor ladder:** Feed the VGA `R`, `G`, and `B` pins through ≈330 Ω
  series resistors (470 Ω + 330 Ω split is fine for the two green bits) so the
  monitor sees ~0.7 V when the RP2350B drives 3.3 V.
- **Sync lines:** Drive `HSYNC` and `VSYNC` directly from the RP2350B GPIO (no
  series resistor needed) and share ground with the VGA connector.
- **Contiguous pins:** GPIO 16→21 map to `HSYNC`, `VSYNC`, `Glo`, `Ghi`, `B`,
  `R` respectively; the Cornell PIO program requires these six pins to be
  consecutive with `HSYNC` as the lowest number.
- **Timing:** The firmware targets the VGA baseline of 640×480 @ 60 Hz. The
  ideal pixel clock is 25.175 MHz, but most displays lock happily to 25.000 MHz
  as documented by Hunter Adams and TinyVGA.

## Verification Checklist
1. Power-up should yield a stable cyan/dark-blue checkerboard with text and a
   cyan frame. Any wobble or tearing suggests sync polarity/connection issues.
2. If the display remains black, reconfirm that the six GPIO pins are wired in
   order starting at HSYNC and that no legacy `set_sys_clock_khz()` calls remain
   in user code (the RP2350B defaults are sufficient).
3. Wrong or missing colors usually trace back to the resistor ladder or swapped
   RGB leads—double-check value, ordering, and solder joints.
4. Some monitors refuse 640×480; try a different VGA display or force the mode
   manually. The timing is part of the base VGA specification.

## References
- Hunter Adams, *PIO Assembly VGA Driver for RP2040* —
  https://vanhunteradams.com/Pico/VGA/VGA.html
- Bruce Land, *RP2040 VGA Graphics Primitives* —
  https://people.ece.cornell.edu/land/courses/ece4760/RP2040/C_SDK_vga16_fonts/index_vga_fonts.html
- TinyVGA, *640×480 @ 60 Hz Timing* — https://tinyvga.com/vga-timing/640x480@60Hz
