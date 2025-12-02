#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "attempt1_pins.h"
#include "vga16_graphics_v2.h"
#include "customize.h"

#define VGA_WIDTH 640
#define VGA_HEIGHT 480

int main(void)
{
    stdio_init_all();
    sleep_ms(10);

    ensure_pin_layout();
    initVGA();

    show_project_name();
    sleep_ms(3000);
    show_loading_sequence();
    show_drum_intro();
    show_drum();
    
    hit_cymbals();
    hit_upper_toms();
    hit_floor_toms();
    hit_bass_drum();

    while (true) {
        move_sticks(-10, 0, 0, +10);
        sleep_ms(100);
    }

    while (true)
    {
        tight_loop_contents();
    }
}
