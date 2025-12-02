#pragma once

// Contiguous GPIO assignment for the Cornell VGA driver.
// HSYNC must be the smallest pin number and the remaining signals follow.
enum {
    ATTEMPT1_VGA_PIN_HSYNC = 16,
    ATTEMPT1_VGA_PIN_VSYNC,
    ATTEMPT1_VGA_PIN_GREEN_LSB,
    ATTEMPT1_VGA_PIN_GREEN_MSB,
    ATTEMPT1_VGA_PIN_BLUE,
    ATTEMPT1_VGA_PIN_RED,
};

// Number of consecutive pins consumed by the VGA PIO program.
#define ATTEMPT1_VGA_PIN_COUNT 6
