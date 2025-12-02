#ifndef CUSTOMIZE_H
#define CUSTOMIZE_H

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "attempt1_pins.h"
#include "vga16_graphics_v2.h"

#define VGA_WIDTH 640
#define VGA_HEIGHT 480
// Keep animations snappy when invoked from the IMU loop.
#ifndef ACTIVATION_TIME
#define ACTIVATION_TIME 120
#endif
#ifndef FUNCTION_LAST_TIME
#define FUNCTION_LAST_TIME 120
#endif

#define COLOR_BACKGROUND WHITE
#define COLOR_SHELL YELLOW
#define COLOR_RIM BLACK
#define COLOR_CYMBAL YELLOW
#define COLOR_STAND BLACK
#define COLOR_STICK BLACK
#define COLOR_HAND WHITE

typedef void (*component_draw_fn)(void);

static short text_pixel_width(const char *text, short size);
static short g_left_stick_offset_x = 0;
static short g_left_stick_offset_y = 0;
static short g_right_stick_offset_x = 0;
static short g_right_stick_offset_y = 0;

static void ensure_pin_layout(void)
{
    // The Cornell driver expects the HSYNC pin to be the lowest value and the
    // remaining signals to be consecutive. Keep the default RP2350 wiring.
#if defined(HSYNC)
    static_assert(HSYNC == ATTEMPT1_VGA_PIN_HSYNC, "HSYNC pin mismatch");
#endif
#if defined(VSYNC)
    static_assert(VSYNC == ATTEMPT1_VGA_PIN_VSYNC, "VSYNC pin mismatch");
#endif
}

static void fill_capsule(short x, short y, short w, short h, char color)
{
    if (w <= 0 || h <= 0)
        return;
    short r = (h > 2) ? (h / 2 - 1) : 0;
    short cxL = x + r;
    short cxR = x + w - r - 1;
    short cy = y + h / 2;

    if (w > 2 * r)
        fillRect(x + r, y, w - 2 * r, h, color);
    fillCircle(cxL, cy, r, color);
    fillCircle(cxR, cy, r, color);
}

// outline for the same capsule
static void draw_capsule(short x, short y, short w, short h, char color)
{
    if (w <= 0 || h <= 0)
        return;
    short r = (h > 2) ? (h / 2 - 1) : 0;
    short cxL = x + r;
    short cxR = x + w - r - 1;
    short cy = y + h / 2;

    if (w > 2 * r)
        drawRect(x + r, y, w - 2 * r, h, color);
    drawCircle(cxL, cy, r, color);
    drawCircle(cxR, cy, r, color);
}

// rounded rectangle from rect + 4 circles (filled)
static void fill_round_rect(short x, short y, short w, short h, short rad, char color)
{
    if (w <= 0 || h <= 0)
        return;
    if (rad < 0)
        rad = 0;
    if (rad * 2 > w)
        rad = w / 2;
    if (rad * 2 > h)
        rad = h / 2;

    // center body
    fillRect(x + rad, y, w - 2 * rad, h, color);
    fillRect(x, y + rad, rad, h - 2 * rad, color);
    fillRect(x + w - rad, y + rad, rad, h - 2 * rad, color);

    // corners
    fillCircle(x + rad, y + rad, rad, color);
    fillCircle(x + w - rad - 1, y + rad, rad, color);
    fillCircle(x + rad, y + h - rad - 1, rad, color);
    fillCircle(x + w - rad - 1, y + h - rad - 1, rad, color);
}

// outline of rounded rectangle
static void draw_round_rect(short x, short y, short w, short h, short rad, char color)
{
    if (w <= 0 || h <= 0)
        return;
    if (rad < 0)
        rad = 0;
    if (rad * 2 > w)
        rad = w / 2;
    if (rad * 2 > h)
        rad = h / 2;

    drawRect(x + rad, y, w - 2 * rad, h, color);
    drawRect(x, y + rad, rad, h - 2 * rad, color);
    drawRect(x + w - rad, y + rad, rad, h - 2 * rad, color);

    drawCircle(x + rad, y + rad, rad, color);
    drawCircle(x + w - rad - 1, y + rad, rad, color);
    drawCircle(x + rad, y + h - rad - 1, rad, color);
    drawCircle(x + w - rad - 1, y + h - rad - 1, rad, color);
}

// cymbal as capsule + center strike line
static void draw_cymbal_capsule(short x, short y, short w, short h, char fill_color, char outline_color)
{
    fill_capsule(x, y, w, h, fill_color);
    draw_capsule(x, y, w, h, outline_color);

    drawLine(x + 6, y + h / 2, x + w - 6, y + h / 2, outline_color);
}

// single drum with shell + rim
static void draw_drum_shell(short cx, short cy, short radius, char shell_color, char rim_color)
{
    if (radius <= 0)
        return;
    fillCircle(cx, cy, radius, shell_color);
    drawCircle(cx, cy, radius, BLACK);
    if (radius > 4)
        drawCircle(cx, cy, radius - 4, rim_color);
}

static void fill_screen_color(char color)
{
    fillRect(0, 0, VGA_WIDTH, VGA_HEIGHT, color);
}

static void draw_thick_stick(short x0, short y0, short x1, short y1, char color)
{
    for (short dx = -2; dx <= 3; ++dx)
    {
        for (short dy = -2; dy <= 3; ++dy)
        {
            drawLine(x0 + dx, y0 + dy, x1 + dx, y1 + dy, color);
        }
    }
}

static void draw_cymbals_base(void)
{
    draw_cymbal_capsule(70, 120, 140, 28, COLOR_CYMBAL, COLOR_STAND);
    draw_cymbal_capsule(260, 70, 120, 26, COLOR_CYMBAL, COLOR_STAND);
    draw_cymbal_capsule(430, 130, 150, 30, COLOR_CYMBAL, COLOR_STAND);
}

static void draw_cymbal_stands_base(void)
{
    drawLine(140, 148, 140, 360, COLOR_STAND);
    drawLine(140, 360, 100, 420, COLOR_STAND);
    drawLine(140, 360, 180, 420, COLOR_STAND);
    drawLine(505, 160, 505, 360, COLOR_STAND);
    drawLine(505, 360, 465, 420, COLOR_STAND);
    drawLine(505, 360, 545, 420, COLOR_STAND);
    drawLine(320, 96, 320, 220, COLOR_STAND);
}

static void draw_upper_toms_base(void)
{
    draw_drum_shell(260, 215, 48, COLOR_SHELL, COLOR_RIM);
    draw_drum_shell(380, 215, 48, COLOR_SHELL, COLOR_RIM);
}

static void draw_floor_toms_base(void)
{
    draw_drum_shell(200, 292, 56, COLOR_SHELL, COLOR_RIM);
    draw_drum_shell(440, 300, 64, COLOR_SHELL, COLOR_RIM);
}

static void draw_bass_drum_base(void)
{
    draw_drum_shell(320, 330, 92, COLOR_SHELL, COLOR_RIM);
    fillCircle(320, 330, 50, COLOR_BACKGROUND);
    drawCircle(320, 330, 50, COLOR_STAND);
    drawLine(320, 420, 320, 462, COLOR_STAND);
    fillRect(300, 462, 40, 12, COLOR_HAND);
    drawRect(300, 462, 40, 12, COLOR_STAND);
}

static void draw_left_stick(void)
{
    draw_thick_stick(190 + g_left_stick_offset_x, 360 + g_left_stick_offset_y,
                     300 + g_left_stick_offset_x, 250 + g_left_stick_offset_y, COLOR_STICK);
}

static void draw_right_stick(void)
{
    draw_thick_stick(450 + g_right_stick_offset_x, 360 + g_right_stick_offset_y,
                     340 + g_right_stick_offset_x, 250 + g_right_stick_offset_y, COLOR_STICK);
}

static void draw_sticks_base(void)
{
    draw_left_stick();
    draw_right_stick();
}

static void draw_hands_base(void)
{
    fill_round_rect(150, 340, 90, 70, 18, COLOR_HAND);
    draw_round_rect(150, 340, 90, 70, 18, COLOR_STAND);
    fill_round_rect(400, 340, 90, 70, 18, COLOR_HAND);
    draw_round_rect(400, 340, 90, 70, 18, COLOR_STAND);
    fill_round_rect(120, 380, 70, 82, 22, COLOR_HAND);
    draw_round_rect(120, 380, 70, 82, 22, COLOR_STAND);
    fill_round_rect(420, 380, 70, 82, 22, COLOR_HAND);
    draw_round_rect(420, 380, 70, 82, 22, COLOR_STAND);
}

static void draw_static_drums(void)
{
    fill_screen_color(COLOR_BACKGROUND);
    draw_cymbals_base();
    draw_cymbal_stands_base();
    draw_upper_toms_base();
    draw_floor_toms_base();
    draw_bass_drum_base();
}

// This function show the drum set
static void show_drum(void)
{
    draw_static_drums();
    draw_sticks_base();
    draw_hands_base();
}

static void move_sticks(short left_dx, short left_dy, short right_dx, short right_dy)
{
    g_left_stick_offset_x += left_dx;
    g_left_stick_offset_y += left_dy;
    g_right_stick_offset_x += right_dx;
    g_right_stick_offset_y += right_dy;
    draw_static_drums();
    draw_left_stick();
    draw_right_stick();
    draw_hands_base();
}

static void draw_hit_label(const char *target_name)
{
    if (!target_name)
        return;

    char message[64];
    snprintf(message, sizeof(message), "You Are Hitting %s", target_name);

    const short text_size = 2;
    const short baseline_padding = 6;
    const short label_height = text_size * 12;
    const short label_y = VGA_HEIGHT - label_height;
    fillRect(0, label_y, VGA_WIDTH, label_height, COLOR_BACKGROUND);

    setTextColor2(COLOR_STAND, COLOR_BACKGROUND);
    setTextSize(text_size);
    setCursor(12, label_y + baseline_padding);
    writeString(message);
}

static void hit_left_cymbal(void);
static void hit_middle_cymbal(void);
static void hit_right_cymbal(void);
static void hit_left_upper_tom(void);
static void hit_right_upper_tom(void);
static void hit_left_floor_tom(void);
static void hit_right_floor_tom(void);

static void hit_cymbals(void)
{
    hit_left_cymbal();
    hit_middle_cymbal();
    hit_right_cymbal();
}

static void hit_left_cymbal(void)
{
    const short frames = 10;
    const uint32_t frame_delay = ACTIVATION_TIME / frames;

    for (int j = 0; j < FUNCTION_LAST_TIME / ACTIVATION_TIME; j++) {
        for (short i = 0; i < frames; ++i)
        {
            show_drum();
            draw_hit_label("Left Cymbal");
            const short shake = (i & 1) ? 8 : -8;
            const short bounce = (i & 2) ? 4 : -4;
            const short inflate = frames - i;

            draw_cymbal_capsule(70 + shake - inflate, 120 + bounce, 140 + inflate * 2, 28 + inflate / 2, COLOR_CYMBAL, COLOR_STAND);
            sleep_ms(frame_delay);
        }
    }
    show_drum();
}

static void hit_middle_cymbal(void)
{
    const short frames = 10;
    const uint32_t frame_delay = ACTIVATION_TIME / frames;

    for (int j = 0; j < FUNCTION_LAST_TIME / ACTIVATION_TIME; j++) {
        for (short i = 0; i < frames; ++i)
        {
            show_drum();
            draw_hit_label("Middle Cymbal");
            const short shake = (i & 1) ? 4 : -4;
            const short bounce = (i & 2) ? 4 : -4;
            const short inflate = frames - i;

            draw_cymbal_capsule(260 + shake - inflate, 70 - bounce, 120 + inflate * 2, 26 + inflate / 2, COLOR_CYMBAL, COLOR_STAND);
            sleep_ms(frame_delay);
        }
    }
    show_drum();
}

static void hit_right_cymbal(void)
{
    const short frames = 10;
    const uint32_t frame_delay = ACTIVATION_TIME / frames;

    for (int j = 0; j < FUNCTION_LAST_TIME / ACTIVATION_TIME; j++) {
        for (short i = 0; i < frames; ++i)
        {
            show_drum();
            draw_hit_label("Right Cymbal");
            const short shake = (i & 1) ? 8 : -8;
            const short bounce = (i & 2) ? 4 : -4;
            const short inflate = frames - i;

            draw_cymbal_capsule(430 - shake - inflate, 130 - bounce, 150 + inflate * 2, 30 + inflate / 2, COLOR_CYMBAL, COLOR_STAND);
            sleep_ms(frame_delay);
        }
    }
    show_drum();
}

static void hit_upper_toms(void)
{
    hit_left_upper_tom();
    hit_right_upper_tom();
}

static void hit_left_upper_tom(void)
{
    const short frames = 10;
    const uint32_t frame_delay = ACTIVATION_TIME / frames;

    for (int j = 0; j < FUNCTION_LAST_TIME / ACTIVATION_TIME; j++)
    {
        for (short i = 0; i < frames; ++i)
        {
            show_drum();
            draw_hit_label("Left Upper Tom");
            const short shake = (i & 1) ? 6 : -6;
            const short bounce = (i & 2) ? 4 : -4;
            const short inflate = (frames - i) / 2;

            draw_drum_shell(260 + shake, 215 + bounce, 48 + inflate, COLOR_SHELL, COLOR_RIM);
            draw_sticks_base();
            sleep_ms(frame_delay);
        }
    }
    show_drum();
}

static void hit_right_upper_tom(void)
{
    const short frames = 10;
    const uint32_t frame_delay = ACTIVATION_TIME / frames;

    for (int j = 0; j < FUNCTION_LAST_TIME / ACTIVATION_TIME; j++)
    {
        for (short i = 0; i < frames; ++i)
        {
            show_drum();
            draw_hit_label("Right Upper Tom");
            const short shake = (i & 1) ? 6 : -6;
            const short bounce = (i & 2) ? 4 : -4;
            const short inflate = (frames - i) / 2;

            draw_drum_shell(380 - shake, 215 - bounce, 48 + inflate, COLOR_SHELL, COLOR_RIM);
            draw_sticks_base();
            sleep_ms(frame_delay);
        }
    }
    show_drum();
}

static void hit_floor_toms(void)
{
    hit_left_floor_tom();
    hit_right_floor_tom();
}

static void hit_left_floor_tom(void)
{
    const short frames = 10;
    const uint32_t frame_delay = ACTIVATION_TIME / frames;

    for (int j = 0; j < FUNCTION_LAST_TIME / ACTIVATION_TIME; j++)
    {
        for (short i = 0; i < frames; ++i)
        {
            show_drum();
            draw_hit_label("Left Floor Tom");
            const short shake = (i & 1) ? 5 : -5;
            const short bounce = (i & 2) ? 4 : -4;
            const short inflate = (frames - i) / 2;

            draw_drum_shell(200 + shake, 292 + bounce, 56 + inflate, COLOR_SHELL, COLOR_RIM);
            draw_sticks_base();
            draw_hands_base();
            sleep_ms(frame_delay);
        }
    }
    show_drum();
}

static void hit_right_floor_tom(void)
{
    const short frames = 10;
    const uint32_t frame_delay = ACTIVATION_TIME / frames;

    for (int j = 0; j < FUNCTION_LAST_TIME / ACTIVATION_TIME; j++)
    {
        for (short i = 0; i < frames; ++i)
        {
            show_drum();
            draw_hit_label("Right Floor Tom");
            const short shake = (i & 1) ? 5 : -5;
            const short bounce = (i & 2) ? 4 : -4;
            const short inflate = (frames - i) / 2;

            draw_drum_shell(440 - shake, 300 - bounce, 64 + inflate, COLOR_SHELL, COLOR_RIM);
            draw_sticks_base();
            draw_hands_base();
            sleep_ms(frame_delay);
        }
    }
    show_drum();
}

static void hit_bass_drum(void)
{
    const short frames = 10;
    const uint32_t frame_delay = ACTIVATION_TIME / frames;

    for (int j = 0; j < FUNCTION_LAST_TIME / ACTIVATION_TIME; j++)
    {
        for (short i = 0; i < frames; ++i)
        {
            show_drum();
            draw_hit_label("Bass Drum");
            const short shake = (i & 1) ? 6 : -6;
            const short bounce = (i & 2) ? 5 : -5;
            const short inflate = frames - i;

            draw_drum_shell(320 + shake, 330 + bounce, 92 + inflate, COLOR_SHELL, COLOR_RIM);
            fillCircle(320 + shake, 330 + bounce, 50 + inflate / 2, COLOR_BACKGROUND);
            drawCircle(320 + shake, 330 + bounce, 50 + inflate / 2, COLOR_STAND);
            drawLine(320 + shake, 420 + bounce / 2, 320 + shake, 462 + bounce / 2, COLOR_STAND);
            fillRect(300 + shake / 2, 462 + bounce / 2, 40, 12, COLOR_HAND);
            drawRect(300 + shake / 2, 462 + bounce / 2, 40, 12, COLOR_STAND);
            draw_sticks_base();
            draw_hands_base();
            sleep_ms(frame_delay);
        }
    }
        show_drum();
    }

static void draw_component_label(const char *label)
{
    if (!label)
        return;
    const short text_size = 3;
    const short text_y = VGA_HEIGHT - 110;
    const short text_width = text_pixel_width(label, text_size);
    setTextColor2(BLACK, WHITE);
    setTextSize(text_size);
    setCursor((VGA_WIDTH - text_width) / 2, text_y);
    writeString((char *)label);
}

static void draw_component_card(component_draw_fn draw_fn, const char *label, uint32_t hold_ms)
{
    fill_screen_color(WHITE);
    if (draw_fn)
    {
        draw_fn();
    }
    draw_component_label(label);
    sleep_ms(hold_ms);
}

static void render_upper_toms_preview(void)
{
    const char shell_color = YELLOW;
    const char rim_color = BLACK;
    draw_drum_shell(240, 210, 55, shell_color, rim_color);
    draw_drum_shell(400, 210, 55, shell_color, rim_color);
}

static void render_cymbals_preview(void)
{
    const char cymbal_color = YELLOW;
    draw_cymbal_capsule(100, 180, 180, 32, cymbal_color, BLACK);
    draw_cymbal_capsule(360, 180, 180, 32, cymbal_color, BLACK);
}

static void render_floor_toms_preview(void)
{
    const char shell_color = YELLOW;
    const char rim_color = BLACK;
    draw_drum_shell(230, 300, 65, shell_color, rim_color);
    draw_drum_shell(410, 300, 70, shell_color, rim_color);
}

static void render_bass_drum_preview(void)
{
    const char shell_color = YELLOW;
    const char rim_color = BLACK;
    draw_drum_shell(320, 260, 110, shell_color, rim_color);
    fillCircle(320, 260, 60, WHITE);
    drawCircle(320, 260, 60, rim_color);
}

static void render_beater_pedal_preview(void)
{
    const char stand_color = BLACK;
    const char pad_color = WHITE;
    drawLine(320, 150, 320, 300, stand_color);
    fillRect(300, 300, 40, 14, pad_color);
    drawRect(300, 300, 40, 14, stand_color);
}

static void render_sticks_preview(void)
{
    const char stick_color = BLACK;
    draw_thick_stick(200, 320, 360, 200, stick_color);
    draw_thick_stick(440, 320, 280, 200, stick_color);
}

static void show_drum_intro(void)
{
    draw_component_card(render_cymbals_preview, "Cymbals", 1000);
    draw_component_card(render_upper_toms_preview, "Upper Toms", 1000);
    draw_component_card(render_floor_toms_preview, "Floor Toms", 1000);
    draw_component_card(render_bass_drum_preview, "Bass Drum", 1000);
    draw_component_card(render_beater_pedal_preview, "Beater + Pedal", 1000);
    draw_component_card(render_sticks_preview, "Sticks", 1000);
}

// This function shows the project name
// and is also the cover page
static short text_pixel_width(const char *text, short size)
{
    if (!text)
        return 0;
    return (short)(strlen(text) * size * 6);
}

static void draw_centered_lines(const char *const *lines, short line_count, short text_size, char fg, char bg)
{
    if (!lines || line_count <= 0 || text_size <= 0)
        return;

    const short line_height = text_size * 12;
    const short total_height = line_count * line_height;
    short cursor_y = (VGA_HEIGHT - total_height) / 2;

    fillRect(0, 0, VGA_WIDTH, VGA_HEIGHT, bg);

    setTextColor2(fg, bg);
    setTextSize(text_size);

    for (short i = 0; i < line_count; ++i)
    {
        short line_width = text_pixel_width(lines[i], text_size);
        short cursor_x = (VGA_WIDTH - line_width) / 2;
        setCursor(cursor_x, cursor_y);
        writeString((char *)lines[i]);
        cursor_y += line_height;
    }
}

static void show_project_name(void)
{
    static const char *lines[] = {
        "ECE 362",
        "Final Project",
        "Drum Set",
    };
    const short line_count = (short)(sizeof(lines) / sizeof(lines[0]));
    const short text_size = 3;
    const short line_height = text_size * 12;

    short max_width = 0;
    for (short i = 0; i < line_count; ++i)
    {
        short width = text_pixel_width(lines[i], text_size);
        if (width > max_width)
        {
            max_width = width;
        }
    }

    const short padding = 24;
    const short block_width = max_width + padding * 2;
    const short block_height = line_count * line_height + padding * 2;
    const short block_x = (VGA_WIDTH - block_width) / 2;
    const short block_y = (VGA_HEIGHT - block_height) / 2;

    fillRect(block_x, block_y, block_width, block_height, WHITE);
    drawRect(block_x, block_y, block_width, block_height, BLACK);

    setTextColor2(BLACK, WHITE);
    setTextSize(text_size);

    short cursor_y = block_y + padding;
    for (short i = 0; i < line_count; ++i)
    {
        short line_width = text_pixel_width(lines[i], text_size);
        short cursor_x = (VGA_WIDTH - line_width) / 2;
        setCursor(cursor_x, cursor_y);
        writeString((char *)lines[i]);
        cursor_y += line_height;
    }

    const char *prompt = "Press Button to Start";
    const short prompt_size = 2;
    const short prompt_width = text_pixel_width(prompt, prompt_size);
    setTextSize(prompt_size);
    setCursor((VGA_WIDTH - prompt_width) / 2, block_y + block_height + 24);
    writeString((char *)prompt);
}

static void show_loading_sequence(void)
{
    static const char *hello_lines[] = {"Hello", "Player 1"};
    static const char *ready_lines[] = {"Are You Ready?"};
    static const char *three_line[] = {"3"};
    static const char *two_line[] = {"2"};
    static const char *one_line[] = {"1"};
    static const char *start_line[] = {"Start"};

    draw_centered_lines(hello_lines, 2, 3, BLACK, WHITE);
    sleep_ms(1000);

    draw_centered_lines(ready_lines, 1, 3, BLACK, WHITE);
    sleep_ms(1000);

    draw_centered_lines(three_line, 1, 4, BLACK, WHITE);
    sleep_ms(1000);

    draw_centered_lines(two_line, 1, 4, BLACK, WHITE);
    sleep_ms(1000);

    draw_centered_lines(one_line, 1, 4, BLACK, WHITE);
    sleep_ms(1000);

    draw_centered_lines(start_line, 1, 4, BLACK, WHITE);
    sleep_ms(1000);
}

#endif
