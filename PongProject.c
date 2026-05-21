#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdint.h>
#include <string.h>
 
/* =========================================================
  HARDWARE WIRING
  ---------------------------------------------------------
  LED Matrix (MAX7219 16x16):
    DIN -> D11 (PB3 / MOSI)
    CS  -> D10 (PB2 / SS)
    CLK -> D13 (PB5 / SCK)
  LCD backpacks (PCF8574):
    SDA -> A4
    SCL -> A5
    LCD Bracket -> 0x27  (no bridges)
    LCD Info    -> 0x26  (A0 solder pad bridged)
  Joysticks:
    Joy1 VRy -> A0   (Player 1, left paddle)
    Joy2 VRx -> A2   (Player 2, right paddle)
    Joy1 SW  -> D2
    Joy2 SW  -> D3
  Restart button:
    One leg -> D4  |  Other -> GND
    Internal pull-up, no resistor needed
  ========================================================= */
 
#define LCD_BRACKET_ADDR   0x27
#define LCD_INFO_ADDR      0x26
 
#define JOY1_SW_PIN   PD2
#define JOY2_SW_PIN   PD3
#define RESTART_PIN   PD4
 
#define JOY1_ADC_CH   0
#define JOY2_ADC_CH   2
 
#define JOY1_INVERT   0
#define JOY2_INVERT   0
 
/* FIX: Increased Deadzone to 200 to eliminate stick drift */
#define JOY_DEADZONE  200
 
static const char player_name[4][9] = { "P1", "P2", "P3", "P4" };
static const char player_tag [4][5] = { "P1", "P2", "P3", "P4" };
 
#define PADDLE_H            4
#define PADDLE_STEP_MS      40
#define BALL_STEP_MS        90
#define SCORE_FLASH_MS      600
#define POINT_PAUSE_MS      400
#define MATCH_WIN_PAUSE_MS  2400
#define BOTH_PRESS_MS       80
#define ADC_SETTLE_US       120
#define ADC_SAMPLES         6
#define WIN_SCORE           5
 
volatile uint32_t g_ms = 0;
 
static uint16_t fb[16];
 
static uint8_t paddle1_y = 6;
static uint8_t paddle2_y = 6;
 
static int8_t  ball_x = 7, ball_y = 7;
static int8_t  ball_dx = 1, ball_dy = 0;
static int8_t  next_serve_dx = 1;
 
static int8_t  flash_x = 0, flash_y = 7;
static uint8_t flashing = 0;
 
static uint16_t joy1_center = 512;
static uint16_t joy2_center = 512;
 
static uint8_t current_match  = 0;
static uint8_t left_player    = 0;
static uint8_t right_player   = 1;
static uint8_t semi_winner[2] = {255, 255};
static uint8_t champion       = 255;
 
static uint8_t score_left  = 0;
static uint8_t score_right = 0;
 
static uint32_t last_paddle_ms      = 0;
static uint32_t last_ball_ms        = 0;
static uint32_t state_start_ms      = 0;
static uint32_t both_press_start    = 0;
static uint32_t last_info_lcd_ms    = 0;
static uint32_t last_bracket_lcd_ms = 0;
 
typedef enum {
    ST_WAIT_BOTH = 0,
    ST_PLAY,
    ST_SCORE_FLASH,
    ST_POINT_PAUSE,
    ST_MATCH_OVER,
    ST_TOURNAMENT_OVER
} GameState;
 
static GameState state = ST_WAIT_BOTH;
 
/* =========================================================
  1 ms tick
  ========================================================= */
 
ISR(TIMER0_COMPA_vect)
{
    g_ms++;
}
 
static uint32_t millis_now(void)
{
    uint32_t t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        t = g_ms;
    }
    return t;
}
 
static void timer0_init_1ms(void)
{
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A  = 249;
    TIMSK0 = (1 << OCIE0A);
}
 
/* =========================================================
  SPI + MAX7219
  ========================================================= */
 
static void spi_init(void)
{
    DDRB  |= (1 << PB2) | (1 << PB3) | (1 << PB5);
    PORTB |= (1 << PB2);
    SPCR   = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}
 
static void spi_tx(uint8_t x)
{
    SPDR = x;
    while (!(SPSR & (1 << SPIF))) {
        ;
    }
}
 
static void max7219_send4(uint8_t reg,
                          uint8_t m0, uint8_t m1,
                          uint8_t m2, uint8_t m3)
{
    PORTB &= ~(1 << PB2);
 
    spi_tx(reg); spi_tx(m0);
    spi_tx(reg); spi_tx(m1);
    spi_tx(reg); spi_tx(m2);
    spi_tx(reg); spi_tx(m3);
 
    PORTB |= (1 << PB2);
}
 
static void matrix_hw_clear(void)
{
    for (uint8_t r = 1; r <= 8; r++) {
        max7219_send4(r, 0, 0, 0, 0);
    }
}
 
static void matrix_init(void)
{
    max7219_send4(0x0F, 0, 0, 0, 0);
    max7219_send4(0x0C, 1, 1, 1, 1);
    max7219_send4(0x0B, 7, 7, 7, 7);
    max7219_send4(0x0A, 2, 2, 2, 2);
    max7219_send4(0x09, 0, 0, 0, 0);
    matrix_hw_clear();
}
 
/* =========================================================
  Framebuffer
  ========================================================= */
 
static void fb_clear(void)
{
    for (uint8_t y = 0; y < 16; y++) {
        fb[y] = 0;
    }
}
 
static void set_pixel(uint8_t x, uint8_t y)
{
    if (x < 16 && y < 16) {
        fb[y] |= ((uint16_t)1 << (15 - x));
    }
}
 
static uint8_t fb_get(uint8_t x, uint8_t y)
{
    if (x >= 16 || y >= 16) return 0;
    return (fb[y] & ((uint16_t)1 << (15 - x))) ? 1 : 0;
}
 
static void draw_paddle(uint8_t x, uint8_t y0)
{
    for (uint8_t i = 0; i < PADDLE_H; i++) {
        set_pixel(x, (uint8_t)(y0 + i));
    }
}
 
static uint8_t rotated_byte_cw(uint8_t dest_row, uint8_t x_start)
{
    uint8_t b = 0;
 
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t src_x = (uint8_t)(15 - dest_row);
        uint8_t src_y = (uint8_t)(x_start + i);
 
        if (fb_get(src_x, src_y)) {
            b |= (uint8_t)(1 << (7 - i));
        }
    }
 
    return b;
}
 
static void render_matrix(void)
{
    for (uint8_t r = 0; r < 8; r++) {
        uint8_t tl = rotated_byte_cw(r,     0);
        uint8_t tr = rotated_byte_cw(r,     8);
        uint8_t bl = rotated_byte_cw(r + 8, 0);
        uint8_t br = rotated_byte_cw(r + 8, 8);
 
        max7219_send4((uint8_t)(r + 1), tl, tr, bl, br);
    }
}
 
/* =========================================================
  Tiny 3x5 font
  ========================================================= */
 
static void glyph3x5(char c, uint8_t out[5])
{
    if (c >= 'a' && c <= 'z') c = (char)(c - 32);
 
    switch (c) {
        case 'A': out[0]=0x2;out[1]=0x5;out[2]=0x7;out[3]=0x5;out[4]=0x5;break;
        case 'B': out[0]=0x6;out[1]=0x5;out[2]=0x6;out[3]=0x5;out[4]=0x6;break;
        case 'C': out[0]=0x3;out[1]=0x4;out[2]=0x4;out[3]=0x4;out[4]=0x3;break;
        case 'D': out[0]=0x6;out[1]=0x5;out[2]=0x5;out[3]=0x5;out[4]=0x6;break;
        case 'E': out[0]=0x7;out[1]=0x4;out[2]=0x6;out[3]=0x4;out[4]=0x7;break;
        case 'F': out[0]=0x7;out[1]=0x4;out[2]=0x6;out[3]=0x4;out[4]=0x4;break;
        case 'G': out[0]=0x3;out[1]=0x4;out[2]=0x5;out[3]=0x5;out[4]=0x3;break;
        case 'H': out[0]=0x5;out[1]=0x5;out[2]=0x7;out[3]=0x5;out[4]=0x5;break;
        case 'I': out[0]=0x7;out[1]=0x2;out[2]=0x2;out[3]=0x2;out[4]=0x7;break;
        case 'J': out[0]=0x1;out[1]=0x1;out[2]=0x1;out[3]=0x5;out[4]=0x2;break;
        case 'K': out[0]=0x5;out[1]=0x5;out[2]=0x6;out[3]=0x5;out[4]=0x5;break;
        case 'L': out[0]=0x4;out[1]=0x4;out[2]=0x4;out[3]=0x4;out[4]=0x7;break;
        case 'M': out[0]=0x5;out[1]=0x7;out[2]=0x7;out[3]=0x5;out[4]=0x5;break;
        case 'N': out[0]=0x5;out[1]=0x7;out[2]=0x7;out[3]=0x7;out[4]=0x5;break;
        case 'O': out[0]=0x7;out[1]=0x5;out[2]=0x5;out[3]=0x5;out[4]=0x7;break;
        case 'P': out[0]=0x6;out[1]=0x5;out[2]=0x6;out[3]=0x4;out[4]=0x4;break;
        case 'Q': out[0]=0x2;out[1]=0x5;out[2]=0x5;out[3]=0x2;out[4]=0x1;break;
        case 'R': out[0]=0x6;out[1]=0x5;out[2]=0x6;out[3]=0x5;out[4]=0x5;break;
        case 'S': out[0]=0x3;out[1]=0x4;out[2]=0x2;out[3]=0x1;out[4]=0x6;break;
        case 'T': out[0]=0x7;out[1]=0x2;out[2]=0x2;out[3]=0x2;out[4]=0x2;break;
        case 'U': out[0]=0x5;out[1]=0x5;out[2]=0x5;out[3]=0x5;out[4]=0x7;break;
        case 'V': out[0]=0x5;out[1]=0x5;out[2]=0x5;out[3]=0x5;out[4]=0x2;break;
        case 'W': out[0]=0x5;out[1]=0x5;out[2]=0x7;out[3]=0x7;out[4]=0x5;break;
        case 'X': out[0]=0x5;out[1]=0x5;out[2]=0x2;out[3]=0x5;out[4]=0x5;break;
        case 'Y': out[0]=0x5;out[1]=0x5;out[2]=0x2;out[3]=0x2;out[4]=0x2;break;
        case 'Z': out[0]=0x7;out[1]=0x1;out[2]=0x2;out[3]=0x4;out[4]=0x7;break;
 
        case '0': out[0]=0x7;out[1]=0x5;out[2]=0x5;out[3]=0x5;out[4]=0x7;break;
        case '1': out[0]=0x2;out[1]=0x6;out[2]=0x2;out[3]=0x2;out[4]=0x7;break;
        case '2': out[0]=0x6;out[1]=0x1;out[2]=0x7;out[3]=0x4;out[4]=0x7;break;
        case '3': out[0]=0x6;out[1]=0x1;out[2]=0x3;out[3]=0x1;out[4]=0x6;break;
        case '4': out[0]=0x5;out[1]=0x5;out[2]=0x7;out[3]=0x1;out[4]=0x1;break;
        case '5': out[0]=0x7;out[1]=0x4;out[2]=0x7;out[3]=0x1;out[4]=0x6;break;
        case '6': out[0]=0x3;out[1]=0x4;out[2]=0x7;out[3]=0x5;out[4]=0x7;break;
        case '7': out[0]=0x7;out[1]=0x1;out[2]=0x1;out[3]=0x1;out[4]=0x1;break;
        case '8': out[0]=0x7;out[1]=0x5;out[2]=0x7;out[3]=0x5;out[4]=0x7;break;
        case '9': out[0]=0x7;out[1]=0x5;out[2]=0x7;out[3]=0x1;out[4]=0x6;break;
 
        case '-': out[0]=0x0;out[1]=0x0;out[2]=0x7;out[3]=0x0;out[4]=0x0;break;
        default:  out[0]=0x0;out[1]=0x0;out[2]=0x0;out[3]=0x0;out[4]=0x0;break;
    }
}
 
static void draw_char3x5(uint8_t x, uint8_t y, char c)
{
    uint8_t g[5];
    glyph3x5(c, g);
 
    for (uint8_t row = 0; row < 5; row++) {
        for (uint8_t col = 0; col < 3; col++) {
            if (g[row] & (1 << (2 - col))) {
                set_pixel((uint8_t)(x + col), (uint8_t)(y + row));
            }
        }
    }
}
 
static void draw_text3x5_centered(const char *s, uint8_t y)
{
    uint8_t len = (uint8_t)strlen(s);
    if (!len) return;
    if (len > 4) len = 4;
 
    uint8_t width = (uint8_t)(len * 4 - 1);
    uint8_t x = (width < 16) ? (uint8_t)((16 - width) / 2) : 0;
 
    for (uint8_t i = 0; i < len; i++) {
        draw_char3x5((uint8_t)(x + i * 4), y, s[i]);
    }
}
 
static void draw_smiley(uint8_t x, uint8_t y)
{
    set_pixel(x+1,y+1);
    set_pixel(x+3,y+1);
    set_pixel(x+0,y+3);
    set_pixel(x+1,y+4);
    set_pixel(x+2,y+4);
    set_pixel(x+3,y+4);
    set_pixel(x+4,y+3);
}
 
static void draw_confetti(void)
{
    set_pixel(1,1);  set_pixel(3,2);  set_pixel(5,1);
    set_pixel(10,1); set_pixel(12,2); set_pixel(14,1);
    set_pixel(2,12); set_pixel(4,13); set_pixel(6,12);
    set_pixel(9,12); set_pixel(11,13); set_pixel(13,12);
}
 
static void draw_winner_screen(uint8_t winner_idx)
{
    fb_clear();
 
    draw_text3x5_centered(player_tag[winner_idx], 3);
 
    if ((millis_now() / 250) & 1) {
        draw_smiley(10, 9);
    } else {
        draw_confetti();
    }
 
    render_matrix();
}
 
static void draw_game_screen(void)
{
    fb_clear();
    draw_paddle(0,  paddle1_y);
    draw_paddle(15, paddle2_y);
    if (flashing) {
        if ((millis_now() / 60) & 1) {
            set_pixel((uint8_t)flash_x, (uint8_t)flash_y);
        }
    } else {
        set_pixel((uint8_t)ball_x, (uint8_t)ball_y);
    }
    render_matrix();
}
 
/* =========================================================
  TWI + LCD
  ========================================================= */
 
#define LCD_BL  0x08
#define LCD_EN  0x04
#define LCD_RW  0x02
#define LCD_RS  0x01
 
static void twi_init(void)
{
    TWSR = 0x00;
    TWBR = 72;
    TWCR = (1 << TWEN);
}
 
static void twi_start(uint8_t addr_rw)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {
        ;
    }
 
    TWDR = addr_rw;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {
        ;
    }
}
 
static void twi_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {
        ;
    }
}
 
static void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    _delay_us(10);
}
 
static void lcd_exp_write(uint8_t addr, uint8_t data)
{
    twi_start((uint8_t)(addr << 1));
    twi_write((uint8_t)(data | LCD_BL));
    twi_stop();
}
 
static void lcd_pulse(uint8_t addr, uint8_t data)
{
    lcd_exp_write(addr, (uint8_t)(data | LCD_EN));
    _delay_us(1);
    lcd_exp_write(addr, (uint8_t)(data & (uint8_t)~LCD_EN));
    _delay_us(50);
}
 
static void lcd_write4(uint8_t addr, uint8_t nibble, uint8_t mode)
{
    uint8_t data = (uint8_t)((nibble << 4) | LCD_BL | (mode ? LCD_RS : 0));
    lcd_pulse(addr, data);
}
 
static void lcd_send(uint8_t addr, uint8_t value, uint8_t mode)
{
    lcd_write4(addr, (uint8_t)(value >> 4),   mode);
    lcd_write4(addr, (uint8_t)(value & 0x0F), mode);
}
 
static void lcd_cmd(uint8_t addr, uint8_t cmd)
{
    lcd_send(addr, cmd, 0);
 
    if (cmd == 0x01 || cmd == 0x02) {
        _delay_ms(2);
    }
}
 
static void lcd_data(uint8_t addr, uint8_t data)
{
    lcd_send(addr, data, 1);
}
 
static void lcd_init_one(uint8_t addr)
{
    _delay_ms(50);
 
    lcd_write4(addr, 0x03, 0);
    _delay_ms(5);
    lcd_write4(addr, 0x03, 0);
    _delay_us(150);
    lcd_write4(addr, 0x03, 0);
    lcd_write4(addr, 0x02, 0);
 
    lcd_cmd(addr, 0x28);
    lcd_cmd(addr, 0x0C);
    lcd_cmd(addr, 0x06);
    lcd_cmd(addr, 0x01);
}
 
static void lcd_set_cursor(uint8_t addr, uint8_t col, uint8_t row)
{
    static const uint8_t row_addr[2] = {0x00, 0x40};
    lcd_cmd(addr, (uint8_t)(0x80 | (row_addr[row] + col)));
}
 
static void lcd_write_line(uint8_t addr, uint8_t row, const char *s)
{
    lcd_set_cursor(addr, 0, row);
 
    uint8_t i = 0;
 
    while (s[i] && i < 16) {
        lcd_data(addr, (uint8_t)s[i]);
        i++;
    }
 
    while (i < 16) {
        lcd_data(addr, ' ');
        i++;
    }
}
 
/* =========================================================
  ADC + joystick / buttons
  ========================================================= */
 
static void input_init(void)
{
    DDRD  &= ~((1 << JOY1_SW_PIN) | (1 << JOY2_SW_PIN) | (1 << RESTART_PIN));
    PORTD |=  (1 << JOY1_SW_PIN)  | (1 << JOY2_SW_PIN)  | (1 << RESTART_PIN);
    ADMUX  = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    DIDR0  = (1 << ADC0D) | (1 << ADC2D);
}
 
static uint8_t joy1_button_pressed(void)
{
    return (uint8_t)(!(PIND & (1 << JOY1_SW_PIN)));
}
 
static uint8_t joy2_button_pressed(void)
{
    return (uint8_t)(!(PIND & (1 << JOY2_SW_PIN)));
}
 
static uint8_t both_buttons_pressed(void)
{
    return (uint8_t)(joy1_button_pressed() && joy2_button_pressed());
}
 
static uint8_t restart_button_pressed(void)
{
    return (uint8_t)(!(PIND & (1 << RESTART_PIN)));
}
 
static uint16_t adc_once(uint8_t ch)
{
    ADMUX = (uint8_t)((1 << REFS0) | (ch & 0x0F));
    _delay_us(ADC_SETTLE_US);
 
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {
        ;
    }
 
    return ADC;
}
 
static uint16_t adc_read_filtered(uint8_t ch)
{
    uint32_t sum = 0;
 
    adc_once(ch);
    adc_once(ch);
 
    for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
        sum += adc_once(ch);
    }
 
    return (uint16_t)(sum / ADC_SAMPLES);
}
 
static void calibrate_joysticks(void)
{
    uint32_t s1 = 0;
    uint32_t s2 = 0;
 
    for (uint8_t i = 0; i < 24; i++) {
        s1 += adc_read_filtered(JOY1_ADC_CH);
        s2 += adc_read_filtered(JOY2_ADC_CH);
        _delay_ms(5);
    }
 
    joy1_center = (uint16_t)(s1 / 24);
    joy2_center = (uint16_t)(s2 / 24);
 
    /* FIX: Widened limits to capture resting drift safely */
    if (joy1_center < 100 || joy1_center > 923) joy1_center = 512;
    if (joy2_center < 100 || joy2_center > 923) joy2_center = 512;
}
 
static int8_t joystick_dir(uint16_t raw, uint16_t center)
{
    int16_t d = (int16_t)raw - (int16_t)center;
 
    if (d < -(int16_t)JOY_DEADZONE) return -1;
    if (d >  (int16_t)JOY_DEADZONE) return  1;
 
    return 0;
}
 
static void step_paddle(uint8_t *p, int8_t dir)
{
    if (dir < 0 && *p > 0) {
        (*p)--;
    }
 
    if (dir > 0 && *p < (uint8_t)(16 - PADDLE_H)) {
        (*p)++;
    }
}
 
static void update_paddles(void)
{
    uint16_t j1 = adc_read_filtered(JOY1_ADC_CH);
    uint16_t j2 = adc_read_filtered(JOY2_ADC_CH);
    if (JOY1_INVERT) j1 = (uint16_t)(1023 - j1);
    if (JOY2_INVERT) j2 = (uint16_t)(1023 - j2);
    step_paddle(&paddle1_y, joystick_dir(j1, joy1_center));
    step_paddle(&paddle2_y, joystick_dir(j2, joy2_center));
}
 
/* =========================================================
  Ball serve and bounce
  ========================================================= */
 
static void serve_ball(int8_t dx)
{
    ball_x  = 7;
    ball_y  = 7;
    ball_dx = dx;
 
    /* Straight serve only at the start of a point */
    ball_dy = 0;
}
 
static int8_t paddle_bounce_dy(int8_t hit)
{
    /* Paddle height is 4:
      hit 0,1 = top half -> up
      hit 2,3 = bottom half -> down
      This prevents the ball from staying straight after paddle hits.
    */
    if (hit <= 1) {
        return -1;
    }
    return 1;
}
 
/* =========================================================
  Tournament helpers
  ========================================================= */
 
static void reset_positions(void)
{
    paddle1_y = 6;
    paddle2_y = 6;
    ball_x    = 7;
    ball_y    = 7;
    ball_dy   = 0;
    flashing  = 0;
}
 
static void load_match(uint8_t match_idx)
{
    current_match = match_idx;
 
    if (match_idx == 0) {
        left_player = 0;
        right_player = 1;
    }
    else if (match_idx == 1) {
        left_player = 2;
        right_player = 3;
    }
    else {
        left_player = semi_winner[0];
        right_player = semi_winner[1];
    }
    score_left       = 0;
    score_right      = 0;
    both_press_start = 0;
    next_serve_dx    = 1;
 
    reset_positions();
 
    state          = ST_WAIT_BOTH;
    state_start_ms = millis_now();
}
 
static void full_reset(void)
{
    semi_winner[0] = 255;
    semi_winner[1] = 255;
    champion       = 255;
 
    load_match(0);
}
 
/* =========================================================
  Ball physics
  ========================================================= */
 
static int8_t update_ball_once(void)
{
    int8_t nx = (int8_t)(ball_x + ball_dx);
    int8_t ny = (int8_t)(ball_y + ball_dy);
 
    if (ny < 0)  {
        ny = 0;
        ball_dy = 1;
    }
 
    if (ny > 15) {
        ny = 15;
        ball_dy = -1;
    }
 
    /* FIX: left paddle (nx <= 0 instead of nx <= 1) */
    if (ball_dx < 0 && nx <= 0) {
        if (ny >= (int8_t)paddle1_y &&
            ny <  (int8_t)(paddle1_y + PADDLE_H)) {
            ball_dx = 1;
 
            /* After a paddle hit, ball_dy is never 0 */
            int8_t hit = (int8_t)(ny - (int8_t)paddle1_y);
            ball_dy = paddle_bounce_dy(hit);
            nx = (int8_t)(ball_x + ball_dx);
            ny = (int8_t)(ball_y + ball_dy);
 
            if (ny < 0)  {
                ny = 0;
                ball_dy = 1;
            }
 
            if (ny > 15) {
                ny = 15;
                ball_dy = -1;
            }
        }
        else if (nx < 0) {
            return 1;
        }
    }
 
    /* FIX: right paddle (nx >= 15 instead of nx >= 14) */
    if (ball_dx > 0 && nx >= 15) {
        if (ny >= (int8_t)paddle2_y &&
            ny <  (int8_t)(paddle2_y + PADDLE_H)) {
            ball_dx = -1;
 
            /* After a paddle hit, ball_dy is never 0 */
            int8_t hit = (int8_t)(ny - (int8_t)paddle2_y);
            ball_dy = paddle_bounce_dy(hit);
            nx = (int8_t)(ball_x + ball_dx);
            ny = (int8_t)(ball_y + ball_dy);
 
            if (ny < 0)  {
                ny = 0;
                ball_dy = 1;
            }
 
            if (ny > 15) {
                ny = 15;
                ball_dy = -1;
            }
        }
        else if (nx > 15) {
            return 0;
        }
    }
 
    ball_x = nx;
    ball_y = ny;
 
    return -1;
}
 
/* =========================================================
  LCD helpers
  ========================================================= */
 
static void score_line(char *buf)
{
    for (uint8_t i = 0; i < 16; i++) {
        buf[i] = ' ';
    }
 
    buf[16] = 0;
 
    buf[0] = player_tag[left_player][0];
 
    if (player_tag[left_player][1]) {
        buf[1] = player_tag[left_player][1];
    }
 
    buf[3] = (char)('0' + (score_left  / 10));
    buf[4] = (char)('0' + (score_left  % 10));
    buf[5] = '-';
    buf[6] = (char)('0' + (score_right / 10));
    buf[7] = (char)('0' + (score_right % 10));
 
    buf[10] = player_tag[right_player][0];
 
    if (player_tag[right_player][1]) {
        buf[11] = player_tag[right_player][1];
    }
}
 
static void bracket_tag_or_dash(char *dst, uint8_t idx)
{
    if (idx == 255) {
        dst[0] = '-';
        dst[1] = '-';
        dst[2] = 0;
    }
    else {
        dst[0] = player_tag[idx][0];
        dst[1] = player_tag[idx][1] ? player_tag[idx][1] : ' ';
        dst[2] = 0;
    }
}
 
static void update_info_lcd(void)
{
    char line1[17];
    char line2[17];
 
    score_line(line1);
 
    for (uint8_t i = 0; i < 16; i++) {
        line2[i] = ' ';
    }
 
    line2[16] = 0;
 
    if (state == ST_WAIT_BOTH) {
        strcpy(line2, "Press both SW");
    }
    else if (state == ST_PLAY) {
        strcpy(line2, "Race to 5");
    }
    else if (state == ST_SCORE_FLASH) {
        strcpy(line2, "Point scored!");
    }
    else if (state == ST_POINT_PAUSE) {
        strcpy(line2, "Point scored!");
    }
    else if (state == ST_MATCH_OVER) {
        strcpy(line2, "Match winner!");
    }
    else {
        strcpy(line1, "Champion");
        strcpy(line2, player_name[champion]);
    }
 
    lcd_write_line(LCD_INFO_ADDR, 0, line1);
    lcd_write_line(LCD_INFO_ADDR, 1, line2);
}
 
static void update_bracket_lcd(void)
{
    char line1[17];
    char line2[17];
    char a[3];
    char b[3];
    char w[3];
 
    uint8_t view = (uint8_t)((millis_now() / 2200UL) % 3UL);
    if (view == 0) {
        bracket_tag_or_dash(a, 0);
        bracket_tag_or_dash(b, 1);
        bracket_tag_or_dash(w, semi_winner[0]);
 
        memset(line1, ' ', 16);
        line1[16] = 0;
 
        memset(line2, ' ', 16);
        line2[16] = 0;
 
        strcpy(line1, "SF1 ");
        strcat(line1, a);
        strcat(line1, " vs ");
        strcat(line1, b);
 
        strcpy(line2, "WIN: ");
        strcat(line2, w);
    }
    else if (view == 1) {
        bracket_tag_or_dash(a, 2);
        bracket_tag_or_dash(b, 3);
        bracket_tag_or_dash(w, semi_winner[1]);
 
        memset(line1, ' ', 16);
        line1[16] = 0;
 
        memset(line2, ' ', 16);
        line2[16] = 0;
 
        strcpy(line1, "SF2 ");
        strcat(line1, a);
        strcat(line1, " vs ");
        strcat(line1, b);
 
        strcpy(line2, "WIN: ");
        strcat(line2, w);
    }
    else {
        bracket_tag_or_dash(a, semi_winner[0]);
        bracket_tag_or_dash(b, semi_winner[1]);
        bracket_tag_or_dash(w, champion);
 
        memset(line1, ' ', 16);
        line1[16] = 0;
 
        memset(line2, ' ', 16);
        line2[16] = 0;
 
        strcpy(line1, "FINAL ");
        strcat(line1, a);
        strcat(line1, "-");
        strcat(line1, b);
 
        strcpy(line2, "CHAMP: ");
        strcat(line2, w);
    }
 
    lcd_write_line(LCD_BRACKET_ADDR, 0, line1);
    lcd_write_line(LCD_BRACKET_ADDR, 1, line2);
}
 
/* =========================================================
  Main
  ========================================================= */
 
int main(void)
{
    cli();
 
    timer0_init_1ms();
    spi_init();
    matrix_init();
    twi_init();
    lcd_init_one(LCD_BRACKET_ADDR);
    lcd_init_one(LCD_INFO_ADDR);
    input_init();
 
    sei();
    lcd_write_line(LCD_INFO_ADDR,    0, "Center sticks");
    lcd_write_line(LCD_INFO_ADDR,    1, "Booting...");
    lcd_write_line(LCD_BRACKET_ADDR, 0, "  Pong Arcade ");
    lcd_write_line(LCD_BRACKET_ADDR, 1, " Tournament!  ");
 
    calibrate_joysticks();
    semi_winner[0] = 255;
    semi_winner[1] = 255;
    champion       = 255;
    load_match(0);
    update_bracket_lcd();
    update_info_lcd();
 
    while (1) {
        uint32_t now = millis_now();
 
        if (restart_button_pressed()) {
            _delay_ms(50);
 
            if (restart_button_pressed()) {
                full_reset();
                update_bracket_lcd();
                update_info_lcd();
 
                while (restart_button_pressed()) {
                    ;
                }
            }
        }
 
        if (state != ST_TOURNAMENT_OVER) {
            if ((now - last_paddle_ms) >= PADDLE_STEP_MS) {
                last_paddle_ms = now;
                update_paddles();
            }
 
            if (state == ST_WAIT_BOTH) {
                if (both_buttons_pressed()) {
                    if (!both_press_start) {
                        both_press_start = now;
                    }
                    else if ((now - both_press_start) >= BOTH_PRESS_MS) {
                        serve_ball(next_serve_dx);
                        state            = ST_PLAY;
                        state_start_ms   = now;
                        both_press_start = 0;
                    }
                }
                else {
                    both_press_start = 0;
                }
            }
            else if (state == ST_PLAY) {
                if ((now - last_ball_ms) >= BALL_STEP_MS) {
                    last_ball_ms = now;
                    int8_t scorer = update_ball_once();
 
                    if (scorer == 0) {
                        score_left++;
                        next_serve_dx = 1;
                        flash_x  = 15;
                        flash_y  = ball_y;
                        flashing = 1;
                        update_info_lcd();
 
                        if (score_left >= WIN_SCORE) {
                            uint8_t winner = left_player;
 
                            if (current_match < 2) {
                                semi_winner[current_match] = winner;
                                state = ST_MATCH_OVER;
                            }
                            else {
                                champion = winner;
                                state = ST_TOURNAMENT_OVER;
                                flashing = 0;
                            }
                        }
                        else {
                            state = ST_SCORE_FLASH;
                        }
 
                        state_start_ms = now;
                    }
                    else if (scorer == 1) {
                        score_right++;
                        next_serve_dx = -1;
                        flash_x  = 0;
                        flash_y  = ball_y;
                        flashing = 1;
                        update_info_lcd();
 
                        if (score_right >= WIN_SCORE) {
                            uint8_t winner = right_player;
 
                            if (current_match < 2) {
                                semi_winner[current_match] = winner;
                                state = ST_MATCH_OVER;
                            }
                            else {
                                champion = winner;
                                state = ST_TOURNAMENT_OVER;
                                flashing = 0;
                            }
                        }
                        else {
                            state = ST_SCORE_FLASH;
                        }
 
                        state_start_ms = now;
                    }
                }
            }
            else if (state == ST_SCORE_FLASH) {
                if ((now - state_start_ms) >= SCORE_FLASH_MS) {
                    flashing       = 0;
                    state          = ST_POINT_PAUSE;
                    state_start_ms = now;
                }
            }
            else if (state == ST_POINT_PAUSE) {
                if ((now - state_start_ms) >= POINT_PAUSE_MS) {
                    serve_ball(next_serve_dx);
                    state          = ST_PLAY;
                    state_start_ms = now;
                }
            }
            else if (state == ST_MATCH_OVER) {
                flashing = 0;
 
                if ((now - state_start_ms) >= MATCH_WIN_PAUSE_MS) {
                    if (current_match == 0) {
                        load_match(1);
                    }
                    else if (current_match == 1) {
                        load_match(2);
                    }
                }
            }
            draw_game_screen();
        }
        else {
            draw_winner_screen(champion);
        }
 
        if ((now - last_info_lcd_ms) >= 120) {
            last_info_lcd_ms = now;
            update_info_lcd();
        }
 
        if ((now - last_bracket_lcd_ms) >= 250) {
            last_bracket_lcd_ms = now;
            update_bracket_lcd();
        }
    }
}