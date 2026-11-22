#include "MK64F12.h"

#include <stdint.h>

#include <stdbool.h>
 
static const uint8_t SEG_NUM[10] = { 0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90 };

static const uint8_t DIG_SEL[4]  = { 0xF1, 0xF2, 0xF4, 0xF8 };
 
#define SEG_BLK 0xFF

#define SEG_DSH 0xBF

#define SEG_TOP 0xFE  // Top bar (meteor high)

#define SEG_BOT 0xF7  // Bottom bar (meteor impact)

#define SEG_ALL 0xB6  // 3 bars (meteor danger!)

#define SEG_r   0xAF

#define SEG_E   0x86

#define SEG_S   0x92

#define SEG_I   0xF9

#define SEG_L   0xC7

#define SEG_G   0x82

#define SEG_O   0xC0

#define SEG_P   0x8C

#define SEG_A   0x88

#define SEG_y   0x91

#define SEG_d   0xA1

#define SEG_H   0x89

#define SEG_n   0xAB
 
// On-board FXOS8700CQ Accelerometer I2C Address

#define ACCEL_ADDR      0x1D

#define ACCEL_CTRL_REG1 0x2A
 
volatile uint32_t sys_ms = 0, tone_end_ms = 0;

volatile bool tone_active = false;

volatile uint8_t disp_buf[4] = { SEG_G, SEG_A, SEG_n, 0xF9 };

volatile uint8_t scan_idx = 0;
 
typedef enum {

    ST_MENU = 0,

    ST_CD,

    ST_R_WAIT,

    ST_R_GO,

    ST_R_RES,

    ST_S_SHOW,

    ST_S_IN,

    ST_S_RETRY,

    ST_S_NEXT,

    ST_S_OVER,

    ST_W_PLAY,

    ST_W_OVER,

    ST_C_PLAY,

    ST_C_PAUSE,

    ST_C_OVER,

    ST_T_PLAY,

    ST_T_HIT,

    ST_T_SCORE,

    ST_T_OVER

} State_t;
 
static State_t state = ST_MENU;

static uint8_t menu_idx = 0, cd_step = 0, s_lvl = 1, s_step = 0, s_lives = 3, s_seq[25];

static uint8_t w_target = 0, w_score = 0, w_lives = 3;

static uint8_t c_pos = 0, c_score = 0, c_lives = 3;

static int8_t  c_dir = 1;

static uint8_t t_lane = 1, t_step = 0, t_ship = 1, t_score = 0, t_lives = 3;

static uint32_t t_ms = 0, s_tmr = 0, r_dly = 0, w_mole_tmr = 0, w_idle_tmr = 0, c_tmr = 0, tilt_tmr = 0, seed = 12345;

static bool s_led = false;
 
void Beep(uint32_t ms) {

    if (!ms) {

        tone_active = false;

        PTA->PSOR = (1U << 1);

        PTA->PDDR &= ~(1U << 1);

        return;

    }

    PTA->PCOR = (1U << 1);

    PTA->PDDR |= (1U << 1);

    tone_end_ms = sys_ms + ms;

    tone_active = true;

}
 
void SetLEDs(bool d1, bool d2, bool d4) {

    if (d1) PTD->PCOR = (1U << 1); else PTD->PSOR = (1U << 1);

    if (d2) PTD->PCOR = (1U << 3); else PTD->PSOR = (1U << 3);

    if (d4) PTD->PCOR = (1U << 0); else PTD->PSOR = (1U << 0);

}
 
void SetDisp(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) {

    disp_buf[0] = d0; disp_buf[1] = d1; disp_buf[2] = d2; disp_buf[3] = d3;

}
 
// --- I2C0 Driver for On-Board FXOS8700CQ Accelerometer (PTE24=SCL, PTE25=SDA) ---

static void I2C_Wait(void) {

    uint32_t timeout = 10000;

    while (!(I2C0->S & I2C_S_IICIF_MASK) && --timeout);

    I2C0->S |= I2C_S_IICIF_MASK;

}
 
static void I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t val) {

    I2C0->C1 |= I2C_C1_TX_MASK;

    I2C0->C1 |= I2C_C1_MST_MASK;

    I2C0->D = (uint8_t)(addr << 1);

    I2C_Wait();

    I2C0->D = reg;

    I2C_Wait();

    I2C0->D = val;

    I2C_Wait();

    I2C0->C1 &= ~I2C_C1_MST_MASK;

    for (volatile int i = 0; i < 300; i++);

}
 
static int16_t I2C_ReadAxis16(uint8_t addr, uint8_t reg) {

    uint8_t msb = 0, lsb = 0;

    I2C0->C1 |= I2C_C1_TX_MASK;

    I2C0->C1 |= I2C_C1_MST_MASK;

    I2C0->D = (uint8_t)(addr << 1);

    I2C_Wait();

    I2C0->D = reg;

    I2C_Wait();
 
    I2C0->C1 |= I2C_C1_RSTA_MASK;

    I2C0->D = (uint8_t)((addr << 1) | 1U);

    I2C_Wait();
 
    I2C0->C1 &= ~I2C_C1_TX_MASK;

    I2C0->C1 &= ~I2C_C1_TXAK_MASK;

    (void)I2C0->D;

    I2C_Wait();
 
    I2C0->C1 |= I2C_C1_TXAK_MASK;

    msb = I2C0->D;

    I2C_Wait();
 
    I2C0->C1 &= ~I2C_C1_MST_MASK;

    lsb = I2C0->D;

    for (volatile int i = 0; i < 300; i++);
 
    return (int16_t)((msb << 8) | lsb) >> 2;

}
 
void Accel_Init(void) {

    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
 
    PORTE->PCR[24] = PORT_PCR_MUX(5) | PORT_PCR_ODE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    PORTE->PCR[25] = PORT_PCR_MUX(5) | PORT_PCR_ODE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
 
    I2C0->F  = 0x14;

    I2C0->C1 = I2C_C1_IICEN_MASK;
 
    I2C_WriteReg(ACCEL_ADDR, ACCEL_CTRL_REG1, 0x00);

    I2C_WriteReg(ACCEL_ADDR, ACCEL_CTRL_REG1, 0x01);

}
 
// Returns ship lane: 0 = Left (D1), 1 = Center (D2), 2 = Right (D4)

uint8_t ReadTiltPos(void) {

    int16_t x = I2C_ReadAxis16(ACCEL_ADDR, 0x01);

    int16_t y = I2C_ReadAxis16(ACCEL_ADDR, 0x03);

    int16_t tilt = (y > 350 || y < -350) ? y : x;
 
    if (tilt < -550) return 0; // Tilt Left  -> LED D1

    if (tilt >  550) return 2; // Tilt Right -> LED D4

    return 1;                  // Level      -> LED D2

}
 
uint8_t RawBtn(void) {

    uint8_t cur = 0;

    if (!(PTB->PDIR & (1U << 3)))  cur |= 1;

    if (!(PTB->PDIR & (1U << 10))) cur |= 2;

    if (!(PTB->PDIR & (1U << 11))) cur |= 4;

    return cur;

}
 
uint8_t ReadBtn(void) {

    static uint8_t prev = 0;

    static uint32_t deb = 0;

    if ((sys_ms - deb) < 20) return 0;

    uint8_t cur = RawBtn();

    uint8_t p = cur & ~prev;

    if (cur != prev) { deb = sys_ms; prev = cur; }

    return p;

}
 
uint32_t Rand(void) {

    seed = seed * 1664525U + 1013904223U + sys_ms;

    return seed;

}
 
void StartCD(void) {

    cd_step = 0;

    t_ms = sys_ms;

    SetLEDs(true, false, false);

    Beep(60);

    SetDisp(SEG_DSH, SEG_NUM[3], SEG_DSH, SEG_DSH);

    state = ST_CD;

}
 
void PIT0_IRQHandler(void) {

    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;

    sys_ms++;

    if (tone_active && sys_ms >= tone_end_ms) {

        tone_active = false;

        PTA->PSOR = (1U << 1);

        PTA->PDDR &= ~(1U << 1);

    }

}
 
void PIT2_IRQHandler(void) {

    PIT->CHANNEL[2].TFLG = PIT_TFLG_TIF_MASK;

    uint8_t d[2] = { disp_buf[scan_idx], DIG_SEL[scan_idx] };

    PTB->PCOR = (1U << 23);

    for (int k = 0; k < 2; k++) {

        for (int i = 7; i >= 0; i--) {

            if (d[k] & (1U << i)) PTC->PSOR = (1U << 12); else PTC->PCOR = (1U << 12);

            PTC->PSOR = (1U << 3); PTC->PCOR = (1U << 3);

        }

    }

    PTB->PSOR = (1U << 23);

    scan_idx = (scan_idx + 1) & 3;

}
 
int main(void) {

    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;

    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
 
    // Buzzer PTA1

    PORTA->PCR[1] = PORT_PCR_MUX(1);

    PTA->PSOR = (1U << 1); PTA->PDDR &= ~(1U << 1);
 
    // 7-Segment Shift Register

    PORTB->PCR[23] = PORT_PCR_MUX(1); PORTC->PCR[3] = PORT_PCR_MUX(1); PORTC->PCR[12] = PORT_PCR_MUX(1);

    PTB->PDDR |= (1U << 23); PTC->PDDR |= (1U << 3) | (1U << 12);
 
    // LEDs D1, D2, D4

    PORTD->PCR[1] = PORT_PCR_MUX(1); PORTD->PCR[3] = PORT_PCR_MUX(1); PORTD->PCR[0] = PORT_PCR_MUX(1);

    PTD->PSOR = (1U << 1) | (1U << 3) | (1U << 0); PTD->PDDR |= (1U << 1) | (1U << 3) | (1U << 0);
 
    // Buttons S1, S2, S3

    PORTB->PCR[3]  = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    PORTB->PCR[10] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    PORTB->PCR[11] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    PTB->PDDR &= ~((1U << 3) | (1U << 10) | (1U << 11));
 
    // Initialize On-Board I2C0 Accelerometer

    Accel_Init();
 
    PIT->MCR = 0;

    PIT->CHANNEL[0].LDVAL = 59999U;  PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

    PIT->CHANNEL[2].LDVAL = 149999U; PIT->CHANNEL[2].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

    NVIC_EnableIRQ(PIT0_IRQn); NVIC_EnableIRQ(PIT2_IRQn);
 
    SetLEDs(false, false, false);

    Beep(90);
 
    while (1) {

        uint8_t btn = ReadBtn();
 
        switch (state) {

            // --- Main Menu (GAM1..GAM5) ---

            case ST_MENU:

                if (menu_idx < 3) {

                    SetLEDs(menu_idx == 0, menu_idx == 1, menu_idx == 2);

                } else if (menu_idx == 3) {

                    SetLEDs(true, true, true);

                } else {

                    SetLEDs(true, false, true);

                }

                SetDisp(SEG_G, SEG_A, SEG_n, SEG_NUM[menu_idx + 1]);
 
                if (btn & 1) {

                    menu_idx = (menu_idx + 1) % 5;

                    Beep(45);

                } else if (btn & 4) {

                    StartCD();

                }

                break;
 
            // --- Game-show Countdown ---

            case ST_CD:

                if (cd_step == 0 && (sys_ms - t_ms) >= 400) {

                    cd_step = 1; t_ms = sys_ms;

                    SetLEDs(true, true, false);

                    Beep(130);

                    SetDisp(SEG_DSH, SEG_DSH, SEG_NUM[2], SEG_DSH);

                } else if (cd_step == 1 && (sys_ms - t_ms) >= 400) {

                    cd_step = 2; t_ms = sys_ms;

                    SetLEDs(true, true, true);

                    Beep(220);

                    SetDisp(SEG_DSH, SEG_DSH, SEG_DSH, SEG_NUM[1]);

                } else if (cd_step == 2 && (sys_ms - t_ms) >= 420) {

                    cd_step = 3; t_ms = sys_ms;

                    SetLEDs(false, false, false);

                    Beep(420);

                    SetDisp(SEG_G, SEG_O, SEG_DSH, SEG_DSH);

                } else if (cd_step == 3 && (sys_ms - t_ms) >= 480 && RawBtn() == 0) {

                    if (menu_idx == 0) {

                        SetDisp(SEG_DSH, SEG_DSH, SEG_DSH, SEG_DSH);

                        r_dly = 800 + (Rand() % 1400);

                        t_ms = sys_ms;

                        state = ST_R_WAIT;

                    } else if (menu_idx == 1) {

                        for (int i = 0; i < 25; i++) s_seq[i] = Rand() % 3;

                        s_lvl = 1; s_step = 0; s_lives = 3; s_led = false; s_tmr = sys_ms;

                        SetDisp(SEG_L, SEG_DSH, SEG_NUM[0], SEG_NUM[1]);

                        state = ST_S_SHOW;

                    } else if (menu_idx == 2) {

                        w_score = 0;

                        w_lives = 3;

                        w_target = Rand() % 3;

                        SetLEDs(w_target == 0, w_target == 1, w_target == 2);

                        w_mole_tmr = sys_ms;

                        w_idle_tmr = sys_ms;

                        SetDisp(SEG_NUM[3], SEG_P, SEG_NUM[0], SEG_NUM[0]);

                        state = ST_W_PLAY;

                    } else if (menu_idx == 3) {

                        c_score = 0;

                        c_lives = 3;

                        c_pos = 0;

                        c_dir = 1;

                        c_tmr = sys_ms;

                        SetLEDs(true, false, false);

                        SetDisp(SEG_NUM[3], SEG_P, SEG_NUM[0], SEG_NUM[0]);

                        state = ST_C_PLAY;

                    } else {

                        // Start Game 5: Visual Tilt Dodger

                        t_score = 0;

                        t_lives = 3;

                        t_lane = Rand() % 3;

                        t_step = 0;

                        tilt_tmr = sys_ms;

                        state = ST_T_PLAY;

                    }

                }

                break;
 
            // --- Game 1: Reaction Timer (GAM1) ---

            case ST_R_WAIT:

                if (btn) {

                    Beep(350);

                    SetDisp(SEG_E, SEG_r, SEG_r, SEG_BLK);

                    t_ms = sys_ms;

                    state = ST_R_RES;

                } else if ((sys_ms - t_ms) >= r_dly) {

                    SetLEDs(true, true, true);

                    Beep(70);

                    t_ms = sys_ms;

                    state = ST_R_GO;

                }

                break;
 
            case ST_R_GO: {

                uint32_t el = sys_ms - t_ms;

                if (el > 9999) el = 9999;

                SetDisp(SEG_NUM[(el/1000)%10], SEG_NUM[(el/100)%10], SEG_NUM[(el/10)%10], SEG_NUM[el%10]);
 
                if (btn || el >= 9999) {

                    SetLEDs(false, false, false);

                    Beep(140);

                    t_ms = sys_ms;

                    state = ST_R_RES;

                }

                break;

            }
 
            case ST_R_RES:

                if (btn & 4) state = ST_MENU;

                else if (btn & 1) StartCD();

                break;
 
            // --- Game 2: Simon Says (GAM2) ---

            case ST_S_SHOW:

                SetDisp(SEG_L, SEG_DSH, SEG_NUM[s_lvl/10], SEG_NUM[s_lvl%10]);
 
                if (!s_led && (sys_ms - s_tmr) >= 150) {

                    uint8_t idx = s_seq[s_step];

                    SetLEDs(idx == 0, idx == 1, idx == 2);

                    Beep(160);

                    s_led = true;

                    s_tmr = sys_ms;

                } else if (s_led && (sys_ms - s_tmr) >= 260) {

                    SetLEDs(false, false, false);

                    s_led = false;

                    s_step++;

                    s_tmr = sys_ms;

                    if (s_step >= s_lvl) {

                        s_step = 0;

                        SetDisp(SEG_P, SEG_L, SEG_A, SEG_y);

                        state = ST_S_IN;

                    }

                }

                break;
 
            case ST_S_IN: {

                int8_t p = (btn & 1) ? 0 : (btn & 2) ? 1 : (btn & 4) ? 2 : -1;
 
                if (p >= 0) {

                    SetLEDs(p == 0, p == 1, p == 2);

                    Beep(80);

                    t_ms = sys_ms;
 
                    if (p == s_seq[s_step]) {

                        s_step++;

                        if (s_step >= s_lvl) {

                            SetDisp(SEG_G, SEG_O, SEG_O, SEG_d);

                            Beep(200);

                            t_ms = sys_ms;

                            state = ST_S_NEXT;

                        }

                    } else {

                        SetLEDs(false, false, false);

                        if (s_lives > 0) s_lives--;

                        if (s_lives == 0) {

                            Beep(350);

                            SetDisp(SEG_L, SEG_O, SEG_S, SEG_E);

                            t_ms = sys_ms;

                            state = ST_S_OVER;

                        } else {

                            Beep(220);

                            SetDisp(SEG_NUM[s_lives], SEG_DSH, SEG_H, SEG_P);

                            t_ms = sys_ms;

                            state = ST_S_RETRY;

                        }

                    }

                } else if ((sys_ms - t_ms) >= 180) {

                    SetLEDs(false, false, false);

                }

                break;

            }
 
            case ST_S_RETRY:

                if ((sys_ms - t_ms) >= 600) {

                    s_step = 0;

                    s_led = false;

                    s_tmr = sys_ms;

                    state = ST_S_SHOW;

                }

                break;
 
            case ST_S_NEXT:

                if ((sys_ms - t_ms) >= 450) {

                    SetLEDs(false, false, false);

                    if (++s_lvl > 25) s_lvl = 25;

                    s_step = 0; s_led = false; s_tmr = sys_ms;

                    state = ST_S_SHOW;

                }

                break;
 
            case ST_S_OVER:

                if (btn & 1) StartCD();

                else if ((btn & 4) || (sys_ms - t_ms) >= 450) state = ST_MENU;

                break;
 
            // --- Game 3: Whack-a-Mole (GAM3) ---

            case ST_W_PLAY: {

                if ((sys_ms - w_idle_tmr) >= 5000) {

                    SetLEDs(false, false, false);

                    Beep(350);

                    SetDisp(SEG_L, SEG_O, SEG_S, SEG_E);

                    t_ms = sys_ms;

                    state = ST_W_OVER;

                    break;

                }
 
                uint32_t speed_ms = (w_score < 15) ? (950 - w_score * 30) : 500;

                int8_t p = (btn & 1) ? 0 : (btn & 2) ? 1 : (btn & 4) ? 2 : -1;

                bool missed = false;
 
                if (p >= 0) {

                    w_idle_tmr = sys_ms;

                    if (p == w_target) {

                        if (w_score < 99) w_score++;

                        Beep(55);

                        uint8_t nxt = Rand() % 3;

                        if (nxt == w_target) nxt = (w_target + 1) % 3;

                        w_target = nxt;

                        SetLEDs(w_target == 0, w_target == 1, w_target == 2);

                        w_mole_tmr = sys_ms;

                    } else {

                        missed = true;

                    }

                } else if ((sys_ms - w_mole_tmr) >= speed_ms) {

                    missed = true;

                }
 
                if (missed) {

                    if (w_lives > 0) w_lives--;

                    if (w_lives == 0) {

                        SetLEDs(false, false, false);

                        Beep(350);

                        SetDisp(SEG_L, SEG_O, SEG_S, SEG_E);

                        t_ms = sys_ms;

                        state = ST_W_OVER;

                        break;

                    } else {

                        Beep(140);

                        uint8_t nxt = Rand() % 3;

                        if (nxt == w_target) nxt = (w_target + 1) % 3;

                        w_target = nxt;

                        SetLEDs(w_target == 0, w_target == 1, w_target == 2);

                        w_mole_tmr = sys_ms;

                    }

                }
 
                SetDisp(SEG_NUM[w_lives], SEG_P, SEG_NUM[(w_score/10)%10], SEG_NUM[w_score%10]);

                break;

            }
 
            case ST_W_OVER:

                if (btn & 1) StartCD();

                else if ((btn & 4) || (sys_ms - t_ms) >= 450) state = ST_MENU;

                break;
 
            // --- Game 4: Cyclone Stop (GAM4) ---

            case ST_C_PLAY: {

                uint32_t step_ms = (c_score < 16) ? (220 - c_score * 10) : 60;
 
                if ((sys_ms - c_tmr) >= step_ms) {

                    c_tmr = sys_ms;

                    if (c_pos == 0) c_dir = 1;

                    else if (c_pos == 2) c_dir = -1;

                    c_pos = (uint8_t)(c_pos + c_dir);

                    SetLEDs(c_pos == 0, c_pos == 1, c_pos == 2);

                    Beep(20);

                }
 
                if (btn) {

                    if (c_pos == 1) {

                        if (c_score < 99) c_score++;

                        Beep(160);

                        SetDisp(SEG_G, SEG_O, SEG_O, SEG_d);

                        t_ms = sys_ms;

                        state = ST_C_PAUSE;

                    } else {

                        if (c_lives > 0) c_lives--;

                        if (c_lives == 0) {

                            SetLEDs(false, false, false);

                            Beep(350);

                            SetDisp(SEG_L, SEG_O, SEG_S, SEG_E);

                            t_ms = sys_ms;

                            state = ST_C_OVER;

                        } else {

                            Beep(200);

                            SetDisp(SEG_NUM[c_lives], SEG_DSH, SEG_H, SEG_P);

                            t_ms = sys_ms;

                            state = ST_C_PAUSE;

                        }

                    }

                } else {

                    SetDisp(SEG_NUM[c_lives], SEG_P, SEG_NUM[(c_score/10)%10], SEG_NUM[c_score%10]);

                }

                break;

            }
 
            case ST_C_PAUSE:

                if ((sys_ms - t_ms) >= 450) {

                    c_pos = (Rand() & 1) ? 0 : 2;

                    c_dir = (c_pos == 0) ? 1 : -1;

                    SetLEDs(c_pos == 0, false, c_pos == 2);

                    c_tmr = sys_ms;

                    state = ST_C_PLAY;

                }

                break;
 
            case ST_C_OVER:

                if (btn & 1) StartCD();

                else if ((btn & 4) || (sys_ms - t_ms) >= 450) state = ST_MENU;

                break;
 
            // --- Game 5: Visual Tilt Dodger (GAM5) ---

            case ST_T_PLAY: {

                t_ship = ReadTiltPos();

                SetLEDs(t_ship == 0, t_ship == 1, t_ship == 2);
 
                uint8_t meteor_glyph = (t_step == 0) ? SEG_TOP : (t_step == 1) ? SEG_DSH : SEG_ALL;

                uint8_t lane0 = (t_lane == 0) ? meteor_glyph : SEG_BLK;

                uint8_t lane1 = (t_lane == 1) ? meteor_glyph : SEG_BLK;

                uint8_t lane2 = (t_lane == 2) ? meteor_glyph : SEG_BLK;

                SetDisp(lane0, lane1, lane2, SEG_NUM[t_lives]);
 
                // Faster meteor speed (380ms per step down to 160ms)

                uint32_t step_ms = (t_score < 22) ? (380 - t_score * 10) : 160;
 
                if ((sys_ms - tilt_tmr) >= step_ms) {

                    tilt_tmr = sys_ms;

                    if (t_step < 2) {

                        t_step++;

                    } else {

                        if (t_ship == t_lane) {

                            if (t_lives > 0) t_lives--;

                            if (t_lives == 0) {

                                SetLEDs(false, false, false);

                                Beep(350);

                                SetDisp(SEG_L, SEG_O, SEG_S, SEG_E);

                                t_ms = sys_ms;

                                state = ST_T_SCORE;

                            } else {

                                Beep(220);

                                SetDisp(SEG_NUM[t_lives], SEG_DSH, SEG_H, SEG_P);

                                t_ms = sys_ms;

                                state = ST_T_HIT;

                            }

                        } else {

                            if (t_score < 99) t_score++;

                            Beep(45);

                            t_step = 0;

                            uint8_t nxt = Rand() % 3;

                            if (nxt == t_lane) nxt = (t_lane + 1) % 3;

                            t_lane = nxt;

                        }

                    }

                }

                break;

            }
 
            case ST_T_HIT:

                if ((sys_ms - t_ms) >= 650) {

                    t_step = 0;

                    t_lane = Rand() % 3;

                    tilt_tmr = sys_ms;

                    state = ST_T_PLAY;

                }

                break;
 
            case ST_T_SCORE:

                if ((sys_ms - t_ms) >= 850) {

                    SetDisp(SEG_P, SEG_DSH, SEG_NUM[(t_score/10)%10], SEG_NUM[t_score%10]);

                    t_ms = sys_ms;

                    state = ST_T_OVER;

                }

                break;
 
            case ST_T_OVER:

                if (btn & 1) StartCD();

                else if ((btn & 4) || (sys_ms - t_ms) >= 1400) state = ST_MENU;

                break;

        }

    }

}
 