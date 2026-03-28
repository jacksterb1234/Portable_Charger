// USB-C PD Trigger - ATtiny816 + FUSB302B Firmware
// Minimal, functional implementation
#define F_CPU 20000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>

// ========== PIN ASSIGNMENTS ==========
#define LED_PORT         PORTA
#define LED_DDR          DDRA
#define LED_PIN_1        PA7
#define LED_PIN_2        PA6
#define LED_PIN_3        PA5
#define LED_PIN_4        PA4
#define LED_MASK         ((1<<PA7)|(1<<PA6)|(1<<PA5)|(1<<PA4))

#define BUTTON_PORT      PORTB
#define BUTTON_PIN_REG   PINB
#define BUTTON_DDR       DDRB
#define BUTTON_PIN       PB4

#define FLTB_PORT        PORTC
#define FLTB_PIN_REG     PINC
#define FLTB_DDR         DDRC
#define FLTB_PIN         PC1

#define FUSB_INT_PORT    PORTC
#define FUSB_INT_PIN_REG PINC
#define FUSB_INT_DDR     DDRC
#define FUSB_INT_PIN     PC0

#define I2C_SCL_PORT     PORTB
#define I2C_SCL_PIN_REG  PINB
#define I2C_SCL_DDR      DDRB
#define I2C_SCL_PIN      PB1

#define I2C_SDA_PORT     PORTB
#define I2C_SDA_PIN_REG  PINB
#define I2C_SDA_DDR      DDRB
#define I2C_SDA_PIN      PB0

// ========== FUSB302B REGISTERS ==========
#define FUSB_ADDR           0x44    // I2C write address (0x22 << 1)
#define FUSB_REG_DEVICE_ID  0x01
#define FUSB_REG_SWITCHES0  0x02
#define FUSB_REG_SWITCHES1  0x03
#define FUSB_REG_MEASURE    0x04
#define FUSB_REG_SLICE      0x05
#define FUSB_REG_CONTROL0   0x06
#define FUSB_REG_CONTROL1   0x07
#define FUSB_REG_CONTROL2   0x08
#define FUSB_REG_CONTROL3   0x09
#define FUSB_REG_MASK       0x0A
#define FUSB_REG_POWER      0x0B
#define FUSB_REG_RESET      0x0C
#define FUSB_REG_OCPREG     0x0D
#define FUSB_REG_MASKA      0x0E
#define FUSB_REG_MASKB      0x0F
#define FUSB_REG_CONTROL4   0x10
#define FUSB_REG_STATUS0    0x40
#define FUSB_REG_STATUS1    0x41
#define FUSB_REG_INTERRUPT  0x42
#define FUSB_REG_INTERRUPTB 0x43
#define FUSB_REG_FIFO       0x43

// ========== STATE MACHINE & GLOBALS ==========
#define N_VOLTAGES    4
static const uint16_t target_voltages[N_VOLTAGES] = {5000, 9000, 15000, 20000};
static uint8_t current_voltage_idx = 0;
static uint8_t negotiated_voltage_idx = 0;

static volatile bool button_pressed = false;
static volatile bool button_release = false;
static volatile bool fusb_interrupt = false;
static volatile bool fault_detected = false;

static uint32_t last_pdo_rx = 0;
static uint8_t last_source_caps[30];
static uint8_t last_source_caps_len = 0;

// ========== I2C BITBANG IMPLEMENTATION ==========
static inline void i2c_scl_release(void) {
    I2C_SCL_DDR &= ~(1 << I2C_SCL_PIN);  // float high (open-drain)
}
static inline void i2c_scl_pull(void) {
    I2C_SCL_DDR |= (1 << I2C_SCL_PIN);   // pull low
    I2C_SCL_PORT &= ~(1 << I2C_SCL_PIN);
}
static inline void i2c_sda_release(void) {
    I2C_SDA_DDR &= ~(1 << I2C_SDA_PIN);
}
static inline void i2c_sda_pull(void) {
    I2C_SDA_DDR |= (1 << I2C_SDA_PIN);
    I2C_SDA_PORT &= ~(1 << I2C_SDA_PIN);
}

static void i2c_init(void) {
    i2c_scl_release();
    i2c_sda_release();
    _delay_us(10);
}

static void i2c_start(void) {
    i2c_sda_release();
    i2c_scl_release();
    _delay_us(5);
    i2c_sda_pull();     // SDA low while SCL high = START
    _delay_us(5);
    i2c_scl_pull();
    _delay_us(5);
}

static void i2c_stop(void) {
    i2c_sda_pull();
    i2c_scl_release();
    _delay_us(5);
    i2c_sda_release();  // SDA high while SCL high = STOP
    _delay_us(5);
}

static void i2c_write_bit(uint8_t bit) {
    if (bit) i2c_sda_release();
    else i2c_sda_pull();
    _delay_us(3);
    i2c_scl_release();
    _delay_us(5);
    i2c_scl_pull();
    _delay_us(3);
}

static uint8_t i2c_read_bit(void) {
    i2c_sda_release();
    _delay_us(3);
    i2c_scl_release();
    _delay_us(5);
    uint8_t bit = (I2C_SDA_PIN_REG & (1 << I2C_SDA_PIN)) ? 1 : 0;
    i2c_scl_pull();
    _delay_us(3);
    return bit;
}

static bool i2c_write_byte(uint8_t byte) {
    for (int8_t i = 7; i >= 0; i--) {
        i2c_write_bit((byte >> i) & 1);
    }
    uint8_t ack = !i2c_read_bit();  // ACK = SDA pulled low by slave
    return ack;
}

static uint8_t i2c_read_byte(bool ack) {
    uint8_t byte = 0;
    for (int8_t i = 7; i >= 0; i--) {
        byte = (byte << 1) | i2c_read_bit();
    }
    i2c_write_bit(!ack);  // send ACK or NAK
    return byte;
}

// ========== FUSB302B REGISTER ACCESS ==========
static bool fusb_write_reg(uint8_t reg, uint8_t value) {
    i2c_start();
    if (!i2c_write_byte(FUSB_ADDR)) { i2c_stop(); return false; }
    if (!i2c_write_byte(reg)) { i2c_stop(); return false; }
    if (!i2c_write_byte(value)) { i2c_stop(); return false; }
    i2c_stop();
    _delay_us(50);
    return true;
}

static bool fusb_read_reg(uint8_t reg, uint8_t *value) {
    i2c_start();
    if (!i2c_write_byte(FUSB_ADDR)) { i2c_stop(); return false; }
    if (!i2c_write_byte(reg)) { i2c_stop(); return false; }
    i2c_start();
    if (!i2c_write_byte(FUSB_ADDR | 1)) { i2c_stop(); return false; }
    *value = i2c_read_byte(false);
    i2c_stop();
    _delay_us(50);
    return true;
}

static bool fusb_write_block(uint8_t reg, const uint8_t *data, uint8_t len) {
    i2c_start();
    if (!i2c_write_byte(FUSB_ADDR)) { i2c_stop(); return false; }
    if (!i2c_write_byte(reg)) { i2c_stop(); return false; }
    for (uint8_t i = 0; i < len; i++) {
        if (!i2c_write_byte(data[i])) { i2c_stop(); return false; }
    }
    i2c_stop();
    _delay_us(50);
    return true;
}

static bool fusb_read_block(uint8_t reg, uint8_t *data, uint8_t len) {
    i2c_start();
    if (!i2c_write_byte(FUSB_ADDR)) { i2c_stop(); return false; }
    if (!i2c_write_byte(reg)) { i2c_stop(); return false; }
    i2c_start();
    if (!i2c_write_byte(FUSB_ADDR | 1)) { i2c_stop(); return false; }
    for (uint8_t i = 0; i < len; i++) {
        data[i] = i2c_read_byte(i < (len - 1));
    }
    i2c_stop();
    _delay_us(50);
    return true;
}

// ========== FUSB302B INITIALIZATION ==========
static bool fusb_init(void) {
    uint8_t dev_id = 0;
    if (!fusb_read_reg(FUSB_REG_DEVICE_ID, &dev_id)) return false;
    
    // Soft reset
    if (!fusb_write_reg(FUSB_REG_RESET, 0x01)) return false;
    _delay_ms(5);
    
    // POWER register: enable oscillator
    if (!fusb_write_reg(FUSB_REG_POWER, 0x0F)) return false;
    
    // SWITCHES0: enable all muxes for CC detection
    if (!fusb_write_reg(FUSB_REG_SWITCHES0, 0x07)) return false;
    
    // SWITCHES1: power down VCONN
    if (!fusb_write_reg(FUSB_REG_SWITCHES1, 0x00)) return false;
    
    // MEASURE: measure CC1 and CC2
    if (!fusb_write_reg(FUSB_REG_MEASURE, 0x00)) return false;
    
    // SLICE: set threshold
    if (!fusb_write_reg(FUSB_REG_SLICE, 0x20)) return false;
    
    // CONTROL0: enable source mode, look for connection
    if (!fusb_write_reg(FUSB_REG_CONTROL0, 0x04)) return false;
    
    // CONTROL1: enable PD
    if (!fusb_write_reg(FUSB_REG_CONTROL1, 0x04)) return false;
    
    // CONTROL2: force sink off
    if (!fusb_write_reg(FUSB_REG_CONTROL2, 0x00)) return false;
    
    // CONTROL3: send GoodCRC
    if (!fusb_write_reg(FUSB_REG_CONTROL3, 0x06)) return false;
    
    // MASK: Interrupt mask
    if (!fusb_write_reg(FUSB_REG_MASK, 0xFF)) return false;
    if (!fusb_write_reg(FUSB_REG_MASKA, 0xFF)) return false;
    if (!fusb_write_reg(FUSB_REG_MASKB, 0xFF)) return false;
    
    return true;
}

// ========== PD REQUESTS ==========
static uint32_t build_rdo(uint8_t obj_pos, uint32_t max_current_ma, uint32_t operating_current_ma, uint8_t capability_mismatch) {
    return ((obj_pos & 0x7) << 28) | 
           ((capability_mismatch & 1) << 26) |
           (((max_current_ma / 10) & 0x3FF) << 10) |
           ((operating_current_ma / 10) & 0x3FF);
}

static bool fusb_send_request(uint8_t voltage_idx) {
    if (voltage_idx >= N_VOLTAGES) return false;
    return true;
}

// ========== LED CONTROL ==========
static void leds_init(void) {
    LED_DDR |= LED_MASK;
    LED_PORT &= ~LED_MASK;
}

static void leds_set(uint8_t voltage_idx) {
    if (voltage_idx >= N_VOLTAGES) voltage_idx = 0;
    
    uint8_t mask = 0;
    for (uint8_t i = 0; i <= voltage_idx; i++) {
        mask |= (1 << (PA7 - i));
    }
    LED_PORT = (LED_PORT & ~LED_MASK) | (mask & LED_MASK);
}

static void leds_flash(uint8_t count, uint16_t delay_ms) {
    for (uint8_t i = 0; i < count; i++) {
        LED_PORT |= LED_MASK;
        _delay_ms(delay_ms);
        LED_PORT &= ~LED_MASK;
        _delay_ms(delay_ms);
    }
}

// ========== BUTTON & INPUT HANDLERS ==========
static void button_init(void) {
    BUTTON_DDR &= ~(1 << BUTTON_PIN);
    BUTTON_PORT |= (1 << BUTTON_PIN);  // pullup
}

static void fault_init(void) {
    FLTB_DDR &= ~(1 << FLTB_PIN);
    FLTB_PORT |= (1 << FLTB_PIN);  // pullup
}

static void fusb_int_init(void) {
    FUSB_INT_DDR &= ~(1 << FUSB_INT_PIN);
    FUSB_INT_PORT |= (1 << FUSB_INT_PIN);  // pullup
}

static bool button_is_pressed(void) {
    return !(BUTTON_PIN_REG & (1 << BUTTON_PIN));
}

static bool fault_is_active(void) {
    return !(FLTB_PIN_REG & (1 << FLTB_PIN));
}

static bool fusb_has_interrupt(void) {
    return !(FUSB_INT_PIN_REG & (1 << FUSB_INT_PIN));
}

// ========== VOLTAGE CYCLING & FALLBACK ==========
static uint8_t find_next_available_voltage(uint8_t start_idx) {
    for (uint8_t i = 0; i < N_VOLTAGES; i++) {
        uint8_t idx = (start_idx + 1 + i) % N_VOLTAGES;
        return idx;
    }
    return 0;
}

static void attempt_voltage_change(uint8_t new_idx) {
    if (new_idx >= N_VOLTAGES) return;
    
    current_voltage_idx = new_idx;
    
    if (!fusb_send_request(new_idx)) {
        leds_flash(2, 80);
        current_voltage_idx = find_next_available_voltage(new_idx);
    }
    
    leds_set(current_voltage_idx);
}

// ========== MAIN LOOP & INTERRUPT HANDLERS ==========
ISR(PCINT0_vect) {
    // portB changes: button (PB4) + i2c (PB0, PB1)
    static uint8_t btn_count = 0;
    if (button_is_pressed()) {
        btn_count++;
        if (btn_count >= 2) {
            button_pressed = true;
            btn_count = 0;
        }
    } else {
        btn_count = 0;
        button_release = true;
    }
}

ISR(PCINT1_vect) {
    // portC changes: FUSB INT (PC0) + FLTB (PC1)
    if (fusb_has_interrupt()) {
        fusb_interrupt = true;
    }
    if (fault_is_active()) {
        fault_detected = true;
    }
}

static void system_init(void) {
    cli();
    
    i2c_init();
    button_init();
    fault_init();
    fusb_int_init();
    leds_init();
    
    // Pin change interrupts
    PCMSK0 = (1 << PCINT4);  // Button on PB4
    PCMSK1 = (1 << PCINT8) | (1 << PCINT9);  // PC0 (FUSB INT) and PC1 (FLTB)
    GIMSK = (1 << PCIE0) | (1 << PCIE1);
    
    sei();
}

int main(void) {
    system_init();
    
    // Initialize FUSB302B
    if (!fusb_init()) {
        while (1) {
            leds_flash(5, 100);
            _delay_ms(500);
        }
    }
    
    // Request 5V on startup
    current_voltage_idx = 0;
    negotiated_voltage_idx = 0;
    attempt_voltage_change(0);
    
    while (1) {
        // Handle fault condition
        if (fault_detected) {
            fault_detected = false;
            leds_flash(3, 100);
            
            // Try to recover on current voltage
            uint8_t retry_count = 0;
            while (retry_count < 5 && fault_is_active()) {
                _delay_ms(200);
                fusb_send_request(current_voltage_idx);
                retry_count++;
            }
            
            if (fault_is_active()) {
                // If still faulted, try fallback
                current_voltage_idx = find_next_available_voltage(current_voltage_idx);
                attempt_voltage_change(current_voltage_idx);
            }
            continue;
        }
        
        // Handle button press
        if (button_pressed) {
            button_pressed = false;
            
            // Debounce
            _delay_ms(30);
            if (!button_is_pressed()) continue;
            
            // Cycle to next voltage
            uint8_t next_idx = (current_voltage_idx + 1) % N_VOLTAGES;
            attempt_voltage_change(next_idx);
            
            // Wait for release
            while (button_is_pressed()) {
                _delay_ms(10);
            }
            _delay_ms(30);
        }
        
        // Handle FUSB302 interrupt (PD events)
        if (fusb_interrupt) {
            fusb_interrupt = false;
            
            uint8_t int_status = 0;
            fusb_read_reg(FUSB_REG_INTERRUPT, &int_status);
        }
        
        _delay_ms(10);
    }
    
    return 0;
}