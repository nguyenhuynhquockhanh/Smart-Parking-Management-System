#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

// ==================================================
//                 TIMER0 → millis()
// ==================================================
volatile uint32_t millis_avr = 0;
ISR(TIMER0_COMPA_vect) { millis_avr++; }
uint32_t millis() { return millis_avr; }

// ==================================================
//             STRUCT SENSOR HC-SR04 + LED + BEEP
// ==================================================
typedef struct {
    volatile uint8_t *ddr_trig;
    volatile uint8_t *port_trig;
    uint8_t trig;

    volatile uint8_t *ddr_echo;
    volatile uint8_t *pin_echo;
    uint8_t echo;

    volatile uint8_t *ddr_led;
    volatile uint8_t *port_led;
    uint8_t led;

    volatile uint8_t *ddr_buz;
    volatile uint8_t *port_buz;
    uint8_t buz;

    uint32_t closeTime;
    uint8_t locked;

    uint16_t beep_interval;
    uint32_t beepTimer;
    uint8_t beep_state;
} SensorPack;

// ==================================================
//                 PROTOTYPE
// ==================================================
uint16_t read_distance(SensorPack *s);
void updateSensor(SensorPack *s);
void updateBeep(SensorPack *s);
void initSensor(SensorPack *s);

// ==================================================
//          3 SENSOR SIÊU ÂM ĐƯỢC GHÉP
// ==================================================
SensorPack S1 = { &DDRD,&PORTD,PD4, &DDRD,&PIND,PD5, &DDRB,&PORTB,PB2, &DDRB,&PORTB,PB3 };
SensorPack S2 = { &DDRD,&PORTD,PD6, &DDRD,&PIND,PD7, &DDRB,&PORTB,PB4, &DDRB,&PORTB,PB5 };
SensorPack S3 = { &DDRB,&PORTB,PB0, &DDRC,&PINC,PC3, &DDRD,&PORTD,PD0, &DDRD,&PORTD,PD1 };

// ==================================================
//                  INIT SENSOR
// ==================================================
void initSensor(SensorPack *s)
{
    *(s->ddr_trig) |=  (1 << s->trig);
    *(s->ddr_echo) &= ~(1 << s->echo);

    *(s->ddr_led) |= (1 << s->led);
    *(s->ddr_buz) |= (1 << s->buz);

    *(s->port_led) &= ~(1 << s->led);
    *(s->port_buz) &= ~(1 << s->buz);

    s->closeTime = 0;
    s->locked = 0;
    s->beep_interval = 0;
    s->beepTimer = 0;
    s->beep_state = 0;
}

// ==================================================
//                  HC-SR04 READ
// ==================================================
uint16_t read_distance(SensorPack *s)
{
    uint32_t timeout;

    *(s->port_trig) &= ~(1 << s->trig);
    _delay_us(2);
    *(s->port_trig) |=  (1 << s->trig);
    _delay_us(10);
    *(s->port_trig) &= ~(1 << s->trig);

    timeout = 30000;
    while ((*(s->pin_echo) & (1 << s->echo)) == 0)
        if (--timeout == 0) return 999;

    uint32_t count = 0;
    while ((*(s->pin_echo) & (1 << s->echo)) != 0)
    {
        _delay_us(1);
        if (++count > 60000) return 999;
    }

    return count / 58;
}

// ==================================================
//                BEEP NON-BLOCKING
// ==================================================
void updateBeep(SensorPack *s)
{
    if (s->beep_interval == 0 || s->locked) {
        *(s->port_buz) &= ~(1 << s->buz);
        s->beep_state = 0;
        return;
    }

    uint32_t now = millis();

    if (s->beep_state == 0)
    {
        if (now - s->beepTimer >= s->beep_interval)
        {
            *(s->port_buz) |= (1 << s->buz);
            s->beepTimer = now;
            s->beep_state = 1;
        }
    }
    else
    {
        if (now - s->beepTimer >= 5)
        {
            *(s->port_buz) &= ~(1 << s->buz);
            s->beepTimer = now;
            s->beep_state = 0;
        }
    }
}

// ==================================================
//                  LOGIC CẢNH BÁO
// ==================================================
#define LOCK_TIME 2000
void updateSensor(SensorPack *s)
{
    uint32_t now = millis();
    uint16_t d = read_distance(s);

    if (d > 0.5 && s->locked)
    {
        s->locked = 0;
        *(s->port_led) &= ~(1 << s->led);
        s->closeTime = 0;
    }

    if (!s->locked)
    {
        if (d < 0.5)
        {
            if (s->closeTime == 0) s->closeTime = now;
            if (now - s->closeTime >= LOCK_TIME)
            {
                s->locked = 1;
                *(s->port_led) |= (1 << s->led);
            }
        }
        else s->closeTime = 0;

        if (d < 0.5)          s->beep_interval = 50;
        else if (d < 1)     s->beep_interval = 300;
        else if (d < 2)    s->beep_interval = 600;
        else                s->beep_interval = 0;
    }
    else {
        s->beep_interval = 0;
    }

    updateBeep(s);
}

// ==================================================
//                 LCD I2C CHUẨN
// ==================================================
#define LCD_ADDR 0x27
#define LCD_BACKLIGHT 0x08
#define ENABLE 0x04

void TWI_init() { TWSR = 0; TWBR = 72; }
void TWI_start() { TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); while (!(TWCR & (1<<TWINT))); }
void TWI_stop() { TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO); while(TWCR & (1<<TWSTO)); }
void TWI_write(uint8_t data) { TWDR=data; TWCR=(1<<TWINT)|(1<<TWEN); while (!(TWCR & (1<<TWINT))); }

void lcd_pulse(uint8_t data)
{
    TWI_start();
    TWI_write(LCD_ADDR<<1);
    TWI_write(data | ENABLE);
    TWI_write(data & ~ENABLE);
    TWI_stop();
}

void lcd_write(uint8_t data, uint8_t rs)
{
    uint8_t high = (data & 0xF0) | LCD_BACKLIGHT | rs;
    uint8_t low  = ((data<<4)&0xF0) | LCD_BACKLIGHT | rs;
    lcd_pulse(high);
    lcd_pulse(low);
}

void lcd_cmd(uint8_t cmd) { lcd_write(cmd,0); _delay_ms(2); }
void lcd_data(uint8_t data) { lcd_write(data,1); _delay_ms(2); }

void lcd_init()
{
    _delay_ms(40);
    lcd_pulse(0x30);
    _delay_ms(5);
    lcd_pulse(0x30);
    _delay_us(200);
    lcd_pulse(0x30);
    _delay_us(200);
    lcd_pulse(0x20);

    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
    _delay_ms(2);
}

void lcd_set(uint8_t col, uint8_t row) { lcd_cmd(0x80 | (row ? 0x40 : 0x00) | col); }
void lcd_print(char *s) { while(*s) lcd_data(*s++); }

// ==================================================
//                 IR – ĐẾM XE
// ==================================================
#define IR3 PC0
#define IR4 PC1
#define IR5 PC2

// ==================================================
//                    SERVO
// ==================================================
void servo_init() {
    DDRB |= (1<<PB1);
    TCCR1A = (1<<COM1A1) | (1<<WGM11);
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);
    ICR1 = 5000;
}

void servo_write(int degree) {
    uint16_t pulse;
    if(degree==0) pulse=600;
    else if(degree==90) pulse=1500;
    else if(degree==180) pulse=2400;
    else pulse=600+degree*10;
    OCR1A=pulse/4;
}
// ==================================================
//            <<< ADD: KHÓA CỬA KHI XE RA
// ==================================================
volatile uint8_t exit_lock = 0;
uint8_t prev_r3 = 1, prev_r4 = 1, prev_r5 = 1;


// ==================================================
//                        MAIN
// ==================================================
int main()
{
    // Timer0
    TCCR0A=(1<<WGM01);
    OCR0A=249;
    TCCR0B=(1<<CS01)|(1<<CS00);
    TIMSK0=(1<<OCIE0A);
    sei();

    // Disable ADC
    ADMUX=0; ADCSRA=0;

    // IR pins PD2-PD3
    DDRD &= ~((1<<PD2)|(1<<PD3));
    PORTD |= (1<<PD2)|(1<<PD3);

    // IR3-IR5 count
    DDRC &= ~((1<<IR3)|(1<<IR4)|(1<<IR5));
    PORTC |= (1<<IR3)|(1<<IR4)|(1<<IR5);

    // LCD + Servo
    TWI_init();
    lcd_init();
    servo_init();
    servo_write(0);

    // Init sensors
    initSensor(&S1); initSensor(&S2); initSensor(&S3);

    enum {IDLE, W_IR2, W_IR1} state=IDLE;

    while(1)
    {
        updateSensor(&S1);
        updateSensor(&S2);
        updateSensor(&S3);

        uint8_t r3 = !(PINC & (1<<IR3));
        uint8_t r4 = !(PINC & (1<<IR4));
        uint8_t r5 = !(PINC & (1<<IR5));
        uint8_t count = r3 + r4 + r5;

            // ================= PHÁT HIỆN XE RA =================
    // IR trong bãi: 1 -> 0  ==> xe đang ra
    if ( (prev_r3 && !r3) || (prev_r4 && !r4) || (prev_r5 && !r5) )
    {
        exit_lock = 1;   // khóa cửa vào
    }

    prev_r3 = r3;
    prev_r4 = r4;
    prev_r5 = r5;
    // ===================================================
    
        // LCD hiển thị

        lcd_set(0,0);
lcd_print("So xe: ");
lcd_data(count + '0');
lcd_print("/3   ");

lcd_set(0,1);
lcd_print("Trong o: ");

// XÓA 7 KÝ TỰ ở vùng 9 → 15
lcd_set(9,1);
lcd_print("       ");   // 7 space

// IN LẠI DANH SÁCH Ô TRỐNG
lcd_set(9,1);
uint8_t empty = 0;

if(!r3){ lcd_print("1"); empty = 1; }
if(!r4){ if(empty) lcd_print(","); lcd_print("2"); empty = 1; }
if(!r5){ if(empty) lcd_print(","); lcd_print("3"); empty = 1; }

if(!empty) lcd_print("-");

        uint8_t lock_ir1 = (count>=3);
        uint8_t ir1 = !(PIND & (1<<PD2));
        uint8_t ir2 = !(PIND & (1<<PD3));

        switch(state)
        {
            case IDLE:
                if(!exit_lock && !lock_ir1 && ir1){ servo_write(90); state=W_IR2; }
                else if(ir2){ servo_write(90); state=W_IR1; }
                break;
            case W_IR2:
                if(ir2){ while(!(PIND & (1<<PD3))); servo_write(0); state=IDLE; }
                break;
            case W_IR1:
                if(ir1){ while(!(PIND & (1<<PD2))); servo_write(0);exit_lock = 0;; state=IDLE; }
                break;
        }

        _delay_ms(50);
    }
}
