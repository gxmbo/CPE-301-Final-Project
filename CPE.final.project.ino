// CPE 301 Final Project - Sound Level Monitor
// Alex Greene & Gambo Valdez


// STATES
#define DISABLED_STATE 0
#define IDLE_STATE     1
#define ACTIVE_STATE   2
#define ERROR_STATE    3

volatile unsigned char currentState = DISABLED_STATE;
volatile unsigned char startFlag = 0;

//  UART
#define RDA 0x80
#define TBE 0x20

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int  *)0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// GPIO 
// Port A -> D22-D29
volatile unsigned char *ddrA  = (unsigned char *)0x21;
volatile unsigned char *portA = (unsigned char *)0x22;
volatile unsigned char *pinA  = (unsigned char *)0x20;

// Port B
volatile unsigned char *ddrB  = (unsigned char *)0x24;
volatile unsigned char *portB = (unsigned char *)0x25;
volatile unsigned char *pinB  = (unsigned char *)0x23;

// Port C -> D30-D37
volatile unsigned char *ddrC  = (unsigned char *)0x27;
volatile unsigned char *portC = (unsigned char *)0x28;
volatile unsigned char *pinC  = (unsigned char *)0x26;

// Port E -> D0-D7
volatile unsigned char *ddrE  = (unsigned char *)0x2D;
volatile unsigned char *portE = (unsigned char *)0x2E;
volatile unsigned char *pinE  = (unsigned char *)0x2C;

// Port F -> ADC0-ADC7
volatile unsigned char *ddrF  = (unsigned char *)0x30;
volatile unsigned char *portF = (unsigned char *)0x31;
volatile unsigned char *pinF  = (unsigned char *)0x2F;

// ADC
volatile unsigned char *myADMUX  = (unsigned char *)0x7C;
volatile unsigned char *myADCSRB = (unsigned char *)0x7B;
volatile unsigned char *myADCSRA = (unsigned char *)0x7A;
volatile unsigned char *myADCL   = (unsigned char *)0x78;
volatile unsigned char *myADCH   = (unsigned char *)0x79;
volatile unsigned char *myDIDR0  = (unsigned char *)0x7E;

// twi / ic2
volatile unsigned char *myTWBR  = (unsigned char *)0xB8;
volatile unsigned char *myTWSR  = (unsigned char *)0xB9;
volatile unsigned char *myTWAR  = (unsigned char *)0xBA;
volatile unsigned char *myTWDR  = (unsigned char *)0xBB;
volatile unsigned char *myTWCR  = (unsigned char *)0xBC;
volatile unsigned char *myTWAMR = (unsigned char *)0xBD;

// output masks
// LED bar on D22-D26 = PA0-PA4
#define LED1_MASK   0x01
#define LED2_MASK   0x02
#define LED3_MASK   0x04
#define LED4_MASK   0x08
#define LED5_MASK   0x10
#define LEDBAR_MASK 0x1F

// Status LED on D27 = PA5
#define STATUS_MASK 0x20

// Buzzer on D12 = PB6
#define BUZZER_MASK 0x40

// Buttons on Port E
// D2 = PE4 start
// D3 = PE5 reset
// D5 = PE3 off
#define START_MASK 0x10
#define RESET_MASK 0x20
#define OFF_MASK   0x08

// LCD on Port C
// D30 = EN = PC7
// D31 = RS = PC6
// D32 = D7 = PC5
// D33 = D6 = PC4
// D34 = D5 = PC3
// D35 = D4 = PC2
#define LCD_D4_MASK 0x04
#define LCD_D5_MASK 0x08
#define LCD_D6_MASK 0x10
#define LCD_D7_MASK 0x20
#define LCD_RS_MASK 0x40
#define LCD_EN_MASK 0x80
#define LCD_ALL_MASK 0xFC

// twi status codes
#define TWI_START_OK     0x08
#define TWI_REP_START_OK 0x10
#define TWI_SLA_W_ACK    0x18
#define TWI_DATA_W_ACK   0x28
#define TWI_SLA_R_ACK    0x40
#define TWI_DATA_R_ACK   0x50
#define TWI_DATA_R_NACK  0x58

// settings
const unsigned int ACTIVE_THRESHOLD      = 220;
const unsigned int ACTIVE_EXIT_THRESHOLD = 180;   // hysteresis to avoid chatter
const unsigned int LEVEL2_THRESHOLD      = 300;
const unsigned int LEVEL3_THRESHOLD      = 380;
const unsigned int LEVEL4_THRESHOLD      = 450;
const unsigned int WARN_THRESHOLD        = 500;
const unsigned int WARN_EXIT_THRESHOLD   = 470;   // hysteresis for warning zone

const unsigned int SENSOR_LOW_FAULT  = 2;
const unsigned int SENSOR_HIGH_FAULT = 1021;

const unsigned long SENSOR_FAULT_TIME_MS = 3000UL;
const unsigned long DEBOUNCE_MS          = 25UL;
const unsigned long IDLE_BLINK_MS        = 500UL;
const unsigned long WARN_BLINK_MS        = 150UL;
const unsigned long ERROR_BLINK_MS       = 250UL;
const unsigned long LCD_REFRESH_MS       = 400UL;
const unsigned long DISPLAY_INTERVAL_MS  = 60000UL;
const unsigned long LOG_INTERVAL_MS      = 60000UL;

// rtc struct
struct RTCData
{
  unsigned char second;
  unsigned char minute;
  unsigned char hour;
  unsigned char day;
  unsigned char month;
  unsigned char year;
};

// global timing
unsigned long lastBlinkTime      = 0;
unsigned long lastLCDRefreshTime = 0;
unsigned long lastDisplayTime    = 0;
unsigned long lastLogTime        = 0;
unsigned long lastIdleBlinkTime  = 0;

unsigned char blinkState = 0;
unsigned char idleBlinkState = 0;

// warning tracking
unsigned char warningActive = 0;

// fault tracking
unsigned char faultTrackingActive = 0;
unsigned long faultStartTime = 0;

// debounce tracking
unsigned char resetLastRaw = 0;
unsigned char offLastRaw   = 0;
unsigned char resetStable  = 0;
unsigned char offStable    = 0;
unsigned long resetChangeTime = 0;
unsigned long offChangeTime   = 0;

// prototypes
void startISR();

void U0init(unsigned long baud);
void U0putchar(unsigned char c);
void printString(const char *str);
void printUInt(unsigned int value);
void print2Digits(unsigned char value);

void adc_init();
unsigned int adc_read0();

void twi_init();
unsigned char twi_status();
unsigned char twi_start();
unsigned char twi_repeated_start();
void twi_stop();
unsigned char twi_read_ack(unsigned char *data);
unsigned char twi_read_nack(unsigned char *data);

unsigned char bcd_to_dec(unsigned char value);
unsigned char rtc_valid(struct RTCData *t);
unsigned char rtc_read(struct RTCData *t);

void lcd_short_wait();
void lcd_long_wait();
void lcd_pulse_enable();
void lcd_write4(unsigned char nibble, unsigned char rs);
void lcd_command(unsigned char cmd);
void lcd_data(unsigned char data);
void lcd_init();
void lcd_clear();
void lcd_set_cursor(unsigned char col, unsigned char row);
void lcd_print(const char *str);
void lcd_print_uint(unsigned int value);
void lcd_print_2digits(unsigned char value);
void lcd_print_padded_uint(unsigned int value, unsigned char width);

void set_status_led(unsigned char on);
void set_buzzer(unsigned char on);
void set_ledbar_pattern(unsigned char pattern);
void clear_ledbar();
void set_ledbar_level(unsigned int value);

unsigned char raw_reset_pressed();
unsigned char raw_off_pressed();
unsigned char button_event(unsigned char rawSample,
                           unsigned char *lastRaw,
                           unsigned char *stable,
                           unsigned long *changeTime);

void go_to_state(unsigned char newState);
unsigned char sensor_fault(unsigned int value);

void show_state_screen();
void show_live_screen(unsigned int sensorValue);
void show_minute_screen(unsigned int sensorValue);

void log_status(unsigned int sensorValue);
void log_error(unsigned int sensorValue);

// setup
void setup()
{
  // outputs
  *ddrA |= (LEDBAR_MASK | STATUS_MASK);
  *ddrB |= BUZZER_MASK;

  // buttons input pull-up
  *ddrE &= ~(START_MASK | RESET_MASK | OFF_MASK);
  *portE |= (START_MASK | RESET_MASK | OFF_MASK);

  // ADC0 input
  *ddrF &= ~0x01;
  *portF &= ~0x01;

  U0init(9600);
  adc_init();
  twi_init();
  lcd_init();

  attachInterrupt(digitalPinToInterrupt(2), startISR, FALLING);

  go_to_state(DISABLED_STATE);
  printString("System booted\r\n");
}

// loop
void loop()
{
  unsigned int sensorValue = 0;

  // off button from any state
  if(button_event(raw_off_pressed(), &offLastRaw, &offStable, &offChangeTime))
  {
    go_to_state(DISABLED_STATE);
  }

  if(currentState != DISABLED_STATE)
  {
    sensorValue = adc_read0();
  }

  switch(currentState)
  {
    case DISABLED_STATE:
      set_status_led(1);
      clear_ledbar();
      set_buzzer(0);

      if(startFlag)
      {
        startFlag = 0;
        go_to_state(IDLE_STATE);
      }
      break;

    case IDLE_STATE:
      if(millis() - lastIdleBlinkTime >= IDLE_BLINK_MS)
      {
        lastIdleBlinkTime = millis();
        idleBlinkState ^= 1;
        set_status_led(idleBlinkState);
      }

      clear_ledbar();
      set_buzzer(0);
      warningActive = 0;

      if(sensor_fault(sensorValue))
      {
        log_error(sensorValue);
        go_to_state(ERROR_STATE);
      }
      else if(sensorValue > ACTIVE_THRESHOLD)
      {
        go_to_state(ACTIVE_STATE);
      }
      break;

    case ACTIVE_STATE:
      set_status_led(0);

      if(sensor_fault(sensorValue))
      {
        set_buzzer(0);
        log_error(sensorValue);
        go_to_state(ERROR_STATE);
      }
      else if(sensorValue < ACTIVE_EXIT_THRESHOLD)
      {
        set_buzzer(0);
        warningActive = 0;
        go_to_state(IDLE_STATE);
      }
      else
      {
        // enter/exit warning region with hysteresis
        if(sensorValue > WARN_THRESHOLD)
        {
          warningActive = 1;
        }
        else if(sensorValue < WARN_EXIT_THRESHOLD)
        {
          warningActive = 0;
        }

        if(warningActive)
        {
          if(millis() - lastBlinkTime >= WARN_BLINK_MS)
          {
            lastBlinkTime = millis();
            blinkState ^= 1;

            if(blinkState)
            {
              set_ledbar_pattern(LEDBAR_MASK);
            }
            else
            {
              clear_ledbar();
            }
          }
          set_buzzer(1);
        }
        else
        {
          set_ledbar_level(sensorValue);
          set_buzzer(0);
        }
      }
      break;

    case ERROR_STATE:
      set_status_led(0);
      set_buzzer(0);
      warningActive = 0;

      if(millis() - lastBlinkTime >= ERROR_BLINK_MS)
      {
        lastBlinkTime = millis();
        blinkState ^= 1;

        if(blinkState)
        {
          set_ledbar_pattern(LED1_MASK | LED3_MASK | LED5_MASK);
        }
        else
        {
          set_ledbar_pattern(LED2_MASK | LED4_MASK);
        }
      }

      if(button_event(raw_reset_pressed(), &resetLastRaw, &resetStable, &resetChangeTime))
      {
        faultTrackingActive = 0;
        faultStartTime = 0;
        go_to_state(IDLE_STATE);
      }
      break;
  }

  if(currentState != DISABLED_STATE && (millis() - lastLCDRefreshTime >= LCD_REFRESH_MS))
  {
    lastLCDRefreshTime = millis();
    show_live_screen(sensorValue);
  }

  if(currentState != DISABLED_STATE && (millis() - lastDisplayTime >= DISPLAY_INTERVAL_MS))
  {
    lastDisplayTime = millis();
    show_minute_screen(sensorValue);
  }

  if(currentState != DISABLED_STATE && (millis() - lastLogTime >= LOG_INTERVAL_MS))
  {
    lastLogTime = millis();
    log_status(sensorValue);
  }
}

// isr
void startISR()
{
  if(currentState == DISABLED_STATE)
  {
    startFlag = 1;
  }
}

// uart
void U0init(unsigned long baud)
{
  unsigned long FCPU = 16000000UL;
  unsigned int tbaud = (FCPU / 16UL / baud) - 1;

  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

void U0putchar(unsigned char c)
{
  while(((*myUCSR0A) & TBE) == 0) {}
  *myUDR0 = c;
}

void printString(const char *str)
{
  while(*str)
  {
    U0putchar(*str);
    str++;
  }
}

void printUInt(unsigned int value)
{
  char buf[6];
  int i = 0;

  if(value == 0)
  {
    U0putchar('0');
    return;
  }

  while(value > 0)
  {
    buf[i++] = (value % 10) + '0';
    value /= 10;
  }

  while(i > 0)
  {
    U0putchar(buf[--i]);
  }
}

void print2Digits(unsigned char value)
{
  U0putchar((value / 10) + '0');
  U0putchar((value % 10) + '0');
}

// adc
void adc_init()
{
  *myDIDR0 |= 0x01;   // ADC0 digital input disable
  *myADMUX  = 0x40;   // AVcc ref, ADC0
  *myADCSRB = 0x00;
  *myADCSRA = 0x87;   // enable ADC, prescaler 128
}

unsigned int adc_read0()
{
  unsigned long sum = 0;
  unsigned char i = 0;

  *myADMUX &= 0xE0;
  *myADMUX |= 0x00;

  for(i = 0; i < 4; i++)
  {
    *myADCSRA |= 0x40;
    while((*myADCSRA & 0x40) != 0) {}

    unsigned char low  = *myADCL;
    unsigned char high = *myADCH;
    sum += (((unsigned int)high << 8) | low);
  }

  return (unsigned int)(sum / 4);
}

// twi / rtc
void twi_init()
{
  *myTWSR  = 0x00;
  *myTWBR  = 72;
  *myTWCR  = 0x04;
  *myTWAR  = 0x00;
  *myTWAMR = 0x00;
}

unsigned char twi_status()
{
  return (*myTWSR & 0xF8);
}

unsigned char twi_start()
{
  *myTWCR = 0xA4;
  while(((*myTWCR) & 0x80) == 0) {}
  return (twi_status() == TWI_START_OK);
}

unsigned char twi_repeated_start()
{
  *myTWCR = 0xA4;
  while(((*myTWCR) & 0x80) == 0) {}
  return (twi_status() == TWI_REP_START_OK);
}

void twi_stop()
{
  *myTWCR = 0x94;
}

unsigned char twi_read_ack(unsigned char *data)
{
  *myTWCR = 0xC4;
  while(((*myTWCR) & 0x80) == 0) {}

  if(twi_status() != TWI_DATA_R_ACK) return 0;
  *data = *myTWDR;
  return 1;
}

unsigned char twi_read_nack(unsigned char *data)
{
  *myTWCR = 0x84;
  while(((*myTWCR) & 0x80) == 0) {}

  if(twi_status() != TWI_DATA_R_NACK) return 0;
  *data = *myTWDR;
  return 1;
}

unsigned char bcd_to_dec(unsigned char value)
{
  return ((value >> 4) * 10) + (value & 0x0F);
}

unsigned char rtc_valid(struct RTCData *t)
{
  if(t->second > 59) return 0;
  if(t->minute > 59) return 0;
  if(t->hour   > 23) return 0;
  if(t->day < 1 || t->day > 31) return 0;
  if(t->month < 1 || t->month > 12) return 0;
  return 1;
}

unsigned char rtc_read(struct RTCData *t)
{
  unsigned char rawSec, rawMin, rawHour, rawDOW, rawDay, rawMonth, rawYear;

  if(!twi_start()) return 0;

  *myTWDR = (0x68 << 1);
  *myTWCR = 0x84;
  while(((*myTWCR) & 0x80) == 0) {}
  if(twi_status() != TWI_SLA_W_ACK)
  {
    twi_stop();
    return 0;
  }

  *myTWDR = 0x00;
  *myTWCR = 0x84;
  while(((*myTWCR) & 0x80) == 0) {}
  if(twi_status() != TWI_DATA_W_ACK)
  {
    twi_stop();
    return 0;
  }

  if(!twi_repeated_start())
  {
    twi_stop();
    return 0;
  }

  *myTWDR = (0x68 << 1) | 1;
  *myTWCR = 0x84;
  while(((*myTWCR) & 0x80) == 0) {}
  if(twi_status() != TWI_SLA_R_ACK)
  {
    twi_stop();
    return 0;
  }

  if(!twi_read_ack(&rawSec))   { twi_stop(); return 0; }
  if(!twi_read_ack(&rawMin))   { twi_stop(); return 0; }
  if(!twi_read_ack(&rawHour))  { twi_stop(); return 0; }
  if(!twi_read_ack(&rawDOW))   { twi_stop(); return 0; }
  if(!twi_read_ack(&rawDay))   { twi_stop(); return 0; }
  if(!twi_read_ack(&rawMonth)) { twi_stop(); return 0; }
  if(!twi_read_nack(&rawYear)) { twi_stop(); return 0; }

  twi_stop();

  t->second = bcd_to_dec(rawSec & 0x7F);
  t->minute = bcd_to_dec(rawMin & 0x7F);
  t->hour   = bcd_to_dec(rawHour & 0x3F);
  t->day    = bcd_to_dec(rawDay & 0x3F);
  t->month  = bcd_to_dec(rawMonth & 0x1F);
  t->year   = bcd_to_dec(rawYear);

  return rtc_valid(t);
}

// lcd
void lcd_short_wait()
{
  for(volatile unsigned int i = 0; i < 300; i++) {}
}

void lcd_long_wait()
{
  for(volatile unsigned long i = 0; i < 20000UL; i++) {}
}

void lcd_pulse_enable()
{
  *portC |= LCD_EN_MASK;
  lcd_short_wait();
  *portC &= ~LCD_EN_MASK;
  lcd_short_wait();
}

void lcd_write4(unsigned char nibble, unsigned char rs)
{
  *portC &= ~LCD_ALL_MASK;

  if(rs) *portC |= LCD_RS_MASK;

  *portC |= ((nibble & 0x0F) << 2);
  lcd_pulse_enable();
}

void lcd_command(unsigned char cmd)
{
  lcd_write4((cmd >> 4) & 0x0F, 0);
  lcd_write4(cmd & 0x0F, 0);
  lcd_long_wait();
}

void lcd_data(unsigned char data)
{
  lcd_write4((data >> 4) & 0x0F, 1);
  lcd_write4(data & 0x0F, 1);
  lcd_short_wait();
}

void lcd_init()
{
  *ddrC |= LCD_ALL_MASK;
  *portC &= ~LCD_ALL_MASK;

  lcd_long_wait();

  lcd_write4(0x03, 0);
  lcd_long_wait();
  lcd_write4(0x03, 0);
  lcd_long_wait();
  lcd_write4(0x03, 0);
  lcd_long_wait();
  lcd_write4(0x02, 0);
  lcd_long_wait();

  lcd_command(0x28);
  lcd_command(0x0C);
  lcd_command(0x06);
  lcd_command(0x01);
}

void lcd_clear()
{
  lcd_command(0x01);
}

void lcd_set_cursor(unsigned char col, unsigned char row)
{
  unsigned char addr = (row == 0) ? col : (0x40 + col);
  lcd_command(0x80 | addr);
}

void lcd_print(const char *str)
{
  while(*str)
  {
    lcd_data(*str);
    str++;
  }
}

void lcd_print_uint(unsigned int value)
{
  char buf[6];
  int i = 0;

  if(value == 0)
  {
    lcd_data('0');
    return;
  }

  while(value > 0)
  {
    buf[i++] = (value % 10) + '0';
    value /= 10;
  }

  while(i > 0)
  {
    lcd_data(buf[--i]);
  }
}

void lcd_print_2digits(unsigned char value)
{
  lcd_data((value / 10) + '0');
  lcd_data((value % 10) + '0');
}

void lcd_print_padded_uint(unsigned int value, unsigned char width)
{
  char buf[6];
  int i = 0;
  unsigned int temp = value;

  if(temp == 0)
  {
    buf[i++] = '0';
  }
  else
  {
    while(temp > 0)
    {
      buf[i++] = (temp % 10) + '0';
      temp /= 10;
    }
  }

  while(i < width)
  {
    lcd_data(' ');
    width--;
  }

  while(i > 0)
  {
    lcd_data(buf[--i]);
  }
}

// output helper
void set_status_led(unsigned char on)
{
  if(on) *portA |= STATUS_MASK;
  else   *portA &= ~STATUS_MASK;
}

void set_buzzer(unsigned char on)
{
  if(on) *portB |= BUZZER_MASK;
  else   *portB &= ~BUZZER_MASK;
}

void set_ledbar_pattern(unsigned char pattern)
{
  *portA &= ~LEDBAR_MASK;
  *portA |= (pattern & LEDBAR_MASK);
}

void clear_ledbar()
{
  *portA &= ~LEDBAR_MASK;
}

void set_ledbar_level(unsigned int value)
{
  unsigned char pattern = 0x00;

  if(value > ACTIVE_THRESHOLD) pattern |= LED1_MASK;
  if(value > LEVEL2_THRESHOLD) pattern |= LED2_MASK;
  if(value > LEVEL3_THRESHOLD) pattern |= LED3_MASK;
  if(value > LEVEL4_THRESHOLD) pattern |= LED4_MASK;
  if(value > WARN_THRESHOLD)   pattern |= LED5_MASK;

  set_ledbar_pattern(pattern);
}

// buttons
unsigned char raw_reset_pressed()
{
  return (((*pinE) & RESET_MASK) == 0);
}

unsigned char raw_off_pressed()
{
  return (((*pinE) & OFF_MASK) == 0);
}

unsigned char button_event(unsigned char rawSample,
                           unsigned char *lastRaw,
                           unsigned char *stable,
                           unsigned long *changeTime)
{
  if(rawSample != *lastRaw)
  {
    *lastRaw = rawSample;
    *changeTime = millis();
  }

  if((millis() - *changeTime) >= DEBOUNCE_MS)
  {
    if(*stable != rawSample)
    {
      *stable = rawSample;
      if(*stable)
      {
        *lastRaw = rawSample;
        return 1;
      }
    }
  }

  *lastRaw = rawSample;
  return 0;
}

// state / fault
void go_to_state(unsigned char newState)
{
  currentState = newState;
  blinkState = 0;
  idleBlinkState = 0;
  warningActive = 0;
  faultTrackingActive = 0;
  faultStartTime = 0;
  set_buzzer(0);

  lastBlinkTime = millis();
  lastIdleBlinkTime = millis();
  lastLCDRefreshTime = 0;
  lastDisplayTime = millis();
  lastLogTime = millis();

  show_state_screen();
}

unsigned char sensor_fault(unsigned int value)
{
  unsigned char extreme = 0;

  if(value <= SENSOR_LOW_FAULT || value >= SENSOR_HIGH_FAULT)
  {
    extreme = 1;
  }

  if(extreme)
  {
    if(!faultTrackingActive)
    {
      faultTrackingActive = 1;
      faultStartTime = millis();
    }
    else if((millis() - faultStartTime) >= SENSOR_FAULT_TIME_MS)
    {
      return 1;
    }
  }
  else
  {
    faultTrackingActive = 0;
    faultStartTime = 0;
  }

  return 0;
}

// lcd screen
void show_state_screen()
{
  lcd_clear();
  lcd_set_cursor(0, 0);

  switch(currentState)
  {
    case DISABLED_STATE:
      lcd_print("State: OFF");
      lcd_set_cursor(0, 1);
      lcd_print("Press Start");
      printString("STATE: OFF\r\n");
      break;

    case IDLE_STATE:
      lcd_print("State: IDLE");
      lcd_set_cursor(0, 1);
      lcd_print("Monitoring...");
      printString("STATE: IDLE\r\n");
      break;

    case ACTIVE_STATE:
      lcd_print("State: ACTIVE");
      lcd_set_cursor(0, 1);
      lcd_print("Sound High");
      printString("STATE: ACTIVE\r\n");
      break;

    case ERROR_STATE:
      lcd_print("State: ERROR");
      lcd_set_cursor(0, 1);
      lcd_print("Press Reset");
      printString("STATE: ERROR\r\n");
      break;
  }
}

void show_live_screen(unsigned int sensorValue)
{
  struct RTCData now;
  unsigned char rtcOK = rtc_read(&now);

  lcd_set_cursor(0, 0);

  switch(currentState)
  {
    case IDLE_STATE:   lcd_print("IDLE "); break;
    case ACTIVE_STATE: lcd_print("ACTV "); break;
    case ERROR_STATE:  lcd_print("ERR  "); break;
    default:           lcd_print("OFF  "); break;
  }

  lcd_print("ADC=");
  lcd_print_padded_uint(sensorValue, 4);

  lcd_set_cursor(0, 1);

  if(rtcOK)
  {
    lcd_print_2digits(now.hour);
    lcd_data(':');
    lcd_print_2digits(now.minute);
    lcd_data(':');
    lcd_print_2digits(now.second);
    lcd_print("        ");
  }
  else
  {
    lcd_print("RTC ERROR      ");
  }
}

void show_minute_screen(unsigned int sensorValue)
{
  struct RTCData now;
  unsigned char rtcOK = rtc_read(&now);

  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_print("Minute ADC=");
  lcd_print_uint(sensorValue);

  lcd_set_cursor(0, 1);

  if(rtcOK)
  {
    lcd_print_2digits(now.hour);
    lcd_data(':');
    lcd_print_2digits(now.minute);
    lcd_data(':');
    lcd_print_2digits(now.second);
    lcd_print(" ");

    switch(currentState)
    {
      case IDLE_STATE:   lcd_print("IDLE"); break;
      case ACTIVE_STATE: lcd_print("ACTV"); break;
      case ERROR_STATE:  lcd_print("ERR "); break;
      default:           lcd_print("OFF "); break;
    }
  }
  else
  {
    lcd_print("RTC ERROR");
  }
}

// logging
void log_status(unsigned int sensorValue)
{
  struct RTCData now;
  unsigned char rtcOK = rtc_read(&now);

  printString("LOG | ");

  if(rtcOK)
  {
    print2Digits(now.month);
    U0putchar('/');
    print2Digits(now.day);
    U0putchar('/');
    printString("20");
    print2Digits(now.year);
    U0putchar(' ');
    print2Digits(now.hour);
    U0putchar(':');
    print2Digits(now.minute);
    U0putchar(':');
    print2Digits(now.second);
  }
  else
  {
    printString("RTC ERROR");
  }

  printString(" | state=");

  switch(currentState)
  {
    case DISABLED_STATE: printString("OFF");    break;
    case IDLE_STATE:     printString("IDLE");   break;
    case ACTIVE_STATE:   printString("ACTIVE"); break;
    case ERROR_STATE:    printString("ERROR");  break;
  }

  printString(" | adc=");
  printUInt(sensorValue);
  printString("\r\n");
}

void log_error(unsigned int sensorValue)
{
  struct RTCData now;
  unsigned char rtcOK = rtc_read(&now);

  printString("ERROR | ");

  if(rtcOK)
  {
    print2Digits(now.month);
    U0putchar('/');
    print2Digits(now.day);
    U0putchar('/');
    printString("20");
    print2Digits(now.year);
    U0putchar(' ');
    print2Digits(now.hour);
    U0putchar(':');
    print2Digits(now.minute);
    U0putchar(':');
    print2Digits(now.second);
  }
  else
  {
    printString("RTC ERROR");
  }

  printString(" | adc=");
  printUInt(sensorValue);
  printString("\r\n");
}