#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <dht.h>
#include <OneWire.h>
#include <DS18B20.h>

#define WX_SERIAL_DEBUG 0

// 'MURICA!
float c2f(float c) {
  return (c * (9.0 / 5.0)) + 32.0;
}

#if WX_SERIAL_DEBUG != 0
// Dump a specified number of bytes in hex
void dumpBytes(unsigned char *ptr, int nBytes) {
  char *tmp = (char*) malloc(nBytes * 5);
  memset(tmp, 0, nBytes * 5);
  for (int i = 0; i < nBytes; i++) {
    sprintf(tmp + (i * 5), "0x%02x ", ptr[i]);
  }
  Serial.println(tmp);
  free(tmp);
}
#endif

// A transaction over the radio: 1 command byte, 3 args.
struct rf_txn {
  byte cmd;
  byte arg1;
  byte arg2;
  byte arg3;
};

// ASCII 4 LYFE
const byte NAK = 0x15;
const byte ENQ = 0x05;
const byte ACK = 0x06;
const byte BEL = 0x07;


// Ensure that we read from sensors periodically, no more frequently than once every N ms
// but without using delay(); reads themselves are still blocking
class PeriodicSensorReader {
  private:
    int m_interval;
    const int m_minInterval = 2000;
    const int m_backoff;
    byte m_ledPin;
  public:
    unsigned long lastRead;

    PeriodicSensorReader(int interval, byte ledPin, int backoff = 2000)
      : m_interval{interval}, m_backoff{backoff}, m_ledPin{ledPin}, lastRead{0}
    {
      if (m_interval < m_minInterval) {
        m_interval = m_minInterval;
      }
    }

    // Subclasses will override
    virtual bool _sensorRead() {
      return true;
    }

    // Subclasses will override
    virtual float _getTempC() {
      return 255.0;
    }

    int getTemp(bool f = false) {
      if (f) {
        return (int) c2f(_getTempC());
      } else {
        return (int) _getTempC();
      }
    }

    // If the read interval has elapsed, toggle the activity LED and
    // actually do the hardware read.
    bool _read() {
      // You can't use pin 0 anyway, so use that a placeholder for "no led"
      if (m_ledPin > 0) {
        digitalWrite(m_ledPin, HIGH);
      }
      bool rv = this->_sensorRead();
      if (m_ledPin > 0) {
        digitalWrite(m_ledPin, LOW);
      }
      return rv;
    }

    // The function we can call every loop
    void read() {
      // This overflows after 2^32-1 ms, but that's fine, because the math below will overflow too
      unsigned long now = millis();
      if ((lastRead == 0) || ((now - lastRead) > m_interval)) {
        if (this->_read()) {
          lastRead = now;
        } else {
          lastRead = now - m_interval + m_backoff;
        }
      }

    }

};


class DHT : public PeriodicSensorReader {
  private:
    dht m_sensor;
    byte m_sensorPin;
    int m_lastErr;
  public:
    DHT(byte sensorPin, int interval, byte ledPin)
      : PeriodicSensorReader{interval, ledPin}, m_lastErr{0}, m_sensorPin {sensorPin}
    {
    }

    bool _sensorRead() {
#if WX_SERIAL_DEBUG != 0
      Serial.print("DHT11: read, result=");
#endif
      m_lastErr = m_sensor.read11(m_sensorPin);
#if WX_SERIAL_DEBUG != 0
      Serial.println(m_lastErr);
#endif
      return (m_lastErr == DHTLIB_OK);
    }

    float _getTempC() {
      return m_sensor.temperature;
    }

    int getHumidity() {
      return (int) m_sensor.humidity;
    }

    //Handle converting the error values into text suitable for the LCD.
    // Takes a ptr to a buffer of at least size 7.
    bool isError(char *errbuf) {
      switch (m_lastErr) {
        case DHTLIB_ERROR_CHECKSUM:
          snprintf(errbuf, 7, "E_CKSM");
          break;
        case DHTLIB_ERROR_TIMEOUT:
          snprintf(errbuf, 7, "E_TIME");
          break;
        case DHTLIB_ERROR_CONNECT:
          snprintf(errbuf, 7, "E_CONN");
          break;
        case DHTLIB_ERROR_ACK_L:
          snprintf(errbuf, 7, "E_ACKL");
          break;
        case DHTLIB_ERROR_ACK_H:
          snprintf(errbuf, 7, "E_ACKH");
          break;
      }
      return (m_lastErr != DHTLIB_OK);
    }
};


class WaterTemp: public PeriodicSensorReader  {
  private:
    int m_timeout;
    DS18B20 *m_sensor;
  public:
    bool timedout = false;
    WaterTemp(OneWire *sens, int interval, byte ledPin, int timeout = 2000)
      : PeriodicSensorReader{interval, ledPin, timeout * 2}, m_timeout {timeout}
    {
      m_sensor = new DS18B20(sens);
      m_sensor->begin();
    }

    bool _sensorRead() {
      timedout = false;
      unsigned long start = millis();
#if WX_SERIAL_DEBUG != 0
      Serial.print("DS18B20: read starts at ");
      Serial.println(start);
#endif
      m_sensor->requestTemperatures();
      while (!m_sensor->isConversionComplete()) {
        // This would block forever if there was a hardware issue
        // So we have a timeout.
        if ((millis() - start) > m_timeout) {
          timedout = true;
          break;
        }
      }
#if WX_SERIAL_DEBUG != 0
      Serial.print("DS18B20: read ends at ");
      Serial.println(millis());
#endif
      return (!timedout);
    }

    float _getTempC() {
      return m_sensor->getTempC();
    }

};


// A class to handle most everything to do with the radio.

class RadioLink {
  private:
    RF24 *radio;
    byte tx_address[6];
    byte rx_address[6];
    uint8_t m_powerLevel;
  public:
    bool initialized = false;
    // ce_pin refers to the Arduino pin to which the CE pin of the RF24 module is connected
    RadioLink(byte ce_pin, byte csn_pin, const char *tx_addr, const char *rx_addr, uint8_t powerLevel = RF24_PA_LOW) {
      radio = new RF24(ce_pin, csn_pin);
      m_powerLevel = powerLevel;
      memcpy(&tx_address, tx_addr, strlen(tx_addr));
      memcpy(&rx_address, rx_addr, strlen(rx_addr));
    }

    bool init() {
      if (! radio->begin()) {
        return false;
      }
      initialized = true;
      radio->openWritingPipe(tx_address);
      radio->openReadingPipe(1, rx_address);
      radio->setPALevel(m_powerLevel);
      radio->setPayloadSize(4);
      radio->startListening();
      return true;
    }

    void debug_print() {
      // requires printf_begin() in setup()
      radio->printPrettyDetails();
    }

    void dump_into_buffer(char *buf) {
      if (initialized) {
        snprintf(buf, 16, "Ch%d Pwr%d Spd%d", radio->getChannel(), radio->getPALevel(), radio->getDataRate());
      } else {
        snprintf(buf, 16, "OFFLINE");
      }
    }

    bool read_command(rf_txn * command) {
      uint8_t pipe;
      if (radio->available(&pipe)) {
#if WX_SERIAL_DEBUG != 0
        Serial.print("RF: bytes waiting on pipe ");
        Serial.println(pipe);
#endif
        radio->read(command, radio->getPayloadSize());
#if WX_SERIAL_DEBUG != 0
        Serial.print("RF: received bytes: ");
        dumpBytes((unsigned char*)command, 4);
#endif
        return true;
      }
      return false;
    }

    bool respond(rf_txn * response) {
#if WX_SERIAL_DEBUG != 0
      Serial.print("RF: responding with: ");
      dumpBytes((unsigned char *)response, 4);
#endif
      radio->stopListening();
      bool rv = radio->write(response, radio->getPayloadSize());
      radio->startListening();
#if WX_SERIAL_DEBUG != 0
      Serial.print("RF: write ");
      Serial.println(rv ? "ok" : "FAILED");
#endif
      return rv;
    }
};


// A class to handle recording a complete button press
// The button must go down and up within 50ms (configurable)
class ButtonReader {
  private:
    byte lastState = LOW;
    unsigned long lastRead = 0;
    unsigned long delay = 0;
    byte pin;
    bool was_pressed = false;
  public:
    ButtonReader(byte pin, int delay = 25) {
      this->pin = pin;
      this->delay = delay;
    }

    bool pressed() {
      if (was_pressed) {
        was_pressed = false;
        return true;
      }
      return false;
    }

    void poll() {
      byte btn = digitalRead(this->pin);
      if (btn != lastState) {

        if (millis() - lastRead > delay) {
          lastState = btn;
          lastRead = millis();
          if (btn == LOW) {
#if WX_SERIAL_DEBUG != 0
            Serial.println("BTN: press");
#endif
            // Record a press
            // This means subsequent presses in quick succession will be lost, but that's ok
            // If the main loop is so slow that that can happen, we have other problems.
            was_pressed = true;
          }
        }
      }
    }
};

class LCDManager {
  private:
    LiquidCrystal_I2C* lcd;
    const int backlightInterval = 5000;
    unsigned long backlightOnTime = 0;
  public:
    // 6 rows of 16 chars
#define NUM_SCREENS 4
    char idx = 0;
    bool backlightOn = false;
    char messageBuffer[NUM_SCREENS][16];
    const char labelBuffer[NUM_SCREENS][16] = { "Air Temp",
                                                "Water Temp",
                                                "Rel. Humidity",
                                                "Radio Status"
                                              };
    LCDManager(byte i2c_addr, byte cols, byte rows) {
      lcd = new LiquidCrystal_I2C(i2c_addr, cols, rows);
    }

    void begin(const char *initial_msg = NULL) {
      lcd->init();
      lcd->clear();
      if (initial_msg) {
        lcd->print(initial_msg);
      }
    }

    void poll() {
      if ((backlightOn) &&
          ((millis() - backlightOnTime) > backlightInterval)) {
        backlightOn = false;
        lcd->noBacklight();
      }
    }

    void show() {
      lcd->backlight();
      backlightOn = true;
      backlightOnTime = millis();
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print(labelBuffer[idx]);
      lcd->setCursor(0, 1);
      lcd->print(messageBuffer[idx]);
    }

    void step(bool rev = false) {
      if (rev) {
        idx--;
      } else {
        idx++;
      }
      if (idx < 0) {
        idx = NUM_SCREENS - 1;
      }
      if (idx >= NUM_SCREENS) {
        idx = 0;
      }
    }

    void dump_messages() {
#if WX_SERIAL_DEBUG != 0
      for (int i = 0; i < NUM_SCREENS; i++) {
        Serial.print("Index ");
        Serial.print(i);
        Serial.print(" ");
        Serial.println(labelBuffer[i]);
        Serial.println(messageBuffer[i]);
      }
#endif
    }
};


// So we don't lose track of pins
#define PIN_BUTTON A1
#define PIN_RF_LED A0
#define PIN_DHT_LED A2
#define PIN_DS18B20_LED A3
#define LCD_I2C_ADDR 0x27
#define PIN_BUZZER 3

ButtonReader btn(PIN_BUTTON);
LCDManager lcd(LCD_I2C_ADDR, 16, 2);
RadioLink rf(9, 8, "RASPI", "WXSTA");
DHT temp(7, 10000, PIN_DHT_LED);
OneWire owb(6);
WaterTemp wt(&owb, 20000, PIN_DS18B20_LED);


void beep() {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(100);
  digitalWrite(PIN_BUZZER, LOW);
}

void setup() {
  lcd.begin();
  pinMode(PIN_RF_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_DHT_LED, OUTPUT);
  pinMode(PIN_DS18B20_LED, OUTPUT);
  pinMode(3, OUTPUT);
#if WX_SERIAL_DEBUG != 0
  Serial.begin(115200);
#endif
  if (!rf.init()) {
#if WX_SERIAL_DEBUG != 0
    Serial.println("Failed to initialize radio hardware");
#endif
  }

  digitalWrite(PIN_RF_LED, HIGH);
  digitalWrite(PIN_DHT_LED, HIGH);
  digitalWrite(PIN_DS18B20_LED, HIGH);
  delay(1000);
  digitalWrite(PIN_RF_LED, LOW);
  digitalWrite(PIN_DHT_LED, LOW);
  digitalWrite(PIN_DS18B20_LED, LOW);
  delay(500);

#if WX_SERIAL_DEBUG != 0
  Serial.println("setup() complete");
#endif
}

void loop() {
  lcd.poll();
  btn.poll();
  temp.read();
  wt.read();
  char tempErr[8];
  if (temp.isError(tempErr)) {
    snprintf(lcd.messageBuffer[0], 16, "Error: %s", tempErr);
    snprintf(lcd.messageBuffer[2], 16, "Error: %s", tempErr);
  } else {
    snprintf(lcd.messageBuffer[0], 16, "%d F (%d C)", temp.getTemp(true), temp.getTemp());
    snprintf(lcd.messageBuffer[2], 16, "%d %%", temp.getHumidity());
  }
  if (wt.timedout) {
    snprintf(lcd.messageBuffer[1], 16, "Error: TIMEOUT");
  } else {
    snprintf(lcd.messageBuffer[1], 16, "%d F (%d C)", wt.getTemp(true), wt.getTemp());
  }
  rf.dump_into_buffer(lcd.messageBuffer[3]);

  if (btn.pressed()) {
    if (lcd.backlightOn) {
      // We're already displaying stuff, next screen
      lcd.step();
    } else {
      // Screen 0
      lcd.idx = 0;
    }
    // Update the screen (and turn on backlight if it isn't)
    lcd.show();
  }
  struct rf_txn txn = {0, 0, 0, 0};
  struct rf_txn resp = {0, 0, 0, 0};
  if ((rf.initialized) && (rf.read_command(&txn))) {
    digitalWrite(PIN_RF_LED, HIGH);
    switch (txn.cmd) {
      case ENQ:
        resp.cmd = ACK;
        switch (txn.arg1) {
          case 0x00:
            break;
          case 'T':
            resp.arg1 = (uint8_t) temp.getTemp((txn.arg2 == 'F'));
            resp.arg2 = txn.arg2 == 'F' ? 'F' : 'C';
            break;
          case 'W':
            resp.arg1 = (uint8_t) wt.getTemp((txn.arg2 == 'F'));
            resp.arg2 = txn.arg2 == 'F' ? 'F' : 'C';
            break;
          case 'H':
            resp.arg1 = (uint8_t) temp.getHumidity();
            break;
          default:
            resp.cmd = NAK;
            break;
        }
        break;
      case BEL:
        beep();
        resp.cmd = ACK;
        break;
      default:
        resp.cmd = NAK;
        break;
    }
    bool rv = rf.respond(&resp);
    digitalWrite(PIN_RF_LED, LOW);
  }
}
