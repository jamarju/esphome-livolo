#include "esphome.h"

#define ST_ON  1
#define ST_OFF 0

#define COM_IN_PIN  D7
#define COM_OUT_PIN D8
#define LED         D4  // On board LED

#define WAIT        100 // ms

// Livolo control cmds. 0 means pull COM down. MSB (always 0) first.
#define SYNC        0b01110
#define FORGET      0b01100

#define RESYNCGANG1 0b00110
#define ACKGANG1    0b00100
#define GANG1ON     0b00011
#define GANG1OFF    0b01011

#define RESYNCGANG2 0b01010
#define ACKGANG2    0b01000
#define GANG2ON     0b00101
#define GANG2OFF    0b01101

#define RESYNCGANG3 0b00010
#define ACKGANG3    0b00000
#define GANG3ON     0b00001
#define GANG3OFF    0b01001

// unused/unknown codes
#define UNK1        0b00111
#define UNK2        0b01111

volatile int bitsToGo;
volatile uint8_t cmd;
volatile bool cmdReady = false;
int gangSt[2];


void rx_on(void);
void rx_off(void);

//=======================================================================

void ICACHE_RAM_ATTR onRxTimerISR() {
  cmd = (cmd << 1) | (digitalRead(COM_IN_PIN) ? 1 : 0); // COM_IN is LOW when COM is pulled down
  if (--bitsToGo) {
    timer1_write(20000 * 5); // 20 ms to center of the next bit
  } else {
    // TODO: should wait another 10 us till end of rx
    cmdReady = true;
    rx_on();
  }
}

void ICACHE_RAM_ATTR onComFallingEdge() {
  rx_off();
  bitsToGo = 5;
  cmd = 0;
  timer1_attachInterrupt(onRxTimerISR);
  timer1_write(10000 * 5); // 10 ms to center of the first bit (falling edge might actually be a glitch due to motors turning on/off)
  digitalWrite(LED, HIGH);
}

void ICACHE_RAM_ATTR onTxTimerISR() {
  if (bitsToGo) {
    bitsToGo--;
    digitalWrite(COM_OUT_PIN, (cmd & (1 << bitsToGo)) ? LOW : HIGH);  // COM_OUT HIGH pulls COM down
    timer1_write(20000 * 5); // schedule next bit in 20 us
  } else {
    digitalWrite(COM_OUT_PIN, LOW);
    rx_on();
  }
}

void rx_on(void) {
  // COM falling edge interrupt
  attachInterrupt(digitalPinToInterrupt(COM_IN_PIN), onComFallingEdge, FALLING);
}

void rx_off(void) {
  // COM falling edge interrupt
  detachInterrupt(digitalPinToInterrupt(COM_IN_PIN));
}

bool tx(uint8_t _cmd) {
  // TODO: wait after every tx/rx
  // TODO: queue them up instead of rejecting
  if (bitsToGo) { // already tx or rxing
    return false;
  }
  rx_off();
  cmd = _cmd;
  bitsToGo = 4;
  timer1_attachInterrupt(onTxTimerISR);
  digitalWrite(COM_OUT_PIN, HIGH);
  timer1_write(20000 * 5); // 20 us to next bit
  return true;
}


class LivoloGang : public Component, public Switch {
protected:
  int _gang;
public:
  LivoloGang(int gang): _gang(gang) {}

  void write_state(bool state) override {
    bool ok = false;
    if      (_gang == 1 && state)   ok = tx(GANG1ON);
    else if (_gang == 1 && !state)  ok = tx(GANG1OFF);
    else if (_gang == 2 && state)   ok = tx(GANG2ON);
    else if (_gang == 2 && !state)  ok = tx(GANG2OFF);
    if (ok) {
      publish_state(state);
    }
  }
};


class LivoloSwitch : public Component {
public:
  std::vector<LivoloGang *> livoloGang { new LivoloGang(1), new LivoloGang(2) };

  // constructor
  // TODO: allow multiple COM lines per ESP and pass the pin number to the constructor
  LivoloSwitch() {}

  void setup() override {
    // This will be called by App.setup()
    pinMode(COM_IN_PIN, INPUT);
    pinMode(COM_OUT_PIN, OUTPUT);
    //pinMode(LED, OUTPUT);
    digitalWrite(COM_OUT_PIN, LOW);

    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE); // TIM_DIV16 -> 5 ticks/us
    rx_on();
    //digitalWrite(LED, LOW);
  }

  void loop() override {
    if (cmdReady) {
      cmdReady = false;
      switch(cmd) {
        case GANG1ON:
          gangSt[0] = ST_ON;
          livoloGang[0]->publish_state(1);
          break;
        case GANG1OFF:
          gangSt[0] = ST_OFF;
          livoloGang[0]->publish_state(0);
          break;
  
        case GANG2ON:
          gangSt[1] = ST_ON;
          livoloGang[1]->publish_state(1);
          break;
        case GANG2OFF:
          gangSt[1] = ST_OFF;
          livoloGang[1]->publish_state(0);
          break;
      }
    }
  }
};
