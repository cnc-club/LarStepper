#define NUMSTEPS 4
#define TIMER_CLOCK_FREQ 2000000.0 //250kHz for /64 prescale from 16MHz
#define PRINT true
#define TESTMODE true
#define TIMER_STEP 125
#define STEP_TIME 200

#define FREQ 7813 // from test

#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(10, 0, 1, 5);
unsigned int localPort = 8888;      // local port to listen on


unsigned int latency;
unsigned char timerLoadValue;

long print__ = 0;
bool print_;
float period;
long time_;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
EthernetUDP Udp;
volatile long count;

unsigned char SetupTimer2(float timeoutFrequency) {
  unsigned char result; //The value to load into the timer to control the timeout interval.
  result = (int)((257.0 - (TIMER_CLOCK_FREQ / timeoutFrequency)) + 0.5); //the 0.5 is for rounding;
  TCCR2A = 0;
  TCCR2B = 0 << CS22 | 1 << CS21 | 0 << CS20;
  TIMSK2 = 1 << TOIE2;
  TCNT2 = result;
  return (result);
}

struct Fly
{
  char    sig[3];
  float   pitch;
  float   bank;
  float   z;
  byte    state;
  byte    sum;

};
Fly *fly;


class LarStepper
{
  public:
    LarStepper(float _scale, int _step, int _dir, int _home);
    void set_cmd(float c);
    void update_freq();
    void print();
    int step_pin;
    int dir_pin;
    int home_pin;
    volatile long timer;
    volatile long counter;
    volatile long home;
    volatile bool homing_dir;
    volatile bool homed = false;
    volatile long pos = 0;
    volatile float v = 0.;
    float a = 200.;
    float cmd = 0;
    float old_cmd;
    float scale = 12.5;
    float maxv = 200.;
    float minl = -1000000000;
    float maxl = 1000000000;
    float e = 0;
    float p = 0;
    float t = 0;
    float t1;
    float module = 360.;
};



LarStepper::LarStepper(float _scale, int _step, int _dir, int _home)
{
  pos = 0;
  step_pin = 1 << _step;
  dir_pin = 1 << _dir;
  home = 0;
  home_pin = 1 << _home;
  homing_dir = 1;
  scale = _scale;
  pinMode(_step, OUTPUT);
  pinMode(_dir, OUTPUT);
  digitalWrite(_dir, HIGH);
  digitalWrite(_step, HIGH);
  pinMode(_home, INPUT);
}


void LarStepper::print() {
  Serial.print("cmd:");
  Serial.print(cmd);
  Serial.print(", pos");
  Serial.print(p);
  Serial.print(", err");
  Serial.print(e);
  Serial.print(", v");
  Serial.print(v);
  Serial.print(", max freq:");
  Serial.print(scale*maxv);
  Serial.println(" ");  
}


void LarStepper::set_cmd(float c)
{
  if (c > maxl) cmd = maxl;
  else if (c < minl) cmd = minl;
  else cmd = c;
  if (module > 0) {
    cmd =  float( (long(cmd * 1000)) % 360000 ) / 1000 ;
  }
}


void LarStepper::update_freq()
{
  p = float(pos) / scale;

  // Get pos error
  if (module > 0)
  {
    p = float(long(p * 1000) % 360000) / 1000;
    float e1 = cmd - p + 360;
    float e2 = cmd - p;
    float e3 = cmd - p - 360;
    if (abs(e1) < abs(e2))
    {
      if (abs(e1) < abs(e3)) e = e1;
      else e = e3;
    }
    else
    {
      if (abs(e2) < abs(e3)) e = e2;
      else e = e3;
    }
  }
  else {
    e = cmd - p;
  }

  float c = e + p; //new cmd

  float vel_cmd = (c - old_cmd) / period;
  old_cmd = c;
  float match_ac = (v < vel_cmd ? a : -a);
  float match_time = (vel_cmd - v) / match_ac;
  float avg_v = (vel_cmd + v) * 0.5;
  float est_out = p + avg_v * match_time;
  float est_cmd = c + vel_cmd * (match_time - 1.5 * period);
  float est_err = est_out - est_cmd;

  if (match_time < period) {
    if (abs(est_err) < 0.01) {
      v = vel_cmd;
    }
    else {
      float new_v = vel_cmd - 0.5 * est_err / period;
      if (new_v > v + a * period) {
        v += a * period;
      }
      else if (new_v < v - a * period) {
        v -= a * period;
      }
      else {
        v = new_v;
      }
    }
  }
  else {

    float dv = -2.0 * match_ac * period;
    float dp = dv * match_time;
    if (abs(est_err + dp * 2.0) < abs(est_err)) {
      match_ac = -match_ac;
    }
    v += match_ac * period;
  }

  if (v < -maxv) v = -maxv;
  if (v > maxv) v = maxv;
  // step timer mks
  if ( abs(v) < 0.001 ) timer =  1000000000;
  else timer = abs(long(1000000 / ( v * scale )));
  //p += v*period;
}


LarStepper st[4] = {
  LarStepper(16., 2, 3, A1),
  LarStepper(16., 4, 5, A2),
  LarStepper(16., 6, 7, A3),
  LarStepper(16., 8, 9, A4)
};


volatile int pins;
//Timer2 overflow interrupt vector handler
ISR(TIMER2_OVF_vect) {
  pins = PINB << 8 | PIND;
  count += 1;

  for (int i = 0; i < NUMSTEPS; i++)
  {
    // home
    if (!st[i].homed && ((pins & st[i].home_pin) > 0) && ( (st[i].v > 0) == st[i].homing_dir ) ) {
      st[i].pos = st[i].home;
      st[i].homed = true;
    }


    // DIR  TODO DIR Set time
    if (
      (((pins & st[i].dir_pin) > 0) && (st[i].v < 0)) ||
      (((pins & st[i].dir_pin) == 0) && (st[i].v > 0))
    ) // have to change dir pin
    {
      if (st[i].v > 0) pins |= st[i].dir_pin;
      else pins &= 65535 -  st[i].dir_pin;
      continue; // skip to next interrupt for dir setup delay.
    }


    st[i].counter += TIMER_STEP;
    if (st[i].counter <= STEP_TIME ) {
      pins |= st[i].step_pin;
    }
    else if (st[i].counter <= 2 * STEP_TIME )
    {
      pins &= 65535 - st[i].step_pin;
    }
    else
    {
      pins &= 65535 - st[i].step_pin;
      if (st[i].counter >= st[i].timer) {
        st[i].counter = 0;//st[i].counter % st[i].timer;
        if (st[i].v > 0) st[i].pos += 1;
        else st[i].pos -= 1;
        pins |= st[i].step_pin;
      }
    }
  }

  PORTD = pins;
  PORTB = (PINB & 192) | (pins >> 8);
  latency = TCNT2;
}

void setup() {
  Serial.begin(115200);

  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  noInterrupts();
  timerLoadValue = SetupTimer2(40000);
  interrupts();
  for (int i = 0; i < NUMSTEPS; i++) {
    Serial.print("Stepper ");
    Serial.print(i);
    Serial.print(" max velocity: ");
    Serial.println(FREQ / 4 / st[i].scale);
  }
}

long period_count = 0;
float max_period = 0;
void loop() {
  period_count += 1;
  period = (micros() - time_) * 0.000001;
  if (period > max_period && millis() > 2000) {
    max_period = period;
  }
  time_ = micros();
  for  (int i = 0; i < NUMSTEPS; i++)
  {
    st[i].update_freq();
  }
  print_ = false;
  if (print__ < millis() && PRINT ) {
    print__ = millis() + 400;
    print_ = true;
    Serial.println();
  }

  if (print_) {
    Serial.print("Average Hz of the base thread:");
    Serial.print(float(count) / (millis()) * 1000);
    Serial.print(" ~ ");
    Serial.print(float(millis()) / count);
    Serial.println("ms");

    Serial.print("Current period (ms):");
    Serial.println(period * 1000);
    Serial.print("Average period (ms):");
    Serial.println(millis() / float(period_count));
    Serial.print("Max period (ms):");
    Serial.println(max_period * 1000);

  }
  if (print_)
  {
    Serial.print("Latency: ");
    Serial.println(latency);
    Serial.print("Pins: ");
    Serial.println(pins, BIN);
    for (int i = 0; i < NUMSTEPS; i++)
    {
      st[i].print();
    }
  }



  if (TESTMODE) {
    int interval = 6;
    if (millis() % (2 * interval * 1000) > interval * 1000) {
      st[0].set_cmd(0);
      st[1].set_cmd(0);
      st[2].set_cmd(0);
      st[3].set_cmd(0);
    }
    else {
      st[0].set_cmd(100);
      st[1].set_cmd(100);
      st[2].set_cmd(200);
      st[3].set_cmd(300);
    }
Ssin(0.0003 * millis()) * 100 );
    st[3].set_cmd(sin(0.0004 * millis()) * 100 );
  }
  else {

    int packetSize = Udp.parsePacket();
    if (packetSize) {
      // read the packet into packetBufffer
      Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      fly = (Fly*)packetBuffer;
      st[0].set_cmd(fly->pitch * 180. / 3.1415);
      st[1].set_cmd(fly->bank * 180. / 3.1415);

      if (print_ & false)
      {
        Serial.println(packetBuffer);
        Serial.println(packetSize);
        Serial.println("Contents:");
        Serial.println(packetBuffer);
        Serial.println(fly->sig);
        Serial.println(fly->pitch);
        Serial.println(fly->bank);
        Serial.println(fly->z);
        Serial.println(fly->state);
        Serial.println(fly->sum);
      }

    }

  }



}












