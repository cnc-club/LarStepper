#define NUMSTEPS 4
#define TIMER_CLOCK_FREQ 2000000.0 //250kHz for /64 prescale from 16MHz
#define PRINT true
#define TESTMODE false
#define TIMER_STEP 125
#define STEP_TIME 200
#define HOME_BUTTON A0
#define OFF_BUTTON A1
#define FREQ 7813 // from test
#define STATE_OFF 0
#define STATE_ON 1



#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

int STATE = STATE_OFF;

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
    void check_home();
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
    float deadband = 0.1;
};



LarStepper::LarStepper(float _scale, int _step, int _dir, int _home)
{
  pos = 0;
  step_pin = 1 << _step;
  dir_pin = 1 << _dir;
  home = 0;
  home_pin = _home; //1 << _home;
  homing_dir = 1;
  scale = _scale;
  pinMode(_step, OUTPUT);
  pinMode(_dir, OUTPUT);
  digitalWrite(_dir, HIGH);
  digitalWrite(_step, HIGH);
  pinMode(_home, INPUT);
  digitalWrite(_home, HIGH);
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
  Serial.print(scale * maxv);
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
  if ( abs(e) < deadband && abs(v) < 0.1) timer =  1000000000;
  else if ( abs(v) < 0.001 ) timer =  1000000000;
  else timer = abs(long(1000000 / ( v * scale )));
  //p += v*period;
}


LarStepper st[4] = {
  LarStepper(10.5555555, 9, 8, A5),
  LarStepper(5.583333327, 7, 6, A4),
  LarStepper(8., 5, 4, A2),
  LarStepper(8., 1, 1, A4)
};


void LarStepper::check_home() {
  if (!homed && (digitalRead(home_pin) == HIGH) && (digitalRead(home_pin) == HIGH) && (digitalRead(home_pin) == HIGH)
      && (digitalRead(home_pin) == HIGH) && (digitalRead(home_pin) == HIGH)
     ) {
    pos = home;
    homed = true;
  }
}

volatile int pins;
//Timer2 overflow interrupt vector handler
ISR(TIMER2_OVF_vect) {
  pins = PINB << 8 | PIND;
  count += 1;

  for (int i = 0; i < NUMSTEPS; i++)
  {
    /*// home
      if (!st[i].homed && ((pins & st[i].home_pin) > 0) && ( (st[i].v > 0) == st[i].homing_dir ) ) {
      st[i].pos = st[i].home;
      st[i].homed = true;
      }*/


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
  pinMode(HOME_BUTTON, INPUT);
  digitalWrite(HOME_BUTTON, HIGH);

  pinMode(OFF_BUTTON, INPUT);
  digitalWrite(OFF_BUTTON, HIGH);


  noInterrupts();
  timerLoadValue = SetupTimer2(40000);
  for (int i = 0; i < NUMSTEPS; i++) {
    Serial.print("Stepper ");
    Serial.print(i);
    Serial.print(" max velocity: ");
    Serial.println(FREQ / 4 / st[i].scale);
  }

  st[2].pos = -150 * st[2].scale;
  st[2].pos = -150 * st[2].scale;
  st[2].a = 2000;
  st[2].maxv = 200;
  st[2].minl = -150;
  st[2].maxl = 150;
  st[2].module = 360;
  st[2].home = -150 * st[2].scale;

  st[0].home = 17 * st[0].scale;
  st[1].home = 15 * st[1].scale;

  interrupts();
  st[0].a = 300;
}

long period_count = 0;
float max_period = 0;
long pack_count = 0;


void print_state() {
  Serial.println();
  Serial.print("Packet count: ");
  Serial.println(pack_count);
  Serial.println("Coords: ");
  Serial.print("   P = ");
  Serial.println(fly->pitch);
  Serial.print("   B = ");
  Serial.println(fly->bank);
  Serial.print("   Z = ");
  Serial.println(fly->z);
  Serial.println("Steppers: ");
  st[0].print();
  st[1].print();
  st[2].print();
  st[3].print();
}

float punch = 0;
int home_axis = 4;


bool check_home() {
  //Serial.println(home_axis);
  if ( (STATE == STATE_OFF)
       && (digitalRead(HOME_BUTTON) == LOW)
       && (digitalRead(HOME_BUTTON) == LOW)
       && (digitalRead(HOME_BUTTON) == LOW)
     ) {
    STATE = STATE_ON;    
  }


  if (home_axis > 3) {
    if (digitalRead(HOME_BUTTON) == LOW) {
      home_axis = 3;
    }
  }
  if (home_axis == 3) {
    st[0].check_home();
    st[0].set_cmd(st[0].p + 0.2);
    if (st[0].homed)
    {
      st[0].set_cmd(0);
      home_axis = 2;
    }
  }
  if (home_axis == 2) {
    st[1].check_home();
    st[1].set_cmd(st[1].p + 0.2);
    if (st[1].homed)
    {
      st[1].set_cmd(0);
      home_axis = 1;
    }
  }
  if (home_axis == 1) {
    st[2].check_home();
    st[2].set_cmd(st[2].p - 0.2);
    if (st[2].homed)
    {
      st[2].set_cmd(0);
      home_axis = 0;
      STATE = STATE_ON;
    }

  }

  return home_axis == 0;
}


bool check_off() {
  if (digitalRead(OFF_BUTTON) == LOW) {
    STATE = STATE_OFF;

    if (st[0].homed) st[0].set_cmd(0);
    if (st[1].homed) st[1].set_cmd(0);
    if (st[2].homed) st[2].set_cmd(-130);
    if (st[3].homed) st[3].set_cmd(0);
  }
  return STATE == STATE_ON;
}

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

  if (!check_home())  {
    Serial.print("Homing....");
    Serial.println(home_axis);
    return ; // go to the next loop
  }

  if (!check_off())  {
    Serial.println("STATE is OFF");
    return ; // go to the next loop
  }

  print_ = false;
  if (print__ < millis() && PRINT ) {
    print__ = millis() + 400;
    print_ = true;
    Serial.println();
  }
  if (print_ )
  {
    print_state();
  }
  if (print_ and false) {
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
  if (print_ and false)
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
    st[0].set_cmd(sin(0.0001 * millis()) * 10 );
    st[1].set_cmd(sin(0.000002 * millis()) * 10 );
    st[2].set_cmd(sin(0.000003 * millis()) * 10 );
    st[3].set_cmd(sin(0.000004 * millis()) * 10 );
  }
  else {

    int packetSize = Udp.parsePacket();
    if (packetSize) {
      pack_count += 1;
      // read the packet into packetBufffer
      Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      fly = (Fly*)packetBuffer;
      fly->pitch = fly->pitch * 180. / 3.1415;
      fly->bank = fly->bank * 180. / 3.1415;

      if (fabs(fly->z) > 1. && punch == 0)
      {
        if (PRINT) {
          Serial.println();
          Serial.println("Punch start.");
          Serial.println("Punch start.");
          Serial.println("Punch start.");
          Serial.println(fly->z);
        }
        if (fly->z > 100) fly->z = 100;
        if (fly->z < -100) fly->z = -100;
        st[2].set_cmd(fly->z);
      }
      st[0].set_cmd(fly->pitch);
      st[1].set_cmd(fly->bank);

      // punch update
      if (fabs(punch) > 0  && (fabs(st[2].p - punch) < 3 || fabs(st[2].p > 120)) )
      {
        if (PRINT) {
          Serial.println();
          Serial.println("Punch back.");
          Serial.println("Punch back.");
          Serial.println("Punch back.");
          Serial.println("Punch back.");
        }
        punch = 0;
        st[2].set_cmd(0);
      }


      if (false and (pack_count % 10 == 0))
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












