#define NUMSTEPS 4
#define TOGGLE_IO        13  //Arduino pin to toggle in timer ISR
#define TIMER_CLOCK_FREQ 2000000.0 //2MHz for /8 prescale from 16MHz
#define TIMER_STEP 150;
#define PRINT true
#define TESTMODE true


#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

unsigned int latency;
unsigned char timerLoadValue;



unsigned char SetupTimer2(float timeoutFrequency) {
  unsigned char result; //The value to load into the timer to control the timeout interval.
  result = (int)((257.0 - (TIMER_CLOCK_FREQ / timeoutFrequency)) + 0.5); //the 0.5 is for rounding;
  TCCR2A = 0;
  TCCR2B = 1 << CS22 | 0 << CS21 | 0 << CS20;
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


volatile long num = 0;

class LarStepper
{
  public:
    LarStepper(float _scale, int _step, int _dir, int _home);
    void set_cmd(float c);
    void update_freq();
    void dash();
    int step_pin;
    int dir_pin;
    int home_pin;
    volatile long timer;
    volatile long counter;
    volatile long home;
    volatile long homing_dir;
    volatile long pos = 0;
    float v = 0.;
    float a = 100.;
    float cmd = 0;
    float scale = 100.;
    float maxv = 100.;
    float minl = -1000000000000;
    float maxl = -1000000000000;
    float e = 0;
    float p = 0;
    float t = 0;
    float t1;
};




LarStepper::LarStepper(float _scale, int _step, int _dir, int _home)
{
  pos = 0;
  step_pin = _step;
  dir_pin = _dir;
  home = 0;
  home_pin = _home;
  homing_dir = 1;
  scale = _scale;
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(home_pin, INPUT);
}


void LarStepper::set_cmd(float c)
{
  if (c > maxl) cmd = maxl;
  else if (c < minl) cmd = minl;
  else cmd = c;
}

float period;
long time_;

void LarStepper::update_freq()
{
  p = float(pos) / scale;
  e = cmd - p;

  if (v * e < 0)
  {
    if (v > 0) v -= a * period;
    else v += a * period;
  }
  else {
    t = abs(v / a);
    t1 = (abs(v) - a * period) * t - a * t * t * .5 ;

    if (abs(t1) < abs(e))
    { // accel
      if (v > 0) v += a * period;
      else v -= a * period;
    }
    else
    { //deccel
      if (v > 0) v -= a * period;
      else v += a * period;
    }
  }

  if (v < -maxv) v = -maxv;
  if (v > maxv) v = maxv;
  // step timer mks
  if ( abs(v) < 0.001 ) timer = 1000000000;
  else timer = abs(long(1000000 / ( v * scale )));
}


LarStepper st[4] = {
  LarStepper(10., 1, 2, 3),
  LarStepper(10., 4, 5, 6),
  LarStepper(10., 7, 8, 9),
  LarStepper(10., 7, 8, 9)
};


byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(10, 0, 1, 5);
unsigned int localPort = 8888;      // local port to listen on
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged";       // a string to send back
EthernetUDP Udp;

//Timer2 overflow interrupt vector handler
ISR(TIMER2_OVF_vect) {
  num += 1;
  if (num > 1000)  digitalWrite(13, !digitalRead(13));
  num = num % 2000;

  int pins = PORTD << 8 || PORTB;
  for (int i = 0; i < NUMSTEPS; i++)
  {
    st[i].counter += TIMER_STEP;
    if (pins & 1 << st[i].step_pin ) {
      pins &= (65535 - 1 << st[i].step_pin);
    }
    else if (st[i].counter >= st[i].timer) {
      st[i].counter = st[i].counter % st[i].timer;
      if (st[i].v > 0) st[i].pos += 1;
      else st[i].pos -= 1;
      pins |= 1 << st[i].step_pin;
    }
    if (st[i].v > 0) pins |=  1 << st[i].step_pin;
    else pins &= (65535 - 1 << st[i].step_pin);
  }
  PORTB = pins;
  PORTD = (PORTD & 192) | pins >> 8;


  //Capture the current timer value. This is how much error we have
  //due to interrupt latency and the work in this function
  latency = TCNT2;
  //Reload the timer and correct for latency.  //Reload the timer and correct for latency.  //Reload the timer and correct for latency.
  //TCNT2 = latency + timerLoadValue;

}


void setup() {
  Serial.begin(115200);

  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  noInterrupts();
  timerLoadValue = SetupTimer2(80000);
  interrupts();
  st[0].cmd = 100;


}

long print__ = 0;
bool print_;
Fly *fly;

void loop() {
  // put your main code here, to run repeatedly:

  period = (micros() - time_) * 0.000001;
  time_ = micros();
  for  (int i = 0; i < NUMSTEPS; i++)
  {
    st[i].update_freq();
  }
  print_ = false;
  if (print__ < millis() && PRINT ) {
    print__ = millis() + 400;
    print_ = true;
  }

  if (print_)
  {Serial.println(latency);
    for (int i = 0; i < NUMSTEPS; i++)
    {
      
      Serial.print(i);
      Serial.print(", ");
      Serial.print(st[i].cmd);
      Serial.print(", ");
      Serial.print(st[i].p);
      Serial.print(", ");
      Serial.print(st[i].e);
      Serial.print(", ");
      Serial.print(st[i].v);
      Serial.println(", ");
    }
    Serial.println(", ");
  }



  if (TESTMODE) {
      st[0].cmd = sin(0.001*float(millis())/40.*3.1415)*360;
      st[1].cmd = cos(0.001*float(millis())/40.*3.1415)*360;
    }
  else {

    int packetSize = Udp.parsePacket();
    if (packetSize) {
      // read the packet into packetBufffer
      Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      fly = (Fly*)packetBuffer;
      st[0].cmd = fly->pitch;
      st[1].cmd = fly->bank;
    }
    if (print_)
    {
      Serial.println("Contents:");
      Serial.println(fly->sig);
      Serial.println(fly->pitch);
      Serial.println(fly->bank);
      Serial.println(fly->z);
      Serial.println(fly->state);
      Serial.println(fly->sum);
      //    delay(100);
    }
  }


  /* Serial.println(latency);
    Serial.print(", cmd");
    Serial.print(st[0].cmd);
    Serial.print(", t1 ");
    Serial.print(st[0].t1);
    Serial.print(", t");
    Serial.print(st[0].t);
    Serial.print(", p");
    Serial.print(st[0].p);
    Serial.print(", e");
    Serial.print(st[0].e);
    Serial.print(", v");
    Serial.print(st[0].v);
    Serial.print(", timer ");
    Serial.print(st[0].timer);
    Serial.print(", pos ");
    Serial.println(st[0].pos);
    Serial.print(", time ");
    Serial.print(time_);
    Serial.print(", period ");
    Serial.println(period);*/



}


