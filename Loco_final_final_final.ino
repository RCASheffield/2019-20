
#include <LiquidCrystal.h>

// Output Pin Definitions
#define safety_pin 10
#define brakes_pin 9
#define H1A 6
#define H1B 8
#define H2A 5
#define H2B 7
#define pwm_pin 3
#define horn_pin  4

// Input Pin Definitions
#define autostop_pin_interrupt  19    // This is also connected via a jumper to input 13
#define encoder_1a_interrupt  18    // This is also connected via a jumper to input 25
#define encoder_1b  26
#define encoder_1i  27  // Index

// LCD Display Setup
LiquidCrystal lcd(50, 51, 49, 48, 47, 46);

// Recieved Variables
byte desired_speed = 0;
char drive = 'S';
char horn = 'h';
char autostop = 'a';

// Interrupt Variables
volatile boolean autostop_flag = 0;
volatile unsigned long auto_start_pulse_count = 0;
volatile unsigned long pulse_count = 0;

// Sent Variables
int actual_speed = 0;
byte pwm = 0;

boolean autostop_end = 0;

void setup()
{
  pin_definitions();    // sets pin modes
  digitalWrite(safety_pin, HIGH); // Safety relay on to sustain power to all electronics
  Serial.begin(115200);   // Baud rate must match controller arduino
  lcd.begin(16, 2);
  lcd.clear();
  attachInterrupt(digitalPinToInterrupt(encoder_1a_interrupt), read_encoder1, RISING);    // Creates inerrupt to read the rotary encoder
}

void loop()
{
  static const char control_scheme = 'c';

  // Variables for calculating speed
  static unsigned long loop_start_pulse_count = 0;
  static unsigned long loop_pulse_count = 0;

  // Loop timing variables
  static unsigned long loop_start_time = 0;
  static const unsigned long loop_period = 10;   // sets loop period

  loop_start_time = millis();

  loop_start_pulse_count = pulse_count;
  actual_speed =  1.104466 * loop_pulse_count; // constant = 0.8835729*(R/(G*T)) where R is the wheel radius (0.15m, don't be an idiot and use the diameter, like I did), G is the gear ratio between the encoder and the wheel, and T is the loop period. Outputs speed such that 150 = 15km/h.

  receive_comms();    // checks for incoming comms, includes timeout if nothing received

  change_outputs();   // changes relay states if necessary and resets variables. Includes relay dead time

  if (control_scheme == 'c')
  {
    if (autostop_flag == 1 && autostop == 'A')   // controls speed for autostop triggered mode
    {
      set_pwm(auto_reference());    // auto reference returns the speed setpoint based on distance travelled in autostop mode
    }
    else
    {
      set_pwm(desired_speed);   // sets pwm based on control box setpoint
    }
  }
  else
  {
    analogWrite(pwm_pin, map(desired_speed, 0, 150, 0, 255)); // open loop control, maps and directly outputs desired speed
  }

  update_lcd();

  send_comms();   // sends speed and pwm value to the control box

  while (millis() - loop_start_time < loop_period)  // fixes loop length
  {
  }

  loop_pulse_count = pulse_count - loop_start_pulse_count;    // for calculating speed in next loop iteration
}

void pin_definitions()
{
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(autostop_pin_interrupt, INPUT);
  pinMode(13, INPUT);
}

void read_encoder1()  // Interrupt service routine for counting encoder pulses
{
  pulse_count++;
}

void receive_comms()
{
  static unsigned long time_last_comms_received = 0;
  static const unsigned long timeout_period = 95;  // sets timeout for loss of communications

  while (Serial.available() > 0)
  {
    Serial.find(',');   // read until delimiter found. Prevents errors if initial messages do not line up
    while (Serial.available() < 4)    // wait for full message
    {
    }
    desired_speed = byte(Serial.read());    // desired speed is sent as a single 8-bit character - converted into a byte for use in the code
    horn = Serial.read();
    drive = Serial.read();
    autostop = Serial.read();
    time_last_comms_received = millis();
  }
  if (millis() - time_last_comms_received > timeout_period)  // shutdown loco if communication is lost
  {
    drive = 'S';
    horn = 'h';
    autostop = 'a';
    desired_speed = 0;
  }
}

void change_outputs()
{
  static char prev_drive = 'S';
  static char prev_horn = 'h';
  static char prev_autostop = 'a';
  static unsigned long bridge_low_time = 0;
  static const unsigned long bridge_dead_time = 200;
  static boolean bridge_switch_flag = 0;

  if (horn != prev_horn)
  {
    switch (horn)
    {
      case 'h':
        lcd.setCursor(0, 0);
        lcd.write("h");
        digitalWrite(horn_pin, LOW);
        break;

      case 'H':
        lcd.setCursor(0, 0);
        lcd.write("H");
        digitalWrite(horn_pin, HIGH);
        break;

      default:
        lcd.setCursor(0, 0);
        lcd.write("E");
        break;
    }
  }

  if (drive != prev_drive)
  {
    switch (drive)
    {
      case 'S':
        lcd.setCursor(0, 1);
        lcd.write("Stopped");
        digitalWrite(brakes_pin, LOW);
        digitalWrite(H1A, LOW);
        digitalWrite(H1B, LOW);
        digitalWrite(H2A, LOW);
        digitalWrite(H2B, LOW);
        break;

      case 'N':
        lcd.setCursor(0, 1);
        lcd.write("Neutral");
        digitalWrite(brakes_pin, HIGH);
        digitalWrite(H1A, LOW);
        digitalWrite(H1B, LOW);
        digitalWrite(H2A, LOW);
        digitalWrite(H2B, LOW);
        break;

      case 'F':
        lcd.setCursor(0, 1);
        lcd.write("Forward");
        digitalWrite(brakes_pin, HIGH);
        digitalWrite(H1A, LOW);
        digitalWrite(H1B, LOW);
        digitalWrite(H2A, LOW);
        digitalWrite(H2B, LOW);
        bridge_low_time = millis();
        bridge_switch_flag = 1;
        break;

      case 'R':
        lcd.setCursor(0, 1);
        lcd.write("Reverse");
        digitalWrite(brakes_pin, HIGH);
        digitalWrite(H1A, LOW);
        digitalWrite(H1B, LOW);
        digitalWrite(H2A, LOW);
        digitalWrite(H2B, LOW);
        bridge_low_time = millis();
        bridge_switch_flag = 1;
        break;

      default:
        lcd.setCursor(0, 1);
        lcd.write("Error  ");
        digitalWrite(brakes_pin, LOW);
        break;
    }
  }

  if (bridge_switch_flag == 1)
  {
    if (millis() - bridge_low_time > bridge_dead_time)
    {
      bridge_switch_flag = 0;
      switch (drive)
      {
        case 'F':
          digitalWrite(H1A, HIGH);
          digitalWrite(H2B, HIGH);
          break;

        case 'R':
          digitalWrite(H1B, HIGH);
          digitalWrite(H2A, HIGH);
          break;
      }
    }
  }

  if (autostop != prev_autostop)
  {
    switch (autostop)
    {
      case 'a':
        lcd.setCursor(1, 0);
        lcd.write("a");
        break;

      case 'A':
        lcd.setCursor(1, 0);
        lcd.write("A");
        autostop_flag = 0;
        autostop_end = 0;
        attachInterrupt(digitalPinToInterrupt(autostop_pin_interrupt), autostop_ISR, LOW);
        break;

      default:
        lcd.setCursor(1, 0);
        lcd.write("E");
        break;
    }
  }
  prev_drive = drive;
  prev_horn = horn;
  prev_autostop = autostop;
}

void autostop_ISR()   // Yay Lawrence did his thing. The train may have a seizure.
{
  auto_start_pulse_count = pulse_count;
  autostop_flag = 1;
  detachInterrupt(digitalPinToInterrupt(autostop_pin_interrupt));   // Prevents autostop triggering again (output generally flickers)
}

byte auto_reference()   // 81487 pulses = 25m travelled 
{
  static const unsigned long stopping_counter = 79400;
  static const unsigned long mech_engage_count = 79470;   // 74870 pulses = 25m - length of train   // 79476 pulses is calibrated 25 metres, accounting for length of loco.

  static const unsigned long stopping_divisor = stopping_counter / 100;
  static unsigned long auto_pulse_count = 0;
  static byte auto_speed_setpoint = 100;    // loco should be going at above 10kmph before autostop entered

  auto_pulse_count = pulse_count - auto_start_pulse_count;
  if (auto_pulse_count < stopping_counter)
  {
    auto_speed_setpoint = 101 - auto_pulse_count / stopping_divisor;
  }
  else if (auto_pulse_count > mech_engage_count)
  {
    auto_speed_setpoint = 0;
    autostop_end = 1;
    autostop_flag = 0;
    digitalWrite(brakes_pin, LOW);
//    digitalWrite(H1A, LOW);
//    digitalWrite(H1B, LOW);
//    digitalWrite(H2A, LOW);
//    digitalWrite(H2B, LOW);
  }
  return auto_speed_setpoint;
}

void set_pwm(int requested)
{
  // Timing variables
  static int pwm_update_period = 50;
  static unsigned long last_pwm_update = 0;

  // Control calculation variables
  static int control_output = 0;
  static int error = 0;
  static long integral_error = 0;
  static long new_integral_error = 0;
  static boolean windup_flag = 1;

  // Gains - need tuning.
  static const float kp = 6;    // higher gain: faster response, lower steady state offset (if ki = 0), more likely to wheelslip
  static const float ki = 0.02 * pwm_update_period / 100;   // increase gain to remove offset faster, but may induce oscillations. Should be several orders of magnitude lower than kp.

  if (millis() - last_pwm_update > pwm_update_period)   // fixes pwm update period
  {
    if (drive == 'F' || drive == 'R')   // only outputs value if in forwards or reverse drive mode
    {
      error = requested - actual_speed;
      new_integral_error = integral_error + error;

      control_output = kp * error + ki * new_integral_error;    // calculation of control output

      // Anti Windup Measures
      windup_flag = 1;
      if (control_output > 255)
      {
        pwm = 255;
        if (error > 0)
        {
          windup_flag = 0;
        }
      }
      else if (control_output < 0)
      {
        pwm = 0;
        if (error < 0)
        {
          windup_flag = 0;
        }
      }
      else
      {
        pwm = control_output;
      }
      if (windup_flag == 1)   // only changes integrator when not wound up
      {
        integral_error = new_integral_error;
      }
    }
    else if (drive == 'S' || drive == 'N')    // if drive is stopped or neutral, make output 0
    {
      integral_error = 0;   // resets integral error to prevent it being non-zero when switched back into forwards or reverse
      pwm = 0;
    }
    if (autostop_end == 1)
    {
      pwm = 0;
    }
    analogWrite(pwm_pin, pwm);    // outputs calculated pwm value
  }
}

void update_lcd()   // prints out some information
{
  static unsigned long lcd_update_time = 0;
  static const unsigned long lcd_update_period = 1000;
  static float actual_speed_kmph = 0;
  static float desired_speed_kmph = 0;

  if (millis() - lcd_update_time > lcd_update_period)   // lcd update speed information
  {
    desired_speed_kmph = desired_speed / 10.0;
    lcd.setCursor(3, 0);
    lcd.print("    ");
    lcd.setCursor(3, 0);
    lcd.print(desired_speed_kmph, 1);
    actual_speed_kmph = actual_speed / 10.0;
    lcd.setCursor(12, 0);
    lcd.print("    ");
    lcd.setCursor(12, 0);
    lcd.print(actual_speed_kmph, 1);
  }
}

void send_comms()
{
  static unsigned long time_last_comms_sent = 0;
  static const unsigned long send_period = 500;     // sets data send interval

  if (millis() - time_last_comms_sent > send_period)  // sends communications at fixed period
  {
    Serial.print(',');
    Serial.print(char(actual_speed));
    Serial.print(char(pwm));
    time_last_comms_sent = millis();
  }
}
