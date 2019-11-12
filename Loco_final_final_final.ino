
#include <LiquidCrystal.h>

// Output pin definitions
#define safety_pin 10
#define brakes_pin 9
#define H1A 6
#define H1B 8
#define H2A 5
#define H2B 7
#define pwm_pin 3
#define horn_pin  4

// Input pin definitions
#define autostop_pin_interrupt  19    // This is also connected via a jumper to input 13
#define encoder_1a_interrupt  18    // This is also connected via a jumper to input 25
#define encoder_1b  26
#define encoder_1i  27  // Index

// LCD display setup
LiquidCrystal lcd(50, 51, 49, 48, 47, 46);

// Variables receiver from control box
byte desired_speed = 0;
char drive = 'S';
char horn = 'h';
char autostop = 'a';

// Variables sent to control box
int actual_speed = 0;
byte pwm = 0;

// Autostop variables
volatile boolean autostop_flag = 0;
volatile unsigned long auto_start_pulse_count = 0;
boolean autostop_end = 0;

// Encoder interruot variables
volatile unsigned long pulse_count = 0;

// Runs once on power on
void setup()
{
  pin_definitions();  // Set pin modes
  digitalWrite(safety_pin, HIGH); // Safety relay on to sustain power to all electronics
  Serial.begin(115200); // Baud rate must match control box
  lcd.begin(16, 2); // Initialise LCD (16 characters and 2 rows)
  lcd.clear();  // CLear LCD
  attachInterrupt(digitalPinToInterrupt(encoder_1a_interrupt), read_encoder1, RISING);  // Attatch interrupt to read the rotary encoder
}

// Runs continuously while powered on
void loop()
{
  // Set control scheme: c = closed loop, C= open loop
  // This is for testing only, cannot change without reuploading
  static const char control_scheme = 'c';

  // Variables for calculating speed
  static unsigned long loop_start_pulse_count = 0;
  static unsigned long loop_pulse_count = 0;

  // Loop timing variables
  static unsigned long loop_start_time = 0;
  static const unsigned long loop_period = 10;  // sets loop period

  loop_start_time = millis();

  loop_start_pulse_count = pulse_count;

  // constant = 0.8835729*(R/(G*T)) where R is the wheel radius (0.15m, don't be an idiot and use the diameter, like I did), 
  // G is the gear ratio between the encoder and the wheel, and T is the loop period. Outputs speed such that 150 = 15km/h.
  actual_speed =  1.104466 * loop_pulse_count;

  receive_comms();    // Check for incoming comms, includes timeout if nothing received
  change_outputs();   // Change relay states if necessary and resets variables. Includes relay dead time

  // Closed loop control
  if (control_scheme == 'c')
  {
    if (autostop_flag == 1 && autostop == 'A')   // Control speed for autostop triggered mode
    {
      set_pwm(auto_reference());    // Auto reference returns the speed setpoint based on distance travelled in autostop mode
    }
    else
    {
      set_pwm(desired_speed);   // Sets pwm based on control box setpoint
    }
  }
  // Open loop control
  else if (control_scheme == 'C')
  {
    analogWrite(pwm_pin, map(desired_speed, 0, 150, 0, 255)); // Open loop control, maps and directly outputs desired speed
  }
  // Unknown mode, default to setting speed to 0
  else 
  {
    analogWrite(pwm_pin, 0);
  }


  update_lcd();   // Update LCD
  send_comms();   // Send current speed and pwm value to control box

  // Delay until loop has met desired duration in order to maintain correct loop frequency
  while (millis() - loop_start_time < loop_period) {}

  loop_pulse_count = pulse_count - loop_start_pulse_count;    // For calculating speed in next loop iteration
}

// Set pin input/output modes
void pin_definitions()
{
  // Outputs
  pinMode(safety_pin, OUTPUT);
  pinMode(brakes_pin, OUTPUT);
  pinMode(H1A, OUTPUT);
  pinMode(H1B, OUTPUT);
  pinMode(H2A, OUTPUT);
  pinMode(H2B, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  pinMode(horn_pin, OUTPUT);

  // Inputs
  pinMode(autostop_pin_interrupt, INPUT);
  pinMode(encoder_1a_interrupt, INPUT);
  pinMode(encoder_1b, INPUT);
  pinMode(encoder_1i, INPUT);
}

// Interrupt service routine for counting encoder pulses
void read_encoder1()
{
  pulse_count++;
}

// Receive data from control box over serial
void receive_comms()
{
  static unsigned long time_last_comms_received = 0;
  static const unsigned long timeout_period = 95;  // Serial timeout duration

  // While there is data to be read
  while (Serial.available() > 0)
  {
    Serial.find(',');   // Read until delimiter found. Prevents errors if initial messages do not line up
    while (Serial.available() < 4)    // Wait for full message
    {
    }
    // Read varials from serial
    desired_speed = byte(Serial.read());    // Desired speed is sent as a single 8-bit character - converted into a byte for use in the code
    horn = Serial.read();
    drive = Serial.read();
    autostop = Serial.read();

    time_last_comms_received = millis();
  }
  if (millis() - time_last_comms_received > timeout_period)  // Shutdown loco if serial timeout exceeded
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

  // Check if horn input has changed
  if (horn != prev_horn)
  {
    switch (horn)
    {
      // Horn off
      case 'h':
        lcd.setCursor(0, 0);
        lcd.write("h");
        digitalWrite(horn_pin, LOW);
        break;

      // Horn on
      case 'H':
        lcd.setCursor(0, 0);
        lcd.write("H");
        digitalWrite(horn_pin, HIGH);
        break;

      // Unknown value
      default:
        lcd.setCursor(0, 0);
        lcd.write("E");
        break;
    }
  }

  // Check if drive input has changed
  if (drive != prev_drive)
  {
    switch (drive)
    {
      // Stop
      case 'S':
        lcd.setCursor(0, 1);
        lcd.write("Stopped");
        digitalWrite(brakes_pin, LOW);
        digitalWrite(H1A, LOW);
        digitalWrite(H1B, LOW);
        digitalWrite(H2A, LOW);
        digitalWrite(H2B, LOW);
        break;

      // Neutral
      case 'N':
        lcd.setCursor(0, 1);
        lcd.write("Neutral");
        digitalWrite(brakes_pin, HIGH);
        digitalWrite(H1A, LOW);
        digitalWrite(H1B, LOW);
        digitalWrite(H2A, LOW);
        digitalWrite(H2B, LOW);
        break;

      // Forwards
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

      // Reverse
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

      // Unknown value
      default:
        lcd.setCursor(0, 1);
        lcd.write("Error  ");
        digitalWrite(brakes_pin, LOW);
        break;
    }
  }

  // Check if motor direction needs changing
  if (bridge_switch_flag == 1)
  {
    // Check if enough time has elapsed before setting new direction
    if (millis() - bridge_low_time > bridge_dead_time)
    {
      bridge_switch_flag = 0;
      switch (drive)
      {
        // Set H-Bridge forwards
        case 'F':
          digitalWrite(H1A, HIGH);
          digitalWrite(H2B, HIGH);
          break;

        // Set H-Bridge reverse
        case 'R':
          digitalWrite(H1B, HIGH);
          digitalWrite(H2A, HIGH);
          break;
      }
    }
  }

  // Check if autostop input has changed
  if (autostop != prev_autostop)
  {
    switch (autostop)
    {
      // No autostop
      case 'a':
        lcd.setCursor(1, 0);
        lcd.write("a");
        break;

      // Setup autostop
      case 'A':
        lcd.setCursor(1, 0);
        lcd.write("A");
        autostop_flag = 0;
        autostop_end = 0;
        attachInterrupt(digitalPinToInterrupt(autostop_pin_interrupt), autostop_ISR, LOW);
        break;

      // Unknown value
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

// Called when autostop mode enabled and autostop pin triggered
void autostop_ISR()
{
  auto_start_pulse_count = pulse_count; // Store current encoder value so control code can calculate distance travelled
  autostop_flag = 1;  // Set flag to tell control code to start decelerate
  detachInterrupt(digitalPinToInterrupt(autostop_pin_interrupt));   // Prevents autostop triggering again (output generally flickers)
}

// Determine pwm output when in autostop mode
// Target speed is proportional to distance from 25m
byte auto_reference()   // 81487 pulses = 25m travelled 
{
  static const unsigned long stopping_counter = 79400;
  static const unsigned long mech_engage_count = 79470;   // 74870 pulses = 25m - length of train   // 79476 pulses is calibrated 25 metres, accounting for length of loco.

  static const unsigned long stopping_divisor = stopping_counter / 100;
  static unsigned long auto_pulse_count = 0;
  static byte auto_speed_setpoint = 100;    // Loco should be going at above 10kmph before autostop entered

  auto_pulse_count = pulse_count - auto_start_pulse_count;  // Calculate distance traveled

  // If haven't reached 25m yet
  if (auto_pulse_count < stopping_counter)
  {
    auto_speed_setpoint = 101 - auto_pulse_count / stopping_divisor;
  }
  // If reached 25m
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

  if (millis() - last_pwm_update > pwm_update_period)   // Fixes pwm update period
  {
    if (drive == 'F' || drive == 'R')   // Only outputs value if in forwards or reverse drive mode
    {
      error = requested - actual_speed;
      new_integral_error = integral_error + error;

      control_output = kp * error + ki * new_integral_error;    // Calculation of control output

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
      if (windup_flag == 1)   // Only changes integrator when not wound up
      {
        integral_error = new_integral_error;
      }
    }
    else if (drive == 'S' || drive == 'N')    // If drive is stopped or neutral, make output 0
    {
      integral_error = 0;   // Resets integral error to prevent it being non-zero when switched back into forwards or reverse
      pwm = 0;
    }
    if (autostop_end == 1)
    {
      pwm = 0;
    }
    analogWrite(pwm_pin, pwm);    // Outputs calculated pwm value
  }
}

// Update LCD contents
void update_lcd()
{
  static unsigned long lcd_update_time = 0;
  static const unsigned long lcd_update_period = 1000;
  static float actual_speed_kmph = 0;
  static float desired_speed_kmph = 0;

  if (millis() - lcd_update_time > lcd_update_period)   // Update at a fixed interval
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

// Send data back to control box
void send_comms()
{
  static unsigned long time_last_comms_sent = 0;
  static const unsigned long send_period = 500;

  if (millis() - time_last_comms_sent > send_period)  // Sends communications at fixed interval
  {
    Serial.print(',');
    Serial.print(char(actual_speed));
    Serial.print(char(pwm));
    time_last_comms_sent = millis();
  }
}
