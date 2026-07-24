#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>  // I2C
#include <LiquidCrystal_I2C.h> // LCD

// ----------- PINS / OBJECTS -----------------------------
Servo servo;  // create servo object to control a servo
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

#define SERVO_CONTROL_PIN 8
#define SELECT_BUTTON 6
#define UP_BUTTON 5
#define DOWN_BUTTON 4 

bool select_button_state = 0, up_button_state = 0, down_button_state = 0;
bool select_button_delay = 0, up_button_delay = 0, down_button_delay = 0;
unsigned long last_debounce_time;
unsigned int DEBOUNCE_TIME = 100;


// ----------------- VARIABLES ---------------------------

int LOCK_TIME_INTERVAL = 15; // seconds

int pos = 0;    // variable to store the servo position
unsigned long now = 0;
unsigned long last_print = 0;

volatile int lock_time = 5; // seconds
unsigned long lock_time_ms;
unsigned long lock_time_start;

bool timer_done = false;

// ---------- FUNCTION DECLARATIONS -------
bool servo_lock(int target_pos);
bool rising_edge_detect(bool button_state, bool carry_state);

// ---------------- FSM SETUP ----------------
enum FSM_States {
  IDLE,
  LOCKED
};

FSM_States state = IDLE;

// ----------------- SETUP -------------------
void setup() {
  Serial.begin(9600);

  // Servo
  servo.attach(SERVO_CONTROL_PIN);  // attaches the servo on pin 9 to the servo object

  // Button inputs
  pinMode(SELECT_BUTTON, INPUT_PULLUP);
  pinMode(UP_BUTTON, INPUT_PULLUP);
  pinMode(DOWN_BUTTON, INPUT_PULLUP); 
  
  // LCD
  lcd.init();      
  lcd.backlight();
  lcd.cursor();              
}

// --------------- LOOP --------------------
void loop() {
  now = millis();

  if (millis() - last_print >= 200){
    Serial.print("State: ");
    Serial.print(state);
    Serial.print("| Servo Position: ");
    Serial.print(pos);
    Serial.print("| Start time: ");
    Serial.print(lock_time_start);
    Serial.print("| Lock Time: " );
    Serial.print(lock_time);
    Serial.println();

    last_print = millis();
  }

  // button states (going through debounce + edge detect)

  if ( (millis() - last_debounce_time) > DEBOUNCE_TIME) {
    // Read the raw button states (invert because INPUT_PULLUP)
    bool select_raw = !digitalRead(SELECT_BUTTON);
    bool down_raw   = !digitalRead(DOWN_BUTTON);
    bool up_raw     = !digitalRead(UP_BUTTON);

    // Detect a new button press
    select_button_state = rising_edge_detect(select_raw, select_button_delay);
    down_button_state   = rising_edge_detect(down_raw, down_button_delay);
    up_button_state     = rising_edge_detect(up_raw, up_button_delay);

    // Save the raw states for the next iteration
    select_button_delay = select_raw;
    down_button_delay   = down_raw;
    up_button_delay     = up_raw;

    last_debounce_time = now;
  }

  switch (state){
    case IDLE:
      if (servo_lock(30)){ // wait for it to finish unlocking

        // whenever SELECT_BUTTON is pressed, lock
        if (select_button_state){
          lock_time_ms = lock_time * 1000;
          lock_time_start = millis();
          select_button_state = false;
          state = LOCKED;
        }

        // time selection
        if (down_button_state) {
          lock_time = (lock_time < LOCK_TIME_INTERVAL)? LOCK_TIME_INTERVAL : lock_time - LOCK_TIME_INTERVAL; // no negative time lmao
          down_button_state = false;
          //delay(200);
        }

        if (up_button_state){
          lock_time = lock_time + LOCK_TIME_INTERVAL;
          up_button_state = false;
          //delay(200);
        }
      }
      break;
      
    case LOCKED:
      if (servo_lock(150)){ // wait for it to finish locking
          if (now - lock_time_start >= lock_time_ms){
          state = IDLE;
        }
      }
      break;
  }
}

// ------------ FUNCTIONS -----------------

// controls servo angle (lock)
bool servo_lock(int target_pos){ 
  if (pos == target_pos){
    return true;
  } 

  // write in increments
  pos = (target_pos > pos)? (pos + 1):(pos - 1);
  servo.write(pos);
  return false;
}

/*

void setup()
{
  
  // Print a message to the LCD.
  

  lcd.setCursor(2,0);
  lcd.print("Hello, world!");

  lcd.setCursor(2,1);
  lcd.print("It's Scarlett!");

  lcd.setCursor(0,2);
  
}


void loop()
{
}

*/
