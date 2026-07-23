#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>  // I2C
#include <LiquidCrystal_I2C.h> // LCD

// ----------- PINS / OBJECTS -----------------------------
Servo servo;  // create servo object to control a servo
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

#define SERVO_CONTROL_PIN 9
#define BUTTON_PIN 7

// ----------------- VARIABLES ---------------------------
int pos = 0;    // variable to store the servo position
unsigned long now = 0;
unsigned long last_print = 0;

int lock_time = 5; // seconds
unsigned long lock_time_ms = lock_time * 1000;
unsigned long lock_time_start;

bool timer_done = false;

// ---------- FUNCTION DECLARATIONS -------
bool servo_lock(int target_pos);


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

  // Button input (for timer start)
  pinMode(BUTTON_PIN, INPUT);
  
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
     Serial.print("| Now: " );
    Serial.print(now);
    Serial.println();

    last_print = millis();
  }

  switch (state){
    case IDLE:
      if (servo_lock(30)){ // wait for it to finish unlocking
        if (digitalRead(BUTTON_PIN) == HIGH){
          lock_time_start = millis();
          state = LOCKED;
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
