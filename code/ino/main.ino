#include <Servo.h>
#include <Wire.h>  // I2C
#include <LiquidCrystal_I2C.h> // LCD
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ----------- PINS / OBJECTS -----------------------------
Servo servo;  // create servo object to control a servo
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
Adafruit_MPU6050 mpu; // MPU6050

#define SERVO_CONTROL_PIN 8
#define SELECT_BUTTON 6
#define UP_BUTTON 5
#define DOWN_BUTTON 4 
#define REED_SWITCH 3

// button intialization
bool select_button_state = 0, up_button_state = 0, down_button_state = 0, reed_sw_state = 0;
bool select_button_delay = 0, up_button_delay = 0, down_button_delay = 0;
unsigned long last_debounce_time;

unsigned int DEBOUNCE_TIME = 100;

// ----------------- VARIABLES ---------------------------
//servo
int pos = 0;  

// selection
unsigned int LOCK_TIME_INTERVAL = 900; // in seconds (15 mins)

// timing
unsigned long now = 0;
unsigned long last_print = 0;

// timer
unsigned volatile int lock_time_s = 900; // seconds
unsigned long lock_time_start;
unsigned long time_left;
unsigned long elapsed;

unsigned int time_disp_s, time_disp_mins, time_disp_hrs;

//display
char lcd_buffer[16]; 
bool update_lcd = false;

// --------- VIBRATION DETECT stuff -----------
float a_z; // we are only using z-accel
unsigned long last_IMU_update = 0;
bool vib_detect_enable = false; // servo causes vibration

// adjustment/filter
float z_DRIFT_CORRECTION = 0;
float z_baseline = 0;
const float HP_COEFF = 0.10;  // high-pass filter

//RMS
float running_sum = 0;
float mean = 0;
float RMS = 0;

const int RMS_SAMPLES = 50;
int rms_counter = 0; 

// thresholds & sliding window :O
const float THRESHOLD_HIGH = 0.40;
const float THRESHOLD_LOW = 0.12;

const int WINDOW = 10;
const int REQUIRED = 8;

bool history[WINDOW] = {0}; // sliding list of bools
int history_index = 0;
int vibration_ct = 0;

bool vibration_detected = false; // final result
unsigned long stabilization_start = 0;


// ---------- FUNCTION DECLARATIONS -------
bool servo_lock(int target_pos);
bool rising_edge_detect(bool button_state, bool carry_state);
void calc_display_times(unsigned long input_time_s);
void reset_all_timer_vars();
void update_accels();
void calibrate_IMU();
bool vibration_detect();
void setup_IMU();
void reset_all_vibration_vars();

// ---------------- FSM SETUP ----------------
enum FSM_States {
  IDLE,
  LOCKED,
  TAMPER, // timer not completed & lid open
  PAUSE   // timer not completed & lid closed, wait for resume
};

FSM_States state = IDLE;

// ----------------- SETUP -------------------
void setup() {
  Serial.begin(9600);

  // Servo
  servo.attach(SERVO_CONTROL_PIN);  // attaches the servo on pin 9 to the servo object

  // Button + reed switch inputs
  pinMode(SELECT_BUTTON, INPUT_PULLUP);
  pinMode(UP_BUTTON, INPUT_PULLUP);
  pinMode(DOWN_BUTTON, INPUT_PULLUP); 
  pinMode(REED_SWITCH, INPUT_PULLUP);
  
  // LCD
  lcd.init();      
  lcd.backlight();
  lcd.cursor();   
  
  lcd.setCursor(0, 0);
  lcd.print("Time to lock in!"); 

  //IMU
  setup_IMU();
}

// --------------- LOOP --------------------
bool sec_tick = false;
unsigned long last_sec_update = 0;

void loop() {

  // ------------------- UPDATES ------------------
  now = millis();
  update_lcd = false;
  sec_tick = false;

  vib_detect_enable = (state == LOCKED) && (now - stabilization_start > 300);

  if (now - last_sec_update >= 1000){
    sec_tick = true;
    last_sec_update = now;
  }

  // debug prints
  if (millis() - last_print >= 200){
    Serial.print("State: ");
    Serial.print(state);
    Serial.print("| Servo Position: ");
    Serial.print(pos);
    //Serial.print("| Start time: ");
    //Serial.print(lock_time_start);
    Serial.print("| Lock Time: " );
    Serial.print(lock_time_s);
    //Serial.print("| seconds_left: " );
    //Serial.print(time_left);
    Serial.print("| " );
    Serial.print(time_disp_mins);
    Serial.print(" : " );
    Serial.print(time_disp_s);
    Serial.print("| Reed Switch" );
    Serial.print(reed_sw_state);
    Serial.print("| Vibration: " );
    Serial.print(vibration_detected);
    Serial.print("| V_en: " );
    Serial.print(vib_detect_enable);
    Serial.print("| RMS: " );
    Serial.print(RMS);
    Serial.print("| vibration ct " );
    Serial.print(vibration_ct);
    //Serial.print("history " );
    //Serial.print(history);
    Serial.println();

    last_print = now;
  }

  // button states (going through debounce + edge detect)
  if ( (now - last_debounce_time) > DEBOUNCE_TIME) {
    // INPUT_PULLUP --> reverse state
    bool select_raw = !digitalRead(SELECT_BUTTON);
    bool down_raw = !digitalRead(DOWN_BUTTON);
    bool up_raw = !digitalRead(UP_BUTTON);

    // detect a new button press
    select_button_state = rising_edge_detect(select_raw, select_button_delay);
    down_button_state = rising_edge_detect(down_raw, down_button_delay);
    up_button_state = rising_edge_detect(up_raw, up_button_delay);

    // save the raw states for the next iteration
    select_button_delay = select_raw;
    down_button_delay = down_raw;
    up_button_delay = up_raw;

    reed_sw_state = !digitalRead(REED_SWITCH);

    last_debounce_time = now;
  }

  if ( (now - last_IMU_update) > 10 && vib_detect_enable){
    update_accels();
    vibration_detected = (vibration_detect());
  } else {
    reset_all_vibration_vars();
  }

  // ----------------------- FSM -------------------------

  switch (state){ 
    case IDLE: 
      if (servo_lock(150)){ // wait for it to finish unlocking

        // whenever SELECT_BUTTON is pressed, lock
        if (select_button_state){
          time_left = lock_time_s;
          select_button_state = false;

          stabilization_start = now;
          state = LOCKED;
        }

        // time selection
        if (down_button_state) {
          lock_time_s = (lock_time_s < LOCK_TIME_INTERVAL)? 0 : lock_time_s - LOCK_TIME_INTERVAL; // no negative time lol
          down_button_state = false;
          
          calc_display_times(lock_time_s);
          snprintf(lcd_buffer, sizeof(lcd_buffer), "    %02d:%02d:%02d", time_disp_hrs, time_disp_mins, time_disp_s);
          update_lcd = true;
        }

        if (up_button_state){
          lock_time_s = lock_time_s + LOCK_TIME_INTERVAL;
          up_button_state = false;

          calc_display_times(lock_time_s);
          snprintf(lcd_buffer, sizeof(lcd_buffer), "    %02d:%02d:%02d", time_disp_hrs, time_disp_mins, time_disp_s);
          update_lcd = true;
        }

      }
      break;
      
    case LOCKED:
      if (servo_lock(30)){ // wait for it to finish locking
        unsigned long old_time_left = time_left;
        time_left = (sec_tick)? time_left - 1:time_left;

        // re-calculating timer things
        if (time_left <= 0){ // timer is finished
            reset_all_timer_vars();
            snprintf(lcd_buffer, sizeof(lcd_buffer), "TIME TO LOCK IN");
            state = IDLE; // yippee :>
        } else {
            calc_display_times(time_left);
        }

        // LCD updates only when time changes  
        if (old_time_left != time_left){
          snprintf(lcd_buffer, sizeof(lcd_buffer), "    %02d:%02d:%02d", time_disp_hrs, time_disp_mins, time_disp_s);
          update_lcd = true;
        }

        // tamper detection
        if (!reed_sw_state){
          snprintf(lcd_buffer, sizeof(lcd_buffer), "    CLOSE LID");
          update_lcd = true;
          state = TAMPER;
        }

        // "phone call" vibration detection
        if (vibration_detected){
          snprintf(lcd_buffer, sizeof(lcd_buffer), "WHITE TO RESUME");
          update_lcd = true;
          state = PAUSE;
        }
      }
      
      break;

    case TAMPER:
        if (servo_lock(150)){ // servo should be unlocked so that you can put the lid back down
            if (reed_sw_state){ // box is closed again, allow to resume

              delay(200); // a bit glitchy
              snprintf(lcd_buffer, sizeof(lcd_buffer), "WHITE TO RESUME");
              update_lcd = true;
              state = PAUSE;
            } else {
              state = TAMPER;
            }
        }
      break;

    case PAUSE:
        if (servo_lock(150)){

          if (reed_sw_state){
            if (select_button_state){ // go back to LOCKED once middle button is pressed
              delay(200); 
              stabilization_start = now;
              state = LOCKED;
            } else {
              state = PAUSE;
            }
          } else { // if lid is opened again, go back to TAMPER :P
            snprintf(lcd_buffer, sizeof(lcd_buffer), "    CLOSE LID");
            update_lcd = true;
            state = TAMPER; 
          }
        } 
      break;

    default: state = IDLE;
  }   
    


  // LCD update (whenever something changes)
  if (update_lcd){
    lcd.setCursor(0,0);
    lcd.print("                    ");  // clear line 
    lcd.setCursor(0,0);
    lcd.print(lcd_buffer);
  }
}

