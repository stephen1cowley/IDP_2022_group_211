// Integrated Design Project
// Final version
// To be uploaded to the Arduino Uno WiFi Rev2
// Stephen Cowley and Yeming Ba (Group 211), November 2022

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <SharpIR.h>

// LED digital ports
#define red_LED 8
#define amber_LED 9
#define green_LED 10

// Infrared constants
#define IR_pin A3 
#define IR_model 1080

// Ultrasound constants
#define echoPin1 7
#define trigPin1 6
#define echoPin2 5
#define trigPin2 4
#define US_threshold 7

// Motor initialisations
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1); // RHS
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); // LHS

// Infrared initialisation
SharpIR IR_sensor = SharpIR(IR_pin, IR_model);

// Robot state definitions
enum line_following_status_t { ON_TRACK = 0, TOO_FAR_RIGHT = 1, TOO_FAR_LEFT = 2, LOST_SOMEWHERE = 3 };
enum last_known_extremity_t { UNKNOWN = 0, TO_THE_RIGHT = 1, TO_THE_LEFT = 2, TUNNEL = 3 };
enum light_level_t { LIGHT = 0, DARK = 1 };
enum working_US_t { NO = 0, YES = 1 };
enum in_box_t { AFFIRMATIVE = 0, NEGATIVE = 1, BLOCK_SEARCHING = 2 };
enum in_box_state_t { state_1 = 0, state_2 = 1, state_3 = 2, state_4 = 3 };
enum ultrasound_state_t { first = 0, second = 1, third = 2 };
enum infrared_detection_state_t { not_on_lookout = 0, on_lookout = 1 };
enum junction_state_t { st_1 = 0, st_2 = 1 };

// Initial robot states
line_following_status_t line_following_status = ON_TRACK;
last_known_extremity_t last_known_extremity = UNKNOWN;
light_level_t light_level = LIGHT;
working_US_t working_US = NO;
ultrasound_state_t ultrasound_state = first;
in_box_t in_box = AFFIRMATIVE;
in_box_state_t in_box_state = state_1;
infrared_detection_state_t infrared_detection_state = on_lookout;
junction_state_t junction_state = st_1;

// Optoswitch (OPB) sensor variables
int RHS_sensor_threshold = 500;
int LHS_sensor_threshold = 550;
int opto_sensor_0;
int opto_sensor_1; 
int RHS_digital_opto;
int LHS_digital_opto;
static double prev_opto_sensor_0 = 0;
static double prev_opto_sensor_1 = 0;

// LDR variables
int LDR;
int LDR_threshold = 500;

// Ultrasound variables
long US_duration_1 = 0;
int US_distance_1 = 100;
long US_duration_2 = 0;
int US_distance_2 = 100;

// IR variables and constants
long duration;
int distance;
int distance_IR;
int critical_dist = 12;
int critical_dist_UT = critical_dist + 8;
const int NumReadings = 80;

// Situation variables
int too_far_situations_parsed = 1;
int initial_situations_parsed = 0;
int junctions_passed = 0;

// Timer variables and constants
unsigned long current_micros;
unsigned long previous_micros_0 = 0;
unsigned long previous_micros_1 = 0;
unsigned long previous_micros_2 = 0;
unsigned long previous_micros_3 = 0;
unsigned long previous_micros_tfl = 0;
const long interval_1 = 2;
const long interval_2 = 10;

// Function prototypes
void setup(void);
void read_sensors(void);
void determine_sensor_states(void);
void initial_box_exit(void);
void normal_line_following(void);
void determine_junctions_passed(void);
void read_ultrasonic_transducers(void);
void begin_block_searching(void);
void block_detection(void);
void tunnel_code(void);
void hard_coded_box_return(void);
void loop(void);

// Functions
// setup() will run initially, loop() will loop indefinetely

void setup() {
  AFMS.begin();
  Serial.begin(9600);

  pinMode(red_LED, OUTPUT);
  pinMode(green_LED, OUTPUT);
  pinMode(amber_LED, OUTPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}

void read_sensors() {
  // Read all sensors and asign to variables
  opto_sensor_0 = analogRead(A0);
  opto_sensor_1 = analogRead(A1); 
  RHS_digital_opto = digitalRead(0);
  LHS_digital_opto = digitalRead(1);
  LDR = analogRead(A2);
  distance_IR = IR_sensor.distance();
  current_micros = micros();
}

void determine_sensor_states() {
  // Determine light sensor states
  if ((opto_sensor_0 >= RHS_sensor_threshold) && (opto_sensor_1 >= LHS_sensor_threshold)) {
    line_following_status = ON_TRACK;
  }
  else if ((opto_sensor_0 <= RHS_sensor_threshold) && (opto_sensor_1 >= LHS_sensor_threshold)) {
    line_following_status = TOO_FAR_RIGHT;
  }
  else if ((opto_sensor_0 >= RHS_sensor_threshold) && (opto_sensor_1 <= LHS_sensor_threshold)) {
    line_following_status = TOO_FAR_LEFT;
  }
  else if ((opto_sensor_0 <= RHS_sensor_threshold) && (opto_sensor_1 <= LHS_sensor_threshold)) {
    line_following_status = LOST_SOMEWHERE;
  }

  // Determine LDR states
  if (LDR <= LDR_threshold) {
    light_level = DARK;
  }
  else if (LDR >= LDR_threshold) {
    light_level = LIGHT;
  }
}

void initial_box_exit() {
  // Leave the start box bearing right and find white line
  if (initial_situations_parsed == 0) {
    delay(3000);
    initial_situations_parsed = 1;
  }
  if ((in_box_state == state_1) && (RHS_digital_opto == 0)) {
    myMotor1->setSpeed(150);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(150);
    myMotor2->run(BACKWARD);
  }
  if ((in_box_state == state_1) && (RHS_digital_opto == 1)) {
    myMotor1->setSpeed(150);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(150);
    myMotor2->run(BACKWARD);
    in_box_state = state_2;
  }
  if ((in_box_state == state_2) && (RHS_digital_opto == 1)) {
    myMotor1->setSpeed(150);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(150);
    myMotor2->run(BACKWARD);
    in_box_state = state_2;
  }
  if ((in_box_state == state_2) && (RHS_digital_opto == 0)) {
    myMotor1->setSpeed(150);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(150);
    myMotor2->run(BACKWARD);
    in_box_state = state_3;
  }
  if ((in_box_state == state_3) && (RHS_digital_opto == 0)) {
    myMotor1->setSpeed(100);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(180);
    myMotor2->run(BACKWARD);
    in_box_state = state_3;
  }
  if ((in_box_state == state_3) && (line_following_status == ON_TRACK) && (current_micros >= 5.5*1000000)) {
    in_box = NEGATIVE;
  }
}

void normal_line_following() {
  // Normal navigation when not in tunnel
  LDR_threshold = 500;
  determine_junctions_passed();
  
  if ((distance_IR <= 9) && (infrared_detection_state == on_lookout)) {
    block_detection();
  }

  if (line_following_status == ON_TRACK) {
    // If both front sensors on line
    too_far_situations_parsed = 0;
    if ((LHS_digital_opto == 1) && (current_micros >= (previous_micros_3 + 30 * 1000000))) {
      in_box = BLOCK_SEARCHING;
      previous_micros_3 = current_micros;
    }
    myMotor1->setSpeed(250);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(250);
    myMotor2->run(BACKWARD);
  }

  if (line_following_status == TOO_FAR_RIGHT) {
    // If right-hand-side sensor falls off line
    last_known_extremity = TO_THE_RIGHT;
    too_far_situations_parsed = 0;
    if ((LHS_digital_opto == 1) && (current_micros >= (previous_micros_3 + 30 * 1000000))) {
      in_box = BLOCK_SEARCHING;
      previous_micros_3 = current_micros;
    }
    myMotor1->setSpeed(255);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(150);
    myMotor2->run(FORWARD);
  }

  if (line_following_status == TOO_FAR_LEFT) {
    // If left-hand-side sensor falls off line
    last_known_extremity = TO_THE_LEFT;
    too_far_situations_parsed = 0;
    if ((LHS_digital_opto == 1) && (current_micros >= (previous_micros_3 + 30 * 1000000))) {
      in_box = BLOCK_SEARCHING;
      previous_micros_3 = current_micros;
    }
    myMotor1->setSpeed(150);
    myMotor1->run(FORWARD);
    myMotor2->setSpeed(255);
    myMotor2->run(BACKWARD);
  }

  if ((line_following_status == LOST_SOMEWHERE) && (last_known_extremity == TO_THE_RIGHT)) {
    // If both sensors fall off line, to the right
    myMotor1->setSpeed(250);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(100);
    myMotor2->run(FORWARD);
  }

  if ((line_following_status == LOST_SOMEWHERE) && (last_known_extremity == TO_THE_LEFT)) {
    // If both sensors fall off line, to the left
    if (too_far_situations_parsed == 0) {
      previous_micros_tfl = current_micros;
      too_far_situations_parsed = 1;
    }
    myMotor1->setSpeed(100);
    myMotor1->run(FORWARD);
    myMotor2->setSpeed(250);
    myMotor2->run(BACKWARD);
    if ((current_micros >= (previous_micros_tfl + 0.75*1000000))) {
      last_known_extremity = TO_THE_RIGHT;
      too_far_situations_parsed = 0;
    }
  }

  if ((line_following_status == LOST_SOMEWHERE) && (last_known_extremity == UNKNOWN)) {
    // If both sensors fall off line, in unknown direction
    too_far_situations_parsed = 0;
    myMotor1->setSpeed(0);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(0);
    myMotor2->run(BACKWARD);
  }

  if ((line_following_status == LOST_SOMEWHERE) && (last_known_extremity == TUNNEL)) {
    // If both sensors are not on line immediately after tunnel
    too_far_situations_parsed = 0;
    myMotor1->setSpeed(155);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(150);
    myMotor2->run(BACKWARD);
  }
}

void determine_junctions_passed() {
  // Determine the number of junctions passed
  if ((RHS_digital_opto == 1) && (junction_state == st_1) && (current_micros >= 60*1000000)) {
    junctions_passed = 1;
    junction_state == st_2;
  }
  if (junctions_passed == 1) {
    hard_coded_box_return();
  }
}

void read_ultrasonic_transducers() {
  // Ultrasonic pulsation and distance calculation for left-hand-side ultrasound
  if ((ultrasound_state == first)) {
    digitalWrite(trigPin1, LOW);
    previous_micros_1 = current_micros;
    ultrasound_state = second;
  }
  if ((current_micros >= previous_micros_1 + interval_1) && (ultrasound_state == second)) {
    digitalWrite(trigPin1, HIGH);
    previous_micros_2 = current_micros;
    ultrasound_state = third;
  }
  if ((current_micros >= previous_micros_2 + interval_2) && (ultrasound_state == third)) {
    digitalWrite(trigPin1, LOW);
    US_duration_1 = pulseIn(echoPin1, HIGH);
    US_distance_1 = US_duration_1 * 0.034 / 2;
    ultrasound_state = first;
  }
  // Now determine if the side ultrasound is giving a value which is reasonable to assume we're in the tunnel
  if (US_distance_1 <= 10) {
    working_US = YES;
  }
  else if (US_distance_1 > 10) {
    working_US = NO;
  }
}

void begin_block_searching() {
  // Allow search for block to begin
  myMotor1->setSpeed(0);
  myMotor1->run(BACKWARD);
  myMotor2->setSpeed(0);
  myMotor2->run(BACKWARD);
  delay(2000);
  in_box = NEGATIVE;
}

void block_detection() {
  // Determine block density
  infrared_detection_state = not_on_lookout;
  distance_IR = IR_sensor.distance();

  myMotor1->setSpeed(15);
  myMotor1->run(BACKWARD);
  myMotor2->setSpeed(15);
  myMotor2->run(BACKWARD);

  if (distance_IR < critical_dist) {
    // If block is within required range

    for (int i=0; i < NumReadings; i++) {
      // Take a number of readings of frontal ultrasonic transducer
      digitalWrite(trigPin2, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin2, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin2, LOW);
      duration = pulseIn(echoPin2, HIGH);
      distance = duration * 0.034 / 2;
      delay(30);
      
      if (distance < critical_dist_UT) {
        // If a single ultrasound beam is returned, block is coarse
        i = NumReadings;
        digitalWrite(green_LED, HIGH);
        myMotor1->setSpeed(0);
        myMotor1->run(BACKWARD);
        myMotor2->setSpeed(0);
        myMotor2->run(BACKWARD);
        delay(5000);
        digitalWrite(green_LED, LOW);
      }
      else if (i > NumReadings - 2) {
        // If not a single ultrasound beam is returned, block is dense
        digitalWrite(red_LED, HIGH);
        myMotor1->setSpeed(0);
        myMotor1->run(BACKWARD);
        myMotor2->setSpeed(0);
        myMotor2->run(BACKWARD);
        delay(5000);
        digitalWrite(red_LED, LOW);
      }
    }
  }
}

void tunnel_code() {
  // Navigation in tunnel using the left side ultrasound when possible
  last_known_extremity = TUNNEL;
  too_far_situations_parsed = 0;
  LDR_threshold = 700;
  
  if (line_following_status == ON_TRACK) {
    // If both front sensors on line
    myMotor1->setSpeed(250);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(250);
    myMotor2->run(BACKWARD);
  }
  else if (line_following_status == TOO_FAR_RIGHT) {
    // If right-hand-side sensor falls off line
    myMotor1->setSpeed(255);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(50);
    myMotor2->run(BACKWARD);
  }
  else if (line_following_status == TOO_FAR_LEFT) {
    // If left-hand-side sensor falls off line
    myMotor1->setSpeed(50);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(255);
    myMotor2->run(BACKWARD);
  }
  else if ((line_following_status == LOST_SOMEWHERE) && (working_US == YES)) {
    // If neither sensor on line and the ultrasound is giving suitable tunnel readings
    if (US_distance_1 > 6) {
      myMotor1->setSpeed(150);
      myMotor1->run(BACKWARD);
      myMotor2->setSpeed(50);
      myMotor2->run(BACKWARD);
    }
    if (US_distance_1 == 6) {
      myMotor1->setSpeed(180);
      myMotor1->run(BACKWARD);
      myMotor2->setSpeed(180);
      myMotor2->run(BACKWARD);
    }
    if (US_distance_1 < 6) {
      myMotor1->setSpeed(50);
      myMotor1->run(BACKWARD);
      myMotor2->setSpeed(150);
      myMotor2->run(BACKWARD);
    }
  }
  else if ((line_following_status == LOST_SOMEWHERE) && (working_US == NO)) {
    // If neither sensor on line and the ultrasound is not giving suitable tunnel readings
    myMotor1->setSpeed(155);
    myMotor1->run(BACKWARD);
    myMotor2->setSpeed(150);
    myMotor2->run(BACKWARD);
  }
  else {
    // In all other tunnel scenarios not described above
    myMotor1->setSpeed(100);
    myMotor1->run(FORWARD);
    myMotor2->setSpeed(100);
    myMotor2->run(FORWARD);
  }
}

void hard_coded_box_return() {
  // Return to start box after penultimate junction passed
  myMotor1->setSpeed(200);
  myMotor1->run(BACKWARD);
  myMotor2->setSpeed(200);
  myMotor2->run(BACKWARD);
  delay(3000);
  myMotor1->setSpeed(150);
  myMotor1->run(BACKWARD);
  myMotor2->setSpeed(200);
  myMotor2->run(BACKWARD);
  delay(3000);
  myMotor1->setSpeed(200);
  myMotor1->run(BACKWARD);
  myMotor2->setSpeed(200);
  myMotor2->run(BACKWARD);
  delay(3500);
  myMotor1->setSpeed(0);
  myMotor1->run(BACKWARD);
  myMotor2->setSpeed(0);
  myMotor2->run(BACKWARD);
  delay(300000);
}

void loop() {
  // Main loop
  read_sensors();
  determine_sensor_states();
  read_ultrasonic_transducers();

  if (in_box == AFFIRMATIVE) {
    initial_box_exit();
  }
  else if (in_box == NEGATIVE) {
    if (light_level == LIGHT){
      normal_line_following();
    }
    if (light_level == DARK) {
      tunnel_code();
    }
  }
  else if (in_box == BLOCK_SEARCHING) {
    begin_block_searching();
  } 
}