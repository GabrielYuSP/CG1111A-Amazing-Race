
/********** Include Header Files **********/
#include "Wire.h"
#include "MeMCore.h"
#include "MeRGBLed.h"

/********** Define Ports **********/
#define MUSIC_PIN       8
#define LDR             A0
#define IR              A1
#define PUSHBUTTON      A7
MeDCMotor         leftWheel(M1);    // assigning LeftMotor to port M1
MeDCMotor         rightWheel(M2);   // assigning RightMotor to port M2
MeLineFollower    lineFinder(PORT_2);
MeUltrasonicSensor  ultraSensor(PORT_1);
MeRGBLed          led(0,30);
MeBuzzer          buzzer;

/********** Define Delay Constants **********/
// Define time delay constants before taking analogue readings (to let voltage stabalise)
#define RGBWait 45  // Time delay (in ms) before taking LDR reading
#define IRWait 20   // Time delay (in ms) before taking IR reading
#define IR_THRESHOLD 140  //Around 6cm (Was 480) // Amount of dip in value to determine proximity of wall on right side. (Difference between AmbientIR and Measured IR Detector Voltage)
#define LDRWait 15  // Time delay (in ms) before taking LDR reading

/********** Constants **********/
// Ultrasound
#define OUT_OF_RANGE 100
#define RIGHT_WALL_NUDGE -60 // Negative value = "turn left"

// Movementa
#define MOTORSPEED              255   //Was 255
#define SIDE_MAX                18    // Side distance threshold (cm)
#define TIME_FOR_LEFT_TURN      360   //Was 320 // The time duration (ms) for turning 90 degrees counter-clockwise      (for red waypoint)
#define TIME_FOR_RIGHT_TURN     360   //Was 320 // The time duration (ms) for turning 90 degrees clockwise            (for green waypoint)
#define TIME_FOR_1_GRID_PINK    855   // The time duration (ms) for moving forward by 1 grid            (for pink waypoint)
#define TIME_FOR_1_GRID_BLUE    855   // The time duration (ms) for moving forward by 1 grid            (for blue waypoint)
#define TIME_FOR_SECOND_LEFT_TURN 370   // The time duration (ms) for second 90 degrees counter-clockwise turn  (for pink waypoint)
#define TIME_FOR_SECOND_RIGHT_TURN 380  // The time duration (ms) for second 90 degrees clockwise turn        (for blue waypoint)
#define TIME_FOR_UTURN          730   //Was 530  // The time duration (ms) for turning 180 degrees clockwise         (for orange waypoint)

/********** Constants & Variables for PID Controller (only PD is used) **********/

const double kp = 20.0; // Proportional Gain/Constant  (P component of PID)
const double kd = 2.0; // Derivative Constant        (D component of PID)

// Variables to hold final motorspeed calculated for each motor
int L_motorSpeed;
int R_motorSpeed;

const double desired_dist = 12.50; //Was 10.5 // Desired distance between ultrasound sensor and wall to keep mBot centered in tile
double error;             // Difference between current position and our desired distance
double prev_error = 0;        // Variable to store previous error, to be used to calculate change in error (For D component of PID)
double error_delta;         // Difference between current error and previous error (For D component of PID)
double correction_dble;     // For calculation of correction for motors
int correction;           // To be used to adjust input to motor

/********** Color Detection **********/
// Pins controlling 2-4 Decoder
int ledArray[2] = { A2, A3 };

// Truth Table to control 2-4 Decoder (for LED Control):
int truth[3][2] =  { { 0, 1 },    // Blue LED ON
                     { 1, 0 },    // Green LED ON
                     { 1, 1 }     // Red LED ON, also used for IR Emitter OFF
                   };

char colourStr[3][5] = {"B = ", "G = ", "R = "};  // array of strings to aid debugging of measured RGB values

// Variable to store resulting classified color [0 - white, 1 - red, 2 - blue, 3 - green, 4 - orange, 5 - pink, 6 - no color classified]
int color;

// Variables to store measured intensity for each color
int measured_blue = 0;
int measured_green = 0;
int measured_red = 0;

// Float arrays to store calibrated values for color arrays
float colourArray[] = {0,0,0};
float whiteArray[] = {811.00, 850.00, 433.00}; //RGB Reading for Pure White
float blackArray[] = {208.00, 255.00, 24.00}; //RGB Reading for Pure Black
float greyDiff[] = {603.00, 595.00, 410.00}; //Pure White - Pure Black


// Struct for each label to be used for color classification
struct Color {
  String name;
  int id; // 0 - white, 1 - red, 2 - blue, 3 - green, 4 - orange, 5 - pink
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

// Data points for each color, obtained by taking average of collected color samples.
// To be used in KNN classification using euclidean distance.
Color colors[] = {
  //  Label/name -   id - R  -  G  -  B
  {   "Red",    1,  255,  114,  104 },
  {   "Blue",   2,  71,   222,  246 },
  {   "Green",  3,  82,   224,  178 },
  {   "Orange", 4,  255,  177,  103 },
  {   "Pink",   5,  255,  229,  225 },
  {   "White",  0,  255,  255,  255 }
};

/********** Global Variables **********/
bool stop = false;  // variable to tell mBot to halt, used for white waypoint (end of maze) [0 = haven't reach end of maze, 1 = reach end of maze, stop all motor and play buzzer]
bool status = false; // variable to tell mBot when to start maze solving algorithm, used to run mBot after pressing button [0 = do nothing, 1 = mBot runs]
int sensorState;    // to keep track of whether black line is detected by line follower
double dist;        // to keep track of distance between wall and ultrasound sensor

/********** Function Declarations **********/
void stopMove(const int i);
void turnLeft(void);
void turnRight(void);
void moveForward(void);
void finishWaypoint(void);


/********** Setup & Loop **********/
void setup()
{
  Serial.begin(9600);
  led.setpin(13);           // led
  pinMode(MUSIC_PIN, OUTPUT); // buzzer
  pinMode(PUSHBUTTON, INPUT); // push button
  pinMode(IR, INPUT);         // ir
  //pinMode(LDR, INPUT);
  
  // 2-4 decoder
  for(int color = 0; color < 2; color++){
    pinMode(ledArray[color],OUTPUT);
  }
  // set all to LOW
  for (int i = 0; i < 2; i++) {
    digitalWrite(ledArray[i], 0);
  }

  // setup complete
  led.setColor(128, 255, 0); // set LED to Green
  led.show();
  buzzer.tone(MUSIC_PIN, 500, 250);
  delay(500);
  buzzer.noTone(MUSIC_PIN);

  
  //delay(2000); // Do nothing for 2000 ms = 2 seconds
}

void loop()
{
  // run mBot only if status is 1
  if (status) {
    if (!on_line()) {       // check if on black line
      
      // --- NEW LOGIC ---
      // 1. Get PD steering correction from the left wall (Ultrasonic)
      int pdCorrection = pd_control_wall_follow();
      
      // 2. Get avoidance correction from the right wall (IR)
      // This function returns RIGHT_WALL_NUDGE (negative) if too close, or 0 otherwise.
      int irCorrection = getRightCorrection();
      
      // 3. Combine corrections
      // The IR correction is directly added to the PD correction.
      // If IR nudges left (e.g., -60), it overrides or strengthens the turn.
      int totalCorrection = pdCorrection + irCorrection;
      
      // 4. Apply the total correction
      // Base speed is MOTORSPEED (e.g., 255)
      L_motorSpeed = MOTORSPEED + totalCorrection;
      R_motorSpeed = MOTORSPEED - totalCorrection;

      // 5. Clamp speeds
      if (L_motorSpeed < 0) L_motorSpeed = 0;
      if (R_motorSpeed < 0) R_motorSpeed = 0;
      if (L_motorSpeed > 255) L_motorSpeed = 255;
      if (R_motorSpeed > 255) R_motorSpeed = 255;

      // 6. Final move command
      move(L_motorSpeed, R_motorSpeed);
      // --- END OF NEW LOGIC ---

    }
    else{
      // black line detected, stop all motors
      stopMove();
      // read color
      read_color();
      // classify color
      int color = classify_color();
      // execute waypoint objectives
      execute_waypoint(color);
    }
  }
  else{
    // entered if status == false, ie. robot is not running maze-solving algorithm
    if (stop == true){
      // entered after white waypoint is executed
      stopMove();
      finishMaze();
      stop = false;
    }
    // check if push button is pressed,
    if (analogRead(PUSHBUTTON) < 100) {
      status = true;      // Toggle status
      delay(500);         // Delay 500ms so that a button push won't be counted multiple times.
    }
  }
}

/********** Functions (Movement, not including waypoint movement) **********/

/**
 * This function is a general movement function used to move robot forward.
 */
void move(int L_spd, int R_spd) {
  if (stop == false) {
    leftWheel.run(-L_spd);
    rightWheel.run(R_spd);
  }
  else {
    stopMove();
  }
}

/**
 * This function is called to stop both motors.
 */
void stopMove() {
  rightWheel.stop();
  leftWheel.stop();
}

/**
 * Calculates the correction value based on the left ultrasonic sensor.
 * A positive value means "turn right".
 * A negative value means "turn left".
 */
int getLeftCorrection() {
  ultrasound(); // Get the distance
  
  if (dist == OUT_OF_RANGE) {
    prev_error = 0; // Reset PD controller
    return 0;       // No wall, so no correction
  }

  // Wall is present, run PD control
  error = desired_dist - dist;
  error_delta = error - prev_error;
  correction_dble = (kp * error) + (kd * error_delta);
  int correction = (int)correction_dble;

  prev_error = error; // Save error for next loop
  
  return correction;
}

/********** Functions (IR) **********/

/**
 * Calculates the correction value based on the right IR sensor.
 * Returns RIGHT_WALL_NUDGE (a negative value) to "turn left" if a wall is too close.
 * Returns 0 otherwise.
 */
int getRightCorrection() {
  int localAmbientIR;
  int irVolt;
  int difference;
  
  // 1. Turn OFF IR Emitter to read ambient light
  // The OFF state is {1, 1} according to your comments
  for(int i = 0; i < 2; i++) { 
    digitalWrite(ledArray[i], 1); 
  }
  delay(IRWait); //Wait for sensor to settle
  localAmbientIR = analogRead(IR);

  // 2. Turn ON IR Emitter to read reflected light
  // The ON state must be {0, 0}
  for(int i = 0; i < 2; i++) { 
    digitalWrite(ledArray[i], 0); 
  }
  delay(IRWait);
  irVolt = analogRead(IR);

  // 3. Turn OFF IR Emitter (Good practice)
  for (int i = 0; i < 2; i++) { 
    digitalWrite(ledArray[i], 1);
  }

  // 4. Calculate the difference
  difference = irVolt - localAmbientIR; 

  // --- Debugging ---
  // Serial.print("Ambient: ");
  // Serial.print(localAmbientIR);
  // Serial.print(", irVolt: ");
  // Serial.print(irVolt);
  // Serial.print(", Difference: ");
  // Serial.println(difference);
  // -----------------

  // 5. Check for a wall
  // (because irVolt goes DOWN near a wall)
  // You may need to adjust this "-100" threshold.
  if (difference < -IR_THRESHOLD) 
  {
    // Wall detected! Return the nudge value.
    // Serial.print("Nudging left...");
    return RIGHT_WALL_NUDGE;
  }
  
  // No wall detected
  return 0;
}


/********** Functions (Ultrasound & Line Sensor) **********/

/**
 * This function updates the global variable dist to current distance between the ultrasonic sensor and the closest object (wall) to it.
 */
void ultrasound() {
  dist = ultraSensor.distanceCm();
  if (dist > SIDE_MAX)
  {
    dist = OUT_OF_RANGE;
  }
}

/**
 * Calculates the PD steering correction based on the left ultrasonic sensor.
 * Positive correction means turn right (R_motorSpeed decreases).
 * Negative correction means turn left (L_motorSpeed decreases).
 */

int pd_control_wall_follow(){
  // 1. Measure distance
  ultrasound();

  // 2. Check for wall presence
  if(dist >= OUT_OF_RANGE || dist <= 1.0){
    prev_error = 0; // Reset PD controller when out of range
    return 0;
  }

  // 3. Wall is present, run PD control
  error = desired_dist - dist;    //P component
  error_delta = error - prev_error;   //D component
  correction_dble = (kp * error) + (kd * error_delta);
  int correction = (int)correction_dble;

  prev_error = error; // Save error for next loop

  return correction;
}

/**
 * This function detects for black line (waypoint)
 */
bool on_line() {
  sensorState = lineFinder.readSensors();
  if (sensorState != S1_OUT_S2_OUT) {
    return true;
  }
  return false;
}

/********** Functions (Color Detection) **********/

/**
 * Function turns on one colour at a time and measure LDR voltage for each colour to estimate respective R/G/B values.
 */
void read_color() {
  for(int c = 0; c <= 2; c++){
    // Serial.print(colourStr[c]);
    // TURN ON LIGHT
    for (int i = 0; i < 2; i++) {
      digitalWrite(ledArray[i], truth[c][i]);
    }
    delay(RGBWait);
    
    colourArray[c] = getAvgReading(3);
    
    int result = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
    if (result > 255) {
      result = 255;
    }
    else if (result < 0) {
      result = 0;
    }
    colourArray[c] = result;
    
    // TURN OFF LIGHT
     for (int i = 0; i < 2; i++) {
      digitalWrite(ledArray[i], 0);
    }
    delay(RGBWait);
    //Serial.begin(9600);
    // Serial.println(int(colourArray[c]));
  }

}

/**
 * Function looks at RGB values stored in colourArray[] and compare it with defined points for each color stored in colors[].
 */
int classify_color() {
  int classified = 6;
  measured_blue = colourArray[0];
  measured_green = colourArray[1];
  measured_red = colourArray[2];
  
  double minDistance = 9999;  // Initialize with a large value

  for (int i = 0; i < 6; i++) { // 6 is the number of known colors
    double distance = sqrt(pow(colors[i].red - measured_red, 2) + pow(colors[i].green - measured_green, 2) + pow(colors[i].blue - measured_blue, 2));
    if (distance < minDistance) {
      minDistance = distance;
      classified = colors[i].id;
    }
  }
  return classified;
}

/**
 * This function finds the average reading of LDR for greater accuracy of LDR readings.
 */
int getAvgReading(int times) {
  int reading;
  int total = 0;
  for (int i = 0; i < times; i++) {
    reading = analogRead(LDR);
    total = reading + total;
    delay(LDRWait);
  }
  return total / times;
}

/********** Functions (Waypoints) **********/

/**
 * Function takes in the color of waypoint classified by mBot and calls respective functions to move mBot to complete waypoint objective.
 */
void execute_waypoint(const int color)
{
  switch(color) {
  case 0:
    // code block for white
    // Serial.println("White Colour!");
    led.setColor(255, 255, 255); // set both LED to WHITE
    led.show();
    stop = true;
    status = false;
    break;
  case 1:
    // code block for red
    // Serial.println("Red Colour!");
    led.setColor(255, 0, 0); // set both LED to RED
    led.show();
    turnLeft(TIME_FOR_LEFT_TURN);
    break;
  case 2:
    // code block for blue
    // Serial.println("Blue Colour!");
    led.setColor(0, 0, 255); // set both LED to BLUE
    led.show();
    doubleRight(TIME_FOR_1_GRID_BLUE);
    break;
  case 3:
    // code block for green
    // Serial.println("Green Colour!");
    led.setColor(0, 255, 0); // set both LED to GREEN
    led.show();
    turnRight(TIME_FOR_RIGHT_TURN);
    break;
  case 4:
    // code block for orange
    // Serial.println("Orange Colour!");
    led.setColor(153, 76, 0); // set both LED to ORANGE
    led.show();
    uTurn();
    break;
  case 5:
    // code block for pink
    // Serial.println("Pink Colour!");
    led.setColor(255, 192, 203); // set both LED to PINK
    led.show();
    doubleLeft(TIME_FOR_1_GRID_PINK);
    break;
  default:
    // code block for no color classified
    // Serial.println("No Colour!");
    led.setColor(0, 0, 0); // set both LED to OFF
    led.show();
    break;
  }
}

/**
 * Function moves mBot forward by a distance of approximately one tile.
 */
void forwardGrid(int time) {
  leftWheel.run(-MOTORSPEED);
  rightWheel.run(MOTORSPEED);
  delay(time);
  stopMove();
}

/**
 * Function allows mBot to make a 90 degrees clockwise turn.
 */
void turnRight(int time) {
  leftWheel.run(-(MOTORSPEED - 15));
  rightWheel.run(-(MOTORSPEED - 15));
  delay(time);
  stopMove();
}

/**
 * Function allows mBot to make a 90 degrees counter-clockwise turn.
 */
void turnLeft(int time) {
  leftWheel.run(MOTORSPEED - 15);
  rightWheel.run(MOTORSPEED - 15);
  delay(time);
  stopMove();
}

/**
 * Function allows mBot to make a 90 degrees clockwise turn, followed by moving forward by 1 tile, lastly following with another 90 degrees clockwise turn.
 */
void doubleRight(int time) {
  turnRight(TIME_FOR_SECOND_RIGHT_TURN);
  delay(10);
  forwardGrid(time);
  delay(100);
  turnRight(TIME_FOR_SECOND_RIGHT_TURN);
}

/**
 * Function allows mBot to make a 90 degrees counter-clockwise turn, followed by moving forward by 1 tile, lastly following with another 90 degrees counter-clockwise turn.
 */
void doubleLeft(int time) {
  turnLeft(TIME_FOR_LEFT_TURN);
  delay(10);
  forwardGrid(time);
  delay(100);
  turnLeft(TIME_FOR_SECOND_LEFT_TURN);
}

/**
 * Function allows mBot to make a 180 degrees clockwise turn.
 */
void uTurn() {
  turnRight(TIME_FOR_UTURN);
}

/**
 * Samsung Washing Machine Chime
 */

void finishMaze() {
  const int BPM = 210;            // close to real tempo
  const int Q = 60000 / BPM;      // quarter note in ms
  const int E = Q / 2;            // eighth note
  const int R = E / 1.5;
  const int F = E / 2;            // 1/16 note

  // Frequencies (Hz)
  const int DSH5 = 622;
  const int E5 = 659;
  const int FSH5 = 740;
  const int GSH5 = 831;
  const int A5 = 880;
  const int B5 = 988;
  const int CSH6 = 1109;
  const int D6 = 1175;

  int melody[] = { E5, A5, CSH6, A5, E5, B5, A5, GSH5, FSH5, E5, E5, A5, CSH6, A5, E5, A5, GSH5, FSH5, GSH5, A5, DSH5, E5, E5, GSH5, A5, GSH5, FSH5, GSH5, A5, E5, A5, GSH5, D6, B5, GSH5, A5, FSH5, A5, E5, B5, GSH5, A5, A5, GSH5, FSH5, A5, GSH5, B5, A5, E5, B5, GSH5, A5};
  int dur[]    = { E, Q, Q, Q, Q, F, F, F, F, Q, E, Q, Q, Q, E, E, E, F, F, E, E, Q, E, Q, F, F, F, F, Q, E, E, Q, F, F, F, Q, Q, Q, Q, E, E, Q, F, F, Q, F, F, F, Q, Q, E, E, Q};

  for (int i = 0; i < 53; i++) {
    buzzer.tone(MUSIC_PIN, melody[i], dur[i]);
    delay((int)(dur[i] * 1.05));
    buzzer.noTone(MUSIC_PIN);
    delay(15);
    if (i == 31) delay(235);
    if (i == 41) delay(135);
    if (i == 4 || i == 9 || i == 21 || i == 35 || i == 38 || i == 49) delay(335);
  }
}
