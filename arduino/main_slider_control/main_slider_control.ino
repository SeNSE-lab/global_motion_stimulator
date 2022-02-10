#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "C:\Users\user\global_motion_stimulator\arduino\libraries\timer_lib\timer_lib.ino"
#include "C:\Users\user\global_motion_stimulator\arduino\libraries\data_struct_lib\data_struct_lib.ino"


#define RETRACT_SPEED 180

#define MOVEMENT_TRIGGER 1
#define TRIGGER_PIN A3
#define MOTOR_SHAFT_FACING_AWAY 0
#define LED_PIN 31

#define DEBUG_MESSAGES 1
#define SAMPLING_EPOCHS 5
#define DEBUG_ONLY 0
// 60 100
#define HOLD_PWM 60   // motor can tolerate about 3V when holding, power is about 12 V  (3/12) * 255 ~= 60
#define MOVE_PWM 100   // 4.7 V

#define CS 49 //Chip Select 

#define ENC_ZERO 90.72
#define STEP_DEG 0.9


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(0x61);


// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *slider_motor = AFMS.getMotor(1);

// Stepper motor object
Adafruit_StepperMotor *direction_motor;

bool debug = true;


int slider_readout = A0;
int STEPS_PER_REVOLUTION = 400;
int state = 0;
int test_stepper = 0;

int current_steps = 0;

// Number of words (argument variables) in a user command line.
char *argument_words[8];
// Buffer to use for receiving special user-defined ASCII commands.
char user_command_buffer[64];
// Index to use when filling the user command buffer.
uint8_t user_command_buffer_index = 0;
uint8_t c = 0;  // character received

char output_line[71];

int read_percent_track = 0;
int set_percent_track = 0;
uint8_t set_speed = 170;   // 0-255

int degrees_to_move = 0;
int direction_to_move = 0;
int step_size = 0;
int current_degrees = 0;

unsigned int slider_buffer_size = 250;
circular_buffer_int *slider_buffer;

unsigned long timestamps[200];
int time_idx = 0;
//unsigned int timestamp_buffer_size = 250;
//circular_buffer_int *timestamp_buffer;

boolean last_led_state = 1;
uint8_t timed_function2;

volatile boolean print_message_timed;
volatile unsigned long times_in_isr = 0;
timed_function *sampling_timer;
sampled_time *sampled_times[SAMPLING_EPOCHS];
uint8_t sample_time_index = 0;
volatile boolean stamp_first = false, stamp_last = false;
unsigned long sampling_period = 1333;  // in microseconds.
unsigned long timer_frequency = 750;      // in Hz.
unsigned long last_trial_time = 0;

// Session parameters
unsigned int orientations[8] = {0, 45, 90, 135, 180, 225, 270, 315};
unsigned int orientation = 0;
unsigned int last_orientation = 0;
boolean in_trial = false;
boolean run_trial = false;
int minimum_position = 30;
int maximum_position = 70;

// Trial parameters.
unsigned int trial_number = 0;
unsigned int inter_trial_interval = 3000; // milliseconds.
uint8_t slider_speed = 160;
int starting_position = 15;
int ending_position = 50;

// Trial results
unsigned long start_trigger_timestamp_us;
unsigned long stop_trigger_timestamp_us;

uint8_t SPI_T (uint8_t msg)    //Repetive SPI transmit sequence
{
  uint8_t msg_temp = 0;  //vairable to hold recieved data
  digitalWrite(CS, LOW);    //select spi device
  msg_temp = SPI.transfer(msg);    //send and recieve
  digitalWrite(CS, HIGH);   //deselect spi device
  return (msg_temp);     //return recieved byte
}


float read_encoder(bool proc = true) {
  // adapted from http://myrobotlab.org/content/code-cui-amt203-absolute-encoder
  // absolute position in deg

  // do a dummy read to clear old data
  if (proc) {
    read_encoder(false);
  }
  uint8_t recieved = 0xA5;    //just a temp vairable (encoder idle)
  uint8_t temp[2];
  short pos = 0;
  uint8_t count = 0;

  SPI.begin();    //start transmition
  digitalWrite(CS, LOW);

  SPI_T(0x10);   //issue read command

  recieved = SPI_T(0x00);    //issue NOP to check if encoder is ready to send

  while (recieved != 0x10)    //loop while encoder is not ready to send
  {
    recieved = SPI_T(0x00);    //cleck again if encoder is still working
    delay(2);    //wait a bit

    // If fails too many times,send read command again
    if (count > 7 && recieved != 0x10) {
      recieved = SPI_T(0x10);
      delay(2);
      count = 0;
    }

    count++;
  }

  temp[0] = SPI_T(0x00);    //Recieve MSB
  temp[1] = SPI_T(0x00);    // recieve LSB

  digitalWrite(CS, HIGH); //just to make sure
  SPI.end();    //end transmition

  temp[0] &= ~ 0xF0;   //mask out the first 4 bits

  pos = temp[0] << 8;    //shift MSB to correct pos in pos message
  pos += temp[1];        // add LSB to pos message to complete message

  delay(20);

  return (float)(pos * 0.08789);    // aprox 360/2**12
}

void parse(char *line, char **argument_words, uint8_t maxArgs) {
  uint8_t argCount = 0;
  while (*line != '\0') {       /* if not the end of line ....... */
    while (*line == ',' || *line == ' ' || *line == '\t' || *line == '\n')
      *line++ = '\0';     /* replace commas and white spaces with 0    */
    *argument_words++ = line;          /* save the argument position     */
    argCount++;
    if (argCount == maxArgs - 1)
      break;
    while (*line != '\0' && *line != ',' && *line != ' ' &&
           *line != '\t' && *line != '\n')
      line++;             /* skip the argument until ...    */
  }
  **argument_words = '\0';                 /* mark the end of argument list  */
}

void move_slider(int percent_of_track, uint8_t speed_of_motor, int direction_of_movement, boolean sampling) {
  // Speed range is 0-255

  //sprintf(output_line, "Moving slider: percent = %d, speed = %d, direction = %d", percent_of_track, speed_of_motor, direction_of_movement);
  //Serial.println(output_line);
  time_idx = 0;
  //  if (debug) {
  //    Serial.print("moving slider: ");
  //    Serial.print(percent_of_track);
  //    Serial.print("\tspeed: ");
  //    Serial.print(speed_of_motor);
  //    Serial.print("\tdirection: ");
  //    if (direction_of_movement == FORWARD)
  //      Serial.println("forward");
  //    else if (direction_of_movement == BACKWARD)
  //      Serial.println("backwards");
  //  }

  percent_of_track = int(10.24 * percent_of_track);

  int read_track = analogRead(slider_readout);
  if (read_track == percent_of_track)
    //Serial.println("Already there bro!");
    return;
  // Wrong direction entered
  else if ((direction_of_movement == FORWARD) && (percent_of_track < read_track)) {
    if (debug) Serial.println(F("Can't do that bro! Move backwards instead"));
  }
  else if ((direction_of_movement == BACKWARD) && (percent_of_track > read_track)) {
    if (debug) Serial.println(F("You mean move forward? We're already past that point!"));
  }
  else {
    boolean not_there_yet = true;

    // Pin to end of track before moving in commanded direction

  if (direction_of_movement == FORWARD) {
      if (read_track > 2) {
        slider_motor->setSpeed(RETRACT_SPEED);
        slider_motor->run(BACKWARD);
        while (read_track > 2) {
          read_track = analogRead(slider_readout);
        }
      }
    }
    else {
      if (read_track < 1021) {
        slider_motor->setSpeed(RETRACT_SPEED);
        slider_motor->run(FORWARD);
        while (read_track < 1021) {
          read_track = analogRead(slider_readout);
        }
      }
    }
    slider_motor->setSpeed(50);
    delay(200);



    
    slider_motor->setSpeed(speed_of_motor);
    if (sampling) {
      stamp_first = true;
      run_timed_function(sampling_timer);
      delay(25);
      if (MOVEMENT_TRIGGER) {
        digitalWrite(TRIGGER_PIN, HIGH);
        start_trigger_timestamp_us = micros();
      }
    }
    slider_motor->run(direction_of_movement);
    digitalWrite(LED_PIN, HIGH);
    while (not_there_yet) {
      //read_percent_track = analogRead(slider_readout);
      if (sampling)
        read_track = read_last_element(slider_buffer);
      else
        read_track = analogRead(slider_readout);

      if (direction_of_movement == FORWARD) {
        if (percent_of_track < read_track)
          not_there_yet = false;
      }
      else if (direction_of_movement == BACKWARD) {
        if (percent_of_track > read_track)
          not_there_yet = false;
      }
      //Serial.println(read_percent_track);
    }
  }
  // TODO HEREHERE return ! if encoder moved, else return something else
  slider_motor->run(RELEASE);
  digitalWrite(LED_PIN, LOW);

  if (sampling) {
    if (MOVEMENT_TRIGGER) {
      digitalWrite(TRIGGER_PIN, LOW);
      stop_trigger_timestamp_us = micros();
    }
    stamp_last = true;
    // delay 5 sampling periods worth to make sure last packet is printed out.
    delay(5 * 1000 / timer_frequency);
  }

  // make sure stepper didn't move during slide
  float pos = read_encoder();
  if (abs((pos - ENC_ZERO) - (float)(current_steps * STEP_DEG)) > STEP_DEG) {
    //read agian to make sure stepper didn't give anamalous reading
    pos = read_encoder();
    if (abs((pos - ENC_ZERO) - (float)(current_steps * STEP_DEG)) > STEP_DEG) {
      Serial.print('!');
      fine_tune_stepper(current_steps * STEP_DEG);
    }
    else Serial.print('1');
  }
  else Serial.print('1');




  //Serial.print("Current slider position at: ");
  //read_percent_track = analogRead(slider_readout);
  //Serial.print(read_percent_track/10.24);
  //Serial.println("%");
  return;
}

void move_stepper(int degrees_to_turn, int rotation_direction, int step_size) {

  if (debug) {
    Serial.print("moving stepper from ");
    Serial.print(current_degrees);
    Serial.print(" by ");
    Serial.print(degrees_to_turn);
    Serial.print(" degrees\tdirection: ");
    if (rotation_direction == FORWARD)
      Serial.println("clockwise");
    else if (rotation_direction == BACKWARD)
      Serial.println("counterclockwise");
  }

  if (degrees_to_turn == 0)
    return;

  int steps_number;
  float steps_per_degree;
  steps_per_degree = STEPS_PER_REVOLUTION / 360.0;
  steps_number = int(degrees_to_turn * steps_per_degree);
  if (debug) Serial.println(steps_number);
  //Serial.print("steps: ");
  //Serial.println(steps_number);
  //if(MOVEMENT_TRIGGER)
  //   digitalWrite(TRIGGER_PIN, HIGH);
  if (!MOTOR_SHAFT_FACING_AWAY) {
    if (rotation_direction == FORWARD)
      rotation_direction = BACKWARD;
    else if (rotation_direction == BACKWARD)
      rotation_direction = FORWARD;
  }
  direction_motor->step(steps_number, rotation_direction, step_size, MOVE_PWM);
  //if(MOVEMENT_TRIGGER)
  //   digitalWrite(TRIGGER_PIN, LOW);

  if (rotation_direction == FORWARD)
    current_degrees = (current_degrees + degrees_to_turn) % 360;
  else if (rotation_direction == BACKWARD)
    current_degrees = (current_degrees - degrees_to_turn) % 360;

  AFMS.setPWM(direction_motor->PWMApin, HOLD_PWM * 16);
  AFMS.setPWM(direction_motor->PWMBpin, HOLD_PWM * 16);
}

void engage_stepper(void) {
  float pos = read_encoder();
  int steps_number;
  float deg_to_turn;
  if (pos > 230) {
    deg_to_turn = ENC_ZERO + (360 - pos);
    steps_number = (int)(deg_to_turn / STEP_DEG);
    direction_motor->step(steps_number, FORWARD, INTERLEAVE, MOVE_PWM);
  }
  else if (pos < ENC_ZERO) {
    deg_to_turn = ENC_ZERO - pos;
    steps_number = (int)(deg_to_turn / STEP_DEG);
    direction_motor->step(steps_number, FORWARD, INTERLEAVE, MOVE_PWM);
  }
  else {
    deg_to_turn = pos - ENC_ZERO;
    steps_number = (int)(deg_to_turn / STEP_DEG);
    direction_motor->step(steps_number, BACKWARD, INTERLEAVE, MOVE_PWM);
  }
  fine_tune_stepper(0);
  current_steps = 0;
}

void fine_tune_stepper(float target_deg) {
  float pos = read_encoder();
  float target_enc = target_deg + ENC_ZERO;

  while (pos - target_enc > .9) {
    direction_motor->step(1, BACKWARD, INTERLEAVE, MOVE_PWM);
    delay(50);
    pos = read_encoder();

  }
  while (pos - target_enc < -.9) {
    direction_motor->step(1, FORWARD, INTERLEAVE, MOVE_PWM);
    delay(50);
    pos = read_encoder();
  }

  AFMS.setPWM(direction_motor->PWMApin, HOLD_PWM * 16);
  AFMS.setPWM(direction_motor->PWMBpin, HOLD_PWM * 16);
}

void move_stepper_to(int target_degrees) {
  volatile float enc_pos;
  int n;
  if (debug) {
    enc_pos = read_encoder();
    Serial.print("start:\t");
    Serial.println(enc_pos);
  }
  //target degrees must be between -90 and 90
  if ((target_degrees < -90) || (target_degrees > 90)) return;

  int target_steps = int(target_degrees * (STEPS_PER_REVOLUTION / 360.0));
  int rotation_direction;
  int steps_number;

  if (target_steps == current_steps) return;
  else if (target_steps > current_steps) {
    steps_number = target_steps - current_steps;
    direction_motor->step(steps_number, FORWARD, INTERLEAVE, MOVE_PWM);
    current_steps += steps_number;
  }
  else {
    steps_number = current_steps - target_steps;
    direction_motor->step(steps_number, BACKWARD, INTERLEAVE, MOVE_PWM);
    current_steps -= steps_number;
  }
  if (debug) {
    Serial.print("Current Pose\tdeg: ");
    Serial.print(current_steps * (360.0 / STEPS_PER_REVOLUTION));
    Serial.print("\tsteps: ");
    Serial.print(current_steps);
    Serial.println();
  }

  fine_tune_stepper(target_degrees);

  AFMS.setPWM(direction_motor->PWMApin, HOLD_PWM * 16);
  AFMS.setPWM(direction_motor->PWMBpin, HOLD_PWM * 16);
  if (debug) {
    enc_pos = read_encoder();
    Serial.print("end:\t");
    Serial.println(enc_pos);
  }
}

void set_degrees(int stepper_position) {
  current_degrees = stepper_position;
}

Adafruit_StepperMotor *create_stepper(int steps, int port) {
  // Connect a stepper motor with steps number of steps and wired to the port
  // number provided. (port#2 = M3 and M4)

  return AFMS2.getStepper(steps, port);

}

// Called from timer ISR
void sample_slider(uint8_t arg) {

  times_in_isr++;
  print_message_timed = true;
  funcs_start = micros();
  read_percent_track = analogRead(slider_readout);  // takes about 100 us.
  if (stamp_first) {

    sampled_times[sample_time_index]->start_time_us = micros();
    sampled_times[sample_time_index]->first_element_index = \
        slider_buffer->fill_next;
    sampled_times[sample_time_index]->sampling_frequency = \
        sampling_timer->launch_period_us;
    sampled_times[sample_time_index]->first_element = read_percent_track;
    stamp_first = false;
  }
  else if (stamp_last) {
    sampled_times[sample_time_index]->end_time_us = micros();
    sampled_times[sample_time_index]->last_element_index = \
        slider_buffer->fill_next;
    sampled_times[sample_time_index]->sampling_frequency = \
        sampling_timer->launch_period_us;
    sampled_times[sample_time_index]->last_element = read_percent_track;
    sample_time_index = (sample_time_index + 1) % SAMPLING_EPOCHS;
    stamp_last = false;
    stamp_first = true;
    sampling_timer->running = false;


  }
  funcs_end = micros();
  timestamps[time_idx] = micros();
  insert_data_buffer(slider_buffer, read_percent_track);
  time_idx++;
  //  insert_data_buffer(timestamp_buffer, micros());
}

void initialize_sampling_times() {

  for (int iteration = 0; iteration < SAMPLING_EPOCHS; iteration++) {
    sampled_times[iteration] = (sampled_time*)malloc(sizeof(sampled_time));
    sampled_times[iteration]->start_time_us = 0;
    sampled_times[iteration]->end_time_us = 0;
    sampled_times[iteration]->first_element_index = 0;
    sampled_times[iteration]->last_element_index = 0;
    sampled_times[iteration]->sampling_frequency = 0;
    sampled_times[iteration]->first_element = 0;
    sampled_times[iteration]->last_element = 0;
  }
}

void setup() {

  // SPI for encoder
  pinMode(CS, OUTPUT); //Slave Select
  digitalWrite(CS, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.end();


  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(13, OUTPUT);

  pinMode(30, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(30, LOW);

  Serial.begin(115200);           // set up Serial library at 9600 bps
  //Serial.println("Starting>");

  int time_prescaler;

  // (frequency in Hz, timer number)
  time_prescaler = setup_timer(timer_frequency, 1);

  //timed_function2 = allocateTimer();
  //startTimer(timed_function2, 0, test_timer, 0);

  sampling_timer = create_timed_function(sample_slider, \
                                         0, \
                                         2000L, \
                                         sampling_period, \
                                         false);
  initialize_sampling_times();
  slider_buffer = initialize_ring_buffer(slider_buffer_size);
  //  timestamp_buffer = initialize_ring_buffer(timestamp_buffer_size);
  stamp_first = true;
  //run_timed_function(sampling_timer);

  /*Serial.print("initialized slider buffer. Size: ");
    Serial.print(slider_buffer->buffer_length);
    Serial.print("\tfill index: ");
    Serial.print(slider_buffer->fill_next);
    Serial.print("\tsent index: ");
    Serial.print(slider_buffer->sent_last);*/
  //Serial.print("free sram: ");
  //Serial.println(free_sram());

  AFMS.begin();  // create with the default frequency 1.6KHz
  AFMS2.begin();
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  slider_motor->setSpeed(0);
  //myMotor->run(FORWARD);
  // turn on motor
  slider_motor->run(RELEASE);

  if (!DEBUG_ONLY) {
    direction_motor = create_stepper(200, 2);
    direction_motor->setSpeed(1);   // 10 rpm
  }
  //TWBR = ((F_CPU /400000l) - 16) / 2; // Change the i2c clock to 400KHz*/
  randomSeed(analogRead(0));
  //  clear_encoder();
}


void loop() {

  int read_track, new_track_position;


  if (DEBUG_MESSAGES) {
    if ((print_message_timed) && !in_trial) {
      //      /*Serial.print("free sram: ");
      //        Serial.println(free_sram());
      //        Serial.print(micros());
      //        Serial.print(" ISR started at: ");
      //        Serial.println(start_of_isr);
      //        Serial.print("Func started at: ");
      //        Serial.println(funcs_start);
      //        Serial.print("Func ended at: ");
      //        Serial.println(funcs_end);
      //        Serial.print("Timed ended at: ");
      //        Serial.println(timed_end);
      //        Serial.print("Func period: ");
      //        Serial.println(funcs_period);
      //        print_message_timed = false;
      //        Serial.println(times_in_isr);*/
      //
      //      for (int i = sample_time_index - 1; i < sample_time_index; i++) {
      //        if (debug) {
      //          Serial.print("\nTrial ");
      //          Serial.print(trial_number);
      //          Serial.print("\tsample ");
      //          Serial.print(i);
      //          Serial.print("\tstart: ");
      //          Serial.print(sampled_times[i]->start_time_us);
      //          Serial.print("\tend: ");
      //          Serial.print(sampled_times[i]->end_time_us);
      //          Serial.print("\ttotal (us): ");
      //          Serial.println(sampled_times[i]->end_time_us - sampled_times[i]->start_time_us);
      //          Serial.print("first index: ");
      //          Serial.print(sampled_times[i]->first_element_index);
      //          Serial.print("\tlast index: ");
      //          Serial.print(sampled_times[i]->last_element_index);
      //          Serial.print("\tsampling frequency: ");
      //          Serial.print(sampled_times[i]->sampling_frequency);
      //          Serial.print("\tfirst number: ");
      //          Serial.print(sampled_times[i]->first_element);
      //          Serial.print("\tlast number: ");
      //          Serial.println(sampled_times[i]->last_element);
      //          Serial.print("Stepper Pose\tdeg: ");
      //          Serial.print(current_steps * (360.0 / STEPS_PER_REVOLUTION));
      //          Serial.print("\tsteps: ");
      //          Serial.println(current_steps);
      //
      //          Serial.println();
      //        }
      //      }
      //      Serial.println("time_idx");
      //      Serial.println(time_idx);
      //      Serial.println(slider_buffer->fill_next);
      //      Serial.println(slider_buffer->sent_last);
      //      Serial.println(slider_buffer->buffer_length);

      //Print timestamps
      if (time_idx) {
        for (int i = 0; i < time_idx; i++) {
          if (debug)Serial.print(timestamps[i]);
          if (debug)Serial.print(", ");
        }
        if (debug)Serial.println();
        time_idx = 0;
      }

      //calculate peak velocity (ish)
      //      int total = 0;
      //      for (int i = 0; i < 5; i++) {
      //        total += slider_buffer -> data[(slider_buffer->fill_next - 1 - i) % slider_buffer->buffer_length] - \
      //                 slider_buffer -> data[(slider_buffer->fill_next - 2 - i) % slider_buffer->buffer_length];
      //      }

      //print position data
      while (buffer_has_data(slider_buffer)) {
        if (debug)Serial.print(grab_data_element(slider_buffer));
        else grab_data_element(slider_buffer);
        if (debug)Serial.print(", ");
      }
      if (debug) Serial.println();


      //      if (debug)Serial.println();
      //      //print peak velocity
      //      if (debug)Serial.print("Avg diff of last 5 positions:\t");
      //      if (debug)Serial.println(abs(total / 5.0));

      print_message_timed = false;

    }
  }


  // Get input for the desired location
  while (Serial.available() > 0) { // PC communication
    c = Serial.read();
    if (c == '\r') {
      user_command_buffer[user_command_buffer_index] = 0;
      parse((char*)user_command_buffer, argument_words, sizeof(argument_words));


      if (strcmp(argument_words[0], "set") == 0) {
        set_percent_track = atoi(argument_words[1]);
        set_speed = (uint8_t)atoi(argument_words[2]);
        if (strcmp(argument_words[3], "forward") == 0)
          move_slider(set_percent_track, set_speed, FORWARD, true);
        else if (strcmp(argument_words[3], "backward") == 0)
          move_slider(set_percent_track, set_speed, BACKWARD, true);
      }
      else if (strcmp(argument_words[0], "f") == 0) {
        set_speed = (uint8_t)atoi(argument_words[1]);
        read_percent_track = analogRead(slider_readout) / 10.24;
        if (read_percent_track > 80) {
          if (debug)Serial.println("Nope. Shoulda moved backwards.");
          else Serial.print('!');
        }
        else
        {
          if (debug)move_slider(90, set_speed, FORWARD, true);
          else move_slider(90, set_speed, FORWARD, false);
        }
      }
      else if (strcmp(argument_words[0], "b") == 0) {
        set_speed = (uint8_t)atoi(argument_words[1]);
        read_percent_track = analogRead(slider_readout) / 10.24;
        if (read_percent_track < 20) {
          if (debug)Serial.println("Nope. Shoulda moved forward.");
          else Serial.print('!');
        }
        else
        {
          if (debug) move_slider(10, set_speed, BACKWARD, true);
          else move_slider(10, set_speed, BACKWARD, false);
        }
      }
      else if (strcmp(argument_words[0], "read") == 0) {
        read_percent_track = analogRead(slider_readout);
        if (debug) {
          Serial.print("Current slider position at: ");
          Serial.print(read_percent_track / 10.24);
          Serial.println("%");
        }
        else {
          short trk = (short)read_percent_track;
          char temp_buff[2];
          memcpy(temp_buff, &trk, 2);
          Serial.write(temp_buff);
        }
      }
      else if (strcmp(argument_words[0], "enc") == 0) {
        float pos = read_encoder();
        if (debug) {
          Serial.print("Encoder deg: ");
          Serial.println(pos);
        }
        else {
          float enc = read_encoder();
          char temp_buff[4];
          memcpy(temp_buff, &enc, 4);
          Serial.write(temp_buff);
        }
      }
      else if (strcmp(argument_words[0], "stepper") == 0) {
        //Serial.print("Moving direction motor ");
        degrees_to_move = atoi(argument_words[1]);
        if (debug) Serial.println(degrees_to_move);
        if (strcmp(argument_words[2], "cc") == 0)
          direction_to_move = FORWARD;
        //        else if (strcmp(argument_words[2], "counterclockwise") == 0)
        else
          direction_to_move = BACKWARD;
        //        else {
        //          Serial.println(F("can only move motor \"clockwise\" or \"counterclockwise\""));
        //          continue;
        //        }
        move_stepper(degrees_to_move, direction_to_move, DOUBLE);
      }
      //      else if (strcmp(argument_words[0], "c") == 0) {//calibrate slider
      //        int forward_speed = atoi(argument_words[1]);
      //        int backward_speed = atoi(argument_words[2]);
      //        debug = true;
      //      }
      else if (strcmp(argument_words[0], "m") == 0) {
        int target_degrees = atoi(argument_words[1]);
        move_stepper_to(target_degrees);
      }
      else if (strcmp(argument_words[0], "deg") == 0) {
        if (debug) {
          Serial.print("Current pose (deg): ");
          Serial.print(int(current_steps * (360.0 / STEPS_PER_REVOLUTION)));
          Serial.println();
        }
      }
      else if (strcmp(argument_words[0], "d") == 0) {
        debug = !debug;
        if (debug)Serial.println("Entered debug mode.");
      }
      else if (strcmp(argument_words[0], "zero_stepper") == 0) {
        //set_degrees(0);
        current_steps = 0;
        if (debug) Serial.println("zeroed");
      }
      else if (strcmp(argument_words[0], "e") == 0) { //engage stepper
        engage_stepper();

      }
      else if (strcmp(argument_words[0], "set_stepper") == 0) {
        set_degrees(atoi(argument_words[1]));
      }
      else if (strcmp(argument_words[0], "start") == 0) {
        run_trial = true;
        trial_number = 0;
      }
      else if (strcmp(argument_words[0], "stop") == 0) {
        run_trial = false;
      }
      else if (strcmp(argument_words[0], "trial_number") == 0)
        trial_number = atoi(argument_words[1]);
      user_command_buffer_index = 0;
    }
    else if (((c == '\b') || (c == 0x7f)) && (user_command_buffer_index > 0))
      user_command_buffer_index--;
    else if ((c >= ' ') && (user_command_buffer_index < sizeof(user_command_buffer) - 1))
      user_command_buffer[user_command_buffer_index++] = c;

  }
}
