// Motor encoder globals
const uint8_t LEFT_ENCODER_PIN_A = 18;
const uint8_t LEFT_ENCODER_PIN_B = 19;
long left_encoder_pos = 0;
boolean left_encoder_updated = false;
const uint8_t RIGHT_ENCODER_PIN_A = 20;
const uint8_t RIGHT_ENCODER_PIN_B = 21;
long right_encoder_pos = 0;
boolean right_encoder_updated = false;
const int MAX_ENCODER_VAL = 32767;
const int MIN_ENCODER_VAL = -32768;


// Setup process for encoders
void setup_encoders() {
    
  // Setup encoders and encoder interrupts
  pinMode(LEFT_ENCODER_PIN_A, INPUT);
  digitalWrite(LEFT_ENCODER_PIN_A, HIGH);
  pinMode(LEFT_ENCODER_PIN_B, INPUT);
  digitalWrite(LEFT_ENCODER_PIN_B, HIGH);
  attachInterrupt(digital_pin_to_interrupt(LEFT_ENCODER_PIN_A), update_left_encoder, CHANGE);
  pinMode(RIGHT_ENCODER_PIN_A, INPUT);
  digitalWrite(RIGHT_ENCODER_PIN_A, HIGH);
  pinMode(RIGHT_ENCODER_PIN_B, INPUT);
  digitalWrite(RIGHT_ENCODER_PIN_B, HIGH);
  attachInterrupt(digital_pin_to_interrupt(RIGHT_ENCODER_PIN_A), update_right_encoder, CHANGE);
}


// Check if new encoder values have been read - if so, publish them
void check_encoders() {
  
  check_left_encoder();
  check_right_encoder();
}


// Check if new left_encoder values have been read - if so, publish them
void check_left_encoder() {
  
  if (left_encoder_updated) {
    left_encoder_msg.data = left_encoder_pos;
    left_encoder_publisher.publish(&left_encoder_msg);
    left_encoder_updated = false;
  }
}


// Check if new right_encoder values have been read - if so, publish them
void check_right_encoder() {
  
  if (right_encoder_updated) {
    right_encoder_msg.data = right_encoder_pos;
    right_encoder_publisher.publish(&right_encoder_msg);
    right_encoder_updated = false;
  }
}


// Reads left encoder pins and determines if there was a tick change
void update_left_encoder() {
  
  int channel_A = digitalRead(LEFT_ENCODER_PIN_A);
  int channel_B = digitalRead(LEFT_ENCODER_PIN_B);
  if (channel_A == channel_B) {
    // Decrement encoder value unless it is at min, in which case it should wrap
    if (left_encoder_pos <= MIN_ENCODER_VAL) {
      left_encoder_pos = MAX_ENCODER_VAL;
    }
    else {
      left_encoder_pos--;
    }
  }
  else {
    // Increment encoder value unless it is at max, in which case it should wrap
    if (left_encoder_pos >= MAX_ENCODER_VAL) {
      left_encoder_pos = MIN_ENCODER_VAL;
    }
    else {
      left_encoder_pos++;
    }
  }
  
  left_encoder_updated = true;
}


// Reads right encoder pins and determines if there was a tick change
void update_right_encoder() {
  
  int channel_A = digitalRead(RIGHT_ENCODER_PIN_A);
  int channel_B = digitalRead(RIGHT_ENCODER_PIN_B);
  if (channel_A == channel_B) {
    // Increment encoder value unless it is at max, in which case it should wrap
    if (right_encoder_pos >= MAX_ENCODER_VAL) {
      right_encoder_pos = MIN_ENCODER_VAL;
    }
    else {
      right_encoder_pos++;
    }
  }
  else {
    // Decrement encoder value unless it is at min, in which case it should wrap
    if (right_encoder_pos <= MIN_ENCODER_VAL) {
      right_encoder_pos = MAX_ENCODER_VAL;
    }
    else {
      right_encoder_pos--;
    }
  }
  
  right_encoder_updated = true;
}
