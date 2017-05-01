
// Sending audio state through digital pins
const uint8_t AUDIO_STATE_WRITE_PIN_1 = 48;
const uint8_t AUDIO_STATE_WRITE_PIN_2 = 49;
const uint8_t AUDIO_STATE_WRITE_PIN_4 = 46;
uint8_t audio_state_write_1 = 0;
uint8_t audio_state_write_2 = 0;
uint8_t audio_state_write_4 = 0;
uint8_t curr_audio_state = 0;
uint8_t prev_audio_state = 0;


// Callback function for a audio_cmd message
void audio_cmd_callback(const std_msgs::Int8& msg) {
  
  prev_audio_state = curr_audio_state;
  if (0 <= msg.data && msg.data <= 7) {
    curr_audio_state = msg.data;
  }
  else {
    debug_print("ERROR: Valid audio states are 0-7. Attempted to set audio state to: " +
        String(msg.data));
    curr_audio_state = 0;
  }
  
  set_audio_state();
}
ros::Subscriber<std_msgs::Int8> audio_cmd_sub("audio_cmd", &audio_cmd_callback);


// Setup process for audio
void setup_audio() {
  
  // ROS subscriber
  nh.subscribe(audio_cmd_sub);
  
  pinMode(AUDIO_STATE_WRITE_PIN_1, OUTPUT);
  pinMode(AUDIO_STATE_WRITE_PIN_2, OUTPUT);
  pinMode(AUDIO_STATE_WRITE_PIN_4, OUTPUT);
  digitalWrite(AUDIO_STATE_WRITE_PIN_1, LOW);
  digitalWrite(AUDIO_STATE_WRITE_PIN_2, LOW);
  digitalWrite(AUDIO_STATE_WRITE_PIN_4, LOW);
}


// Convert the current audio state into a 3 bit binary number with each bit stored in a separate variable
void set_audio_state() {
  
  // If this is a new audio state
  if (curr_audio_state != prev_audio_state) {
    
    // Decompose curr_audio_state into a 3 bit binary number
    uint8_t decomposed_audio_state = curr_audio_state;
    
    // 0X00
    if (decomposed_audio_state >= 4) {
      decomposed_audio_state -= 4;
      audio_state_write_4 = 1;
    }
    else {
      audio_state_write_4 = 0;
    }
    
    // 00X0
    if (decomposed_audio_state >= 2) {
      decomposed_audio_state -= 2;
      audio_state_write_2 = 1;
    }
    else {
      audio_state_write_2 = 0;
    }
    
    // 000X
    if (decomposed_audio_state >= 1) {
      decomposed_audio_state -= 1;
      audio_state_write_1 = 1;
    }
    else {
      audio_state_write_1 = 0;
    }
    
    send_audio_state();
  }
}


// Send the audio state over digital pins to the second Arduino so it can play the correct audio file
void send_audio_state() {
      debug_print("Sending audio state: DEC(" + String(curr_audio_state) + "), BIN(" +
        String(audio_state_write_4) + String(audio_state_write_2) + String(audio_state_write_1) + ")");
    
    // Write audio state as 3 bit binary number where each bit is a digital pin
    digitalWrite(AUDIO_STATE_WRITE_PIN_4, audio_state_write_4*255); // 0X00
    digitalWrite(AUDIO_STATE_WRITE_PIN_2, audio_state_write_2*255); // 00X0
    digitalWrite(AUDIO_STATE_WRITE_PIN_1, audio_state_write_1*255); // 000X
}
