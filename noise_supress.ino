// Stepper Motor Position Decoder with State Validation
// Reads the two coil signals from a miniature stepper motor to track position
// Uses pin change interrupts with multi-sample validation instead of time-based debouncing

// Define the digital input pins for reading stepper motor coils
#define STEPPER_PIN_A 2  // Use digital pin 2 (supports interrupts)
#define STEPPER_PIN_B 3  // Use digital pin 3 (supports interrupts)

// Define the number of steps per revolution for the stepper motor
#define STEPS_PER_REV 200

// Number of consecutive valid samples required to accept a state change
#define VALIDATION_SAMPLES 3

// Global variables for position tracking (volatile because they're modified in ISR)
volatile long stepper_position = 0;  // Absolute position (can be negative)
volatile bool position_changed = false;  // Flag for main loop

// Current confirmed state
volatile uint8_t current_state = 0;

// State validation buffer
volatile uint8_t state_sample_buffer[VALIDATION_SAMPLES];
volatile uint8_t sample_index = 0;

// Quadrature decoder state transition table
// Rows: previous state (00, 01, 10, 11)
// Cols: current state (00, 01, 10, 11)
// Values: direction (-1 = CCW, 0 = invalid/no change, 1 = CW)
const int8_t state_table[4][4] = {
  { 0, -1,  1,  0},  // From state 00: valid transitions to 01(CCW) or 10(CW)
  { 1,  0,  0, -1},  // From state 01: valid transitions to 00(CW) or 11(CCW)
  {-1,  0,  0,  1},  // From state 10: valid transitions to 00(CCW) or 11(CW)
  { 0,  1, -1,  0}   // From state 11: valid transitions to 01(CW) or 10(CCW)
};

// State validity table - identifies which transitions are physically possible
// This helps filter out noise by rejecting impossible state jumps
// 1 = valid transition, 0 = invalid/impossible transition
const uint8_t valid_transition_table[4][4] = {
  {1, 1, 1, 0},  // From 00: can go to 00(stay), 01, 10, but NOT 11(impossible)
  {1, 1, 0, 1},  // From 01: can go to 00, 01(stay), 11, but NOT 10(impossible)
  {1, 0, 1, 1},  // From 10: can go to 00, 10(stay), 11, but NOT 01(impossible)
  {0, 1, 1, 1}   // From 11: can go to 01, 10, 11(stay), but NOT 00(impossible)
};

// Error tracking for diagnostics
volatile unsigned long invalid_transition_count = 0;
volatile unsigned long valid_transition_count = 0;

// Read current state of both pins
inline uint8_t read_encoder_state() {
  return (digitalRead(STEPPER_PIN_A) << 1) | digitalRead(STEPPER_PIN_B);
}

// Check if all samples in buffer are the same
bool validate_state_samples() {
  uint8_t first_sample = state_sample_buffer[0];
  for (uint8_t i = 1; i < VALIDATION_SAMPLES; i++) {
    if (state_sample_buffer[i] != first_sample) {
      return false;  // Samples don't match - noise detected
    }
  }
  return true;  // All samples match - state is stable
}

// Interrupt service routine for encoder state changes
void encoder_isr() {
  // Read current state
  uint8_t new_sample = read_encoder_state();
  
  // Add sample to circular buffer
  state_sample_buffer[sample_index] = new_sample;
  sample_index = (sample_index + 1) % VALIDATION_SAMPLES;
  
  // Check if we have enough consistent samples
  if (validate_state_samples()) {
    uint8_t validated_state = state_sample_buffer[0];
    
    // Check if this is a different state from current
    if (validated_state != current_state) {
      
      // Check if this is a valid transition
      if (valid_transition_table[current_state][validated_state]) {
        // Valid transition - look up direction
        int8_t direction = state_table[current_state][validated_state];
        
        // Update position if there's movement (direction != 0)
        if (direction != 0) {
          stepper_position += direction;
          position_changed = true;
          valid_transition_count++;
        }
        
        // Update current state
        current_state = validated_state;
        
      } else {
        // Invalid transition detected - likely noise
        // Reject this state change and clear the buffer
        invalid_transition_count++;
        
        // Reset buffer to current valid state
        for (uint8_t i = 0; i < VALIDATION_SAMPLES; i++) {
          state_sample_buffer[i] = current_state;
        }
        sample_index = 0;
      }
    }
  }
}

// Get current position (thread-safe)
long get_position() {
  long pos;
  noInterrupts();
  pos = stepper_position;
  interrupts();
  return pos;
}

// Reset position to zero (thread-safe)
void reset_position() {
  noInterrupts();
  stepper_position = 0;
  position_changed = true;
  interrupts();
}

// Set position to specific value (thread-safe)
void set_position(long new_position) {
  noInterrupts();
  stepper_position = new_position;
  position_changed = true;
  interrupts();
}

// Get position within single revolution (0 to STEPS_PER_REV-1)
int get_position_in_revolution() {
  long pos = get_position();
  int revolution_pos = pos % STEPS_PER_REV;
  // Handle negative positions correctly
  if (revolution_pos < 0) {
    revolution_pos += STEPS_PER_REV;
  }
  return revolution_pos;
}

// Get number of complete revolutions (can be negative)
long get_revolutions() {
  long pos = get_position();
  if (pos >= 0) {
    return pos / STEPS_PER_REV;
  } else {
    return (pos + 1) / STEPS_PER_REV - 1;
  }
}

// Get angle in degrees (0-360)
float get_angle_degrees() {
  return (get_position_in_revolution() * 360.0) / STEPS_PER_REV;
}

// Get error statistics (thread-safe)
void get_error_stats(unsigned long *valid, unsigned long *invalid) {
  noInterrupts();
  *valid = valid_transition_count;
  *invalid = invalid_transition_count;
  interrupts();
}

// Reset error statistics
void reset_error_stats() {
  noInterrupts();
  valid_transition_count = 0;
  invalid_transition_count = 0;
  interrupts();
}

// Calculate error rate percentage
float get_error_rate() {
  unsigned long valid, invalid;
  get_error_stats(&valid, &invalid);
  unsigned long total = valid + invalid;
  if (total == 0) return 0.0;
  return (invalid * 100.0) / total;
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Stepper Motor Position Decoder v2.0");
  Serial.println("State-Validated Decoding with Noise Rejection");
  Serial.println("==============================================");
  
  // Configure pins as inputs with pull-ups
  pinMode(STEPPER_PIN_A, INPUT_PULLUP);
  pinMode(STEPPER_PIN_B, INPUT_PULLUP);
  
  // Read and initialize state
  current_state = read_encoder_state();
  
  // Initialize sample buffer with current state
  for (uint8_t i = 0; i < VALIDATION_SAMPLES; i++) {
    state_sample_buffer[i] = current_state;
  }
  sample_index = 0;
  
  // Attach interrupts to both pins for any state change
  attachInterrupt(digitalPinToInterrupt(STEPPER_PIN_A), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEPPER_PIN_B), encoder_isr, CHANGE);
  
  Serial.println("Ready. Rotate the stepper motor...");
  Serial.println("Commands: 'r'=reset, 's'=status, 'e'=error stats, 'c'=clear stats");
  Serial.println();
}

void loop() {
  // Check if position has changed
  if (position_changed) {
    position_changed = false;
    
    // Get position data
    long pos = get_position();
    int rev_pos = get_position_in_revolution();
    long revolutions = get_revolutions();
    float angle = get_angle_degrees();
    
    // Display position information
    Serial.print("Pos: ");
    Serial.print(pos);
    Serial.print(" | Rev: ");
    Serial.print(revolutions);
    Serial.print(" | Step: ");
    Serial.print(rev_pos);
    Serial.print("/");
    Serial.print(STEPS_PER_REV);
    Serial.print(" | Angle: ");
    Serial.print(angle, 1);
    Serial.println("°");
    
    // ==== ADD YOUR CONTROL LOGIC HERE ====
    // Example: Control something based on position
    /*
    if (pos > 100) {
      // Do something when position exceeds 100 steps
    }
    
    if (angle > 180.0) {
      // Do something when angle is in second half of revolution
    }
    */
  }
  
  // Check for serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'r':
      case 'R':
        reset_position();
        Serial.println("\n>>> Position reset to zero <<<\n");
        break;
        
      case 's':
      case 'S':
        Serial.println("\n=== Status ===");
        Serial.print("Position: ");
        Serial.print(get_position());
        Serial.println(" steps");
        Serial.print("Revolutions: ");
        Serial.println(get_revolutions());
        Serial.print("Angle: ");
        Serial.print(get_angle_degrees(), 2);
        Serial.println("°");
        Serial.println();
        break;
        
      case 'e':
      case 'E':
        {
          unsigned long valid, invalid;
          get_error_stats(&valid, &invalid);
          Serial.println("\n=== Error Statistics ===");
          Serial.print("Valid transitions: ");
          Serial.println(valid);
          Serial.print("Invalid transitions (rejected): ");
          Serial.println(invalid);
          Serial.print("Error rate: ");
          Serial.print(get_error_rate(), 2);
          Serial.println("%");
          Serial.println();
        }
        break;
        
      case 'c':
      case 'C':
        reset_error_stats();
        Serial.println("\n>>> Error statistics cleared <<<\n");
        break;
        
      case '?':
      case 'h':
      case 'H':
        Serial.println("\n=== Commands ===");
        Serial.println("r - Reset position to zero");
        Serial.println("s - Show current status");
        Serial.println("e - Show error statistics");
        Serial.println("c - Clear error statistics");
        Serial.println("h - Show this help");
        Serial.println();
        break;
    }
  }
  
  // Small delay to prevent overwhelming serial output
  delay(10);
}
