// Stepper Motor Position Decoder
// Reads the two coil signals from a miniature stepper motor to track position
// Uses pin change interrupts for efficient, real-time decoding

// Define the digital input pins for reading stepper motor coils
#define STEPPER_PIN_A 2  // Use digital pin 2 (supports interrupts)
#define STEPPER_PIN_B 3  // Use digital pin 3 (supports interrupts)

// Define the number of steps per revolution for the stepper motor
#define STEPS_PER_REV 200

// Debounce time in microseconds (adjust based on your motor characteristics)
#define DEBOUNCE_TIME 50

// Global variables for position tracking (volatile because they're modified in ISR)
volatile long stepper_position = 0;  // Absolute position (can be negative)
volatile bool position_changed = false;  // Flag for main loop

// Previous state of the encoder pins
volatile uint8_t prev_state = 0;

// Debounce timing
volatile unsigned long last_interrupt_time = 0;

// Quadrature decoder state transition table
// Rows: previous state (00, 01, 10, 11)
// Cols: current state (00, 01, 10, 11)
// Values: direction (-1 = CCW, 0 = invalid/no change, 1 = CW)
const int8_t state_table[4][4] = {
  { 0, -1,  1,  0},  // From state 00
  { 1,  0,  0, -1},  // From state 01
  {-1,  0,  0,  1},  // From state 10
  { 0,  1, -1,  0}   // From state 11
};

// Read current state of both pins
inline uint8_t read_encoder_state() {
  return (digitalRead(STEPPER_PIN_A) << 1) | digitalRead(STEPPER_PIN_B);
}

// Interrupt service routine for encoder state changes
void encoder_isr() {
  // Simple debouncing
  unsigned long interrupt_time = micros();
  if (interrupt_time - last_interrupt_time < DEBOUNCE_TIME) {
    return;  // Ignore if within debounce period
  }
  last_interrupt_time = interrupt_time;
  
  // Read current state
  uint8_t curr_state = read_encoder_state();
  
  // Look up direction from state table
  int8_t direction = state_table[prev_state][curr_state];
  
  // Update position if valid transition (direction != 0)
  if (direction != 0) {
    stepper_position += direction;
    position_changed = true;
  }
  
  // Update previous state
  prev_state = curr_state;
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

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Stepper Motor Position Decoder");
  Serial.println("===============================");
  
  // Configure pins as inputs with pull-ups
  pinMode(STEPPER_PIN_A, INPUT_PULLUP);
  pinMode(STEPPER_PIN_B, INPUT_PULLUP);
  
  // Read initial state
  prev_state = read_encoder_state();
  
  // Attach interrupts to both pins for any state change
  attachInterrupt(digitalPinToInterrupt(STEPPER_PIN_A), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEPPER_PIN_B), encoder_isr, CHANGE);
  
  Serial.println("Ready. Rotate the stepper motor...");
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
    Serial.print("Position: ");
    Serial.print(pos);
    Serial.print(" steps | Revolution: ");
    Serial.print(revolutions);
    Serial.print(" | Step in rev: ");
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
  
  // Add any other non-blocking code here
  // The main loop can handle other tasks while the ISR tracks position
  
  // Example: Check for serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      reset_position();
      Serial.println("\n>>> Position reset to zero <<<\n");
    } else if (cmd == 's' || cmd == 'S') {
      Serial.print("\nCurrent position: ");
      Serial.print(get_position());
      Serial.println(" steps\n");
    }
  }
  
  // Small delay to prevent overwhelming serial output if motor spins very fast
  delay(10);
}
