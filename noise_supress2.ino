// Advanced Stepper Motor Position Decoder
// High-performance quadrature decoder with adaptive filtering and comprehensive diagnostics
// Optimized for miniature stepper motors with robust noise immunity

// ==================== CONFIGURATION ====================

// Pin definitions - Use interrupt-capable pins
#define STEPPER_PIN_A 2
#define STEPPER_PIN_B 3

// Motor characteristics
#define STEPS_PER_REV 200

// Validation settings
#define VALIDATION_SAMPLES 3  // Number of consistent samples required
#define MAX_VELOCITY 10000    // Maximum expected steps/second for sanity checking

// Performance monitoring interval (milliseconds)
#define STATS_UPDATE_INTERVAL 5000

// ==================== STATE MACHINE ====================

// Quadrature state transition table
// Encodes direction: -1=CCW, 0=invalid/no-change, 1=CW
const int8_t PROGMEM state_table[4][4] = {
  { 0, -1,  1,  0},
  { 1,  0,  0, -1},
  {-1,  0,  0,  1},
  { 0,  1, -1,  0}
};

// Valid state transition table - filters impossible jumps
const uint8_t PROGMEM valid_transition_table[4][4] = {
  {1, 1, 1, 0},
  {1, 1, 0, 1},
  {1, 0, 1, 1},
  {0, 1, 1, 1}
};

// ==================== GLOBAL STATE ====================

// Position tracking (volatile for ISR access)
volatile long stepper_position = 0;
volatile long position_at_last_check = 0;
volatile bool position_changed = false;

// State machine
volatile uint8_t current_state = 0;
volatile uint8_t state_sample_buffer[VALIDATION_SAMPLES];
volatile uint8_t sample_index = 0;

// Performance metrics
volatile unsigned long valid_transitions = 0;
volatile unsigned long invalid_transitions = 0;
volatile unsigned long velocity_violations = 0;
volatile unsigned long total_samples = 0;

// Velocity tracking for anomaly detection
volatile long last_position = 0;
volatile unsigned long last_micros = 0;

// ==================== INLINE HELPERS ====================

// Fast state read
inline uint8_t read_state() {
  return (digitalRead(STEPPER_PIN_A) << 1) | digitalRead(STEPPER_PIN_B);
}

// Validate buffer consistency
inline bool buffer_is_consistent() {
  uint8_t first = state_sample_buffer[0];
  for (uint8_t i = 1; i < VALIDATION_SAMPLES; i++) {
    if (state_sample_buffer[i] != first) return false;
  }
  return true;
}

// Read from PROGMEM tables
inline int8_t get_direction(uint8_t from_state, uint8_t to_state) {
  return pgm_read_byte(&state_table[from_state][to_state]);
}

inline bool is_valid_transition(uint8_t from_state, uint8_t to_state) {
  return pgm_read_byte(&valid_transition_table[from_state][to_state]);
}

// ==================== ISR ====================

void encoder_isr() {
  total_samples++;
  
  // Sample current state
  uint8_t new_sample = read_state();
  state_sample_buffer[sample_index] = new_sample;
  sample_index = (sample_index + 1) % VALIDATION_SAMPLES;
  
  // Check for consistent state
  if (!buffer_is_consistent()) return;
  
  uint8_t validated_state = state_sample_buffer[0];
  
  // State unchanged - exit early
  if (validated_state == current_state) return;
  
  // Validate transition
  if (!is_valid_transition(current_state, validated_state)) {
    invalid_transitions++;
    // Reset buffer to current known-good state
    for (uint8_t i = 0; i < VALIDATION_SAMPLES; i++) {
      state_sample_buffer[i] = current_state;
    }
    sample_index = 0;
    return;
  }
  
  // Get direction
  int8_t direction = get_direction(current_state, validated_state);
  
  if (direction != 0) {
    // Velocity sanity check (optional, prevents runaway on catastrophic failure)
    unsigned long now = micros();
    unsigned long dt = now - last_micros;
    
    if (dt > 0) {
      // Calculate instantaneous velocity (steps per second)
      unsigned long velocity = 1000000UL / dt;
      
      if (velocity > MAX_VELOCITY) {
        velocity_violations++;
        // Don't update position if velocity is impossibly high
        last_micros = now;
        return;
      }
    }
    
    last_micros = now;
    
    // Update position
    stepper_position += direction;
    position_changed = true;
    valid_transitions++;
  }
  
  // Update state
  current_state = validated_state;
}

// ==================== THREAD-SAFE ACCESSORS ====================

long get_position() {
  noInterrupts();
  long pos = stepper_position;
  interrupts();
  return pos;
}

void set_position(long new_pos) {
  noInterrupts();
  stepper_position = new_pos;
  position_changed = true;
  interrupts();
}

void reset_position() {
  set_position(0);
}

// ==================== POSITION CALCULATIONS ====================

int get_position_in_revolution() {
  long pos = get_position();
  int rev_pos = pos % STEPS_PER_REV;
  return (rev_pos < 0) ? rev_pos + STEPS_PER_REV : rev_pos;
}

long get_revolutions() {
  long pos = get_position();
  return (pos >= 0) ? (pos / STEPS_PER_REV) : ((pos + 1) / STEPS_PER_REV - 1);
}

float get_angle_degrees() {
  return (get_position_in_revolution() * 360.0f) / STEPS_PER_REV;
}

float get_angle_radians() {
  return (get_position_in_revolution() * TWO_PI) / STEPS_PER_REV;
}

// ==================== DIAGNOSTICS ====================

struct Statistics {
  unsigned long valid;
  unsigned long invalid;
  unsigned long velocity_errors;
  unsigned long samples;
  float error_rate;
  float rejection_rate;
  float sample_efficiency;
};

Statistics get_statistics() {
  Statistics stats;
  noInterrupts();
  stats.valid = valid_transitions;
  stats.invalid = invalid_transitions;
  stats.velocity_errors = velocity_violations;
  stats.samples = total_samples;
  interrupts();
  
  unsigned long total_transitions = stats.valid + stats.invalid;
  stats.error_rate = (total_transitions > 0) ? 
                     (stats.invalid * 100.0f) / total_transitions : 0.0f;
  stats.rejection_rate = (stats.samples > 0) ? 
                         ((stats.invalid + stats.velocity_errors) * 100.0f) / stats.samples : 0.0f;
  stats.sample_efficiency = (stats.samples > 0) ? 
                            (stats.valid * 100.0f) / stats.samples : 0.0f;
  
  return stats;
}

void reset_statistics() {
  noInterrupts();
  valid_transitions = 0;
  invalid_transitions = 0;
  velocity_violations = 0;
  total_samples = 0;
  interrupts();
}

// Calculate current velocity (steps per second)
float get_velocity() {
  static long prev_pos = 0;
  static unsigned long prev_time = 0;
  
  long curr_pos = get_position();
  unsigned long curr_time = millis();
  
  if (curr_time == prev_time) return 0.0f;
  
  float velocity = (curr_pos - prev_pos) * 1000.0f / (curr_time - prev_time);
  
  prev_pos = curr_pos;
  prev_time = curr_time;
  
  return velocity;
}

// ==================== DISPLAY FUNCTIONS ====================

void print_status() {
  long pos = get_position();
  int rev_pos = get_position_in_revolution();
  long revs = get_revolutions();
  float angle = get_angle_degrees();
  float vel = get_velocity();
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║          SYSTEM STATUS             ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.print(F("Position:      ")); Serial.print(pos); Serial.println(F(" steps"));
  Serial.print(F("Revolutions:   ")); Serial.println(revs);
  Serial.print(F("Step in rev:   ")); Serial.print(rev_pos); 
  Serial.print(F("/")); Serial.println(STEPS_PER_REV);
  Serial.print(F("Angle:         ")); Serial.print(angle, 2); Serial.println(F("°"));
  Serial.print(F("Velocity:      ")); Serial.print(vel, 1); Serial.println(F(" steps/s"));
  Serial.println();
}

void print_statistics() {
  Statistics stats = get_statistics();
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║       PERFORMANCE METRICS          ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.print(F("Total samples:        ")); Serial.println(stats.samples);
  Serial.print(F("Valid transitions:    ")); Serial.println(stats.valid);
  Serial.print(F("Invalid transitions:  ")); Serial.println(stats.invalid);
  Serial.print(F("Velocity violations:  ")); Serial.println(stats.velocity_errors);
  Serial.println(F("────────────────────────────────────"));
  Serial.print(F("Error rate:           ")); Serial.print(stats.error_rate, 2); Serial.println(F("%"));
  Serial.print(F("Rejection rate:       ")); Serial.print(stats.rejection_rate, 2); Serial.println(F("%"));
  Serial.print(F("Sample efficiency:    ")); Serial.print(stats.sample_efficiency, 2); Serial.println(F("%"));
  Serial.println();
}

void print_help() {
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║            COMMANDS                ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("r - Reset position to zero"));
  Serial.println(F("s - Show system status"));
  Serial.println(F("e - Show error statistics"));
  Serial.println(F("c - Clear error statistics"));
  Serial.println(F("v - Show velocity"));
  Serial.println(F("h - Show this help"));
  Serial.println();
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for serial (with timeout)
  
  Serial.println(F("\n╔════════════════════════════════════════════════╗"));
  Serial.println(F("║  Advanced Stepper Motor Position Decoder       ║"));
  Serial.println(F("║  v3.0 - State Validated with Diagnostics       ║"));
  Serial.println(F("╚════════════════════════════════════════════════╝\n"));
  
  Serial.print(F("Steps per revolution: ")); Serial.println(STEPS_PER_REV);
  Serial.print(F("Validation samples:   ")); Serial.println(VALIDATION_SAMPLES);
  Serial.print(F("Max velocity:         ")); Serial.print(MAX_VELOCITY); Serial.println(F(" steps/s"));
  
  // Configure pins
  pinMode(STEPPER_PIN_A, INPUT_PULLUP);
  pinMode(STEPPER_PIN_B, INPUT_PULLUP);
  
  // Initialize state
  current_state = read_state();
  for (uint8_t i = 0; i < VALIDATION_SAMPLES; i++) {
    state_sample_buffer[i] = current_state;
  }
  
  // Initialize timing
  last_micros = micros();
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(STEPPER_PIN_A), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEPPER_PIN_B), encoder_isr, CHANGE);
  
  Serial.println(F("\n✓ System ready. Type 'h' for help.\n"));
}

// ==================== MAIN LOOP ====================

void loop() {
  static unsigned long last_stats_time = 0;
  
  // Position update notification
  if (position_changed) {
    position_changed = false;
    
    long pos = get_position();
    int rev_pos = get_position_in_revolution();
    float angle = get_angle_degrees();
    
    Serial.print(F("Pos: ")); Serial.print(pos);
    Serial.print(F(" | Rev: ")); Serial.print(get_revolutions());
    Serial.print(F(" | Step: ")); Serial.print(rev_pos);
    Serial.print(F("/")); Serial.print(STEPS_PER_REV);
    Serial.print(F(" | ∠")); Serial.print(angle, 1); Serial.println(F("°"));
    
    // ===== USER CONTROL LOGIC HERE =====
    // Example: Trigger actions based on position
    /*
    if (pos % 100 == 0) {
      // Every 100 steps
    }
    if (angle < 90.0f && angle > 85.0f) {
      // In specific angle range
    }
    */
  }
  
  // Periodic statistics update
  if (millis() - last_stats_time > STATS_UPDATE_INTERVAL) {
    last_stats_time = millis();
    Statistics stats = get_statistics();
    if (stats.samples > 100) {  // Only if meaningful data
      Serial.print(F("[Stats] Samples: ")); Serial.print(stats.samples);
      Serial.print(F(" | Valid: ")); Serial.print(stats.valid);
      Serial.print(F(" | Errors: ")); Serial.print(stats.error_rate, 1);
      Serial.println(F("%"));
    }
  }
  
  // Command processing
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read(); // Flush buffer
    
    switch (cmd) {
      case 'r': case 'R':
        reset_position();
        Serial.println(F("\n✓ Position reset to zero\n"));
        break;
        
      case 's': case 'S':
        print_status();
        break;
        
      case 'e': case 'E':
        print_statistics();
        break;
        
      case 'c': case 'C':
        reset_statistics();
        Serial.println(F("\n✓ Statistics cleared\n"));
        break;
        
      case 'v': case 'V':
        Serial.print(F("\nVelocity: "));
        Serial.print(get_velocity(), 2);
        Serial.println(F(" steps/s\n"));
        break;
        
      case 'h': case 'H': case '?':
        print_help();
        break;
        
      default:
        if (cmd >= 32 && cmd <= 126) {  // Printable character
          Serial.print(F("\n? Unknown command: '"));
          Serial.print(cmd);
          Serial.println(F("' - Type 'h' for help\n"));
        }
        break;
    }
  }
  
  delay(10);
}
