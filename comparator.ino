// Advanced Stepper Motor Position Decoder with Analog Comparator
// Uses Arduino's built-in analog comparator for differential signal detection
// Ideal for low-voltage stepper motor coil signals with superior noise immunity

// ==================== CONFIGURATION ====================

// Analog Comparator pins (hardware fixed on most Arduinos)
// AIN0 (Positive input) = D6 (Arduino Uno/Nano) - Connect to Coil A
// AIN1 (Negative input) = D7 (Arduino Uno/Nano) - Connect to Coil B
// Note: These are hardware pins and cannot be changed

// Alternative phase B detection pin (for full quadrature)
#define PHASE_B_PIN 3  // Interrupt-capable digital pin for second coil pair

// Motor characteristics
#define STEPS_PER_REV 200

// Validation settings
#define VALIDATION_SAMPLES 3
#define MAX_VELOCITY 10000

// Performance monitoring
#define STATS_UPDATE_INTERVAL 5000

// Comparator detection mode
enum ComparatorMode {
  DIFFERENTIAL,    // Compare two coil voltages directly (highest sensitivity)
  THRESHOLD_AIN0,  // AIN0 vs internal bandgap reference
  THRESHOLD_AIN1   // AIN1 vs internal bandgap reference
};

ComparatorMode COMP_MODE = DIFFERENTIAL;

// ==================== STATE MACHINE ====================

// Modified state encoding for comparator-based detection
// State bits: [PhaseB_digital][Comparator_output]
// Comparator output: 1 when AIN0 > AIN1, 0 when AIN0 < AIN1

const int8_t PROGMEM state_table[4][4] = {
  { 0, -1,  1,  0},
  { 1,  0,  0, -1},
  {-1,  0,  0,  1},
  { 0,  1, -1,  0}
};

const uint8_t PROGMEM valid_transition_table[4][4] = {
  {1, 1, 1, 0},
  {1, 1, 0, 1},
  {1, 0, 1, 1},
  {0, 1, 1, 1}
};

// ==================== GLOBAL STATE ====================

volatile long stepper_position = 0;
volatile bool position_changed = false;

volatile uint8_t current_state = 0;
volatile uint8_t state_sample_buffer[VALIDATION_SAMPLES];
volatile uint8_t sample_index = 0;

// Performance metrics
volatile unsigned long valid_transitions = 0;
volatile unsigned long invalid_transitions = 0;
volatile unsigned long velocity_violations = 0;
volatile unsigned long total_samples = 0;
volatile unsigned long comparator_triggers = 0;

// Velocity tracking
volatile unsigned long last_micros = 0;

// ==================== ANALOG COMPARATOR SETUP ====================

void setup_analog_comparator() {
  // Disable digital input buffers on AIN0 (D6) and AIN1 (D7) to reduce power
  DIDR1 |= (1 << AIN1D) | (1 << AIN0D);
  
  switch (COMP_MODE) {
    case DIFFERENTIAL:
      // Compare AIN0 (D6) with AIN1 (D7)
      // ACO bit in ACSR will be 1 when AIN0 > AIN1
      ACSR = (1 << ACIE) |   // Analog Comparator Interrupt Enable
             (0 << ACIS1) | (0 << ACIS0);  // Interrupt on toggle
      break;
      
    case THRESHOLD_AIN0:
      // Compare AIN0 with internal 1.1V bandgap reference
      ACSR = (1 << ACIE) |   // Enable interrupt
             (1 << ACBG) |   // Enable bandgap reference
             (0 << ACIS1) | (0 << ACIS0);
      break;
      
    case THRESHOLD_AIN1:
      // Compare AIN1 with bandgap (requires swapping connection)
      ACSR = (1 << ACIE) | (1 << ACBG) |
             (0 << ACIS1) | (0 << ACIS0);
      break;
  }
  
  // Clear any pending interrupt
  ACSR |= (1 << ACI);
  
  Serial.print(F("Comparator mode: "));
  switch (COMP_MODE) {
    case DIFFERENTIAL:
      Serial.println(F("DIFFERENTIAL (AIN0 vs AIN1)"));
      Serial.println(F("  Connect Coil A+ to D6 (AIN0)"));
      Serial.println(F("  Connect Coil A- to D7 (AIN1)"));
      break;
    case THRESHOLD_AIN0:
      Serial.println(F("THRESHOLD (AIN0 vs 1.1V)"));
      break;
    case THRESHOLD_AIN1:
      Serial.println(F("THRESHOLD (AIN1 vs 1.1V)"));
      break;
  }
}

// Read comparator output bit
inline bool read_comparator() {
  return (ACSR & (1 << ACO)) != 0;  // Returns true if AIN0 > AIN1
}

// ==================== STATE READING ====================

inline uint8_t read_state() {
  // Bit 1: Phase B digital pin
  // Bit 0: Analog comparator output (differential Phase A)
  uint8_t phase_b = digitalRead(PHASE_B_PIN) ? 1 : 0;
  uint8_t comp_out = read_comparator() ? 1 : 0;
  return (phase_b << 1) | comp_out;
}

inline bool buffer_is_consistent() {
  uint8_t first = state_sample_buffer[0];
  for (uint8_t i = 1; i < VALIDATION_SAMPLES; i++) {
    if (state_sample_buffer[i] != first) return false;
  }
  return true;
}

inline int8_t get_direction(uint8_t from_state, uint8_t to_state) {
  return pgm_read_byte(&state_table[from_state][to_state]);
}

inline bool is_valid_transition(uint8_t from_state, uint8_t to_state) {
  return pgm_read_byte(&valid_transition_table[from_state][to_state]);
}

// ==================== INTERRUPT HANDLERS ====================

// Analog Comparator ISR - triggered when comparator output changes
ISR(ANALOG_COMP_vect) {
  comparator_triggers++;
  process_state_change();
}

// Phase B pin change ISR
void phase_b_isr() {
  process_state_change();
}

// Common state change processing
void process_state_change() {
  total_samples++;
  
  // Sample current state
  uint8_t new_sample = read_state();
  state_sample_buffer[sample_index] = new_sample;
  sample_index = (sample_index + 1) % VALIDATION_SAMPLES;
  
  // Check for consistent state
  if (!buffer_is_consistent()) return;
  
  uint8_t validated_state = state_sample_buffer[0];
  
  // State unchanged
  if (validated_state == current_state) return;
  
  // Validate transition
  if (!is_valid_transition(current_state, validated_state)) {
    invalid_transitions++;
    // Reset buffer
    for (uint8_t i = 0; i < VALIDATION_SAMPLES; i++) {
      state_sample_buffer[i] = current_state;
    }
    sample_index = 0;
    return;
  }
  
  // Get direction
  int8_t direction = get_direction(current_state, validated_state);
  
  if (direction != 0) {
    // Velocity sanity check
    unsigned long now = micros();
    unsigned long dt = now - last_micros;
    
    if (dt > 0 && dt < 100) {  // Less than 100µs is suspicious
      unsigned long velocity = 1000000UL / dt;
      if (velocity > MAX_VELOCITY) {
        velocity_violations++;
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
  unsigned long comp_triggers;
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
  stats.comp_triggers = comparator_triggers;
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
  comparator_triggers = 0;
  interrupts();
}

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
  Serial.print(F("Comparator:    ")); 
  Serial.println(read_comparator() ? F("HIGH (AIN0>AIN1)") : F("LOW (AIN0<AIN1)"));
  Serial.println();
}

void print_statistics() {
  Statistics stats = get_statistics();
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║       PERFORMANCE METRICS          ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.print(F("Total samples:        ")); Serial.println(stats.samples);
  Serial.print(F("Comparator triggers:  ")); Serial.println(stats.comp_triggers);
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
  Serial.println(F("d - Show raw comparator state"));
  Serial.println(F("h - Show this help"));
  Serial.println();
}

void print_raw_state() {
  uint8_t state = read_state();
  bool comp = read_comparator();
  bool phase_b = digitalRead(PHASE_B_PIN);
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║         RAW SIGNAL STATE           ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.print(F("Comparator output:  ")); Serial.println(comp ? F("1 (AIN0>AIN1)") : F("0 (AIN0<AIN1)"));
  Serial.print(F("Phase B (D3):       ")); Serial.println(phase_b ? F("1 (HIGH)") : F("0 (LOW)"));
  Serial.print(F("Combined state:     ")); Serial.print(state, BIN); 
  Serial.print(F(" (0b")); 
  if (state < 2) Serial.print(F("0"));
  Serial.print(state, BIN); Serial.println(F(")"));
  Serial.print(F("State decimal:      ")); Serial.println(state);
  Serial.println();
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Serial.println(F("\n╔════════════════════════════════════════════════╗"));
  Serial.println(F("║  Advanced Stepper Decoder - Comparator Mode    ║"));
  Serial.println(F("║  v4.0 - Analog Comparator for Low Voltages    ║"));
  Serial.println(F("╚════════════════════════════════════════════════╝\n"));
  
  Serial.print(F("Steps per revolution: ")); Serial.println(STEPS_PER_REV);
  Serial.print(F("Validation samples:   ")); Serial.println(VALIDATION_SAMPLES);
  Serial.print(F("Max velocity:         ")); Serial.print(MAX_VELOCITY); Serial.println(F(" steps/s"));
  Serial.println();
  
  // Configure Phase B pin
  pinMode(PHASE_B_PIN, INPUT_PULLUP);
  
  // Setup analog comparator
  setup_analog_comparator();
  
  // Initialize state
  current_state = read_state();
  for (uint8_t i = 0; i < VALIDATION_SAMPLES; i++) {
    state_sample_buffer[i] = current_state;
  }
  
  // Initialize timing
  last_micros = micros();
  
  // Attach Phase B interrupt
  attachInterrupt(digitalPinToInterrupt(PHASE_B_PIN), phase_b_isr, CHANGE);
  
  Serial.println(F("✓ System ready. Type 'h' for help.\n"));
  Serial.print(F("Initial state: "));
  print_raw_state();
}

// ==================== MAIN LOOP ====================

void loop() {
  static unsigned long last_stats_time = 0;
  
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
  }
  
  if (millis() - last_stats_time > STATS_UPDATE_INTERVAL) {
    last_stats_time = millis();
    Statistics stats = get_statistics();
    if (stats.samples > 100) {
      Serial.print(F("[Stats] Samples: ")); Serial.print(stats.samples);
      Serial.print(F(" | CompTrig: ")); Serial.print(stats.comp_triggers);
      Serial.print(F(" | Valid: ")); Serial.print(stats.valid);
      Serial.print(F(" | Errors: ")); Serial.print(stats.error_rate, 1);
      Serial.println(F("%"));
    }
  }
  
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();
    
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
      case 'd': case 'D':
        print_raw_state();
        break;
      case 'h': case 'H': case '?':
        print_help();
        break;
      default:
        if (cmd >= 32 && cmd <= 126) {
          Serial.print(F("\n? Unknown command: '"));
          Serial.print(cmd);
          Serial.println(F("' - Type 'h' for help\n"));
        }
        break;
    }
  }
  
  delay(10);
}
