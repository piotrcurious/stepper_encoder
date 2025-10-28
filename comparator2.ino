// Advanced Stepper Motor Position Decoder with Analog Comparator
// Compares Coil A vs Coil B voltages for superior differential detection
// Ideal for low-voltage stepper signals with excellent common-mode noise rejection

// ==================== CONFIGURATION ====================

// Analog Comparator pins (hardware fixed)
// AIN0 (D6) - Connect to Coil A (other end to GND)
// AIN1 (D7) - Connect to Coil B (other end to GND)
// The comparator detects which coil has higher voltage

// Secondary detection method for full quadrature
#define USE_ANALOG_READS true  // Use ADC to read absolute coil levels
#define COIL_A_ANALOG A0       // Analog pin for Coil A absolute level
#define COIL_B_ANALOG A1       // Analog pin for Coil B absolute level
#define THRESHOLD 512          // Mid-scale threshold for analog reads

// Motor characteristics
#define STEPS_PER_REV 200

// Validation settings
#define VALIDATION_SAMPLES 3
#define MAX_VELOCITY 10000
#define STATS_UPDATE_INTERVAL 5000

// ==================== STATE MACHINE RETHOUGHT ====================

// State encoding for differential + absolute detection:
// We now need to track BOTH:
// 1. Differential: Which coil is stronger (Comparator: A>B or A<B)
// 2. Absolute: Are coils energized above/below midpoint
//
// State bits [1:0]: Represents quadrature phase
// Bit 1: Coil A state (high/low from ADC or derived from comparator transitions)
// Bit 0: Coil B state (high/low from ADC)
//
// Alternative simpler encoding using just comparator + timing:
// Track comparator output and infer position from transition patterns

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

volatile unsigned long last_micros = 0;

// Track last comparator value for edge detection
volatile bool last_comparator_state = false;

// ==================== COMPARATOR SETUP ====================

void setup_analog_comparator() {
  // Disable digital input buffers on AIN0 and AIN1 for lower power
  DIDR1 |= (1 << AIN1D) | (1 << AIN0D);
  
  // Configure comparator:
  // - Compare AIN0 (Coil A on D6) with AIN1 (Coil B on D7)
  // - Interrupt on toggle (any change)
  // - ACO bit = 1 when Coil A voltage > Coil B voltage
  ACSR = (1 << ACIE) |   // Analog Comparator Interrupt Enable
         (0 << ACBG) |   // Don't use bandgap (compare pins directly)
         (0 << ACIS1) | (0 << ACIS0);  // Interrupt on toggle
  
  // Clear any pending comparator interrupt
  ACSR |= (1 << ACI);
  
  Serial.println(F("╔════════════════════════════════════════════════╗"));
  Serial.println(F("║        DIFFERENTIAL COMPARATOR MODE            ║"));
  Serial.println(F("╚════════════════════════════════════════════════╝"));
  Serial.println(F("Comparing Coil A vs Coil B voltages"));
  Serial.println(F("  D6 (AIN0) ← Coil A (other end to GND)"));
  Serial.println(F("  D7 (AIN1) ← Coil B (other end to GND)"));
  Serial.println(F("  A0        ← Coil A (for absolute level)"));
  Serial.println(F("  A1        ← Coil B (for absolute level)"));
  Serial.println();
  Serial.println(F("Comparator detects: HIGH when V(CoilA) > V(CoilB)"));
  Serial.println(F("                    LOW  when V(CoilA) < V(CoilB)"));
  Serial.println();
}

// ==================== STATE READING ====================

// Read comparator output
inline bool read_comparator() {
  return (ACSR & (1 << ACO)) != 0;  // true if Coil A > Coil B
}

// Read absolute coil states from ADC
inline uint8_t read_state_from_adc() {
  uint16_t coil_a = analogRead(COIL_A_ANALOG);
  uint16_t coil_b = analogRead(COIL_B_ANALOG);
  
  uint8_t a_state = (coil_a > THRESHOLD) ? 1 : 0;
  uint8_t b_state = (coil_b > THRESHOLD) ? 1 : 0;
  
  return (a_state << 1) | b_state;
}

// Hybrid state reading: Use comparator transitions + ADC samples
inline uint8_t read_state() {
  if (USE_ANALOG_READS) {
    // Method 1: Direct ADC reading of both coils
    // More traditional, gives absolute phase info
    return read_state_from_adc();
  } else {
    // Method 2: Derive state from comparator + one ADC
    // More sensitive to small signals, uses differential advantage
    bool comp = read_comparator();
    uint8_t b_state = (analogRead(COIL_B_ANALOG) > THRESHOLD) ? 1 : 0;
    
    // If comparator says A>B:
    //   - When B is LOW, A must be HIGH → state = 10 (2)
    //   - When B is HIGH, A must be HIGH → state = 11 (3)
    // If comparator says A<B:
    //   - When B is LOW, A must be LOW → state = 00 (0)
    //   - When B is HIGH, A must be LOW → state = 01 (1)
    
    uint8_t a_state;
    if (comp) {
      a_state = 1;  // A > B means A is high
    } else {
      a_state = 0;  // A < B means A is low (relatively)
    }
    
    return (a_state << 1) | b_state;
  }
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

// Analog Comparator ISR - triggered when Coil A vs Coil B relationship changes
ISR(ANALOG_COMP_vect) {
  comparator_triggers++;
  
  // The comparator changing means the coils have crossed over
  // This is a key moment in the quadrature sequence
  bool comp_state = read_comparator();
  
  // Only process if comparator state actually changed
  if (comp_state != last_comparator_state) {
    last_comparator_state = comp_state;
    process_state_change();
  }
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
    
    if (dt > 0 && dt < 100) {
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

// Read raw coil voltages
void get_coil_voltages(float *v_coil_a, float *v_coil_b) {
  uint16_t raw_a = analogRead(COIL_A_ANALOG);
  uint16_t raw_b = analogRead(COIL_B_ANALOG);
  
  *v_coil_a = (raw_a * 5.0f) / 1023.0f;
  *v_coil_b = (raw_b * 5.0f) / 1023.0f;
}

// ==================== DISPLAY FUNCTIONS ====================

void print_status() {
  long pos = get_position();
  int rev_pos = get_position_in_revolution();
  long revs = get_revolutions();
  float angle = get_angle_degrees();
  float vel = get_velocity();
  
  float v_a, v_b;
  get_coil_voltages(&v_a, &v_b);
  bool comp = read_comparator();
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║          SYSTEM STATUS             ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.print(F("Position:      ")); Serial.print(pos); Serial.println(F(" steps"));
  Serial.print(F("Revolutions:   ")); Serial.println(revs);
  Serial.print(F("Step in rev:   ")); Serial.print(rev_pos); 
  Serial.print(F("/")); Serial.println(STEPS_PER_REV);
  Serial.print(F("Angle:         ")); Serial.print(angle, 2); Serial.println(F("°"));
  Serial.print(F("Velocity:      ")); Serial.print(vel, 1); Serial.println(F(" steps/s"));
  Serial.println(F("────────────────────────────────────"));
  Serial.print(F("Coil A:        ")); Serial.print(v_a, 3); Serial.println(F(" V"));
  Serial.print(F("Coil B:        ")); Serial.print(v_b, 3); Serial.println(F(" V"));
  Serial.print(F("Difference:    ")); Serial.print(v_a - v_b, 3); Serial.println(F(" V"));
  Serial.print(F("Comparator:    ")); 
  Serial.println(comp ? F("HIGH (A>B)") : F("LOW (A<B)"));
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
  Serial.println(F("d - Show raw signal state"));
  Serial.println(F("w - Show waveforms (live)"));
  Serial.println(F("h - Show this help"));
  Serial.println();
}

void print_raw_state() {
  uint8_t state = read_state();
  bool comp = read_comparator();
  uint16_t raw_a = analogRead(COIL_A_ANALOG);
  uint16_t raw_b = analogRead(COIL_B_ANALOG);
  float v_a = (raw_a * 5.0f) / 1023.0f;
  float v_b = (raw_b * 5.0f) / 1023.0f;
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║         RAW SIGNAL STATE           ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.print(F("Coil A voltage:     ")); Serial.print(v_a, 3); Serial.print(F(" V (ADC: ")); 
  Serial.print(raw_a); Serial.println(F(")"));
  Serial.print(F("Coil B voltage:     ")); Serial.print(v_b, 3); Serial.print(F(" V (ADC: ")); 
  Serial.print(raw_b); Serial.println(F(")"));
  Serial.print(F("Voltage diff:       ")); Serial.print(v_a - v_b, 3); Serial.println(F(" V"));
  Serial.print(F("Comparator:         ")); Serial.println(comp ? F("1 (A>B)") : F("0 (A<B)"));
  Serial.print(F("Combined state:     ")); Serial.print(state, BIN); 
  Serial.print(F(" (decimal: ")); Serial.print(state); Serial.println(F(")"));
  Serial.println();
}

void show_live_waveforms() {
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║       LIVE WAVEFORM DISPLAY        ║"));
  Serial.println(F("║    (Rotate motor, press any key)   ║"));
  Serial.println(F("╚════════════════════════════════════╝\n"));
  
  while (!Serial.available()) {
    float v_a, v_b;
    get_coil_voltages(&v_a, &v_b);
    bool comp = read_comparator();
    
    // Create ASCII waveform display
    int bar_a = (int)(v_a * 10);  // Scale to 0-50
    int bar_b = (int)(v_b * 10);
    
    Serial.print(F("A:"));
    for (int i = 0; i < bar_a && i < 50; i++) Serial.print(F("█"));
    Serial.print(F(" ")); Serial.print(v_a, 2); Serial.println(F("V"));
    
    Serial.print(F("B:"));
    for (int i = 0; i < bar_b && i < 50; i++) Serial.print(F("█"));
    Serial.print(F(" ")); Serial.print(v_b, 2); Serial.println(F("V"));
    
    Serial.print(F("Comp: ")); Serial.println(comp ? F("[A > B]") : F("[A < B]"));
    Serial.println();
    
    delay(100);
  }
  while (Serial.available()) Serial.read();
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Serial.println(F("\n╔════════════════════════════════════════════════╗"));
  Serial.println(F("║  Stepper Decoder - Differential Comparator     ║"));
  Serial.println(F("║  v4.1 - Coil A vs Coil B Comparison           ║"));
  Serial.println(F("╚════════════════════════════════════════════════╝\n"));
  
  Serial.print(F("Steps per revolution: ")); Serial.println(STEPS_PER_REV);
  Serial.print(F("Validation samples:   ")); Serial.println(VALIDATION_SAMPLES);
  Serial.print(F("Max velocity:         ")); Serial.print(MAX_VELOCITY); Serial.println(F(" steps/s"));
  Serial.print(F("Use analog reads:     ")); Serial.println(USE_ANALOG_READS ? F("YES") : F("NO (Comparator only)"));
  Serial.println();
  
  // Setup analog comparator
  setup_analog_comparator();
  
  // Initialize state
  current_state = read_state();
  last_comparator_state = read_comparator();
  
  for (uint8_t i = 0; i < VALIDATION_SAMPLES; i++) {
    state_sample_buffer[i] = current_state;
  }
  
  last_micros = micros();
  
  Serial.println(F("✓ System ready. Type 'h' for help.\n"));
  delay(500);
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
      case 'w': case 'W':
        show_live_waveforms();
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
