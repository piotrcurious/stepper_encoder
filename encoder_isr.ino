// Define the analog input pins for the stepper motor
#define STEPPER_PIN_A A0
#define STEPPER_PIN_B A1

// Define the number of steps per revolution for the stepper motor
#define STEPS_PER_REV 200

// Define the timer interrupt frequency in Hz
#define TIMER_FREQ 1000

// Define a global variable to store the current position of the stepper motor in steps
volatile int stepper_position = 0;

// Define a global variable to store the previous state of the stepper motor pins
volatile byte stepper_prev_state = 0;

// Define a lookup table for the state transitions of the stepper motor pins
// The table has 4 rows and 4 columns, corresponding to the 4 possible states of each pin
// The table values are -1, 0, or 1, corresponding to the direction of rotation (CCW, stop, CW)
const int8_t stepper_state_table[4][4] = {
  {0, -1, 1, 0},
  {1, 0, 0, -1},
  {-1, 0, 0, 1},
  {0, 1, -1, 0}
};

// This function reads the analog values of the stepper motor pins and returns a state value
// The state value is a byte with the two least significant bits representing the pin states
// For example, if pin A is high and pin B is low, the state value is 0b10 or 2
byte read_stepper_state() {
  // Read the analog values and map them to 0 or 1
  byte pin_a = analogRead(STEPPER_PIN_A) > 512 ? 1 : 0;
  byte pin_b = analogRead(STEPPER_PIN_B) > 512 ? 1 : 0;
  
  // Return the state value by shifting and combining the pin values
  return (pin_a << 1) | pin_b;
}

// This function updates the position of the stepper motor based on the state transitions
void update_stepper_position() {
  // Read the current state of the stepper motor pins
  byte stepper_curr_state = read_stepper_state();
  
  // If the state has changed since the last update
  if (stepper_curr_state != stepper_prev_state) {
    // Lookup the direction of rotation from the state table
    int8_t direction = stepper_state_table[stepper_prev_state][stepper_curr_state];
    
    // Update the position by adding or subtracting one step depending on the direction
    stepper_position += direction;
    
    // Constrain the position to be within [0, STEPS_PER_REV - 1]
    stepper_position = (stepper_position + STEPS_PER_REV) % STEPS_PER_REV;
    
    // Save the current state as the previous state for the next update
    stepper_prev_state = stepper_curr_state;
    
    // Print the position for debugging purposes
    Serial.println(stepper_position);
    
    // TODO: Add any other logic that depends on the position here
    // For example, you can control another device based on the position value
    // ...
    
  }
}

// This function sets up the timer interrupt to call update_stepper_position at a fixed frequency
void setup_timer_interrupt() {
  // Calculate the timer compare value based on the desired frequency and the clock prescaler
  // For example, if using a 16 MHz clock and a prescaler of 64, the compare value for a frequency of 1000 Hz is (16000000 / (64 * TIMER_FREQ)) - 1 = 
249

  // Set the timer compare value to the calculated value
  OCR1A = 249;
  
  // Clear the timer counter value
  TCNT1 = 0;
  
  // Enable the output compare match interrupt for timer 1A
  TIMSK1 |= (1 << OCIE1A);
  
  // Set the timer mode to CTC (clear timer on compare match)
  TCCR1A = 0;
  TCCR1B = (1 << WGM12);
  
  // Set the clock prescaler to 64
  TCCR1B |= (1 << CS11) | (1 << CS10);
}

// This is the interrupt service routine that is called when the timer compare match occurs
ISR(TIMER1_COMPA_vect) {
  // Call the update_stepper_position function
  update_stepper_position();
}

// This function sets up the serial communication and the timer interrupt
void setup() {
  // Initialize the serial communication at 9600 baud rate
  Serial.begin(9600);
  
  // Initialize the stepper motor pins as inputs with pull-up resistors
  pinMode(STEPPER_PIN_A, INPUT_PULLUP);
  pinMode(STEPPER_PIN_B, INPUT_PULLUP);
  
  // Read and save the initial state of the stepper motor pins
  stepper_prev_state = read_stepper_state();
  
  // Set up the timer interrupt
  setup_timer_interrupt();
}

// This function runs in a loop and does nothing
void loop() {
  // Do nothing
}
