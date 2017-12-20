// The fft code is from http://learn.adafruit.com/fft-fun-with-fourier-transforms/

#define ARM_MATH_CM4
#include <arm_math.h>

////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION
// These values can be changed to alter the behavior of the spectrum display.
////////////////////////////////////////////////////////////////////////////////

int SAMPLE_RATE_HZ = 2000;             // Sample rate of the audio in hertz.
const int FFT_SIZE = 256;              // Size of the FFT.  Realistically can only be at most 256
// without running out of memory for buffers and other state.
const int AUDIO_INPUT_PIN = 14;        // Input ADC pin for audio data.
const int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
const int ANALOG_READ_AVERAGING = 16;  // Number of samples to average with each ADC reading.
const int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 3.0's onboard LED).
// any other changes to the program.
const int MAX_CHARS = 65;              // Max size of the input command buffer


////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE (based on FFT library
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////

IntervalTimer samplingTimer;
float samples[FFT_SIZE * 2];
float magnitudes[FFT_SIZE];
int sampleCounter = 0;
char commandBuffer[MAX_CHARS];
int tonePosition = 0;
unsigned long toneStart = 0;

////////////////////////////////////////////////////////////////////////////////
// MOTOR STATES and PINS
////////////////////////////////////////////////////////////////////////////////

volatile int currentIncrement = 0;
volatile byte stateA = LOW;
volatile byte stateB = LOW;

//Motor pins
int encoderA = 11;
int encoderB = 12;
int input1 = 7;
int input2 = 8;
int enable1 = 6;
int targetFreq = 440;
int maxVal = 0;                     // Highest dB (in magnitude) 
int maxFreq = 0;                    //Corresponding highest frequency
int listening_thresh = 5000;        //only update current frequency if maxVal above this threshold
const int BUTTON_FIRST_NOTE = 22;   //pin for first note
const int BUTTON_SECOND_NOTE = 23;  //pin for second note

//checks when last bounce was at button push
volatile unsigned long lastBounce = 0;
//how long after button push to check for bounces
unsigned long debounce = 500;

////////////////////////////////////////////////////////////////////////////////
// MAIN SKETCH FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Set up serial port.
  Serial.begin(38400);

  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);

  // Turn on the power indicator LED.
  pinMode(POWER_LED_PIN, OUTPUT);
  digitalWrite(POWER_LED_PIN, HIGH);

  // Motor and H-bridge setup
  pinMode(encoderA,INPUT);//encoder A
  pinMode(encoderB,INPUT);//encoder B
  pinMode(input1, OUTPUT);//input 1
  pinMode(input2, OUTPUT);//input 2
  pinMode(enable1, OUTPUT);//enable 1
  
  // set initial setup to low for the following pins
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);
  digitalWrite(enable1, LOW);

  stateA = digitalRead(encoderA);
  stateB = digitalRead(encoderB);

  // Button Pins
  pinMode(BUTTON_FIRST_NOTE, INPUT_PULLUP);
  pinMode(BUTTON_SECOND_NOTE, INPUT_PULLUP);

  //Checks if buttons pressed
  attachInterrupt(digitalPinToInterrupt(BUTTON_FIRST_NOTE), switchToA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_SECOND_NOTE), switchToB, CHANGE);
  //checks if encoder moved
  attachInterrupt(digitalPinToInterrupt(encoderA), changeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), changeB, CHANGE);

  // Clear the input command buffer
  memset(commandBuffer, 0, sizeof(commandBuffer));

  // Begin sampling audio
  samplingBegin();
}

void loop() {
  // Calculate FFT if a full sample is available. (From fft library)
  if (samplingIsDone()) {
    // Run FFT on sample data.
    arm_cfft_radix4_instance_f32 fft_inst;
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&fft_inst, samples);
    // Calculate magnitude of complex numbers output by the FFT.
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);

    //get chosen note
    thresh();

    // Restart audio sampling.
    samplingBegin();
  }

}

void thresh() {

  int kprop = 2;
  
  getMaxFreq(magnitudes); //Get max frequency and max magnitude

  //make sure that sound is qualified to be analyzed
  if (maxVal > listening_thresh && maxFreq < 700 && maxFreq > 200) {

    // if current frequency is less than target frequency
    if (maxFreq < targetFreq - 13) {
      moveMotorClockwise(kprop*(targetFreq - maxFreq));
    }

    // if current frequency is more than target frequency
    else if (maxFreq > targetFreq + 13) {
      moveMotorCounterClockwise(kprop*(maxFreq - targetFreq));
    }
    else {
      analogWrite(enable1, 0);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// MOTOR MOVEMENT
////////////////////////////////////////////////////////////////////////////////

//Move motor clockwise
void moveMotorClockwise(int error) {
  digitalWrite(input1, HIGH);
  digitalWrite(input2, LOW);
  digitalWrite(enable1, HIGH);
  //make sure the encoder moves enough to account for the error
  int temp = currentIncrement;
  while (abs(currentIncrement - temp) < error) {
  }
  digitalWrite(enable1, LOW);  
  
}

// Move motor counter clockwise
void moveMotorCounterClockwise(int error) {
  digitalWrite(input1, LOW);
  digitalWrite(input2, HIGH);
  digitalWrite(enable1, HIGH);
  //make sure the encoder moves enough to account for the error
  int temp = currentIncrement;
  while (abs(currentIncrement - temp) < error) {
  }
  digitalWrite(enable1, LOW);  
}

////////////////////////////////////////////////////////////////////////////////
// SOUND INFORMATION EXTRACTION
////////////////////////////////////////////////////////////////////////////////

// Compute the average magnitude of a target frequency window vs. all other frequencies.
void getMaxFreq(float* magnitudes) {
  maxVal = 0;
  maxFreq = 0;
  int bin_size = 8;
  // Notice the first magnitude bin is skipped because it represents the
  // average power of the signal.
  for (int i = 1; i < FFT_SIZE / 2; ++i) {
    
    //get current frequency played
    if (magnitudes[i] > maxVal) {
      maxVal = magnitudes[i]; //change 4
      maxFreq = (i * bin_size);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// INTERRUPTS
////////////////////////////////////////////////////////////////////////////////

//change counter according to change in output A
void changeA() {
    stateA = !stateA;
    //if A and B are different, add one to the counter - if they are the same, subtract one from the counter
    if (stateA != stateB) {
      currentIncrement += 1;
    }
    else {
      currentIncrement -= 1;
    }
}
//change counter according to change in output B
void changeB() {
    stateB = !stateB;
    //if A and B are the same, add one to the counter - if they are different, subtract one from the counter
    if (stateA == stateB) {
      currentIncrement += 1;
    }
    else {
      currentIncrement -= 1;
    }
}


//swtiches target frequency to note A
void switchToA() {
  //checks to see if 500ms has passed since last bounce
  if (millis() - lastBounce > debounce) {
    targetFreq = 440;
    Serial.println(targetFreq);
  }
  lastBounce = millis(); //set new lastBounce
}

//swtiches target frequency to note B
void switchToB() {
  //checks to see if 500ms has passed since last bounce
  if (millis() - lastBounce > debounce) {
    targetFreq = 550;
    Serial.println(targetFreq);
  }
  lastBounce = millis(); //set new lastBounce
}


////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS (From fft library)
////////////////////////////////////////////////////////////////////////////////

void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter + 1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE * 2) {
    samplingTimer.end();
  }
}

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000 / SAMPLE_RATE_HZ);
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE * 2;
}


