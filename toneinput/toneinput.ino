// Audio Tone Input
// Copyright 2013 Tony DiCola (tony@tonydicola.com)

// This code is part of the guide at http://learn.adafruit.com/fft-fun-with-fourier-transforms/

#define ARM_MATH_CM4
#include <arm_math.h>
#include <Adafruit_NeoPixel.h>


////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION
// These values can be changed to alter the behavior of the spectrum display.
////////////////////////////////////////////////////////////////////////////////

int SAMPLE_RATE_HZ = 2000;             // Sample rate of the audio in hertz.
const int TONE_LOWS[] = {              // Lower bound (in hz) of each tone in the input sequence.
  439
};
const int TONE_HIGHS[] = {             // Upper bound (in hz) of each tone in the input sequence.
  441
};
int TONE_ERROR_MARGIN_HZ = 1;         // Allowed fudge factor above and below the bounds for each tone input.
int TONE_WINDOW_MS = 4000;             // Maximum amount of milliseconds allowed to enter the full sequence.
int maxVal = 0;                       //OUR MAX VAL
int maxFreq = 0;
float TONE_THRESHOLD_DB = 25.0;        // Threshold (in decibels) each tone must be above other frequencies to count.
const int FFT_SIZE = 256;              // Size of the FFT.  Realistically can only be at most 256
// without running out of memory for buffers and other state.
const int AUDIO_INPUT_PIN = 14;        // Input ADC pin for audio data.
const int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
const int ANALOG_READ_AVERAGING = 16;  // Number of samples to average with each ADC reading.
const int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 3.0's onboard LED).
const int NEO_PIXEL_PIN = 3;           // Output pin for neo pixels.
const int NEO_PIXEL_COUNT = 4;         // Number of neo pixels.  You should be able to increase this without
// any other changes to the program.
const int MAX_CHARS = 65;              // Max size of the input command buffer


////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////

IntervalTimer samplingTimer;
float samples[FFT_SIZE * 2];
float magnitudes[FFT_SIZE];
int sampleCounter = 0;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NEO_PIXEL_COUNT, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);
char commandBuffer[MAX_CHARS];
int tonePosition = 0;
unsigned long toneStart = 0;

//motor states
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
//only update current frequency if above this threshold
int listening_thresh = 5000;
const int BUTTON_NOTE_A = 22;
const int BUTTON_NOTE_B = 23;

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

  // Motor stuff
  pinMode(encoderA,INPUT);//encoder A
  pinMode(encoderB,INPUT);//encoder B
  pinMode(input1, OUTPUT);//input 1
  pinMode(input2, OUTPUT);//input 2
  pinMode(enable1, OUTPUT);//enable 1
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);
  digitalWrite(enable1, LOW);

  stateA = digitalRead(encoderA);
  stateB = digitalRead(encoderB);

  // Button Pins
  pinMode(BUTTON_NOTE_A, INPUT_PULLUP);
  pinMode(BUTTON_NOTE_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BUTTON_NOTE_A), switchToA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_NOTE_B), switchToB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA), changeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), changeB, CHANGE);

  // Initialize neo pixel library and turn off the LEDs
  pixels.begin();
  pixels.show();

  // Clear the input command buffer
  memset(commandBuffer, 0, sizeof(commandBuffer));

  // Begin sampling audio
  samplingBegin();
}

void loop() {
  // Calculate FFT if a full sample is available.
  if (samplingIsDone()) {
    // Run FFT on sample data.
    arm_cfft_radix4_instance_f32 fft_inst;
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&fft_inst, samples);
    // Calculate magnitude of complex numbers output by the FFT.
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);

    // Detect tone sequence.
    //toneLoop();
    testThresh();

    // Restart audio sampling.
    samplingBegin();
  }

  // Parse any pending commands.
  parserLoop();
}

void testThresh() {

  int kprop = 2;
  
  getMaxFreq(magnitudes);
  if (maxVal > listening_thresh && maxFreq < 700 && maxFreq > 200) {
    
    Serial.print("Max Val: ");
    Serial.println(maxVal);
    Serial.print("Freq: ");
    Serial.println(maxFreq);
    
    if (maxFreq < targetFreq - 13) {
      moveMotorClockwise(kprop*(targetFreq - maxFreq));
      //Serial.println("CLOCKWISE");
    }
    else if (maxFreq > targetFreq + 13) {
      moveMotorCounterClockwise(kprop*(maxFreq - targetFreq));
      //Serial.println("COUNTERCLOCKWISE");
    }
    else {
      Serial.println("SAME");
      analogWrite(enable1, 0);
    }
    //analogWrite(enable1, 0);
  }
}

void moveMotorClockwise(int error) {
  digitalWrite(input1, HIGH);
  digitalWrite(input2, LOW);
  digitalWrite(enable1, HIGH);
  int temp = currentIncrement;
  while (abs(currentIncrement - temp) < error) {
  }
  digitalWrite(enable1, LOW);  
  
}
void moveMotorCounterClockwise(int error) {
  digitalWrite(input1, LOW);
  digitalWrite(input2, HIGH);
  digitalWrite(enable1, HIGH);
  int temp = currentIncrement;
  while (abs(currentIncrement - temp) < error) {
  }
  digitalWrite(enable1, LOW);  
}

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
// UTILITY FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

// Compute the average magnitude of a target frequency window vs. all other frequencies.
void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
  *windowMean = 0;
  *otherMean = 0;
  // Notice the first magnitude bin is skipped because it represents the
  // average power of the signal.
  for (int i = 1; i < FFT_SIZE / 2; ++i) {
    if (i >= lowBin && i <= highBin) {
      *windowMean += magnitudes[i];
    }
    else {
      *otherMean += magnitudes[i];
    }
  }
  *windowMean /= (highBin - lowBin) + 1;
  *otherMean /= (FFT_SIZE / 2 - (highBin - lowBin));
}

// Convert a frequency to the appropriate FFT bin it will fall within.
int frequencyToBin(float frequency) {
  float binFrequency = float(SAMPLE_RATE_HZ) / float(FFT_SIZE);
  return int(frequency / binFrequency);
}

// Convert intensity to decibels
float intensityDb(float intensity) {
  return 20.0 * log10(intensity);
}


////////////////////////////////////////////////////////////////////////////////
// SPECTRUM DISPLAY FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

void toneLoop() {
  // Calculate the low and high frequency bins for the currently expected tone.
  int lowBin = frequencyToBin(TONE_LOWS[tonePosition] - TONE_ERROR_MARGIN_HZ);
  int highBin = frequencyToBin(TONE_HIGHS[tonePosition] + TONE_ERROR_MARGIN_HZ);
  // Get the average intensity of frequencies inside and outside the tone window.
  float window, other;
  windowMean(magnitudes, lowBin, highBin, &window, &other);
  window = intensityDb(window);
  other = intensityDb(other);
  // Check if tone intensity is above the threshold to detect a step in the sequence.
  if ((window - other) >= TONE_THRESHOLD_DB) {
    // Start timing the window if this is the first in the sequence.
    unsigned long time = millis();
    if (tonePosition == 0) {
      toneStart = time;
    }
    // Increment key position if still within the window of key input time.
    if (toneStart + TONE_WINDOW_MS - 3 > time) {
      tonePosition += 1;
    }
    else {
      // Outside the window of key input time, reset back to the beginning key.
      tonePosition = 0;
    }
  }

  // Check if the entire sequence was passed through.
  if (tonePosition >= sizeof(TONE_LOWS) / sizeof(int)) {
    toneDetected();
    tonePosition = 0;
  }
}

void toneDetected() {
  // Flash the LEDs four times.
  int pause = 500;

  //for (int i = 0; i < 4; ++i) {

  digitalWrite(POWER_LED_PIN, LOW);
  delay(pause);
  digitalWrite(POWER_LED_PIN, HIGH);
  //}

}

////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
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


////////////////////////////////////////////////////////////////////////////////
// COMMAND PARSING FUNCTIONS
// These functions allow parsing simple commands input on the serial port.
// Commands allow reading and writing variables that control the device.
//
// All commands must end with a semicolon character.
//
// Example commands are:
// GET SAMPLE_RATE_HZ;
// - Get the sample rate of the device.
// SET SAMPLE_RATE_HZ 400;
// - Set the sample rate of the device to 400 hertz.
//
////////////////////////////////////////////////////////////////////////////////

void parserLoop() {
  // Process any incoming characters from the serial port
  while (Serial.available() > 0) {
    char c = Serial.read();
    // Add any characters that aren't the end of a command (semicolon) to the input buffer.
    if (c != ';') {
      c = toupper(c);
      strncat(commandBuffer, &c, 1);
    }
    else
    {
      // Parse the command because an end of command token was encountered.
      parseCommand(commandBuffer);
      // Clear the input buffer
      memset(commandBuffer, 0, sizeof(commandBuffer));
    }
  }
}

// Macro used in parseCommand function to simplify parsing get and set commands for a variable
#define GET_AND_SET(variableName) \
  else if (strcmp(command, "GET " #variableName) == 0) { \
    Serial.println(variableName); \
  } \
  else if (strstr(command, "SET " #variableName " ") != NULL) { \
    variableName = (typeof(variableName)) atof(command+(sizeof("SET " #variableName " ")-1)); \
  }

void parseCommand(char* command) {
  if (strcmp(command, "GET MAGNITUDES") == 0) {
    for (int i = 0; i < FFT_SIZE; ++i) {
      Serial.println(magnitudes[i]);
    }
  }
  else if (strcmp(command, "GET SAMPLES") == 0) {
    for (int i = 0; i < FFT_SIZE * 2; i += 2) {
      Serial.println(samples[i]);
    }
  }
  else if (strcmp(command, "GET FFT_SIZE") == 0) {
    Serial.println(FFT_SIZE);
  }
  GET_AND_SET(SAMPLE_RATE_HZ)
  GET_AND_SET(TONE_ERROR_MARGIN_HZ)
  GET_AND_SET(TONE_WINDOW_MS)
  GET_AND_SET(TONE_THRESHOLD_DB)
}
