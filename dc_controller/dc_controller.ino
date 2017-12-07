//declare variables
volatile int currentIncrement = 0;
volatile byte stateA = LOW;
volatile byte stateB = LOW;
const float pi = 3.14159265;

int input1 = 7;
int input2 = 8;
int enable1 = 6;
//enable1 pin 6 from 11
//input1 from 4 to 7
//input2 8
//

void setup() {
  //begin communication with the Serial Monitor
  Serial.begin(9600);
  //set up pins
  //pinMode(2,INPUT);//encoder A
  //pinMode(3,INPUT);//encoder B
  pinMode(input1, OUTPUT);//input 1
  pinMode(input2, OUTPUT);//input 2
  pinMode(enable1, OUTPUT);//enable 1
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);
  digitalWrite(enable1, LOW);


 // stateA = digitalRead(2);
 // stateB = digitalRead(3);
  
  //attach interrupt routines to watch for changes in encoder outputs
  //attachInterrupt(digitalPinToInterrupt(2), changeA, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(3), changeB, CHANGE);
  currentIncrement= (analogRead(2)/1023.0)*1633;
}
float count = analogRead(16);
void loop() {
  // put your main code here, to run repeatedly:
  //print out the value of the counter

  //Get the desired count (angle) from the potentiometer
  int potValue = analogRead(A2);
  count = (potValue/1023.0)*1633;

  //Get the difference between desired angle and current angle
  float error = count - currentIncrement;


  Serial.print(potValue);
  Serial.print('\t');
  Serial.println(currentIncrement);
  error = (error/1633)*2*pi;

  //set kprop and calculate control signal
  int Kprop = 5;
  float control = Kprop*error;
  //change direction of motor based on control signal
  if (control > 0.0){
    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);
  } else{
    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH);
  }
  
  //constrain control signal
  control = control*255/5.0;
  int val = constrain(abs(round(control)), 0 ,255);
  //set the enable1 pin to specified value
  analogWrite(enable1, val);
  
}
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
