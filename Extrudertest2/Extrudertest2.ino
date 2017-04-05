

/**************** MAX31855K_Thermocouple_Digitizer_Example.ino *****************
 *                                                                             *
   MAX31855K Thermocouple Breakout Example Code
   brent@sparkfun.com
   March 26th 2015
   https://github.com/sparkfun/MAX31855K_Thermocouple_Digitizer
 *                                                                             *
   Use the "serial monitor" window to read a temperature sensor.
 *                                                                             *
   Circuit:
   MAX31855K breakout attached to the following pins
    SS:   pin 10
    MOSI: pin 11 (NC)
    MISO: pin 12
    SCK:  pin 13
 ******************************************************************************/
#include <SparkFunMAX31855k.h> // Using the max31855k driver
#include <SPI.h>  // Included here too due Arduino IDE; Used in above header

// Define SPI Arduino pin numbers (Arduino Pro Mini)
const uint8_t CHIP_SELECT_PIN = 10; // Using standard CS line (SS)
// SCK & MISO are defined by Arduiino
const uint8_t VCC = 0;//14; // Powering board straight from Arduino Pro Mini
const uint8_t GND = 0;//15;

float temperature;
float evenEvenOlderTemperature, evenOlderTemperature, oldestTemperature, olderTemperature, oldTemperature;

// Instantiate an instance of the SparkFunMAX31855k class
SparkFunMAX31855k probe(CHIP_SELECT_PIN, VCC, GND);



/******************************************************************************/
// ***  PID declare
/******************************************************************************/
#include <PID_v1.h>
const int HEATER_PIN = 5;  //Pin to control the heater element

double Setpoint, Input, Output;  //Setpoint = desired temperature, Input = Thermocouple reading, Output = Power 256-0
double KP = 18.0 ;
double KI = 0.82 ;
double KD = 99.0 ;


PID myPID(&Input, &Output, &Setpoint, KP, KI, KD, REVERSE);  // set the pid parameters

// PID settings meassured in the Vertex
//DEFAULT_Kp 35.0
//DEFAULT_Ki 5.0
//DEFAULT_Kd 60.0


/******************************************************************************/
// ***  AccelStepper declare
/******************************************************************************/
#include <AccelStepper.h>
const int STEP_PIN = 3;
const int DIR_PIN = 2;
const int DISSABLE_PIN = 4;
long max_speed;

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN, 0, 0, true); // Use Driver pcb, 3 step, 2 dir, disable enable

/******************************************************************************/
// ***  HX711 Load Cell declare
/******************************************************************************/
#include <Q2HX711.h>
const int HX711_DATA_PIN = 7;
const int HX711_CLOCK_PIN = 6;

Q2HX711 hx711(HX711_DATA_PIN, HX711_CLOCK_PIN);  //Inititalte

long firstLoadCellReading, LoadCellReading;

/******************************************************************************/
// ***  TimerOne declare
/******************************************************************************/
#include <TimerOne.h>

/******************************************************************************/
// ***  Program flow declare
/******************************************************************************/
const long interval = 20;           // interval for writing data to Mac (milliseconds), 200 for extrudertest and 20 for retract
// The HX711 normally reads 10SPS but can do 80SPS=13ms
unsigned long previousMillis = 0;        // will store last time data was sent
unsigned long currentMillis = millis();

const long intervalTemp = 100; // interval to read temperature. There is a min for the MAX31855K, 100ms
unsigned long previousTempMillis = 0;

unsigned long microsLoop;
unsigned long counterLoop;


/******************************************************************************/
// ***  Serial Event declare
/******************************************************************************/
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

/******************************************************************************/
// ************* Declare for Retract test ********************
/******************************************************************************/
int PreExtrusionSpeed = 59;    //steps/sec  59=1mm/s
int PreExtrusionSteps = 590;  //microsteps 590=10sec=10mm
int RetractSpeed = 590;        // 590=10mm/s
int RetractSteps = -59;         // Retract 1mm
unsigned long RetractDelay = 2000;        // milliseconds
unsigned long StartRetractDelay;
int UnRetractSpeed = 590;
int UnRetractSteps = 59;
int PostExtrusionSpeed = 59;
int PostExtrusionSteps = 590;

boolean DoRetract = false;
byte RetractSeqenceStep = 0;


/******************************************************************************/
//  ***********  SetUp **************************
/******************************************************************************/
void setup() {
  // put your setup code here, to run once:

  Serial.begin(57600);


  // ************* Setup for PID ********************

  // ************* Setup for MAX ********************
  temperature = probe.readTempC(); //read to power up
  temperature = 20;


  // ************* Setup for PID ********************
  pinMode(HEATER_PIN, OUTPUT); // Pin connected to the optocoupler. The signal is inverted.
  // Used as AnalogWrite and the mod does not need to be set

  Input = 20; //initial temperature
  Output = 255; //initial Power- 255 is off, 0 max
  Setpoint = 20;  // The temperature to controll towards

  //turn the PID on
  myPID.SetOutputLimits(0, 255); // Range 0-255. 200 is the max value and 255 the min
  myPID.SetControllerDirection(REVERSE);  //The optocoupler will reverse the signal
  myPID.SetMode(MANUAL);  //MANUAL or AUTOMATIC
  myPID.SetSampleTime(1000);  //Time interval to evaluate the PID loop (25 normal)


  // ************* Setup for AccelStepper ********************
  pinMode(DISSABLE_PIN, OUTPUT);
  digitalWrite(DISSABLE_PIN, HIGH);  //dissable stepper
  stepper.setAcceleration(80000);  // set the acceleration to a fixed value
  max_speed = 59;  //Set an initial speed, 59=1mm/s filament
  stepper.setMaxSpeed(max_speed);

  //476 steps/mm for BondTech used in Vertex at 16 micro steps
  // 1,75mm filament and 0,4mm nozzle has a ratio of 19,14.
  // This results in 24,86 steps/mm extruded wire
  // Extruding a full diameter at 100mm/s print speed then requires 2486,8 steps/s
  // If 4 microsteps are used the value becomes 621,7 steps/s
  // the Uno can achieve 4000steps/s and the Due 4 times more.


  // ************* Setup for Loadcell HX711 ********************
  firstLoadCellReading = hx711.read();
  LoadCellReading = hx711.read();



  // ************* Setup flow controll ********************

  // ************* Setup for TimerOne ********************
  Timer1.initialize(200);  //microseconds interval for the timer that calls the stepper, 200 normal
  Timer1.attachInterrupt(step_me); // Attach the step_me to the interrupt so that it will run each interrupt time

  // ************* Setup for Serial Event ********************

  inputString.reserve(20);  // reserve 20 bytes for the inputString:



}



void loop() {    // run repeatedly:
  // Temporary to check the loop time
  //  counterLoop = counterLoop + 1;
  //  if (counterLoop == 10000)
  //  {
  //    counterLoop=0;
  //    Serial.println("LoopTime: " + String(micros()-microsLoop));
  //  }
  //  microsLoop=micros();


  currentMillis = millis();
  if (DoRetract)
  {
    if (stepper.distanceToGo() == 0)
    {
      switch (RetractSeqenceStep)
      {
        case 1:
          {
            max_speed = PreExtrusionSpeed;
            stepper.setMaxSpeed(PreExtrusionSpeed);
            stepper.setCurrentPosition(0);
            Serial.println("PreExtrusionSpeed: " + String(PreExtrusionSpeed));

            digitalWrite(DISSABLE_PIN, LOW); //Enable the stepper
            stepper.moveTo(PreExtrusionSteps);
            Serial.println("Move stepper to: " + String(PreExtrusionSteps));

            RetractSeqenceStep = RetractSeqenceStep + 1;
          }
          break;
        case 2:
          {
            max_speed = RetractSpeed;
            stepper.setMaxSpeed(RetractSpeed);
            stepper.setCurrentPosition(0);
            Serial.println("RetractionSpeed: " + String(RetractSpeed));

            digitalWrite(DISSABLE_PIN, LOW); //Enable the stepper
            stepper.moveTo(RetractSteps);
            Serial.println("Move stepper to: " + String(RetractSteps));

            RetractSeqenceStep = RetractSeqenceStep + 1;
          }
          break;

        case 3:
          {

            StartRetractDelay = currentMillis; //Store the time when the delay simulating a move starts
            Serial.println("Start Retract Delay");
            stepper.setMaxSpeed(0); //Set the speed to zero so that it is possible to see it in the datalog on the Mac
            max_speed = 0;
            RetractSeqenceStep = RetractSeqenceStep + 1;
          }
          break;

        case 4:
          {

            if (currentMillis - StartRetractDelay > RetractDelay)  //The delay has elapsed, move to next step
            { Serial.println("End Retract Delay");
              RetractSeqenceStep = RetractSeqenceStep + 1;
            }
          }
          break;

        case 5:
          {
            max_speed = UnRetractSpeed;
            stepper.setMaxSpeed(UnRetractSpeed);
            stepper.setCurrentPosition(0);
            Serial.println("UnRetractSpeed: " + String(UnRetractSpeed));

            digitalWrite(DISSABLE_PIN, LOW); //Enable the stepper
            stepper.moveTo(UnRetractSteps);
            Serial.println("Move stepper to: " + String(UnRetractSteps));

            RetractSeqenceStep = RetractSeqenceStep + 1;
          }
          break;


        case 6:
          {
            max_speed = PostExtrusionSpeed;
            stepper.setMaxSpeed(PostExtrusionSpeed);
            stepper.setCurrentPosition(0);
            Serial.println("PostExtrusionSpeed: " + String(PostExtrusionSpeed));

            digitalWrite(DISSABLE_PIN, LOW); //Enable the stepper
            stepper.moveTo(PostExtrusionSteps);
            Serial.println("Move stepper to: " + String(PostExtrusionSteps));

            RetractSeqenceStep = RetractSeqenceStep + 1;
          }
          break;

        case 7:
          {
            DoRetract = false;
            RetractSeqenceStep = 0;
            Serial.println("Retraction finished");
          }
          break;


        default:
          {
            Serial.print("Switch statement in retract not handled correctly");
          }
          break;
      }  // switch
    }  // if stepper
  } //if doRetract

  //temperature = probe.readCJT();  //Coldjunction temperature - not used really
  //  if (!isnan(temperature)) {                 //Only write if the number is NOT a NAN
  //    Serial.print("CJT is [C]: ");
  //   Serial.print(temperature);
  //  }


  // Read the temperature at intervalTemp to avoid reading temperature to often
  if (currentMillis - previousTempMillis >= intervalTemp) {
    previousTempMillis = currentMillis;
    temperature = probe.readTempC();  // Read the temperature in Celsius
    //Serial.print("  Temp[C]=");
    //Serial.println(temperature);
    if (!isnan(temperature))          //Only transfer the reading to the PID if  NOT a NAN
      if (temperature < 600)
        if (temperature > 5)  {
          // transfer the temperature to the PID Input for clearity
          Input = (evenEvenOlderTemperature + evenOlderTemperature + oldestTemperature + olderTemperature + oldTemperature + temperature) / 6;
          evenEvenOlderTemperature = evenOlderTemperature;
          evenOlderTemperature = oldestTemperature;
          oldestTemperature = olderTemperature;
          olderTemperature = oldTemperature;
          oldTemperature = temperature;

          //myPID.Compute();
          //analogWrite(HEATER_PIN, Output);  //PWM to Heater pin. This has to be called also when the mod is manual
          // Serial.print("  Temp[C]=");
          //Serial.println(olderTemperature);
        }
  } // End if current Millis



  //if (myPID.Compute())  // Only set a new value if there is a new calculated
  myPID.Compute();

  analogWrite(HEATER_PIN, Output);  //PWM to Heater pin. This has to be called also when the mod is manual

  if (stepper.distanceToGo() == 0)   //If the stepper has moved to final destination - dissable stepper driver
  {
    digitalWrite(DISSABLE_PIN, HIGH); //Disable the stepper
  }


  // Send information to serial Monitor after interval time
  //currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read the force cell at the time of transmision to avoid disturbing the stepper.
    // this is slow due to the shiftIn, I think. The interrupt is dissabled during this call
    // since the reading from the hx711 is disturbed otherwise.
    if (hx711.readyToSend()) {
      noInterrupts();
      LoadCellReading = hx711.read();
      interrupts();
    }
    // Uncomment the line below to include ASCII in the sending
    // Serial.println("Millis: " + String(currentMillis) + " Temp: " + String(temperature)  + " Output: " + String(Output) + " Input: " + String(Input) + " Force: " + String(LoadCellReading - firstLoadCellReading) );


    //  Send status in a binary form
    Serial.write("xy");       // Two bytes to identify the start of the binary block
    sendBinary((currentMillis));
    sendBinary(long(Input * 100));  //The temperature used in PID that is read from thermocouple
    sendBinary(int(Output));        // The PWM power
    sendBinary(long(LoadCellReading - firstLoadCellReading));
    sendBinary(long(stepper.distanceToGo()));   //The remaining distance to go in steps
    sendBinary(long(max_speed));    //The speed of the stepper after accelerationSet

    sendBinary(int(Setpoint)); // The desired Temperature
    sendBinary(int(KP * 100));
    sendBinary(int(KI * 100));
    sendBinary(int(KD * 100));
    Serial.write(RetractSeqenceStep);  //The position of the retract sequence


    //   end sending status in binary form

  }  //end of currentMillis

  //------------------------------------------------------------
  // Handle incomming commands
  //------------------------------------------------------------
  // Handle the command when terminat character is recieved
  if (stringComplete) {
    //Serial.println(inputString); //Remove, only for testing
    // Extract the command and the value
    String Command;
    String CommandValueString;
    long CommandValue;
    int Position, Position2;
    
    String Slask; //Store the original inputstring
    Slask=inputString;
    
    Command = inputString.substring(0, 2);
    Position = inputString.indexOf('#'); //The position of the terminating character -new
    CommandValueString = inputString.substring(2, Position);
    CommandValue = CommandValueString.toInt();

    inputString = inputString.substring(Position + 1); //Removes the part that has been handled -new
    Position2 = inputString.indexOf('#'); //The position of the terminating character -new
    Serial.println("Position2 : "+ String(Position2)); //Print the remaining string - new
    if (Position2 == -1)   //new
      //No more # in string -new
    {
      stringComplete = false; //-new
      //Serial.println("Next String NOT Complete"+ inputString); //Print the remaining string - new
    }
    else     //-new
      // More commands to handle in the next loop
    { stringComplete = true;  //-new
      Serial.println("Next String Complete" + inputString); //Print the remaining string - new
    }


    Serial.print(Command);
    Serial.println(CommandValue);
    if (CommandValue > 10000) 
    {
    Serial.println("**** Warning **** Large CommandValue" + CommandValue);
   Serial.println(Slask);
    }

     if (CommandValue == 0) 
    {
    Serial.println("**** Warning **** Zero CommandValue" + CommandValue);
    Serial.println(Slask);
    }

    if (Command == "PO") {        //Stepper position in microsteps
      digitalWrite(DISSABLE_PIN, LOW); //Enable the stepper
      stepper.moveTo(CommandValue);
      Serial.println("Move stepper to: " + String(CommandValue));
    }
    else if (Command == "TE") { //Temperature
      Setpoint = CommandValue;
      Serial.println("SetPoint set to: " + String(Setpoint));
    }
    else if (Command == "ST") // Stop the motion
    {
      stepper.stop();
      Serial.println("Stepper stopped");
    }

    else if (Command == "RE") // Retract sequence
    {
      DoRetract = true;
      RetractSeqenceStep = 1;
      Serial.println("Retract sequence started");
    }

    else if (Command == "CP") // Set current position
    {
      stepper.setCurrentPosition(CommandValue);
      Serial.println("Current Pos: " + String(CommandValue));
    }
    else if (Command == "SP") {//Set max speed
      max_speed = CommandValue;
      stepper.setMaxSpeed(max_speed);
      Serial.println("Speed: " + String(max_speed));
    }
    else if (Command == "SU") // Turn Supply on or off
      if (CommandValue == 0) {
        // Turn the supply off
        Serial.println("Supply turned off");
      }
      else {
        // Turn the supply on
        Serial.println("Supply turned on");
      }

    else if (Command == "HE") //Turn Heating on or off
      if (CommandValue == 0) {
        //Turn the heating off
        myPID.SetMode(MANUAL);
        Output = 255;  //Turn heating off
        Serial.println("Heating off, Manual mode with Output: " + String(Output));
      }
      else {
        // Turn the heating on where the PID loop controlls the temperature
        myPID.SetMode(AUTOMATIC);
        Serial.println("Automatic mode");
      }

    else if (Command == "HP") { //Turn to manual and set the heating power to the sent value
      myPID.SetMode(MANUAL);
      Output = CommandValue;
      Serial.println("Manual mode with Output: " + String(Output));
    }

    else if (Command == "KP") {//Set Kp value, sent 100 times larger
      // SetTunings(Kp, Ki, Kd) // these values muste me defined as variables. Exist also for auto tune
      //Divide the value with 100
      KP = CommandValue / 100.0;
      myPID.SetTunings(KP, KI, KD);
      Serial.println("Kp: " + String(KP));
    }
    else if (Command == "KI") {
      KI = CommandValue / 100.0;
      myPID.SetTunings(KP, KI, KD);
      Serial.println("Ki: " + String(KI));
    }
    else if (Command == "KD") {
      KD = CommandValue / 100.0;
      myPID.SetTunings(KP, KI, KD);
      Serial.println("Kd: " + String(KD));
    }

    else if (Command == "R1") {
      PreExtrusionSpeed = CommandValue;
      Serial.println("PreExtrusionSpeed: " + String(PreExtrusionSpeed));
    }

    else if (Command == "R2") {
      PreExtrusionSteps = CommandValue;
      Serial.println("PreExtrusionSteps: " + String(PreExtrusionSteps));
    }

    else if (Command == "R3") {
      RetractSpeed = CommandValue;
      Serial.println("RetractSpeed: " + String(RetractSpeed));
    }

    else if (Command == "R3") {
      RetractSpeed = CommandValue;
      Serial.println("RetractSpeed: " + String(RetractSpeed));
    }

    else if (Command == "R4") {
      RetractSteps = CommandValue;
      Serial.println("RetractSteps: " + String(RetractSteps));
    }

    else if (Command == "R5") {
      RetractDelay = CommandValue;
      Serial.println("RetractDelay: " + String(RetractDelay));
    }

    else if (Command == "R6") {
      UnRetractSpeed = CommandValue;
      Serial.println("UnRetractSpeed: " + String(UnRetractSpeed));
    }

    else if (Command == "R7") {
      UnRetractSteps = CommandValue;
      Serial.println("UnRetractSteps: " + String(UnRetractSteps));
    }

    else if (Command == "R8") {
      PostExtrusionSpeed = CommandValue;
      Serial.println("PostExtrusionSpeed: " + String(PostExtrusionSpeed));
    }

    else if (Command == "R9") {
      PostExtrusionSteps = CommandValue;
      Serial.println("PostExtrusionSteps: " + String(PostExtrusionSteps));
    }


    else {
      Serial.println("Unknown Command: " + Command);
    }
    // End if Command



    // clear the string to recieva a new command
    Command = "";
    CommandValueString = "";
    CommandValue = 0;
    //  inputString = "";
    //stringComplete = false;
  }  // end string complete


} // end of main loop ------------------------------------------------------------------



void step_me() {    //interrupt procedure to move the stepper
  stepper.run();  // Uses acceleration and dedeleration
  //stepper.runSpeed(); //No acceleration
}

/*
  SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '#') {
      stringComplete = true;
    }
  }
}

// function to send the given integer value to the serial port
void sendBinary(int value)
{
  // send the two bytes that comprise a two byte (16 bit) integer
  Serial.write(lowByte(value));  // send the low byte
  Serial.write(highByte(value)); // send the high byte
}

//The following function sends the value of a long (4 - byte) integer by first sending the
//two low (rightmost) bytes, followed by the high (leftmost) bytes:

// function to send the given long integer value to the serial port
void sendBinary(long value)
{
  // first send the low 16 bit integer value
  int temp = value & 0xFFFF;  // get the value of the lower 16 bits
  sendBinary(temp);
  // then send the higher 16 bit integer value:
  temp = value >> 16;  // get the value of the higher 16 bits
  sendBinary(temp);
}


// function to send the given unsigned long integer value to the serial port
void sendBinary(unsigned long value)
{
  // first send the low 16 bit integer value
  int temp = value & 0xFFFF;  // get the value of the lower 16 bits
  sendBinary(temp);
  // then send the higher 16 bit integer value:
  temp = value >> 16;  // get the value of the higher 16 bits
  sendBinary(temp);
}


