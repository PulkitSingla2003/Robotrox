/***************************************************************
 * @author Akshar Sistu                                        *
 * @breif 'Mode changer' part of 'Floor Cleaner Robot' project *
 ***************************************************************/


// Defining all the pins of LCD display connected to respective Arduino Digital I/O pins:
#define ENABLE_PIN  12
#define RS_PIN      13
#define D4  11
#define D5  10
#define D6  9
#define D7  8

// Defining Input pins of L293D to switch the motors on/off(unidirectional):
#define SUCTION_MOTOR   3
#define SWEEPER_MOTOR   2

// Defining Enable pin of the motors to stop the motors when idle:
#define MOTOR_ENABLE    4

// Defining Slideswitch input pins:
#define SWEEPER_SUCTION_SWITCH      A2
#define CLEANING_SWITCH             A3

// Including LiquidCrystal Library to interface and control LCD:
#include <LiquidCrystal.h>

// Declaring lcd with pins:
LiquidCrystal lcd(RS_PIN, ENABLE_PIN, D4, D5, D6, D7);


// Creating Enum called state with different states of robot as values:
enum state {
  null_state,                       // During initialization
  notCleaning,                      // When Idle
  cleaning                          // When cleaning
};

// Creating Enum called mode with different modes the robot works in:
enum mode {
  null_mode,                        //  During initialization
  suction,                          //  When in suction mode
  sweeper                           //  When in sweeper mode
};

//  Creating global variables to hold current state and mode of the machine:
state currentState;
mode currentMode;

// Declaring all the functions that are implemented below void loop() :
void setParams(state s, mode m);
void setState(state s);
void setMode(mode m);
void printState(state s);
void printMode(mode m);


void setup() {
  lcd.begin(16, 2);                 // Initializing lcd to begin with character resolution 16x2
  Serial.begin(9600);               // Initializing serial interface to communicate with automation part of project
  
  // Assigning Slideswitch input pins as Input:
  pinMode(SWEEPER_SUCTION_SWITCH ,INPUT);
  pinMode(CLEANING_SWITCH ,INPUT);

  // Assigning motor output pins as output:
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(SWEEPER_MOTOR, OUTPUT);
  pinMode(SUCTION_MOTOR, OUTPUT);
  
  // Intro Screen on lcd:
  lcd.print("Floor Cleaner");       // Prints 'Floor Cleaner Robot' on the LCD Screen
  lcd.setCursor(0, 1);              // Changing cursor to the second line
  delay(500);                       // 1 second delay for initialization
  lcd.print("Loading");             // Prints 'Loading' on the LCD Screen
  delay(500);                       // 1 second delay for initialization
  lcd.clear();                      // Clearing Screen
}


void loop() {
  // To read the current state of switches and assign state and mode values to the enum variables:
  state s = digitalRead(CLEANING_SWITCH) ? cleaning : notCleaning;
  mode m = digitalRead(SWEEPER_SUCTION_SWITCH) ? sweeper : suction;

  // Calling the setParams() function with state and mode parameteres from above: 
  setParams(s, m);  
}

// setParams function to assign parameteres for machine to work in: 
void setParams(state s, mode m){
  if(currentState != s){                            //  If not already in the same state
    setState(s);                                    //  Calling setState() function to assign the state to the machine
    printState(s);                                  //  Calling printState() function to print the state of machine on the LCD display
    currentState = s;                               //  Updating the currentState Variable
  }
  if(currentState == cleaning){ //  If the machine is in cleaning state and not already in the same mode
    if(currentMode != m){                           //  To change the mode only if not in the same mode already
      setMode(m);                                   //  Calling setMode() function to assign the cleaning mode to the machine 
    }
    printMode(m);                                   //  Calling printMode() function to print the mode of machine on the LCD display
    currentMode = m;                                // Updating the currentMode variable
  }
}

// setState function to make changes to state of the machine:
void setState(state s){
  if(s == cleaning){                                //  Enables the motors to run on L293D when in cleaing mode
    digitalWrite(MOTOR_ENABLE, HIGH); 
  }else{                                            // Disables the motors to run on L293D when in Idle mode
    digitalWrite(MOTOR_ENABLE, LOW);
  }
}

//  setMode function to make changes to the working mode of the machine:
void setMode(mode m){
  if(m == sweeper){                                 //  Turns on Sweeper Motor and turns off Suction Motor for sweeper mode
    digitalWrite(SUCTION_MOTOR, LOW);
    digitalWrite(SWEEPER_MOTOR, HIGH);
  }else if(m == suction){                           //  Turns on Suction Motor and turns off Sweeper Motor for suction mode
    digitalWrite(SWEEPER_MOTOR, LOW);
    digitalWrite(SUCTION_MOTOR, HIGH);
  }
}

// printState function to print current state on to the LCD display whenever changes occur:
void printState(state s){
  lcd.clear();                                      //  Clears the display and makes it blank
  lcd.home();                                       //  Resets the cursor of the display to (0, 0)
  if(s == notCleaning){                             //  To print "IDLE" on the display when not cleaning
    lcd.print("IDLE");
  }else if(s == cleaning){                          //  To print "Cleaning Mode:" on the top half of the display when cleaning
    lcd.print("Cleaning Mode:");
  }
}

// printMode function to print current mode on to the LCD display whenever changes occur:
void printMode(mode m){
  lcd.setCursor(0,1);                               //  Sets the cursor to second line on the display
  if(m == sweeper){                                 //  To print "Sweeper" on the lower half of the display when in sweeper mode
    lcd.print("Sweeper");
  }else if(m == suction){                           //  To print "Suction" on the lower half of the display when in suction mode
    lcd.print("Suction");
  }
}

/*--------------------------------------------- END OF THE CODE --------------------------------------------------*/
