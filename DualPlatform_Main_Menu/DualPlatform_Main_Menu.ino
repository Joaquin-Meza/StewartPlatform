//__________________________________________________________________________
//__________________ General Setup _________________________________________
//__________________________________________________________________________
#define MIN_CONTROL_VALUE 25  // Minimum control threshold to prevent small noise activation
#define NUM_ACTUATORS 6  // Number of actuators per Stewart platform
#define NUM_PLATFORMS 2  // Number of Stewart platforms
#define EMERGENCY_STOP_PIN 2  // Pin for emergency stop button

// --------VARIABLES --------------------
volatile bool emergencyStop = false;  // Flag to indicate emergency stop

// Define motor control pins for two platforms
const int DIR_A[NUM_PLATFORMS][NUM_ACTUATORS] = {
    {30, 33, 35, 36, 38, 40}, // Platform 1
    {42, 44, 46, 48, 50, 52}  // Platform 2
};
const int DIR_B[NUM_PLATFORMS][NUM_ACTUATORS] = {
    {31, 32, 34, 37, 39, 41},
    {43, 45, 47, 49, 51, 53}
};
const int POT[NUM_PLATFORMS][NUM_ACTUATORS] = {
    {A0, A1, A2, A3, A4, A5},
    {A6, A7, A8, A9, A10, A11}
};


// Variables for actuator control
int receivedValues[NUM_PLATFORMS][NUM_ACTUATORS] = {0};
float actuatorPosition[NUM_PLATFORMS][NUM_ACTUATORS] = {0.0};
int actuatorControl[NUM_PLATFORMS][NUM_ACTUATORS] = {0};

// Communication control
int cnt_com = 0;
const int threshold_cnt = 1000; // Maximum counter before stopping actuators


void setup() {
  Serial.begin(250000);
    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), emergencyStopISR, FALLING);
    
    for (int p = 0; p < NUM_PLATFORMS; p++) {
        for (int i = 0; i < NUM_ACTUATORS; i++) {
            pinMode(DIR_A[p][i], OUTPUT);
            pinMode(DIR_B[p][i], OUTPUT);
            pinMode(POT[p][i], INPUT);
        }
    }
    //moveToHomePosition();
    Serial.println("Arduino Ready to work");
}

void loop() {
     if (emergencyStop) {
        stopActuators();
        while(1); // Lock system
    }
    // Handle structured serial commands (HOME, TEST, POSITION, ACTUATOR_TEST)
    if (Serial.available()){
      mainLoop();         // Handle full-line commands
    }
    // Continously check for incoming control values and apply them
    parseAndApplyLengths(); 
    
}
