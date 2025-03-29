// CORRECTED CODE
//__________________________________________________________________________
//__________________ General Setup _________________________________________
//__________________________________________________________________________
#define MIN_CONTROL_VALUE 25  // Minimum control threshold to prevent small noise activation
#define NUM_ACTUATORS 6  // Number of actuators per Stewart platform
#define NUM_PLATFORMS 2  // Number of Stewart platforms
#define EMERGENCY_STOP_PIN 2  // Pin for emergency stop button


volatile bool emergencyStop = false;  // Flag to indicate emergency stop

// Define motor control pins for two platforms
const int DIR_A[NUM_PLATFORMS][NUM_ACTUATORS] = {
    {30, 32, 34, 36, 38, 40}, // Platform 1
    {42, 44, 46, 48, 50, 52}  // Platform 2
};
const int DIR_B[NUM_PLATFORMS][NUM_ACTUATORS] = {
    {31, 33, 35, 37, 39, 41},
    {43, 45, 47, 49, 51, 53}
};

// The cables are connected in the reverse order
// 0  1  2 3 4 5   6 7 8 9 10 11 
// 11 10 9 8 7 6   5 4 3 2 1  0

const int POT[NUM_PLATFORMS][NUM_ACTUATORS] = {
    {A11, A10, A9, A8, A7, A6},
    {A5, A4, A3, A2, A1, A0}
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
    //Serial.println("Arduino Ready to work");
    Serial.flush();
}

void loop() {
     if (emergencyStop) {
        stopActuators();
        while(1);
    }
    Serial.flush();
    mainLoop();
    Serial.flush();
}
