//__________________________________________________________________________
//__________________ FUNCTION DEFINITIONS __________________________________
//__________________________________________________________________________

void handleEmergencyStop(){
  // Emergency stop function triggered by Python
  Serial.println("Emergency Stop Activated!");
  emergencyStop = true;
  stopActuators();
}
void emergencyStopISR() {
  // Interrupt service routine for emergency stop
    emergencyStop = true;
}

//--------------------------------------------------------------

void stopActuators() {
  // Stops all actuators by setting PWM signals to zero
    for (int p = 0; p < NUM_PLATFORMS; p++) {
        for (int i = 0; i < NUM_ACTUATORS; i++) {
            analogWrite(DIR_A[p][i], 0);
            analogWrite(DIR_B[p][i], 0);
        }
    }
    //Serial.println("Emergency Stop Activated!");
}

//--------------------------------------------------------------

float readPot(int platform, int index) {
  // Reads and averages potentiometer readings for more stable position feedback
    int readings = 5;
    float sum = 0;
    for (int i = 0; i < readings; i++) {
        sum += analogRead(POT[platform][index]);
    }
    return sum / readings * (5.0 / 1023.0);
}

//--------------------------------------------------------------

void updatePositionMeasurement(int platform, int index) {
  // Updates the measured position of a given actuator
    float voltage = readPot(platform, index);
    if (platform == 0){
        if (index == 0){     // Characteristic equation for Actuator 1
          actuatorPosition[platform][index] = -(6.7408 * voltage)+34.548;
        }
        if (index == 1){     // Characteristic equation for Actuator 2
          actuatorPosition[platform][index] = -(6.7016 * voltage)+34.499;
        }
        if (index == 2){     // Characteristic equation for Actuator 3
          actuatorPosition[platform][index] = -(6.7202 * voltage)+34.605;
        }
        if (index == 3){     // Characteristic equation for Actuator 4
          actuatorPosition[platform][index] = -(6.7202 * voltage)+34.605;
        }
        if (index == 4){     // Characteristic equation for Actuator 5
          actuatorPosition[platform][index] = -(6.6775 * voltage)+34.615;
        }
        if (index == 5){     // Characteristic equation for Actuator 6
          actuatorPosition[platform][index] = -(6.7129 * voltage)+34.876;
        }
    }
    if (platform == 1){
        if (index == 0){     // Characteristic equation for Actuator 1
          actuatorPosition[platform][index] = -(6.6893 * voltage)+34.395;
        }
        if (index == 1){     // Characteristic equation for Actuator 2
          actuatorPosition[platform][index] = -(6.7267 * voltage)+34.632;
        }
        if (index == 2){     // Characteristic equation for Actuator 3
          actuatorPosition[platform][index] = -(6.7267 * voltage)+34.497;
        }
        if (index == 3){     // Characteristic equation for Actuator 4
          actuatorPosition[platform][index] = -(6.7133 * voltage)+34.538;
        }
        if (index == 4){     // Characteristic equation for Actuator 5
          actuatorPosition[platform][index] = -(6.6871 * voltage)+34.552;
        }
        if (index == 5){     // Characteristic equation for Actuator 6
          actuatorPosition[platform][index] = -(6.7042 * voltage)+34.471;
        }
    }
}

//--------------------------------------------------------------

void pwmWrite(int control, int dirA, int dirB) {
  // Sends PWM signals to control motor direction and speed
    if (emergencyStop) {
        stopActuators();
        return;
    }
    if (abs(control) < MIN_CONTROL_VALUE) {
        control = 0;
    }
    if (control > 0) {
        analogWrite(dirA, 0);
        analogWrite(dirB, abs(control));
    } else if (control < 0) {
        analogWrite(dirA, abs(control));
        analogWrite(dirB, 0);
    } else {
        analogWrite(dirA, 0);
        analogWrite(dirB, 0);
    }
}

//--------------------------------------------------------------

void moveToHomePosition() {
  // Moves both Stewart platforms to the predefined home position
    const float homePosition = 5.0;
    const float tolerance = 0.2;          // Stopping range (+- 0.2)
    const int maxAttepmts = 15;           // Prevent infite loops (timeout)
    int attemptCounter = 0;
    
    bool homeReached = false;
    
    while (!homeReached && attemptCounter < maxAttepmts) {
      // Check if the interruption instance hasn't been activated
        if (emergencyStop) {
            stopActuators();
            return;
        }
        
        homeReached = true; // Assume succes until proven otherwise
        
        for (int p = 0; p < NUM_PLATFORMS; p++) {               // Loop through each platform
            for (int i = 0; i < NUM_ACTUATORS; i++) {           // Loop through each actuator
                updatePositionMeasurement(p, i);                // Update recording of actuator's actual position

                float error = actuatorPosition[p][i]-homePosition;      // Compute the error of the actuator's position
                
                if (error > tolerance) {                        // Move down
                    pwmWrite(-180, DIR_A[p][i], DIR_B[p][i]);
                    homeReached = false;
                } else if (error < tolerance) {                 // Move up
                    pwmWrite(180, DIR_A[p][i], DIR_B[p][i]);
                    homeReached = false;
                } else {                                        // Stop
                    pwmWrite(0, DIR_A[p][i], DIR_B[p][i]);
                }
            }
        }
        delay(200);
        attemptCounter++;   // Prevent infinite loops
    }
    if (homeReached){Serial.println("Home position reached");}
    else{Serial.println("Home position NOT reached! Check actuators.");}
    sendActuatorFeedback();
}

//--------------------------------------------------------------
void parseAndApplyLengths(){
  if (emergencyStop){
    stopActuators();
    return;
  }
  cnt_com+=1;
  if (cnt_com >= threshold_cnt) {
    cnt_com = threshold_cnt;
    stopActuators();
  }
  if (Serial.available()){
    // Read the incoming data and retrieve the control data
    cnt_com = 0;
    String receivedData = Serial.readStringUntil('\n');   // Read full line
    int valueIndex = 0;
    char *token = strtok((char *)receivedData.c_str(),",");
    
    // Save the values of the control corresponding to each actuator
    while (token != NULL && valueIndex < NUM_PLATFORMS*NUM_ACTUATORS){
      int platform = valueIndex / NUM_ACTUATORS;
      int actuator = valueIndex % NUM_ACTUATORS;
      receivedValues[platform][actuator] = atoi(token); // Convert control data to int
      valueIndex++;
      token = strtok(NULL,",");
    }
    // Once all values are stored move the actuators
    if (valueIndex == NUM_PLATFORMS*NUM_ACTUATORS){
      for (int p = 0; p < NUM_PLATFORMS; p++) {
           for (int i = 0; i < NUM_ACTUATORS; i++) {
                actuatorControl[p][i] = receivedValues[p][i] - 255;
                pwmWrite(actuatorControl[p][i], DIR_A[p][i], DIR_B[p][i]);
                updatePositionMeasurement(p, i);
           }
    }

    // Send current position of each actuator
    sendActuatorFeedback();
    }
  }
}

//--------------------------------------------------------------

void sendActuatorFeedback(){
  // Print the first element to avoid leading ", "
  Serial.print(actuatorPosition[0][0]);
  
  for (int p = 0; p< NUM_PLATFORMS; p++){
    for (int i = 0; i < NUM_ACTUATORS; i++){
      updatePositionMeasurement(p, i);
      Serial.print(", ");
      Serial.print(actuatorPosition[p][i]);
    }
  }
  Serial.println();
}
//--------------------------------------------------------------
void testActuatorTrajectory(){
  if (emergencyStop){
    stopActuators();
    return;
  }

  if (Serial.available()){
    // Read incoming serial data
    String receivedData = Serial.readStringUntil('\n');   // Read full line
    receivedData.trim();    // Remove any extra spaces

    if(receivedData == "EMERGENCY_STOP"){
      handleEmergencyStop();
      return;
    }

    if(receivedData.startsWith("ACTUATOR_TEST")){
      int comma1 = receivedData.indexOf(',');
      int comma2 = receivedData.indexOf(',',comma1+1);
      int comma3 = receivedData.lastIndexOf(',');

      if(comma1 == -1 || comma2 == -1 || comma3 == -1){
        Serial.println("Error: Invalid ACTUATOR_TEST format");
        return;
      }

      // Extract platform index, actuator index, and target position
      int platform_idx = receivedData.substring(comma1 + 1, comma2).toInt();
      int actuator_idx = receivedData.substring(comma2 + 1, comma3).toInt();
      int control_signal = receivedData.substring(comma3 + 1).toInt();

      // Convert control signal back to range (-255 to 255)
      control_signal -=255;

      // Validate the recived data
      // Validate indices
      if (platform_idx < 0 || platform_idx > 1 || actuator_idx < 0 || actuator_idx >5){
        Serial.println("Error: Invalid platform or actuator index");
        return;
      }

      // Apply control signal to actuator
      pwmWrite(control_signal, DIR_A[platform_idx][actuator_idx],DIR_B[platform_idx][actuator_idx]);
      
      // Update position measurement
      updatePositionMeasurement(platform_idx, actuator_idx);

      // Send updated actuator position back to python
      Serial.println(actuatorPosition[platform_idx][actuator_idx]);
    }
  }
}
//--------------------------------------------------------------


//____________________________________________________________________________________
void mainLoop(){
  if (Serial.available()){
    String receivedData = Serial.readStringUntil('\n');   // Read the entire line
    receivedData.trim();    // Remove whitespaces

    if (receivedData == "EMERGENCY_STOP"){
      handleEmergencyStop();
    }

    if (receivedData == "HOME"){
      Serial.println("Setting home configuration");
      moveToHomePosition();
      
    }
    else if(receivedData == "POSITION"){
      sendActuatorFeedback();
    }
    else if (receivedData == "STOP"){
      emergencyStop = true;
      stopActuators();
      Serial.println("Emergency stop activated");
    }
    else if(receivedData == "TEST"){
      Serial.println("Running simulation");
      parseAndApplyLengths();
    }
    else if(receivedData.startsWith("ACTUATOR_TEST")){
      testActuatorTrajectory();
    }
    else{
      Serial.println("Invalid option");
    }
  }
  if (emergencyStop){
    stopActuators();
    while(1);
  }
}
