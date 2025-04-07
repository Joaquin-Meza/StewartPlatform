//__________________________________________________________________________
//__________________ FUNCTION DEFINITIONS __________________________________
//__________________________________________________________________________

void printArray(int receivedValues[2][6]) {
    Serial.println("Printing receivedValues:");
    for (int i = 0; i < 2; i++) {  // Loop over platforms
        for (int j = 0; j < 6; j++) {  // Loop over actuators
            Serial.print(receivedValues[i][j]);  // Print element
            Serial.print("\t");  // Tab space between values
        }
        Serial.println();  // New line for the next row
    }
}
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
          actuatorPosition[platform][index] = -(6.7408 * voltage)+34.548 - 3.49;
        }
        if (index == 1){     // Characteristic equation for Actuator 2
          actuatorPosition[platform][index] = -(6.7016 * voltage)+34.499 -3.30;
        }
        if (index == 2){     // Characteristic equation for Actuator 3
          actuatorPosition[platform][index] = -(6.7202 * voltage)+34.605 -3.55;
        }
        if (index == 3){     // Characteristic equation for Actuator 4
          actuatorPosition[platform][index] = -(6.7202 * voltage)+34.605 -3.4;
        }
        if (index == 4){     // Characteristic equation for Actuator 5
          actuatorPosition[platform][index] = -(6.6775 * voltage)+34.615 -3.5;
        }
        if (index == 5){     // Characteristic equation for Actuator 6
          actuatorPosition[platform][index] = -(6.7129 * voltage)+34.876 -3.62;
        }
    }
    if (platform == 1){
        if (index == 0){     // Characteristic equation for Actuator 1
          actuatorPosition[platform][index] = -(6.6893 * voltage)+34.395-3.17;
        }
        if (index == 1){     // Characteristic equation for Actuator 2
          actuatorPosition[platform][index] = -(6.7267 * voltage)+34.632-3.2;
        }
        if (index == 2){     // Characteristic equation for Actuator 3
          actuatorPosition[platform][index] = -(6.7267 * voltage)+34.497-3.45;
        }
        if (index == 3){     // Characteristic equation for Actuator 4
          actuatorPosition[platform][index] = -(6.7133 * voltage)+34.538-3.41;
        }
        if (index == 4){     // Characteristic equation for Actuator 5
          actuatorPosition[platform][index] = -(6.6871 * voltage)+34.552-3.37;
        }
        if (index == 5){     // Characteristic equation for Actuator 6
          actuatorPosition[platform][index] = -(6.7042 * voltage)+34.471-3;
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
    if (control < 0) {
        analogWrite(dirA, 0);
        analogWrite(dirB, abs(control));
    } else if (control > 0) {
        analogWrite(dirA, abs(control));
        analogWrite(dirB, 0);
    } else {
        analogWrite(dirA, 0);
        analogWrite(dirB, 0);
    }
}

//--------------------------------------------------------------
void moveToHomePosition2(){
  for (int p = 0; p < NUM_PLATFORMS; p++) {
        for (int i = 0; i < NUM_ACTUATORS; i++) {
            analogWrite(DIR_A[p][i], 0);
            analogWrite(DIR_B[p][i], 255);
        }
    }
  delay(5000);
  stopActuators();
    
}
void moveToHomePosition() {
  // Moves both Stewart platforms to the predefined home position
    const float homePosition = 1;
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
void test_platforms(String receivedData){
  // Check if the emergency stop has been triggered
  if (emergencyStop){
    stopActuators();    // Immediately stop all actuators
    return;             // Exit function
  }
  
 // If the received command is an emergency stop, execute the stop funtion 
  if (receivedData == "EMERGENCY_STOP"){
    handleEmergencyStop();
    return;
  }

  // Ensure receivedData starts with "TEST,"
  if(!receivedData.startsWith("TEST,")){
    Serial.println("Error: Invalid TEST command format");
    return;
  }

  // Remove "TEST," prefix and extrac the numeric control values
  receivedData = receivedData.substring(5);   // Remove "TEST,"
  //Serial.print("Control data received: ");
  //Serial.println(receivedData);
  // Convert String to character array for parsing
  char receivedCharArray[receivedData.length()+1];
  receivedData.toCharArray(receivedCharArray, receivedData.length()+1);
  
  // Communication timeout check: if the command counter exceeds the threshold, stop all actuators
  cnt_com+=1;
  if (cnt_com >= threshold_cnt) {
    cnt_com = threshold_cnt;      // Prevent overflow
    stopActuators();              // Stop actuators due to timeout
  }

  
  cnt_com = 0;                // Reset the command counter to ensure active communication
  
  // Parse the received control values
  int valueIndex = 0;         // Initialize the variable for parsin incoming data
  // Tokenize the received string, splitting values separated by commas
  char *token = strtok(receivedCharArray,",");
  int receivedValues[NUM_PLATFORMS][NUM_ACTUATORS]; // Store control values
    
  // Loop through each extracted token to assign control values to actuators
  while (token != NULL && valueIndex < NUM_PLATFORMS*NUM_ACTUATORS){
    // Determine the platform index (0 or 1) based on position in the list
    int platform = valueIndex / NUM_ACTUATORS;
    
    // Determine the actuator index (0 to 5) within the platform
    int actuator = valueIndex % NUM_ACTUATORS;

    // Convert the extracted token to an integer control value and store it
    receivedValues[platform][actuator] = round(atoi(token)); // Convert control data to the closest int

    // Move to the next token in the string
    valueIndex++;
    token = strtok(NULL,",");
  }
  //printArray(receivedValues);     // Debug line

  // Ensure the number of received values matches the expected actuator count before proceeding
  if (valueIndex == NUM_PLATFORMS*NUM_ACTUATORS){
    // Iterate over each platform
    for (int p = 0; p < NUM_PLATFORMS; p++) {
      // Iterate over each actuator within the platform
      for (int i = 0; i < NUM_ACTUATORS; i++) {
        // Assign the received control value to the corresponding actuator
        actuatorControl[p][i] = receivedValues[p][i];

        // Apply the control signal to the actuator (moving it accordingly)
        pwmWrite(actuatorControl[p][i], DIR_A[p][i], DIR_B[p][i]);
        // Wait for actuator to move (modify this delay if needed)
        //delay(500);  // Adjust this delay based on actual actuator response time
        // Stop the actuator after a brief delay to prevent continuous movement
        //delay(10);
        //pwmWrite(0, DIR_A[p][i], DIR_B[p][i]);    // Stop movement  
        
        // Update the recorded position of the actuator using its feedback encoder
        updatePositionMeasurement(p, i);
        }
    }

    // Send updated actuator position data to the external controller (Python)
    sendActuatorFeedback();
    // Ensure Serial buffer is cleared AFTER sending feedback to avoid data overflow
    Serial.flush();
    }
}

//--------------------------------------------------------------

void sendActuatorFeedback(){
  // Print the first element to avoid leading ", "
  //Serial.print(actuatorPosition[0][0]);
  
  for (int p = 0; p< NUM_PLATFORMS; p++){
    for (int i = 0; i < NUM_ACTUATORS; i++){
      updatePositionMeasurement(p, i);

      if (p==0 && i ==0){
        Serial.print(actuatorPosition[p][i]);
      }
      else{
      Serial.print(", ");
      Serial.print(actuatorPosition[p][i]);
      }
    }
  }
  Serial.println();
}
//--------------------------------------------------------------
void testActuatorTrajectory(String receivedData){
  if (emergencyStop){
    stopActuators();
    return;
  }
    
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
      control_signal = round(control_signal);

      // Convert control signal back to range (-255 to 255)
      //control_signal -=255;
      //Serial.print("Commas:");
      //Serial.print(receivedData.substring(comma1 + 1, comma2));
      //Serial.print(receivedData.substring(comma2 + 1, comma3));
      //Serial.println(receivedData.substring(comma3 + 1));
      // Debugging information
      //Serial.print("Processing Platform: ");
      //Serial.print(platform_idx);
      //Serial.print(", Actuator: ");
      //Serial.print(actuator_idx);
      //Serial.print(", Control: ");
      //Serial.println(control_signal);
      //Serial.print("Posicion: ");

      // Validate the recived data
      // Validate indices
      if (platform_idx < 0 || platform_idx > 1 || actuator_idx < 0 || actuator_idx >5){
        Serial.println("Error: Invalid platform or actuator index");
        return;
      }

      // Apply control signal to actuator
      pwmWrite(control_signal, DIR_A[platform_idx][actuator_idx],DIR_B[platform_idx][actuator_idx]);
      //delay(100);
      //pwmWrite(0, DIR_A[platform_idx][actuator_idx],DIR_B[platform_idx][actuator_idx]);
      // Update position measurement
      updatePositionMeasurement(platform_idx, actuator_idx);
      delay(10);
      // Debug: Print actuator position before sending
      //Serial.print("Sending Actuator Position: ");
      //Serial.println(actuatorPosition[platform_idx][actuator_idx]);
      
      // Ensure Arduino sends only One response per request
      // Clear the buffer to prevent overflow
      //Serial.flush();

      // Send updated actuator position back to python
      Serial.println(actuatorPosition[platform_idx][actuator_idx]);
    }
}
//--------------------------------------------------------------

void testIndividualActuator(int platform_idx, int actuator_idx, float target_Position){
  Serial.flush();
  const float tolerance = 0.2;  // Allowable error range
  const int maxAttempts = 20;   // Prevent infinite loops

  int attemptCounter = 0;
  bool positionReached = false;
  Serial.print("Platform ");
  Serial.print(platform_idx);
  Serial.print("Testing actuator ");
  Serial.print(actuator_idx);
  Serial.print(" to position ");
  Serial.println(target_Position);

  while (!positionReached && attemptCounter < maxAttempts){
    if(emergencyStop){
      stopActuators();
      return;
    }
    updatePositionMeasurement(platform_idx, actuator_idx);
    float error = actuatorPosition[0][actuator_idx] - target_Position;

    if (error>tolerance){
      pwmWrite(-180,DIR_A[platform_idx][actuator_idx],DIR_B[platform_idx][actuator_idx]);  // Move down
    }
    else if (error < tolerance){
      pwmWrite(180,DIR_A[platform_idx][actuator_idx],DIR_B[platform_idx][actuator_idx]); // Move up
    }
    else {
      pwmWrite(0,DIR_A[platform_idx][actuator_idx],DIR_B[platform_idx][actuator_idx]); //Stop
      positionReached= true;
    }
    delay(200);
    Serial.println(actuatorPosition[platform_idx][actuator_idx]);
    attemptCounter++;
    }

    if (positionReached) {
        Serial.println("Test Complete: Position reached.");
    } 
    else {
        Serial.println("Test Failed: Position not reached.");
    }
}

//____________________________________________________________________________________
void mainLoop(){
  if (Serial.available()){
    String receivedData = Serial.readStringUntil('\n');   // Read the entire line
    receivedData.trim();    // Remove whitespaces
    

    //Serial.print("mainLoop received: ");   // Debugging: Print received data
    //Serial.println(receivedData);


    if (receivedData == "EMERGENCY_STOP"){
      emergencyStop = true;
      handleEmergencyStop();
      Serial.println("Emergency stop activated");
    }

    else if (receivedData == "HOME"){
      Serial.println("Setting home configuration");
      moveToHomePosition2();      
    }
    
    else if(receivedData == "POSITION"){
      sendActuatorFeedback();
    }
    else if (receivedData == "STOP"){
      stopActuators();
    }
    else if(receivedData.startsWith("TEST")){
      //Serial.println("Running simulation");
      test_platforms(receivedData);
      Serial.flush();
    }
    else if(receivedData.startsWith("ACTUATOR_TEST")){
      //Serial.println("FUNCTION ACTUATOR TEST");
      testActuatorTrajectory(receivedData);
      //Serial.println(receivedData);
      Serial.flush();
    }
    else{
      Serial.println("Invalid option");
    }
    Serial.flush();
  }
  Serial.flush();
  if (emergencyStop){
    stopActuators();
    while(1);
  }
}
