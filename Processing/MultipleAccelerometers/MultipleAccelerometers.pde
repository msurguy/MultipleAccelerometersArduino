import processing.serial.*;

Serial myPort;  // The serial port
String serialString = "";  // Input string from serial port
String tipValuesString;
String refValuesString;
int serialSplitIndex;
float[] tipValues;
float[] refValues;
float refTheta;
float tipTheta;

String refThetaText;
String tipThetaText;

int interval = 0;

void setup() {
  size(400, 200);
  background(255);

  // List all the available serial ports
  printArray(Serial.list());
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, "/dev/cu.usbmodem1421", 115000);
  myPort.write('r'); // write single character to trigger DMP init
  myPort.clear();            // flush buffer
  myPort.bufferUntil('\n');  // set buffer full flag on receipt of carriage return
}

void draw() {
  background(255); // Clear the scene
  
   if (millis() - interval > 1000) {
        // resend single character to trigger DMP init/start
        // in case the MPU is halted/reset while applet is running
        myPort.write('r');
        interval = millis();
    }

  //print(serialString);
  serialSplitIndex = serialString.indexOf("tip ypr");
  if (serialSplitIndex >= 0 ) {
    refValuesString = trim(serialString.substring(0, serialSplitIndex));
    tipValuesString = trim(serialString.substring(serialSplitIndex, serialString.length()-1));
    refValues = float(split(refValuesString, '\t'));
    tipValues = float(split(tipValuesString, '\t'));
    //println(tipValues);
    if (tipValues.length > 3){
      //println(tipValues.length);
      tipTheta = tipValues[3];
    }
    
    if (refValues.length > 3){
      //println(refValues[3]);
      refTheta = refValues[3];
    }
    
  }
  
  
  pushMatrix();
  rectMode(CORNER);
  refThetaText = str(refTheta);
  fill(50);
  text(refThetaText, 10, 10, 70, 80);
  popMatrix();
  
  pushMatrix();
  rectMode(CORNER);
  tipThetaText = str(tipTheta);
  fill(50);
  text(tipThetaText, width - 90, 10, 70, 80);
  popMatrix();
  
  pushMatrix();
  stroke(0);
  fill(175);
  // Translate origin to center
  translate(100, height/2);

  // The greek letter, theta, is often used as the name of a variable to store an angle

  // Rotate by the angle theta
  rotate(radians(refTheta));

  // Display rectangle with CENTER mode
  rectMode(CENTER);
  rect(0, 0, 100, 100);
  popMatrix();
  
  pushMatrix();
  stroke(0);
  fill(175);
  // Translate origin to center
  translate(300, height/2);

  // The greek letter, theta, is often used as the name of a variable to store an angle

  // Rotate by the angle theta
  rotate(radians(tipTheta));

  // Display rectangle with CENTER mode
  rectMode(CENTER);
  rect(0, 0, 100, 100);
  popMatrix();
}

void serialEvent(Serial p) { 
  serialString = trim(p.readStringUntil('\n')); 
} 