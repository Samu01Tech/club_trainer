/* 
* ClubTrainer Processing sketch
*
* Author: Samuele Mazzei
* Date: 11/09/2019
*
*/

import controlP5.*; 
import oscP5.*;
import netP5.*;
import java.util.Arrays;
  
OscP5 oscP5;
NetAddress myRemoteLocation;

ControlP5 cp5;

Knob launch_angle;
Slider bar_roll;
Slider2D end_rotation;
Slider swing_rotation;
Slider launch_power;
Textlabel results_table;

Group row_1;
int row_2 = 450;

ArrayList<String> valueList;

final int WINDOW_WIDTH = 1024;
final int WINDOW_HEIGHT = 768;

int BACKGROUND_COLOR = #100F0F;
int FOREGROUND_COLOR = #FFFCF0;
int GREEN = #879A39;
int RED = #D14D41;
int BLUE = #4385BE;

int MIN_SWING_HEIGHT = 0;
int MAX_SWING_HEIGHT = 160;
int MAX_PITCH = 50;
int MIN_PITCH = -50;

void setup() {
  size(1024,768);
  
  oscP5 = new OscP5(this,32000); /* start oscP5, listening for incoming messages at port 12000 */
  myRemoteLocation = new NetAddress("127.0.0.1",12000);
  
  
  cp5 = new ControlP5(this);
  cp5.setFont(createFont("Arial",16));

  row_1 = cp5.addGroup("Row 1").setPosition(WINDOW_WIDTH/2 - 250, 100).hideBar();
   
    end_rotation = cp5.addSlider2D("End Rotation")
          .setSize(400,200)
          .setGroup(row_1)
          .setMaxX(100)
          .setMinX(0)
          .setMaxY(10)
          .setMinY(0)
          ;
     
    swing_rotation = cp5.addSlider("Swing Rotation")
      .setPosition(450,0)
     .setSize(50,200)
     .setRange(MIN_SWING_HEIGHT, MAX_SWING_HEIGHT)
     .setValue(0)
     .setColorBackground(FOREGROUND_COLOR)
     .setGroup(row_1)
     ;    

    // row_2 = cp5.addGroup("Row 2").setPosition(WINDOW_WIDTH/2 - 400, 450).hideBar();
     
    launch_angle = cp5.addKnob("Launch Angle")  // Define the knob function name
    .setPosition(50, row_2)  // Knob position
    .setSize(200, 200)  // Knob size
    .setRange(-50, 50)  // Set knob range to -50 to 50 degrees
    .setValue(0)  // Initial value at the center (0 degrees)
    .setNumberOfTickMarks(10)  // Set the number of tick marks
    .setColorForeground(color(#1FDB8E))  // Set the knob color
    ;

    launch_power = cp5.addSlider("Launch Power")
      .setPosition(300,row_2)
     .setSize(50,200)
     .setRange(MIN_SWING_HEIGHT, MAX_SWING_HEIGHT)
     .setValue(0)
     .setColorBackground(FOREGROUND_COLOR)
     ;   

     
  oscP5.plug(this,"setEndPitch","/end_pitch");   
  oscP5.plug(this,"setSwingRotation","/swing_rotation");   
  oscP5.plug(this,"setLaunchAngle","/launch_angle");      
  oscP5.plug(this,"setLaunchPower","/launch_power");      

  valueList = new ArrayList<String>();  
  valueList.add("10, 10, 10");
}

void draw() {
  background(color(BACKGROUND_COLOR));  
  
  textSize(28);
  textAlign(CENTER);
  fill(FOREGROUND_COLOR);
  text("ClubTrainer", WINDOW_WIDTH/2, 50);

  // Display the list of last 5 values
  displayValueTable();
}

void roll(float input_value) {
  cp5.getController("bar_roll").setValue(input_value);   
}


void setEndPitch(float input_value) {
  end_rotation.setArrayValue(new float[] {input_value, 5}); 
}


void setSwingRotation(float input_value) {
  swing_rotation.setValue(input_value);   

  // Set the active color based on the value of the angle
  if (input_value > (MAX_SWING_HEIGHT-20)) {
    swing_rotation.setColorForeground(RED);
  }  else {
    swing_rotation.setColorForeground(GREEN);
  }
}

void setLaunchPower(float input_value) {
  launch_power.setValue(input_value);   
}

void setLaunchAngle(float input_value) {
  launch_angle.setValue(input_value);   

  // get the current values 
  float pitch = cp5.getController("end_rotation").getValue();
  float yaw = swing_rotation.getValue();
  float c = launch_angle.getValue();

  addToList(pitch, yaw, c);
}

void addToList(float pitch, float yaw, float launch_angle) {
  // Format the entry as a string
  String entry = round(pitch) + "," + round(yaw) + "," + round(launch_angle);
  
  // Add the entry to the list
  valueList.add(entry);
  
  // If the list exceeds 5 entries, remove the oldest one
  if (valueList.size() > 5) {
    valueList.remove(0);
  }
}

void displayValueTable() {
  // Table headers
  fill(255);
  textSize(20);
  textAlign(LEFT, TOP);


  float xPos = 450;  // Starting position for columns
  float yPos = row_2;  // Starting position for headers
  float columnSpacing = 120; // Adjust spacing between columns
  
  text("Last 5 launches", xPos, yPos);  // Display the title


  textSize(16);
  yPos += 30;
  // Display headers in columns
  text("Pitch", xPos + 0 * columnSpacing, yPos);
  text("Swing Rotation", xPos + 1 * columnSpacing, yPos);
  text("Launch Angle", xPos + 2 * columnSpacing, yPos);

  yPos += 30;
    
  for (int i = 0; i < valueList.size(); i++) {
    String[] values = split(valueList.get(i), ',');  // Split the values by comma
    for (int j = 0; j < values.length; j++) {
      text(values[j], xPos + j * columnSpacing, yPos + i * 20);  // Display each entry in the table format
    }
  }
}
