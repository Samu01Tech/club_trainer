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
  
OscP5 oscP5;
NetAddress myRemoteLocation;

ControlP5 cp5;

Knob deg_launch_angle;
Slider bar_roll;
Slider bar_pitch;
Slider bar_yaw;

ArrayList<String> valueList;

int bar_row = 80;

void setup() {
  size(900,700);
  
  oscP5 = new OscP5(this,32000); /* start oscP5, listening for incoming messages at port 12000 */
  myRemoteLocation = new NetAddress("127.0.0.1",12000);
  
  
  cp5 = new ControlP5(this);
  cp5.setFont(createFont("Inter",16));
  
      
  // add a vertical slider
  bar_roll = cp5.addSlider("bar_roll")
     .setPosition(100, bar_row)
     .setSize(20,200)
     .setRange(90,270)
     .setValue(0)
     .setColorBackground(color(#C9FCE8))
     .setColorForeground(color(#1FDB8E))
     .setColorActive(color(#1FDB8E))
     ;       
 
     
  // add a vertical slider
  bar_pitch = cp5.addSlider("bar_pitch")
     .setPosition(200,bar_row)
     .setSize(20,200)
     .setRange(0,-200)
     .setValue(0)
     .setColorBackground(color(#C9FCE8))
     .setColorForeground(color(#1FDB8E))
     .setColorActive(color(#1FDB8E))
     ;      
     
    bar_yaw = cp5.addSlider("bar_yaw")
     .setPosition(300,bar_row)
     .setSize(20,200)
     .setRange(-20,20)
     .setValue(0)
     .setColorBackground(color(#C9FCE8))
     .setColorForeground(color(#1FDB8E))
     .setColorActive(color(#1FDB8E))
     ;    
     
    deg_launch_angle = cp5.addKnob("deg_launch_angle")  // Define the knob function name
    .setPosition(110, bar_row + 250)  // Knob position
    .setSize(200, 200)  // Knob size
    .setRange(-50, 50)  // Set knob range to -50 to 50 degrees
    .setValue(0)  // Initial value at the center (0 degrees)
    .setNumberOfTickMarks(10)  // Set the number of tick marks
    .setColorForeground(color(#1FDB8E))  // Set the knob color
    ;
  

     
  oscP5.plug(this,"roll","/roll");   
  oscP5.plug(this,"pitch","/pitch");   
  oscP5.plug(this,"yaw","/yaw");   
  oscP5.plug(this,"launch_angle","/launch_angle");      

  valueList = new ArrayList<String>();  
}

void draw() {
  background(0);  
  
  textSize(28);
  textAlign(LEFT, TOP);
  fill(color(#C9FCE8));
  text("ClubTrainer", 100, 30);

  float current_pitch = bar_pitch.getValue();
  // Set the active color based on the value of the angle
  if (current_pitch < -170) {
    bar_pitch.setColorForeground(color(#db2e1e)); // Red for values over 30
  }  else {
    bar_pitch.setColorForeground(color(#1FDB8E)); // Green for values between -30 and 30
  }

  // Set red for angles over 30 degrees or under -30 degrees
  float current_angle = deg_launch_angle.getValue();
  if (current_angle > 30 || current_angle < -30) {
    deg_launch_angle.setColorForeground(color(#db2e1e));
  } else {
    deg_launch_angle.setColorForeground(color(#1FDB8E));
  }

    // Display the list of last 5 values
  displayValueTable();
}

void roll(float input_value) {
  cp5.getController("bar_roll").setValue(input_value);   
}


void pitch(float input_value) {
  cp5.getController("bar_pitch").setValue(input_value);   
}


void yaw(float input_value) {
  cp5.getController("bar_yaw").setValue(input_value);   
}


void launch_angle(float input_value) {
  cp5.getController("deg_launch_angle").setValue(input_value);   

  // get the current values 
  float roll = cp5.getController("bar_roll").getValue();
  float pitch = cp5.getController("bar_pitch").getValue();
  float yaw = cp5.getController("bar_yaw").getValue();
  float launch_angle = cp5.getController("deg_launch_angle").getValue();

  addToList(roll, pitch, yaw, launch_angle);
}

void addToList(float roll, float pitch, float yaw, float launch_angle) {
  // Format the entry as a string
  String entry = round(roll) + "," + round(pitch) + "," + round(yaw) + "," + round(launch_angle);
  
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


  float xPos = 400;  // Starting position for columns
  float yPos = bar_row;  // Starting position for headers
  float columnSpacing = 70; // Adjust spacing between columns
  
  text("Last 5 launches", 400, bar_row);  // Display the title

  textSize(16);
  yPos += 30;
  // Display headers in columns
  text("Roll", xPos, yPos);
  text("Pitch", xPos + columnSpacing, yPos);
  text("Yaw", xPos + 2 * columnSpacing, yPos);
  text("Launch Angle", xPos + 3 * columnSpacing, yPos);

  yPos += 30;
    
  for (int i = 0; i < valueList.size(); i++) {
    String[] values = split(valueList.get(i), ',');  // Split the values by comma
    for (int j = 0; j < values.length; j++) {
      text(values[j], xPos + j * columnSpacing, yPos + i * 20);  // Display each entry in the table format
    }
  }
}


