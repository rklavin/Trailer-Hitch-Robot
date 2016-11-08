import processing.serial.*;

 Serial myPort;        // The serial port
 int xPos = 1;         // horizontal position of the graph
 int pastxPos = 0;
 float inByte = 0;
 float motor1v = 0;
 float pastmotor1v = 0;
 float motor2v = 0;
 float pastmotor2v = 0;
 

 void setup () {
   // set the window size:
   size(1000, 800);

   // List all the available serial ports
   // if using Processing 2.1 or later, use Serial.printArray()
   println(Serial.list());

   // I know that the first port in the serial list on my mac
   // is always my  Arduino, so I open Serial.list()[0].
   // Open whatever port is the one you're using.
   myPort = new Serial(this, Serial.list()[1], 9600);

   // don't generate a serialEvent() unless you get a newline character:
   myPort.bufferUntil('\n');

   // set inital background:
   background(0);
 }
 void draw () {
   //Draw Lines and Text
   int line1 = int(height*.33);
   int line2 = int(height*.66);
   
   //Top Line
   strokeWeight(3);
   stroke(100,0,0);
   line(0,line1,width,line1);
   text("Motor 1 (inverted)", 0,line1+10,200,line1);
   //Requires Redraw
   //text(str(motor1v),1,line1+30,200,line1);
   
   //Bottom Line
   strokeWeight(3);
   stroke(0,100,0);
   line(0,line2,width,line2);
   text("Motor 2", 0,line2+10,200,line2);
   //Requires Redraw
   //text(str(motor2v),1,line2+30,200,line2);
   
   //Motor 1 output
   strokeWeight(1);
   stroke(255,0,0);
   //point(xPos, motor1v+line1);
   line(pastxPos, pastmotor1v+line1, xPos, motor1v+line1); 
   
   //Motor 2 output
   strokeWeight(1);
   stroke(0,255,0);
   //point(xPos, motor2v+line2);
   line(pastxPos, pastmotor2v+line2, xPos, motor2v+line2);
   
   pastxPos = xPos;
   pastmotor1v = motor1v;
   pastmotor2v = motor2v;
  
   if (xPos >= width) {
     xPos = 0;
     pastxPos = 0;
     background(0);
   } else {
     // increment the horizontal position:
     xPos++;
     strokeWeight(3);
     stroke(0,0,100);
     line(pastxPos, 1, xPos, 1);
   }
   
  /* 
   // draw the line:
   stroke(127, 34, 255);
   line(xPos, height, xPos, height - inByte);

   // at the edge of the screen, go back to the beginning:
   if (xPos >= width) {
     xPos = 0;
     background(0);
   } else {
     // increment the horizontal position:
     xPos++;
   }
   */
 }

 void serialEvent (Serial myPort) {
   // get the ASCII string:
   /*
   String inString = myPort.readStringUntil('\n');

   if (inString != null) {
     // trim off any whitespace:
     inString = trim(inString);
     // convert to an int and map to the screen height:
     inByte = float(inString);
     println(inByte);
     inByte = map(inByte, 0, 1023, 0, height);
   }
   */
   
   String inString = myPort.readStringUntil('\n');
   if (inString != null){
     inString = trim(inString);
     println(inString);
     String[] values = split(inString,",");
     
     //Motor 1 code
     String motor1 = values[0];
     motor1v = float(motor1);  
     if(motor1v<92){
     motor1v = map(motor1v,40,92,-50,0);
     }
     else{
       motor1v = map(motor1v,92,150,0,50);
     }
     motor1v = motor1v * -1;
     
     //Motor 2 code
     String motor2 = values[1];
     motor2v = float(motor2);
     if(motor2v<92){
     motor2v = map(motor2v,40,92,-50,0);
     }
     else{
       motor2v = map(motor2v,92,150,0,50);
     }
     
   }
 }