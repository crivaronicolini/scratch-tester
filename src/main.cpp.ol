// ProportionalControl.pde
// -*- mode: C++ -*-
//
// Make a single stepper follow the analog value read from a pot or whatever
// The stepper will move at a constant speed to each newly set posiiton, 
// depending on the value of the pot.
//
// Copyright (C) 2012 Mike McCauley
// $Id: ProportionalControl.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
#include <Arduino.h>
#include <AccelStepper.h>

#define dirX 25
#define stepX 33

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, stepX, dirX);

// This defines the analog input pin for reading the control voltage
// Tested with a 10k linear pot between 5v and GND
#define ANALOG_IN 36
void setup()
{  
  // pinMode(stepX, OUTPUT);
	// pinMode(dirX, OUTPUT); 
  // Change these to suit your stepper if you want
  stepper.setMaxSpeed(500);
  stepper.setSpeed(100);
  // stepper.setAcceleration(20);
  // stepper.moveTo(500);
}

void loop()
{
    // If at the end of travel go to the other end
    // if (stepper.distanceToGo() == 0)
    //   stepper.moveTo(-stepper.currentPosition());

    // stepper.run();
    stepper.runSpeed();
}

// void setup()
// {  
//   stepper.setMaxSpeed(1000);
// }

// void loop()
// {
//   // Read new position
//   int analog_in = analogRead(ANALOG_IN);
//   stepper.moveTo(analog_in);
//   stepper.setSpeed(100);
//   stepper.runSpeedToPosition();
// }
