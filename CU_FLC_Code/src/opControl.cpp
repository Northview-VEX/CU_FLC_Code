#include "vex.h"
#include "opControl.h"

using namespace vex;

bool finishedUnfolding = false;

int joyStickControlCallback()
{
  while (true)
  {
    //Get the raw sums of appropriate joystick axes
    double front_left = (double)(Controller1.Axis3.position(pct) + Controller1.Axis4.position(pct) + Controller1.Axis1.position(pct));
    double back_left = (double)(Controller1.Axis3.position(pct) - Controller1.Axis4.position(pct) + Controller1.Axis1.position(pct));
    double front_right = (double)(Controller1.Axis3.position(pct) - Controller1.Axis4.position(pct) - Controller1.Axis1.position(pct));
    double back_right = (double)(Controller1.Axis3.position(pct) + Controller1.Axis4.position(pct) - Controller1.Axis1.position(pct));

    //Find the largest raw sum or 100
    double max_raw_value = std::max(front_left,std::max(back_left,std::max(front_right,std::max(back_right,100.0))));
    
    //Scale down each value if there was one larger than 100, otherwise leave them alone
    //The largest value will be scaled down to 100, and the others will be reduced by the same factor
    front_left = front_left / max_raw_value * 100;
    back_left = back_left / max_raw_value * 100;
    front_right = front_right / max_raw_value * 100;
    back_right = back_right / max_raw_value * 100;

    //Write the scaled sums out to the various motors
    front_L.spin(fwd, front_left, velocityUnits::pct);
    back_L.spin(fwd, back_left, velocityUnits::pct);
    front_R.spin(fwd, front_right, velocityUnits::pct);
    back_R.spin(fwd, back_right, velocityUnits::pct);

    //Debugging info
    printf("left side %f\n", left_encoder.rotation(deg));
    printf("right side%f\n", right_encoder.rotation(deg));
  }
  return 1;
}


void unfold ()
{
  task joyStickControl = task (joyStickControlCallback);
  sorter.spin(fwd, 100, pct);
  left_intake.spin(fwd, -80, pct);
  right_intake.spin(fwd, -80, pct);
  wait(0.25, sec);

  left_intake.stop(brake);
  right_intake.stop(brake);
  wait (0.1, sec);

  sorter.stop(brake);
  left_intake.spin(fwd, 80, pct);
  right_intake.spin(fwd, 80, pct);
  wait(0.2, sec);
  finishedUnfolding = true;

}

float threshold1 = 5;
float threshold2 = 5;

int autoIndexCallback() 
{
  while (true) 
  { 
    if (ballPos2.reflectivity() < threshold2 && Controller1.ButtonR1.pressing() == false && Controller1.ButtonR2.pressing() == false)
    {
      indexer.spin(fwd, 50, pct);
      sorter.spin(fwd, 50, pct);
    } 
    
    else if (ballPos1.reflectivity() < threshold1 && ballPos2.reflectivity() > threshold2 && Controller1.ButtonR1.pressing() == false && Controller1.ButtonR2.pressing() == false)
    {
      indexer.spin(fwd, 50, pct);
      //wait (0.25, sec);
    }

    else 
    {
      break;
    }

    task::sleep(10);
  }
return 1; 
}


void intakeControl ()
{
  task joyStickControl = task (joyStickControlCallback);
  task autoIndex = task (autoIndexCallback);

  if (Controller1.ButtonL1.pressing())
  {
    left_intake.spin(fwd, 100, pct);
    right_intake.spin(fwd, 100, pct);
  }
       
  else if (Controller1.ButtonL2.pressing())
  {
    left_intake.spin(fwd, -100, pct);
    right_intake.spin(fwd, -100, pct);
  }

  else
  {
    left_intake.stop(brake);
    right_intake.stop(brake);
  }

}

void manualIndexerControl(){
  if (Controller1.ButtonR1.pressing())
  {
    sorter.spin(fwd, 100, pct);
    
    if (ballPos2.reflectivity() < threshold1)
    {
      indexer.spin(fwd, 100, pct);
    }
  }
  else if (Controller1.ButtonR2.pressing())
  {
    indexer.spin(fwd, -100, pct);
    sorter.spin(fwd, -100, pct);
    left_intake.spin(fwd, -10, pct);
    right_intake.spin(fwd, -10, pct);
  }

  else
  {
    indexer.stop(brake);
    sorter.stop(brake);
  }
}