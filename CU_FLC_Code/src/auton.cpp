#include "vex.h"
#include <algorithm>
#include "auton.h"

using namespace vex;

int intakeSpeedPCT = 100;

bool finishedAutonUnfolding = false;

void inertialCalibration(){
  right_inertial.calibrate();
  left_inertial.calibrate();
  while (right_inertial.isCalibrating() || left_inertial.isCalibrating()) 
  {
    wait(100, msec);
  }
}

void resetFunction() {
  back_L.resetRotation();
  back_R.resetRotation();
  front_L.resetRotation();
  front_R.resetRotation();
  left_encoder.resetRotation();
  right_encoder.resetRotation();
  back_encoder.resetRotation();
}

//Unfolding procedure
void autonUnfold ()
{
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
  left_intake.stop(brake);
  right_intake.stop(brake);
  finishedAutonUnfolding = true;

}

//Autoindexing task
float autonThreshold1 = 5;
float autonThreshold2 = 5;

int progAutoIndexCallback() 
{
  while (true) 
  { 
    if (ballPos2.reflectivity() < autonThreshold2)
    {
      indexer.spin(fwd, 50, pct);
      sorter.spin(fwd, 50, pct);
    } 
    
    else if (ballPos1.reflectivity() > autonThreshold1 && ballPos2.reflectivity() > autonThreshold2)
    {
      indexer.spin(fwd, 100, pct);
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

void setDriveSpeed(int leftSpeed, int rightSpeed)
{
  front_L.spin(fwd, leftSpeed, velocityUnits::pct);
  front_R.spin(fwd, rightSpeed, velocityUnits::pct);
  back_L.spin(fwd, rightSpeed, velocityUnits::pct);
  back_R.spin(fwd, leftSpeed, velocityUnits::pct);
}


int debugging()
{
  while(true)
  {
    printf("frontL %f\n", front_L.velocity(pct));
    printf("frontR %f\n", front_R.velocity(pct));
    printf("backL %f\n", back_L.velocity(pct));
    printf("backR %f\n", back_R.velocity(pct));
    task::sleep(100);
  }
  task::sleep(10);
}

void holdDrive()
{
  front_L.stop(hold);
  front_R.stop(hold);
  back_L.stop(hold);
  back_R.stop(hold);
}

void brakeDrive()
{
  front_L.stop(brake);
  front_R.stop(brake);
  back_L.stop(brake);
  back_R.stop(brake);
}

void coastDrive()
{
  front_L.stop(coast);
  front_R.stop(coast);
  back_L.stop(coast);
  back_R.stop(coast);
}

void setIntakeSpeed(int speed)
{
  left_intake.spin(fwd, speed, pct);
  right_intake.spin(fwd, speed, pct);
}

void brakeIntake()
{
  right_intake.stop(brake);
  left_intake.stop(brake);
}

void setIndexerSpeed(int speed)
{
  indexer.spin(fwd, speed, pct);
  sorter.spin(fwd, speed, pct);
}

void setSortingSpeed(int speed)
{
  indexer.spin(fwd, speed, pct);
  sorter.spin(fwd, -speed, pct);
}

void brakeConveyor(){
  indexer.stop(brake);
  sorter.stop(brake);
}

void moveForward(double distanceToTravel, int speed) 
{
  double wheelDiameterIN = 3.25;
  double travelTargetCM = distanceToTravel; // this is the distance it goes which is set as a variable
  double circumference = wheelDiameterIN * 3.141592;
  double degreesToRotate = ((360 * travelTargetCM) / circumference) * sin(45);

  back_L.setVelocity(speed, vex::velocityUnits::pct);
  back_R.setVelocity(speed, vex::velocityUnits::pct);
  front_L.setVelocity(speed, vex::velocityUnits::pct);
  front_R.setVelocity(speed, vex::velocityUnits::pct);

  back_L.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  back_R.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  front_L.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  front_R.rotateFor(degreesToRotate, vex::rotationUnits::deg, true);
}

void moveForwardSimple(int speed) 
{
  back_L.spin(fwd, speed, pct);
  back_R.spin(fwd, speed, pct);
  front_L.spin(fwd, speed, pct);
  front_R.spin(fwd, speed, pct);
}

void strafeSimpleRight(int speed) 
{
  back_R.spin(fwd, speed, pct);
  front_L.spin(fwd, speed, pct);
}

void strafeSimpleLeft(int speed) 
{
  back_L.spin(fwd, speed, pct);
  front_R.spin(fwd, speed, pct);
}

void strafe(double distanceToTravel, int speed)
{
  double wheelDiameterIN = 3.25;
  double travelTargetCM = distanceToTravel; // this is the distance it goes which is set as a variable
  double circumference = wheelDiameterIN * 3.141592;
  double degreesToRotate = ((360 * travelTargetCM) / circumference) * sin(45);

  back_L.setVelocity(-speed, vex::velocityUnits::pct);
  back_R.setVelocity(speed, vex::velocityUnits::pct);
  front_L.setVelocity(speed, vex::velocityUnits::pct);
  front_R.setVelocity(-speed, vex::velocityUnits::pct);

  back_L.rotateFor(-degreesToRotate, vex::rotationUnits::deg, false);
  back_R.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  front_L.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  front_R.rotateFor(-degreesToRotate, vex::rotationUnits::deg, true);
}

int intakeOn() {
  while(true){
    right_intake.spin(directionType::fwd, intakeSpeedPCT, voltageUnits::volt);
    left_intake.spin(directionType::fwd, intakeSpeedPCT, voltageUnits::volt);
    /*if(ballPos1.reflectivity() >= 10) {
      task intakingBalls = task(scoreGoal);
    }*/
  }
}

float get_average_inertial() 
{
  float robotDirection = (-right_inertial.rotation(deg) - left_inertial.rotation(deg)) / 2;
  //printf("heading average  %f\n", get_average_inertial());
  return robotDirection;
}

void rotate(int heading, int speed)
{
  double currentHeading = get_average_inertial();

  if (currentHeading < heading)
  {
    speed = speed * 1;
  }
  else
  {
    speed = speed * -1;
  }

  while (fabs(heading - currentHeading) > 0.5 && fabs(heading - currentHeading) < -0.5)
  {
    back_L.setVelocity(-speed, vex::velocityUnits::pct);
    back_R.setVelocity(speed, vex::velocityUnits::pct);
    front_L.setVelocity(-speed, vex::velocityUnits::pct);
    front_R.setVelocity(speed, vex::velocityUnits::pct);
  }

}

void intakeOff(){
  right_intake.stop(brake);
  left_intake.stop(brake);
}

void score1Ball()
{
  sorter.spin(fwd, 100, pct);
    
  if (ballPos2.reflectivity() < 5)
  {
    sorter.spin(fwd, 100, pct);
    wait (1, sec);
  }
}

void createIntakeOnTask(){
  task intakeOnTask = task(intakeOn);
}

void stopIntakeOn(){
  task::stop(intakeOn);
}


//Radius of robot from center = 8.75 inches
//Distance from center to auto-aligner = 8.25 inches
//Distance from center to front of intakes = 14.25 inches

void testRun()
{
  task progAutoIndex = task (progAutoIndexCallback);
  stopIntakeOn();
  brakeIntake();
  strafe(15.25, 80); // Move in front of front left corner goal
  autonUnfold();


  // Score preload ball in front left corner goal
  rotate(45, 90); // Face back left corner goal
  moveForward(10.71, 80); // Align with front left corner goal
  score1Ball();
  wait(0.5, sec);
  moveForward(-10.71, 80); // Back up from front left corner goal
  rotate(0, 90); // Rotate back to facing left side of field


  // Pick up left field wall ball #1
  strafe(12, 80); // Strafe behind field wall ball #1
  createIntakeOnTask();
  moveForward(9.75, 90); // Approach field wall ball
  wait(0.5, sec);
  stopIntakeOn();
  brakeIntake();
  moveForward(-9.75, 90); // Back up from left field wall


  //Score in left center goal
  strafe(36, 80); // Move behind left center goal
  moveForward(3.39, 80); // Align with left center goal
  score1Ball();
  moveForward(-3.39, 80); // Back up from left center goal
  wait (0.5, sec);
  

  // Pick up left field wall ball #2
  strafe(36, 80); // Move behind field wall ball #2
  createIntakeOnTask();
  moveForward(9.75, 90); // Approach field wall ball
  wait(0.5, sec);
  stopIntakeOn();
  brakeIntake();
  moveForward(-9.75, 90); // Back up from left field wall


  // Score in back left corner goal
  strafe(12, 80); // Move in front of back left corner goal
  rotate(-45, 90); // Face back left corner goal
  moveForward(10.71, 80); // Align with front left corner goal
  score1Ball();
  wait(0.5, sec);
  moveForward(-10.71, 80); // Back up from front left corner goal
  
}