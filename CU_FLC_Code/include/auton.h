  
#ifndef AUTONFUNCTIONS_H
#define AUTONFUNCTIONS_H

//Optimized calibration
void inertialCalibration();

//Resets encoder values
void resetFunction();

//Custom unfolding sequence
void autonUnfold();

//Debugging info
int debugging( void );

//Driving forward/backward functions
void setDriveSpeed(int leftSpeed, int rightSpeed);
void moveForward(double distanceToTravel, int speed);
void moveForwardSimple(int speed);

//Strafing functions
void strafeSimpleRight(int speed);
void strafeSimpleLeft(int speed);
void strafe(double distanceToTravel, int speed);

//Stop drive motor functions
void holdDrive(); 
void brakeDrive(); 
void coastDrive();

//Turning functions
void rotate(int heading, int speed);

//Intake functions
void setIntakeSpeed(int speed);
void brakeIntake(); 
int intakeOn( void );
void intakeOff( void );
void createIntakeOnTask( void );
void stopIntakeOn( void );

//Indexer/sorter functions
void setIndexingSpeed(int speed);
void setSortingSpeed(int speed);
void brakeConveyor();

//Custom made scoring function:
void score1Ball();

//FLC Skills Run
void testRun( void );

#endif 