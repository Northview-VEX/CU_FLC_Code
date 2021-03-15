using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor back_R;
extern motor back_L;
extern motor front_R;
extern motor right_intake;
extern motor sorter;
extern motor indexer;
extern motor left_intake;
extern motor front_L;
extern controller Controller1;
extern line ballPos1;
extern line ballPos2;
extern inertial right_inertial;
extern inertial left_inertial;
extern encoder left_encoder;
extern encoder right_encoder;
extern encoder back_encoder;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );