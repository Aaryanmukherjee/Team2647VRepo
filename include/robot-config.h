using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor FL;
extern controller Controller1;
extern motor FR;
extern motor BR;
extern motor BL;
extern motor IntakeUp;
extern motor Sorter;
extern motor IntakeLeft;
extern motor IntakeRight;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );