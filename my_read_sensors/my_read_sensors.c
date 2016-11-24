/*******************************************************************************
* my_read_sensors.c
*
* Assignment 4: Read the sensors
*******************************************************************************/

#include <usefulincludes.h>
#include <roboticscape.h>


// Hash defines
#define SAMPLE_FREQUENCY 20
#define WRITE_FREQUENCY  10

// function declarations
int on_pause_pressed();
int on_pause_released();
int imu_callback();
void* write_imu();

// variable declarations
imu_data_t data;

/*******************************************************************************
* int main() 
*	
* This template main function contains these critical components
* - call to initialize_cape
* - main while loop that checks for EXITING condition
* - cleanup_cape() at the end
*******************************************************************************/
int main()
{
	// always initialize cape library first
	initialize_cape();

	// do your own initialization here
  printf("\n");
  printf("--------------------------\n");
	printf("Welcome to my_read_sensors\n");
  printf("--------------------------\n");
	set_pause_pressed_func(&on_pause_pressed);
	set_pause_released_func(&on_pause_released);

  // Initialize DMP Mode on IMU
  imu_config_t imu_config = get_default_imu_config();
  imu_config.enable_magnetometer=1;
  imu_config.dmp_sample_rate=SAMPLE_FREQUENCY;
  if(initialize_imu_dmp(&data,imu_config))
  {
    printf("Could not initialize IMU\n");
    return -1;
  }
      
  set_imu_interrupt_func(&imu_callback);

	// done initializing so set state to RUNNING
	set_state(RUNNING);
  
  // start writing to the screen
  pthread_t write_thread;
  pthread_create(&write_thread, NULL, write_imu, (void*) NULL);

  // print out the state stuff
  printf("\n  Accel X | Accel Y | Accel Z |  Angle\n");

  // Keep looping until state changes to EXITING
	while(get_state()!=EXITING)
  {
    // We'll deal with everything in different threads, so just chill.
		usleep(100000);
	}

  // Say goodbye
  printf("Goodbye Cruel World\n");
	
  // exit cleanly
	cleanup_cape();
	return 0;
}


/*******************************************************************************
* int on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
int on_pause_released()
{
	// toggle betewen paused and running modes
	if(get_state()==RUNNING)   		set_state(PAUSED);
	else if(get_state()==PAUSED)	set_state(RUNNING);
  return 0;
}

/*******************************************************************************
* int on_pause_pressed() 
*	
* If the user holds the pause button for 2 seconds, set state to exiting which 
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
int on_pause_pressed()
{
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++)
  {
		usleep(us_wait/samples);
		if(get_pause_button() == RELEASED) return 0;
	}
	printf("\nlong press detected, shutting down\n");
	set_state(EXITING);
	return 0;
}

/*******************************************************************************
 * int print_imu()
 *
 * prints in a nice and formatted way the current IMU values
 ******************************************************************************/
int print_imu()
{
  printf("\r ");
  
  printf(" %7.4f | %7.4f | %7.4f | %7.4f ",data.accel[0],data.accel[1],data.accel[2],atan2(-data.accel[2],data.accel[1]));

  fflush(stdout);
  return 0;
}

/*******************************************************************************
 * void* write_imu()
 *
 * Write IMU DMP values to the screen
 ******************************************************************************/
void* write_imu(void* ptr)
{
  while(get_state()!=EXITING)
  {
    // write IMU data
    print_imu();
    
    // always sleep at some point
    usleep(1000000/WRITE_FREQUENCY);
  }
  return NULL;
}

/*******************************************************************************
 * int imu_callback()
 * 
 * Called when a new IMU DMP value is received
 ******************************************************************************/
int imu_callback()
{
  // Do something?
//  printf("Callback\n"); 
//  print_imu();
  return 0;
}
