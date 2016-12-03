/*******************************************************************************
* daniel_filters.c
*
* Assignment 6: Read the sensors and filter them.
*******************************************************************************/

#include <usefulincludes.h>
#include <roboticscape.h>

// Hash defines
#define SAMPLE_FREQUENCY   100
#define WRITE_FREQUENCY    10
#define FILENAME           "filtered_angles.csv"
#define TIME_CONSTANT      1.0

// function declarations
int on_pause_pressed();
int on_pause_released();
int imu_callback();
void* write_imu();
void* write_csv();

// variable declarations
imu_data_t data;
float g_angle;
float a_angle;
float bbb_angle;
d_filter_t lpass;
d_filter_t hpass;

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
  printf("-------------------------\n");
	printf("Welcome to daniel_filters\n");
  printf("-------------------------\n");
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

  // Initialize gyro angle to 0
  g_angle   = 0.0;
  a_angle   = 0.0;
  bbb_angle = 0.0;
  
  set_imu_interrupt_func(&imu_callback);

  // Initialize filters
  lpass   = create_first_order_lowpass(1.0/(float)SAMPLE_FREQUENCY, TIME_CONSTANT);
  hpass  = create_first_order_highpass(1.0/(float)SAMPLE_FREQUENCY, TIME_CONSTANT);
  //reset_filter(&low_pass);
  //reset_filter(&high_pass);

  printf("dt:  %f \n",1.0/( (float)SAMPLE_FREQUENCY ));
  printf("tau: %f \n",(float) TIME_CONSTANT);
  // done initializing so set state to RUNNING
	set_state(RUNNING);
  
  // start writing to the screen
  pthread_t write_thread;
  pthread_create(&write_thread, NULL, write_imu, (void*) NULL);

  // start writing to csv file
  pthread_t csv_thread;
  pthread_create(&csv_thread, NULL, write_csv, (void*) NULL);

  // print out the state stuff
  printf("\n  a_angle | g_angle | bbb_angle \n");

  // Keep looping until state changes to EXITING
	while(get_state()!=EXITING)
  {
    // We'll deal with everything in different threads, so just chill.
		usleep(100000);
	}

  // Say goodbye
  printf("Goodbye Cruel World\n");
  
  print_filter_details(&lpass);

  // exit cleanly
	destroy_filter(&lpass);
  destroy_filter(&hpass);
  power_off_imu();
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
 * void* write_imu()
 *
 * Write IMU DMP values to the screen
 ******************************************************************************/
void* write_imu(void* ptr)
{
  while(get_state()!=EXITING)
  {
    printf("\r");
    printf("  %7.4f | %7.4f | %7.4f ",a_angle,g_angle,bbb_angle);
    fflush(stdout);
    
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
  g_angle += data.gyro[0]/SAMPLE_FREQUENCY*DEG_TO_RAD;
  a_angle = atan2(-data.accel[2],data.accel[1]);
  bbb_angle = march_filter(&hpass,g_angle) + march_filter(&lpass,a_angle);
  return 0;
}

/*******************************************************************************
 * void* write_csv()
 *
 * Write the IMU data to a CSV file
 ******************************************************************************/
void* write_csv()
{
  // Initialize file
  FILE *csv; // pointer to file (stream)
  csv = fopen(FILENAME,"w");
  fprintf(csv,"time,a_angle,g_angle,bbb_angle\n");
  float i = 0.0;

  while(get_state()!=EXITING)
  {
    fprintf(csv,"%f,%f,%f,%f\n",i/WRITE_FREQUENCY,a_angle,g_angle,bbb_angle);
    i++;
    usleep(1000000/WRITE_FREQUENCY);
  }

  fclose(csv);
  return NULL;
}
