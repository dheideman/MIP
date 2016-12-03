/*******************************************************************************
* filters_by_daniel.c
*
* Assignment 6: Read the sensors and filter them with custom filters.
*******************************************************************************/

#include <usefulincludes.h>
#include <roboticscape.h>
#include "./filters_by_daniel.h"

// Hash defines
#define SAMPLE_FREQUENCY   100
#define WRITE_FREQUENCY    10
#define FILENAME           "custom_filtered_angles.csv"
#define TIME_CONSTANT      1.0

// function declarations
int on_pause_pressed();
int on_pause_released();
int imu_callback();
void* write_imu();
void* write_csv();
daniel_filter_t create_daniel_filter(int order, float dt, float* num, float* den);
float step_filter(daniel_filter_t* filter, float new_input);

// variable declarations
imu_data_t data;
float g_angle;
float a_angle;
float bbb_angle;
//d_filter_t lpass;
daniel_filter_t lpass;
//d_filter_t hpass;
daniel_filter_t hpass;

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
  printf("----------------------------\n");
	printf("Welcome to filters_by_daniel\n");
  printf("----------------------------\n");
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
  float dt = 1.0/(float)SAMPLE_FREQUENCY;
  float lpass_num[] = {dt/TIME_CONSTANT,0};
  float lpass_den[] = {1, dt/TIME_CONSTANT-1};
  lpass = create_daniel_filter(1,dt,lpass_num,lpass_den);
  
  float hpass_num[] = {1-dt/TIME_CONSTANT,dt/TIME_CONSTANT-1};
  float hpass_den[] = {1,dt/TIME_CONSTANT-1};
  hpass  = create_daniel_filter(1,dt,hpass_num,hpass_den);

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
  
  // exit cleanly
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
  //printf("%f\n",step_filter(&lpass,a_angle));
  bbb_angle = -1*step_filter(&hpass,g_angle) + step_filter(&lpass,a_angle);
  //bbb_angle = -1*step_filter(&hpass,1) + step_filter(&lpass,1);
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

/*******************************************************************************
 * daniel_filter_t create_daniel_filter(int order, float dt, float* num, float* den)
 *
 * Create a filter.  Yay!
 ******************************************************************************/
daniel_filter_t create_daniel_filter(int order, float dt, float* num, float* den)
{
  daniel_filter_t filter;
  int i;
  int n = 3-order;
  filter.order = order;
  filter.dt = dt;
  filter.gain = 1;
  for(i=0; i<n; i++)
  {
    filter.num[i] = 0;
    filter.den[i] = 0;
    filter.inputs[i]  = 0;
    filter.outputs[i]  = 0;
  }
  for(i=n; i<4; i++)
  {
    filter.num[i] = num[i-n];
    filter.den[i] = den[i-n];
    filter.inputs[i]  = 0;
    filter.outputs[i] = 0;
  }
  filter.step = 0;
  filter.initialized = 1;
  return filter;
}

/*******************************************************************************
 * float step_filter(daniel_filter_t* filter, float new_input)
 *
 * Move forward one step in the filter
 ******************************************************************************/
float step_filter(daniel_filter_t* filter, float new_input)
{
  float gain = filter->gain;
  float new_output = 0;
  int i;
  int n = 3 - filter->order;
  
  // Advance inputs and outputs
  for(i=n; i<3; i++)
  {
    filter->inputs[i] = filter->inputs[i+1];
    filter->outputs[i] = filter->outputs[i+1];
  }
  filter->inputs[3] = new_input;
  
  // Calculate output
  for(i=n; i<4; i++)
  {
    new_output += gain*filter->num[i]*filter->inputs[i];
    //printf("%f\n",gain*filter->num[i]*filter->inputs[i]);
  }
  for(i=n+1; i<4; i++)
  {
    new_output -= filter->den[i]*filter->outputs[i];
    //printf("%f\n",filter->den[i]*filter->outputs[i]);
  }
  
  // Divide out a0
  new_output = new_output/filter->den[n];
  //printf("%f\n",filter->num[n+1]);

  filter->outputs[3] = new_output;
  
  filter->step++;
  return new_output;
}
