/*******************************************************************************
* balance_by_daniel.c
*
* Assignment 7: Balance the MiP!
*******************************************************************************/

#include <usefulincludes.h>
#include <roboticscape.h>
#include "./balance_by_daniel.h"

// function declarations
int on_pause_pressed();
int on_pause_released();
int imu_callback();
void* inner_loop();
void* outer_loop();
int initialize_angle_filters();
int reset_controllers();
int disarm_mip();
int arm_mip();
int zero_filter();
daniel_filter_t create_daniel_filter(int order, float dt, float* num, float* den, float gain, float sat);
float step_filter(daniel_filter_t* filter, float new_input);

// variable declarations
imu_data_t data;
float g_angle;
float a_angle;
mip_state_t mip_state;
mip_refs_t  mip_refs;
daniel_filter_t lpass;
daniel_filter_t hpass;
daniel_filter_t iloop;
daniel_filter_t oloop;

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
  
  // Initialize the mip as disarmed
  disarm_mip();
	
  // do your own initialization here
  printf("\n");
  printf("----------------------------\n");
  printf("Welcome to balance_by_daniel\n");
  printf("----------------------------\n");
  
  // Initialize pause button stuff
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

  initialize_angle_filters();
  
  // Initialize gyro angle to 0
  g_angle   = 0.0;
  a_angle   = 0.0;
  mip_state.phi_right = 0.0;
  mip_state.phi_left  = 0.0;
  mip_state.theta     = 0.0;
  mip_state.phi       = 0.0;
  mip_state.u         = 0.0;
  mip_refs.theta_r    = 0.0;
  mip_refs.phi_r      = PHI_REF;
  
  set_imu_interrupt_func(&imu_callback);
  
  // done initializing so set state to RUNNING
  set_state(RUNNING);

  // pause to let some important initialization to occur
  usleep(100000);
  
  // start inner loop
  pthread_t inner_loop_thread;
  pthread_create(&inner_loop_thread, NULL, inner_loop, (void*) NULL);

  // start outer loop
  pthread_t outer_loop_thread;
  pthread_create(&outer_loop_thread, NULL, outer_loop, (void*) NULL);
  
  usleep(1000000*START_DELAY);
  printf("\n\n");
  //printf(" %7.3f |", hpass.num[0]);
  //printf(" %7.3f |", hpass.num[1]);
  //printf(" %7.3f |", hpass.num[2]);
  //printf(" %7.3f |", hpass.num[3]);
  //printf("\n");
  //printf(" %7.3f |", hpass.den[0]);
  //printf(" %7.3f |", hpass.den[1]);
  //printf(" %7.3f |", hpass.den[2]);
  //printf(" %7.3f |", hpass.den[3]);
  //printf("\n");


  // Keep looping until state changes to EXITING
	while(get_state()!=EXITING)
  {
    /*
    printf("\r");
    printf(" %7.2f |", mip_state.theta);
    printf(" %7.2f |", a_angle);
    printf(" %10.5f |", lpass.outputs[0]);
    printf(" %10.5f |", lpass.outputs[1]);
    printf(" %10.5f |", lpass.outputs[2]);
    printf(" %10.5f |", lpass.outputs[3]);
    //printf(" %7.2f |", iloop.num[2]);
    //printf(" %7.2f |", iloop.num[3]);
    //printf(" %d |", mip_state.armed);
    //printf(" %d |", get_state());
    fflush(stdout);
    // */

    if(mip_state.armed)
    {
      //printf("armed\n");
      if(fabs(mip_state.theta)>TIP_ANGLE || get_state()==PAUSED) disarm_mip();
    }
    else
    {
      if(fabs(mip_state.theta)<START_ANGLE && get_state()==RUNNING) arm_mip();
    }
      
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
  if(get_state()==RUNNING)
  {
    set_state(PAUSED);
    set_led(RED,ON);
  }
  else if(get_state()==PAUSED)
  {
    set_state(RUNNING);
    set_led(RED,OFF);
  }
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
 * int reset_controllers()
 *
 * Reset both the balance controllers
 ******************************************************************************/
int reset_controllers()
 {
  zero_filter(&iloop);
  zero_filter(&oloop);
  return 0;
 }

/*******************************************************************************
 * int disarm_mip()
 *
 * Disarm the controllers, disable motors
 ******************************************************************************/
int disarm_mip()
 {
  disable_motors();
  //set_motor_all(0);
  mip_state.armed = 0;
  return 0;
 }
 
 /******************************************************************************
 * int arm_mip()
 *
 * Arm the controllers, zero out stuff to start over balancing
 ******************************************************************************/
int arm_mip()
 {
  reset_controllers();
  set_encoder_pos(ENCODER_CHANNEL_L,0);
  set_encoder_pos(ENCODER_CHANNEL_R,0);
  mip_state.armed = 1;
  enable_motors();
  return 0;
 }

/*******************************************************************************
 * void* inner_loop()
 *
 * Inner loop controller
 ******************************************************************************/
void* inner_loop(void* ptr)
{
  // Initialize shit
  float dt = 1.0/INNER_LOOP_FREQUENCY;
  float iloop_num[] = D1_NUM;
  float iloop_den[] = D1_DEN;
  iloop = create_daniel_filter(D1_ORDER,dt,iloop_num,iloop_den,D1_GAIN,D1_SAT);
  
  float theta_error;
  
  while(get_state()!=EXITING)
  {
    // Run balance filter
    theta_error = mip_refs.theta_r - mip_state.theta;
    mip_state.u = step_filter(&iloop,theta_error);
    if(mip_state.armed)
    {
      set_motor(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * mip_state.u); 
      set_motor(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * mip_state.u);
    }
    // always sleep at some point
    usleep(1000000/INNER_LOOP_FREQUENCY);
  }
  //disarm_mip();
  return NULL;
}

/*******************************************************************************
 * void* outer_loop()
 *
 * Outer loop controller
 ******************************************************************************/
void* outer_loop()
{
  // Initialize shit
  float dt = 1.0/OUTER_LOOP_FREQUENCY;
  float oloop_num[] = D2_NUM;
  float oloop_den[] = D2_DEN;
  oloop = create_daniel_filter(D2_ORDER,dt,oloop_num,oloop_den,D2_GAIN,D2_SAT);
  
  float phi_error;

  while(get_state()!=EXITING)
  {
    mip_state.phi_right = (get_encoder_pos(ENCODER_CHANNEL_R) * TWO_PI)\
                          /(ENCODER_POLARITY_R * GEAR_RATIO * ENCODER_TICKS);
    mip_state.phi_left  = (get_encoder_pos(ENCODER_CHANNEL_L) * TWO_PI)\
                          /(ENCODER_POLARITY_L * GEAR_RATIO * ENCODER_TICKS);
                          
    mip_state.phi = (mip_state.phi_right + mip_state.phi_left)/2.0;
    phi_error = mip_refs.phi_r - mip_state.phi - mip_state.theta;
    mip_refs.theta_r = step_filter(&oloop,phi_error);
    usleep(1000000/OUTER_LOOP_FREQUENCY);
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
  mip_state.theta = step_filter(&hpass,g_angle) + step_filter(&lpass,a_angle)\
                    + CAPE_MOUNT_ANGLE;
  return 0;
}

/*******************************************************************************
 * int initialize_angle_filters()
 *
 * Initialize the filters that work on the angle stuff
 ******************************************************************************/
int initialize_angle_filters()
{
  // Initialize filters
  float dt = 1.0/(float)SAMPLE_FREQUENCY;
  float lpass_num[] = {dt/TIME_CONSTANT,0};
  float lpass_den[] = {1, dt/TIME_CONSTANT-1};
  lpass = create_daniel_filter(1,dt,lpass_num,lpass_den,1,0);
  
  float hpass_num[] = {1-dt/TIME_CONSTANT,dt/TIME_CONSTANT-1};
  float hpass_den[] = {1,dt/TIME_CONSTANT-1};
  hpass  = create_daniel_filter(1,dt,hpass_num,hpass_den,1,0);
  
  return 1;
}

/*******************************************************************************
 * daniel_filter_t create_daniel_filter(int order, float dt, float* num, float* den, float gain, float sat)
 *
 * Create a filter.  Yay!
 ******************************************************************************/
daniel_filter_t create_daniel_filter(int order, float dt, float* num, float* den, float gain, float sat)
{
  daniel_filter_t filter;
  int i;
  int n = 3-order;
  filter.order = order;
  filter.dt = dt;
  filter.gain = gain;
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
  filter.sat = sat;
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
  for(i=3; i>n; i--)
  {
    filter->inputs[i] = filter->inputs[i-1];
    filter->outputs[i] = filter->outputs[i-1];
  }
  filter->inputs[n] = new_input;
  
  // Calculate output
  for(i=n; i<4; i++)
  {
    new_output += gain*filter->num[i]*filter->inputs[i];
  }
  for(i=n+1; i<4; i++)
  {
    new_output -= filter->den[i]*filter->outputs[i];
  }
  
  // Divide out a0
  new_output = new_output/filter->den[n];
  
  if(filter->sat > 0)
  {
    if(new_output > filter->sat) new_output = filter->sat;
    else if(new_output < -1*filter->sat) new_output = -1*filter->sat;
  }
  
  filter->outputs[n] = new_output;
  
  //printf("%8.4f\n",new_output);

  filter->step++;
  return new_output;
}

/*******************************************************************************
 * int zero_filter(daniel_filter_t* filter)
 *
 * Zero out all the values in a filter
 ******************************************************************************/
int zero_filter(daniel_filter_t* filter)
{
  int i;
  for(i=0; i<4; i++)
  {
    filter->inputs[i] = 0.0;
    filter->outputs[i] = 0.0;
  }
  return 0;
}
