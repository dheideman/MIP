/*******************************************************************************
* danielblink.c
*
* This is meant to be a skeleton program for robotics cape projects. 
* Change this description and file name 
*******************************************************************************/

#include <usefulincludes.h>
#include <roboticscape.h>


// Hash defines


// function declarations
int on_pause_pressed();
int on_pause_released();
int on_mode_released();
void* custom_blink();
void* write_state();

// variable declarations
const int mode_delay[] = {2000000,1000000,750000,500000};
int mode;
int last_mode;
state_t last_state;

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
  printf("----------------------\n");
	printf("Welcome to danielblink\n");
  printf("----------------------\n");
	set_pause_pressed_func(&on_pause_pressed);
	set_pause_released_func(&on_pause_released);
  set_mode_released_func(&on_mode_released);
  
	// done initializing so set state to RUNNING
	set_state(RUNNING);
  
  // Initial mode set to 0
  mode = 0;
  last_mode = -1;
  
  // Set up last_state
  last_state = UNINITIALIZED;

  // start blinking
  pthread_t blink_thread;
  pthread_create(&blink_thread, NULL, custom_blink, (void*) NULL);
  
  // start writing to the screen
  pthread_t write_thread;
  pthread_create(&write_thread, NULL, write_state, (void*) NULL);

  // print out the state stuff
  printf("\n  STATE  |  MODE\n");
	print_state(get_state(),mode);

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
 * int on_mode_released()
 *
 * increment mode variable
*******************************************************************************/
int on_mode_released()
{
  mode = mode<3?mode+1:0;
  return 0;
}

/*******************************************************************************
 * int print_state(state_t state, int mode)
 *
 * prints in a nice and formatted way the current state and mode
 ******************************************************************************/
int print_state(state_t state, int mode)
{
  printf("\r ");
  
  if(state==RUNNING)
  {
    printf("Running");
  }
  else if(state==PAUSED)
  {
    printf("Paused ");
  }
  else
  {
    printf("-------");
  }
  
  printf(" |   %d   ",mode);

  fflush(stdout);
  return 0;
}

/*******************************************************************************
 * void* custom_blink()
 *
 * Multi-threaded blink function
 ******************************************************************************/
void* custom_blink(void* ptr)
{
  while(get_state()!=EXITING)
  {
    if(get_state()==RUNNING)
    {
      set_led(GREEN, ON);
      set_led(RED, OFF);
    }
    usleep(mode_delay[mode]*1/6);
    
    if(get_state()==RUNNING)
    {
      set_led(GREEN, OFF);
      set_led(RED, ON);
    }
    usleep(mode_delay[mode]*3/6);
    
    if(get_state()==RUNNING)
    {
      set_led(GREEN, ON);
      set_led(RED, OFF);
    }
    usleep(mode_delay[mode]*1/6);
    
    if(get_state()==RUNNING)
    {
      set_led(GREEN, OFF);
      set_led(RED, ON);
    }
    usleep(mode_delay[mode]*1/6);
  }
  return NULL;
}

/*******************************************************************************
 * void* write_state()
 *
 * Write state to the screen
 ******************************************************************************/
void* write_state(void* ptr)
{
  while(get_state()!=EXITING)
  {
    // handle change in state
    if(get_state()!=last_state || mode!=last_mode)
    {
      print_state(get_state(),mode);
      last_state = get_state();
    }
    
    // always sleep at some point
    usleep(100000);
  }
  return NULL;
}
