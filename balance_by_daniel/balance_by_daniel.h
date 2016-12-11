/*******************************************************************************
 * balance_by_daniel.h
 * 
 * Configurations and definitions and stuff for "balance_by_daniel.c"
 ******************************************************************************/

// Timing
#define SAMPLE_FREQUENCY       200
#define INNER_LOOP_FREQUENCY   200
#define OUTER_LOOP_FREQUENCY   10
#define TIME_CONSTANT          1.0

// MiP Physical Properties
#define CAPE_MOUNT_ANGLE      0.40
#define GEAR_RATIO            35.577
#define ENCODER_TICKS         60
#define WHEEL_RADIUS          0.034
#define WHEEL_TRACK           0.035

// Inner Loop Controller
#define D1_GAIN    -4.0
#define D1_ORDER   2
#define D1_NUM     { 1.0000, -1.9470, 0.9480 }
#define D1_DEN     { 1.0000, -1.9048, 0.9048 }
#define D1_SAT     1

// Outer Loop Controller
#define D2_GAIN    0.20766
#define D2_ORDER   1
#define D2_NUM     { 1.0000, -0.9882 }
#define D2_DEN     { 1.0000, -0.6065 }
#define D2_SAT     0.75

// Wiring Parameters
#define MOTOR_CHANNEL_L       3
#define MOTOR_CHANNEL_R       2
#define MOTOR_POLARITY_L      1
#define MOTOR_POLARITY_R      -1
#define ENCODER_CHANNEL_L     3
#define ENCODER_CHANNEL_R     2
#define ENCODER_POLARITY_L    1
#define ENCODER_POLARITY_R    -1

// Safety Parameters
#define TIP_ANGLE        0.75
#define START_ANGLE      0.3
#define PICKUP_TIME      0.5
#define PHI_REF          0.0


// Implement custom filter struct up to 3th order
typedef struct daniel_filter_t
{
  // basic stuff
  int order;
  float dt;
  float gain;
  float sat;
  uint64_t step;
  int initialized;

  // Filter values
  float num[4];
  float den[4];
  float inputs[4];
  float outputs[4];
} daniel_filter_t;

// Robot state
typedef struct mip_state_t
{
  // Basic robot state variables
  float phi_left;
  float phi_right;
  float theta;
  float phi;
  float u;
  int   armed;
  
} mip_state_t;

// Robot reference values
typedef struct mip_refs_t
{
  float theta_r;
  float phi_r;
  
} mip_refs_t;
