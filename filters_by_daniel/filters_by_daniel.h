
// Implement custom filter struct up to 4th order
typedef struct daniel_filter_t
{
  // basic stuff
  int order;
  float dt;
  float gain;
  uint64_t step;
  int initialized;

  // Filter values
  float num[4];
  float den[4];
  float inputs[4];
  float outputs[4];
} daniel_filter_t;
