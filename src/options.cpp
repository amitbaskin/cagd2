#include "options.h"

// Define the variables
unsigned int DEF_NUM_STEPS = NUM_SAMPS;
unsigned int DEF_DEGREE = 3;
unsigned char CURVE_COLOR[ 3 ] = { 255, 0, 0 };
bool HIDE_CTRL_POLYS = false;

const unsigned char *get_curve_color()
{
  return CURVE_COLOR;
}

void set_curve_color( unsigned char new_curve_color[ 3 ] )
{
  CURVE_COLOR[ 0 ] = new_curve_color[ 0 ];
  CURVE_COLOR[ 1 ] = new_curve_color[ 1 ];
  CURVE_COLOR[ 2 ] = new_curve_color[ 2 ];
}

void get_curve_color( unsigned char *red, unsigned char *green, unsigned char *blue )
{
  *red   = CURVE_COLOR[0];
  *green = CURVE_COLOR[1];
  *blue  = CURVE_COLOR[2];
}

unsigned int get_def_degree()
{
  return DEF_DEGREE;
}

void set_def_degree( unsigned int val )
{
  DEF_DEGREE = val;
}

unsigned int get_default_num_steps()
{
  return DEF_NUM_STEPS;
}

void set_default_num_steps( unsigned int val )
{
  DEF_NUM_STEPS = val;
}

void set_hide_ctrl_polys( bool hide )
{
  HIDE_CTRL_POLYS = hide;
}

bool get_hide_ctrl_polys()
{
  return HIDE_CTRL_POLYS;
}
