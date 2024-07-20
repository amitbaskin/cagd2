#pragma once

#define NUM_SAMPS 2000

// user controlled through GUI
static unsigned int DEF_NUM_STEPS = NUM_SAMPS;
static unsigned int DEF_DEGREE = 3;
static unsigned char CURVE_COLOR[ 3 ] = { 255, 0 , 0 };

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

unsigned int get_def_degree()
{
  return DEF_DEGREE;
}

void set_def_degree( unsigned int val  )
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
