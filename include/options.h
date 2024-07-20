#pragma once

#define NUM_SAMPS 2000

// user controlled through GUI
extern unsigned int DEF_NUM_STEPS;
extern unsigned int DEF_DEGREE;
extern unsigned char CURVE_COLOR[ 3 ];

const unsigned char *get_curve_color();
void set_curve_color( unsigned char new_curve_color[ 3 ] );
unsigned int get_def_degree();
void set_def_degree( unsigned int val );
unsigned int get_default_num_steps();
void set_default_num_steps( unsigned int val );
