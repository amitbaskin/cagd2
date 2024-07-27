#pragma once

#define NUM_SAMPS 2000

const unsigned char *get_curve_color();
void set_curve_color( unsigned char new_curve_color[ 3 ] );
void get_curve_color( unsigned char *red, unsigned char *green, unsigned char *blue );
unsigned int get_def_degree();
void set_def_degree( unsigned int val );
unsigned int get_default_num_steps();
void set_default_num_steps( unsigned int val );
