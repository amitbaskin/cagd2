#pragma once

#include <vector>
#include "cagd.h"

#define IS_DEBUG 1

enum curve_type
{
  CURVE_TYPE_NONE = 0,
  CURVE_TYPE_BEZIER,
  CURVE_TYPE_BSPLINE
};

typedef std::vector<CAGD_POINT> point_vec;
typedef std::vector<double> double_vec;

typedef struct
{
  curve_type type;
  UINT       order;
  point_vec  ctrl_pts;
  double_vec knots;
  GLubyte    color[3];
} curve;

void load_curve( int dummy1, int dummy2, void *p_data );
void save_curve( int dummy1, int dummy2, void *p_data );

void free_curve_data( curve *curveData );
void print_err( char *str );
void print_debug_curve_data( curve *curve_data );