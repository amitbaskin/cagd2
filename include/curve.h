#pragma once

#include <vector>
#include "cagd.h"
#include "BezierCurve.h"
#include "BSpline.h"

#define IS_DEBUG 1
#define K_NOT_USED -1

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
  curve_type   type;
  BezierCurve *bz_crv;
  BSpline     *bs_crv;
  UINT         order;
  point_vec    ctrl_pts;
  double_vec   knots;
  GLubyte      color[3];
  int          poly_id;
  int          curve_id;
} curve;

void load_curve( int dummy1, int dummy2, void *p_data );
void save_curve( int dummy1, int dummy2, void *p_data );

void clean_all_curves();
void clean_cur_curves_vec();

void free_curve_data( curve *curveData );
void print_err( char *str );
void print_debug_curve_data( curve *curve_data );

void show_curve( curve *curve_data, bool redraw_ctrl_polyline );

void show_ctrl_pts_polyline( curve *curve_data );

void show_bezier_curve( curve *curve_data );
void show_bspline_curve( curve *curve_data );

void redraw_all_curves();

static double get_jump_sample_val( double start, double end, int num_pnts );