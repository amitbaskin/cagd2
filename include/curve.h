#pragma once

#include <vector>
#include "cagd.h"
#include "BezierCurve.h"
#include "BSpline.h"

#define IS_DEBUG 1
#define K_NOT_USED -1

enum class curve_type
{
  CURVE_TYPE_NONE    = 0,
  CURVE_TYPE_BEZIER  = 1,
  CURVE_TYPE_BSPLINE = 2
};

typedef std::vector<CAGD_POINT> point_vec;
typedef std::vector<double> double_vec;

class Curve
{
public:
  ~Curve()
  {
    if (bz_crv != nullptr)
      delete bz_crv;

    if (bs_crv != nullptr)
      delete bs_crv;
  }

  CAGD_POINT evaluate(double param);
  double get_end_param();

  curve_type   type;
  BezierCurve *bz_crv;
  BSpline     *bs_crv;
  UINT         order;
  point_vec    ctrl_pts;
  double_vec   knots;
  GLubyte      color[3];
  int          poly_id;
  int          curve_id;
};

void load_curve( int dummy1, int dummy2, void *p_data );
void save_curve( int dummy1, int dummy2, void *p_data );

void clean_all_curves();
void clean_cur_curves_vec();

void print_err( char *str );
void print_debug_curve_data( Curve *curve_data );

void show_curve( Curve *curve_data, bool redraw_ctrl_polyline );

void show_ctrl_pts_polyline( Curve *curve_data );

void show_curve_helper( Curve *curve_data );
void show_bspline_curve( Curve *curve_data );

void redraw_all_curves();

static double get_jump_sample_val( double start, double end, int num_pnts );