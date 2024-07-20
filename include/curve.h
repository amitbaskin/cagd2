#pragma once

#include <sstream>
#include <vector>
#include "cagd.h"

#define IS_DEBUG 1
#define K_NOT_USED -1


enum class CurveType
{
  CURVE_TYPE_NONE    = 0,
  CURVE_TYPE_BEZIER  = 1,
  CURVE_TYPE_BSPLINE = 2
};

typedef std::vector< CAGD_POINT > point_vec;
typedef std::vector< double > double_vec;
typedef std::vector< int > int_vec;


class Curve
{
public:
  Curve();
  Curve( int order_, point_vec ctrl_pnts_ );

  virtual void show_crv() const = 0;
  virtual bool is_miss_ctrl_pnts() const = 0;
  virtual CAGD_POINT evaluate( double param ) const = 0;

  virtual void print() const;
  void show_ctrl_poly();
  void add_ctrl_pnt( std::istringstream &line );

  int order_;
  point_vec ctrl_pnts_;
  mutable int_vec seg_ids_;
  GLubyte color_[ 3 ];
  int     poly_seg_id_;
};

void load_curves( int dummy1, int dummy2, void *p_data );

void clean_all_curves();
void clean_cur_curves_vec();

void print_err( char *str );

void redraw_all_curves();

void map_seg_to_crv( int seg_id, Curve *p_curve );
void map_pnt_to_crv_ctrl( int pnt_id, Curve *p_curve, int ctrl_idx );

std::tuple< Curve *, int > get_pnt_crv_ctrl( int pnt_id );
Curve *get_seg_crv( int seg_id );
