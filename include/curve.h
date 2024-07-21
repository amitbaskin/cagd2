#pragma once

#include <sstream>
#include <vector>
#include "cagd.h"

#define IS_DEBUG 1
#define K_NOT_USED -1

enum class CtrlOp
{
  NONE = 0,
  ADD = 1,
  RMV = 2
};

enum class CurveType
{
  CURVE_TYPE_NONE = 0,
  CURVE_TYPE_BEZIER = 1,
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

  virtual void show_crv( int chg_ctrl_idx = K_NOT_USED,
                         CtrlOp op = CtrlOp::NONE ) const = 0;

  virtual bool is_miss_ctrl_pnts() const = 0;
  virtual CAGD_POINT evaluate( double param ) const = 0;

  virtual void add_ctrl_pnt( CAGD_POINT &ctrl_pnt, int idx );
  virtual void rmv_ctrl_pnt( int idx );
  virtual void print() const;

  void show_ctrl_poly();
  void add_ctrl_pnt( std::istringstream &line );

  int order_;
  point_vec ctrl_pnts_;
  mutable int_vec seg_ids_;
  mutable int_vec pnt_ids_;
  GLubyte color_[ 3 ];
  int     poly_seg_id_;
};
