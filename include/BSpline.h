#pragma once

#include <vector>
#include "cagd.h"
#include "Curve.h"

class BSpline : public Curve
{
public:
  BSpline::BSpline(){}

  BSpline::BSpline( int order,
                    const point_vec &ctrl_pnts,
                    const double_vec &knots ) :
    Curve( order, ctrl_pnts),
    knots_( knots ) {}

  virtual CAGD_POINT evaluate( double t ) const;

  virtual void show_crv( int chg_ctrl_idx = K_NOT_USED,
                         CtrlOp op = CtrlOp::NONE ) const;

  virtual void print() const;
  virtual void add_ctrl_pnt( CAGD_POINT &ctrl_pnt, int idx );
  virtual void rmv_ctrl_pnt( int idx );
  void show_crv_helper( std::vector< int > u_vec ) const;

  virtual bool is_miss_ctrl_pnts() const
  {
    return ctrl_pnts_.size() < knots_.size() - order_;
  }

  int findKnotSpan( double t ) const;

  void evaluateBasisFunctions( int span, double t, double *N ) const;

  std::vector< int > findAffectedSegments( int controlPointIndex ) const;

  double_vec knots_;
  double_vec u_vec_;
};
