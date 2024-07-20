#pragma once

#include <vector>
#include "cagd.h"
#include "Curve.h"

class BSpline : public Curve
{
public:
  BSpline::BSpline(){}

  BSpline::BSpline( const point_vec &ctrl_pnts_, const double_vec &knots_, int order_ )
    : knots_( knots_ )
  {}

  virtual CAGD_POINT evaluate( double t ) const;
  virtual void show_crv() const;
  virtual void print() const;

  virtual bool is_miss_ctrl_pnts() const
  {
    return ctrl_pnts_.size() < knots_.size() - order_;
  }

  // Utility method to find the knot span index
  int findKnotSpan( double t ) const;

  // Utility method to evaluate basis functions
  void evaluateBasisFunctions( int span, double t, double *N ) const;

  double_vec knots_;
  double_vec u_vec_;
};
