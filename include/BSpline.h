#ifndef BSPLINE_H
#define BSPLINE_H

#include <vector>
#include "cagd.h" // Ensure this is included properly

typedef std::vector<CAGD_POINT> point_vec;
typedef std::vector<double> double_vec;

class BSpline
{
public:
  BSpline::BSpline()
  {}

  BSpline::BSpline( const point_vec &ctrl_pnts, const double_vec &knots, int order )
    : order( order ), ctrl_pnts( ctrl_pnts ), knots( knots )
  {}

// Method to evaluate the B-spline at parameter t
  CAGD_POINT evaluate( double t ) const;

  // Utility method to find the knot span index
  int findKnotSpan( double t ) const;

  // Utility method to evaluate basis functions
  void evaluateBasisFunctions( int span, double t, double *N ) const;

  void print();

  int order;
  point_vec ctrl_pnts;
  double_vec knots;
};

#endif // BSPLINE_H