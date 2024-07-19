#ifndef BSPLINE_H
#define BSPLINE_H

#include <vector>
#include "cagd.h" // Ensure this is included properly

typedef std::vector<CAGD_POINT> point_vec;
typedef std::vector<double> double_vec;

class BSpline
{
public:
  BSpline::BSpline( const point_vec &controlPoints, const double_vec &knots, int order )
    : order( order ), controlPoints( controlPoints ), knots( knots )
  {}

// Method to evaluate the B-spline at parameter t
  CAGD_POINT evaluate( double t ) const;

private:
  int order;
  point_vec controlPoints;
  double_vec knots;

  // Utility method to find the knot span index
  int findKnotSpan( double t ) const;

  // Utility method to evaluate basis functions
  void evaluateBasisFunctions( int span, double t, double *N ) const;
};

#endif // BSPLINE_H
