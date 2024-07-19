// BSpline.h
#ifndef BSPLINE_H
#define BSPLINE_H

#include "curve.h"
#include <vector>
#include <unordered_map>
#include <stdexcept>

class BSpline
{
  public:
  BSpline( const point_vec &controlPoints, const double_vec &knots, UINT order );

  // Evaluate the B-spline at a given parameter t
  CAGD_POINT evaluate( double t ) const;

  private:
  point_vec controlPoints;
  double_vec knots;
  UINT order;
  UINT degree;

  mutable std::unordered_map<double, CAGD_POINT> cache; // Cache for evaluated points

  // Helper functions for B-spline evaluation
  double basisFunction( size_t i, UINT k, double t ) const;
  CAGD_POINT computePoint( double t ) const;

  // Helper functions for point operations
  CAGD_POINT pointMultiply( const CAGD_POINT &p, double scalar ) const;
  CAGD_POINT pointAdd( const CAGD_POINT &p1, const CAGD_POINT &p2 ) const;
};

#endif // BSPLINE_H
