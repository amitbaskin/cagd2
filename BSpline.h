// BSpline.h
#ifndef BSPLINE_H
#define BSPLINE_H

#include <vector>
#include "curve.h" // Assuming this contains point_vec and double_vec

class BSpline
{
  public:
      // Constructor
  BSpline( const point_vec &controlPoints, const double_vec &knots, int order );

  // Evaluate the B-spline at a given parameter value
  CAGD_POINT evaluate( double t ) const;

  private:
      // Control points for the B-spline
  point_vec controlPoints;

  // Knot vector for the B-spline
  double_vec knots;

  // Order of the B-spline
  int order;

  // Utility function to compute the basis function value
  double basisFunction( int i, int k, double t ) const;

  // Utility function to find the knot span index
  int findKnotSpan( double t ) const;

  // Update a control point
  void updateControlPoint( size_t index, const CAGD_POINT &newPoint );

};

#endif // BSPLINE_H
