#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include <vector>
#include <unordered_map>
#include "cagd.h" // Include for CAGD_POINT

typedef std::vector<CAGD_POINT> point_vec;

class BezierCurve
{
  public:
      // Constructor with a vector of control points
  BezierCurve( const point_vec &controlPoints );

  // Method to evaluate the curve at a given parameter value t (0 <= t <= 1)
  CAGD_POINT evaluate( double t ) const;

  // Method to update a single control point and recalculate if necessary
  void updateControlPoint( int index, const CAGD_POINT &newPoint );

  private:
  point_vec controlPoints;
  mutable std::unordered_map<int, double> basisMatrixCache;

  // Method to compute binomial coefficients
  double binomialCoefficient( int n, int k ) const;

  // Method to compute the Bernstein polynomial
  double bernsteinPolynomial( int i, int n, double t ) const;

  // Method to compute and cache the basis matrix
  void computeBasisMatrix() const;
};

#endif // BEZIER_CURVE_H
