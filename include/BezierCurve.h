#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include <vector>
#include "cagd.h" // For CAGD_POINT

// Define CAGD_POINT if not already defined in cagd.h
// typedef struct {
//   GLdouble x, y, z;
// } CAGD_POINT;

typedef std::vector<CAGD_POINT> point_vec;

class BezierCurve
{
  private:
  point_vec control_points;
  std::vector<std::vector<GLdouble>> base_matrix_cache;

  // Computes the Bernstein base matrix for the Bezier curve
  void computeBaseMatrix();

  // Helper function to compute binomial coefficient C(n, k)
  int binomialCoefficient( int n, int k ) const;

  public:
      // Constructor
  BezierCurve( const point_vec &points );

  // Evaluates the Bezier curve at parameter t
  CAGD_POINT evaluate( GLdouble t ) const;
};

#endif // BEZIER_CURVE_H
