#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include <vector>
#include "cagd.h"

class Bezier
{
public:
  Bezier(){}

  Bezier( const std::vector<CAGD_POINT> &points );

  void print();

  CAGD_POINT evaluate( GLdouble t ) const;
  void addControlPoint( const CAGD_POINT &new_point );
  void removeControlPoint( size_t index );
  void updateControlPoint( size_t index, const CAGD_POINT &new_point );

  UINT order;
  std::vector<CAGD_POINT> ctrl_pnts;
  mutable std::vector<CAGD_POINT> MP_cache;

  void computeMP() const;
  void constructMatrixM( std::vector<std::vector<GLdouble>> &M ) const;
  void calculateMatrixM( std::vector<std::vector<GLdouble>> &M ) const; // New method
};

#endif // BEZIER_CURVE_H
