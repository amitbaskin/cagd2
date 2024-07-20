#pragma once

#include <vector>
#include "cagd.h"
#include "Curve.h"

class Bezier : public Curve
{
public:
  Bezier(){}

  virtual CAGD_POINT evaluate( GLdouble t ) const;
  virtual void show_crv( int chg_ctrl_idx = K_NOT_USED ) const;
  virtual void print() const;

  virtual bool is_miss_ctrl_pnts() const
  {
    return ctrl_pnts_.size() < ( size_t )order_;
  }

  void computeMP() const;
  void calculateMatrixM( std::vector<std::vector<GLdouble>> &M ) const;

private:
  mutable std::vector<CAGD_POINT> MP_cache_;
};
