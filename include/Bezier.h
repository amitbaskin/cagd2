#pragma once

#include <vector>
#include "cagd.h"
#include "Curve.h"

class Bezier : public Curve
{
public:
  Bezier(){}

  Bezier( int order,
          const point_vec &ctrl_pnts ) :
    Curve( order, ctrl_pnts) {}

  virtual CAGD_POINT evaluate( GLdouble t ) const;

  virtual void show_crv( int chg_ctrl_idx = K_NOT_USED,
                         CtrlOp op = CtrlOp::NONE ) const;

  virtual void print() const;
  virtual void add_ctrl_pnt( CAGD_POINT &ctrl_pnt, int idx );
  virtual void rmv_ctrl_pnt( int idx );

  virtual bool is_miss_ctrl_pnts() const
  {
    return ctrl_pnts_.size() < ( size_t )order_;
  }

  void computeMP() const;
  void calculateMatrixM( std::vector<std::vector<GLdouble>> &M ) const;

private:
  mutable std::vector< CAGD_POINT > MP_cache_;
};
