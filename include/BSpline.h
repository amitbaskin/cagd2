#pragma once

#include <vector>
#include "cagd.h"
#include "Curve.h"

class Bezier;

enum class UniKnots
{
  NONE = 0,
  FLOAT = 1,
  OPEN = 2
};

enum class EndKnots
{
  NONE = 0,
  FLOAT = 1,
  OPEN = 2
};

class BSpline : public Curve
{
public:
  BSpline::BSpline() :
    uni_type_( UniKnots::NONE ),
    end_type_( EndKnots::NONE )
  {}

  BSpline::BSpline( int order,
                    const point_vec &ctrl_pnts,
                    const double_vec &knots ) :
    Curve( order, ctrl_pnts),
    knots_( knots ),
    uni_type_( UniKnots::NONE ),
    end_type_( EndKnots::NONE )
  {}

  virtual CAGD_POINT evaluate( double t ) const;

  virtual void show_crv( int chg_ctrl_idx = K_NOT_USED,
                         CtrlOp op = CtrlOp::NONE ) const;

  virtual void print() const;
  virtual void add_ctrl_pnt( CAGD_POINT &ctrl_pnt, int idx );
  virtual void rmv_ctrl_pnt( int idx );
  void show_crv_helper( std::vector< int > u_vec ) const;

  virtual bool is_miss_ctrl_pnts() const
  {
    return ctrl_pnts_.size() < knots_.size() - order_;
  }

  int findKnotSpan( double t ) const;

  void evaluateBasisFunctions( int span, double t, double *N ) const;

  std::vector< int > findAffectedSegments( int controlPointIndex ) const;

  void generateUniformFloatingKnotVector();

  virtual void connectC0_bezier( const Bezier &other );
  virtual void connectC1_bezier( const Bezier &other );
  virtual void connectG1_bezier( const Bezier &other );

  virtual void connectC0_bspline( const BSpline &bspline );
  virtual void connectC1_bspline( const BSpline &bspline );
  virtual void connectG1_bspline( const BSpline &bspline );

  double_vec knots_;
  double_vec u_vec_;
  UniKnots uni_type_;
  EndKnots end_type_;
};
