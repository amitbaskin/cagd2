#pragma once

#include <vector>
#include "cagd.h"
#include "Curve.h"

class Bezier;

class BSpline : public Curve
{
public:
  BSpline::BSpline() :
    is_uni_( false ),
    is_open_( false )
  {}

  BSpline::BSpline( int order,
                    const point_vec &ctrl_pnts,
                    const double_vec &knots ) :
    Curve( order, ctrl_pnts),
    knots_( knots ),
    is_uni_( false ),
    is_open_( false )
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

  void update_u_vec();

  void makeUniformKnotVector();
  void makeOpenKnotVector();

  int findKnotSpan( double t ) const;

  void evaluateBasisFunctions( int span, double t, double *N ) const;

  std::vector< int > findAffectedSegments( int controlPointIndex ) const;

  virtual void connectC0_bezier( const Bezier &other );
  virtual void connectC1_bezier( const Bezier &other );
  virtual void connectG1_bezier( const Bezier &other );

  virtual void connectC0_bspline( const BSpline &bspline );
  virtual void connectC1_bspline( const BSpline &bspline );
  virtual void connectG1_bspline( const BSpline &bspline );

  void addKnot( double new_knot );
  void removeKnot( double knot );
  void onKnotDrag( int knot_index, const CAGD_POINT &new_position );
  void ensureNonDecreasingKnotVector();
  void updateUniqueKnotsAndMultiplicity();
  CAGD_POINT interpolate( const CAGD_POINT &P1, const CAGD_POINT &P2, double t ) const;
  double distance( const CAGD_POINT &P1, const CAGD_POINT &P2 ) const;
  double mapPositionToParam( const CAGD_POINT &position ) const;
  CAGD_POINT mapParamToPosition( double param ) const;

  double_vec knots_;
  double_vec u_vec_;
  double_vec multiplicity_;
  bool is_uni_;
  bool is_open_;
};
