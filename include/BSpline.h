#pragma once

#include <vector>
#include "cagd.h"
#include "Curve.h"

#define DEF_ORDER 4

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

  virtual void dump( const std::string &path ) const;
  void dumpKnots( std::ofstream &ofs ) const;

  virtual CAGD_POINT evaluate( double t ) const;

  virtual void show_ctrl_poly();
  virtual bool show_crv( int chg_ctrl_idx = K_NOT_USED,
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

  void makeUniformKnotVector( bool use = false, double mn = 0.0, double mx = 1.0 );
  void interpolateEnd();
  void makeOpenKnotVector();

  int findKnotSpan( double t ) const;

  void evaluateBasisFunctions( int span, double t, double *N ) const;

  std::vector< int > findAffectedSegments( int controlPointIndex ) const;

  virtual double get_dom_start() const;
  virtual double get_dom_end() const;

  virtual void connectC0_bezier( const Bezier *other );
  virtual void connectC1_bezier( const Bezier *other );
  virtual void connectG1_bezier( const Bezier *other );

  void adjustForContinuity( const BSpline *other, bool isG1 );
  virtual void connectC0_bspline( const BSpline *bspline );
  virtual void connectC1_bspline( const BSpline *bspline );
  virtual void connectG1_bspline( const BSpline *bspline );

  void addKnot( double new_knot );
  void rmvKnot( int knot_idx );
  void updateKnot( int knot_idx, double new_param );
  void ensureNonDecreasingKnotVector();
  void updateUniqueKnotsAndMultiplicity();
  CAGD_POINT interpolate( const CAGD_POINT &P1, const CAGD_POINT &P2, double t ) const;
  double distance( const CAGD_POINT &P1, const CAGD_POINT &P2 ) const;

  const char *getKnotsDescription() const;
  bool parseKnotsDescription( const std::string &description );

  double_vec knots_;
  double_vec u_vec_;
  double_vec multiplicity_;
  bool is_uni_;
  bool is_open_;
};
