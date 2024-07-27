#pragma once

#include <sstream>
#include <vector>
#include "cagd.h"

#define IS_DEBUG 1
#define K_NOT_USED -1
#define DEF_START_DOM 0.0
#define DEF_END_DOM 1.0

class Bezier;
class BSpline;

enum class CtrlOp
{
  NONE = 0,
  ADD = 1,
  RMV = 2
};

typedef std::vector< CAGD_POINT > point_vec;
typedef std::vector< double > double_vec;
typedef std::vector< int > int_vec;

class Curve
{
public:
  Curve();
  Curve( int order_, point_vec ctrl_pnts_ );

  virtual void dump( std::ofstream &ofs ) const;
  void dumpOrder( std::ofstream &ofs ) const;
  void dumpControlPoints( std::ofstream &ofs ) const;
  void dumpPoint( std::ofstream &ofs, const CAGD_POINT &point ) const;

  virtual bool show_crv( int chg_ctrl_idx = K_NOT_USED,
                         CtrlOp op = CtrlOp::NONE ) const = 0;

  virtual void show_ctrl_poly();

  virtual bool is_miss_ctrl_pnts() const = 0;
  virtual CAGD_POINT evaluate( double param ) const = 0;

  virtual double get_dom_start() const { return 0.0; }
  virtual double get_dom_end() const { return 1.0; }

  virtual void connectC0_bezier( const Bezier *other ) = 0;
  virtual void connectC1_bezier( const Bezier *other ) = 0;
  virtual void connectG1_bezier( const Bezier *other ) = 0;

  virtual void connectC0_bspline( const BSpline *bspline ) = 0;
  virtual void connectC1_bspline( const BSpline *bspline ) = 0;
  virtual void connectG1_bspline( const BSpline *bspline ) = 0;

  void connect_tangents( const Curve *other, bool is_g1 );

  virtual void add_ctrl_pnt( CAGD_POINT &ctrl_pnt, int idx );
  virtual void rmv_ctrl_pnt( int idx );
  virtual void print() const;

  void clean_ctrl_poly();
  void hide_ctrl_poly();
  int get_pnt_id_idx( int pnt_id );
  void add_ctrl_pnt_from_str( std::istringstream &line );
  void update_weight( int pnt_idx, double val );
  void change_color( BYTE red, BYTE green, BYTE blue );

  int order_;
  point_vec ctrl_pnts_;
  mutable int_vec seg_ids_;
  mutable int_vec pnt_ids_;
  GLubyte color_[ 3 ];
  mutable int_vec poly_seg_ids_;
};
