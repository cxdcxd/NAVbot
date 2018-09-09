#ifndef _ROBOLAND_COMMAND_TOOLS_HH_
#define _ROBOLAND_COMMAND_TOOLS_HH_

#include <Eigen/Dense>

#include <array>

using namespace std;

namespace roboland {

namespace tools {

class BSpline
{
public:
  class Exception : public std::exception
  {
  private:
    string msg;
  public:
    Exception(const string msg):
      msg("Exception in spline: " + msg)
    {
    }


    const char *what() const noexcept override
    {
      return msg.data();
    }
  };

  typedef Eigen::Matrix<double, 1, 5> Coeffs;
  typedef Eigen::Matrix<double, 5, 1> CoeffsCompl;

  struct Point
  {
    Point() = default;
    Point(const double x,
          const double y);

    bool operator==(const BSpline::Point &other) const;
    double x;
    double y;
    double getSlope(const BSpline::Point &other) const;
  };

  struct Spline
  {
    Spline() = default;
    Spline(const array<Point, 3> &points,
           const double s_i_1);
    Spline(const array<BSpline::Point, 2> &init_point,
           const double s_i_1);

    double start_x;
    double end_x;
    array<Point, 3> points;
    array<double, 4> control_points;

    int isInRange(const double x) const;
    double scaleX(const double x) const;
    void calculateControlPoints(const double s_i_1);
    double getValue(const double x) const;
    double getDerValue(const double x) const;
    double get2DerValue(const double x) const;
    bool validSpline() const;
    string getRangeStr() const;
  };

  BSpline() = default;
  BSpline(const vector<BSpline::Point> &initial_points,
               const float min_thr=INFINITY,
               const float max_thr=INFINITY,
               const float slope_thr=INFINITY);
  BSpline::Spline createSpline(const array<Point, 4> &points);

  int addPoint(const Point &point);
  double getValue(const double x) const;
  double getDerValue(const double x) const;
  double get2DerValue(const double x) const;
  void erase(const double x);
  void eraseUpper(const double x);
  bool isInRange(const double x) const;
private:
  vector<Spline> splines;
  double min;
  double max;
  double min_thr;
  double max_thr;
  double slope_thr;

  static CoeffsCompl getCoeffsCompl(const double x);
  static CoeffsCompl getDerCoeffsCompl(const double x);
};

double radianRound(double theta);

} // namespace tools

} // namespace roboland

#endif /* _ROBOLAND_COMMAND_TOOLS_HH_ */
