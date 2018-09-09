#include <tools/tools.hh>

#include <ros/console.h>

namespace roboland {

namespace tools {

#define EPS 1E-5

BSpline::Point::Point(const double x,
                      const double y):
  x(x),
  y(y)
{}

bool BSpline::Point::operator==(const BSpline::Point &other) const
{
  return (fabs(this->x - other.x) < EPS &&
          fabs(this->y - other.y) < EPS);
}

double BSpline::Point::getSlope(const BSpline::Point &other) const
{
  return (other.y - this->y) / (other.x - this->x);
}

BSpline::Spline::Spline(const array<BSpline::Point, 3> &points,
                             const double s_i_1):
  start_x(points[0].x),
  end_x(points[1].x),
  points(points)
{
  this->calculateControlPoints(s_i_1);
}

/// for the last spline construction
BSpline::Spline::Spline(const array<BSpline::Point, 2> &init_points,
                        const double s_i_1):
  start_x(init_points[0].x),
  end_x(init_points[1].x)
{
  this->points[0] = init_points[0];
  this->points[1] = init_points[0];
  this->points[2] = init_points[1];

  this->control_points[0] = s_i_1;
  this->control_points[1] = (2 * init_points[0].y + init_points[1].y) / 3;
  this->control_points[2] = (init_points[0].y + 2 * init_points[1].y) / 3;
  this->control_points[3] = init_points[1].y;
}

void BSpline::Spline::calculateControlPoints(const double s_i_1)
{
  this->control_points[0] = s_i_1;
  this->control_points[1] = (2 * this->points[0].y + this->points[1].y) / 3;
  this->control_points[2] = (this->points[0].y + 2 * this->points[1].y) / 3;
  this->control_points[3] = (this->points[0].y + 4 * this->points[1].y + this->points[2].y) / 6;
}

int BSpline::Spline::isInRange(const double x) const
{
  if (x < this->start_x)
    return -1;
  if (x > this->end_x)
    return 1;

  return 0;
}

double BSpline::Spline::scaleX(const double x) const
{
  // If we have a invalid spline which end_x and start_x were
  // relatively near together we must just send 0
  auto dem = this->end_x - this->start_x;
  if (dem > EPS)
    return (x - this->start_x) / (this->end_x - this->start_x);
  else
    return 0;
}

double BSpline::Spline::getValue(const double x) const
{
  vector<double> ps;

  std::copy(this->control_points.begin(), this->control_points.end(), std::back_inserter(ps));

  auto temp_ps = ps;

  auto scaled_x = this->scaleX(x);

  while (ps.size() > 1)
  {
    temp_ps.clear();
    for (unsigned int i = 0; i < ps.size() - 1; ++i)
    {
      auto val = (1 - scaled_x) * ps[i] + scaled_x * ps[i + 1];
      temp_ps.push_back(val);
    }
    ps = temp_ps;
  }

  return ps[0];
}

double BSpline::Spline::getDerValue(const double x) const
{
  auto &ps = this->control_points;

  auto scaled_x = this->scaleX(x);

  return 3 * pow((1 - scaled_x), 2) * (ps[1] - ps[0]) + \
    6 * (1 - scaled_x) * scaled_x * (ps[2] - ps[1]) + \
    3 * pow(scaled_x, 2) * (ps[3] - ps[2]);
}

double BSpline::Spline::get2DerValue(const double x) const
{
  auto &ps = this->control_points;

  auto scaled_x = this->scaleX(x);

  return 6 * (1 - scaled_x) * (ps[2] - 2 * ps[1] + ps[0]) + \
    6 * x * (ps[3] - 2 * ps[2] + ps[1]);
}

bool BSpline::Spline::validSpline() const
{
  if (fabs(this->start_x - this->end_x) < 3 * EPS) // EPS distance between each point
    return false;

  return true;
}

string BSpline::Spline::getRangeStr() const
{
  return std::to_string(this->start_x) + "-" + std::to_string(this->end_x);
}

BSpline::BSpline(const vector<BSpline::Point> &initial_points,
                           const float min_thr,
                           const float max_thr,
                           const float slope_thr):
  min_thr(min_thr),
  max_thr(max_thr),
  slope_thr(slope_thr)
{
  if (initial_points.size() < 2)
  {
    ROS_ERROR("At least 2 points required for constructing a bezier spline.");

    return;
  }

  auto start_index = 0u;
  while (initial_points[start_index] == initial_points[start_index + 1])
  {
    ++start_index;
  }

  this->splines.push_back(BSpline::Spline(array<BSpline::Point, 2>({initial_points[start_index], initial_points[start_index + 1]}),
                                          initial_points[start_index].y));
  this->min = initial_points[start_index].x;
  this->max = initial_points[start_index + 1].x;

  for (size_t i = start_index + 2; i < initial_points.size(); ++i)
  {
    this->addPoint(initial_points[i]);
  }
}

int BSpline::addPoint(const BSpline::Point &cpoint)
{
  if (this->splines.size() == 0)
  {
    // This situation could happen for example all splines deleted
    // previously. this is an invalid spline we just create it to able
    // to construct a valid spline further.
    this->splines.push_back(BSpline::Spline(array<BSpline::Point, 2>({cpoint, cpoint}),
                                            cpoint.y));

    return 1;
  }

  auto last_spline = this->splines.back();

  if (last_spline.isInRange(cpoint.x) != 1)
  {
    throw(Exception("addPoint: Want to add point " +                    \
                    std::to_string(cpoint.x) + " ," + std::to_string(cpoint.y) + \
                    " which is not upper than last_spline exist " +            \
                    last_spline.getRangeStr()));

    return -1;
  }

  this->splines.pop_back();

  if (!last_spline.validSpline())
  {
    // For example when we just have a spline which is constructed
    // using just a point, other situations currently do not could
    // imagine.
    this->splines.push_back(BSpline::Spline(array<BSpline::Point, 2>({last_spline.points[1], cpoint}),
                                            last_spline.points[0].y));
    return 1;
  }

  const auto &points = last_spline.points;

  // first spline get initial point as initial control point
  double last_control_point = this->splines.size() ? this->splines.back().control_points[3] : points[0].y;

  auto point = cpoint;
  auto slope = points.back().getSlope(point);

  if (!std::isinf(this->min_thr) && !std::isinf(this->max_thr))
  {
    if (slope < -this->slope_thr)
    {
      do {
        point.y = this->max_thr + (point.y - this->min_thr);
      } while (point.y < this->min_thr);
    }
    else if (slope > this->slope_thr)
    {
      do {
        point.y = this->min_thr - (this->max_thr - point.y);
      } while (point.y > this->max_thr);
    }
  }

  this->splines.push_back(BSpline::Spline({points[1], points[2], point}, last_control_point));

  // Always we add a spline that connect the last point(currently
  // adding point) to part of the trajectory
  this->splines.push_back(BSpline::Spline(array<BSpline::Point, 2>({points[2], point}),
                                          this->splines.back().control_points[3]));

  if (point.x > this->max)
    this->max = point.x;

  return 0;
}

double BSpline::getValue(const double x) const
{
  for (auto &spline : this->splines)
  {
    if (spline.isInRange(x) == 0)
    {
      auto value = spline.getValue(x);

      if (!std::isinf(this->min_thr) && !std::isinf(this->max_thr))
      {
        if (value <= this->min_thr)
        {
          value = this->max_thr - (this->min_thr - value);
        }
        else if (value > this->max_thr)
        {
          value = this->min_thr + (value - this->max_thr);
        }
      }
      return value;
    }
  }

  throw(Exception("getValue: Could not find any spline in range for " + \
                  std::to_string(x) +                                   \
                  " last spline exist: " +                              \
                  (this->splines.size() ?                                \
                   this->splines.back().getRangeStr() :                 \
                   "No spline exist.")));

  return 0.0;
}

double BSpline::getDerValue(const double x) const
{
  for (auto &spline : this->splines)
  {
    if (spline.isInRange(x) == 0)
    {
      return spline.getDerValue(x);
    }
  }

  throw(Exception("getDerValue: Could not find any spline in range for " + \
                  std::to_string(x) +                                   \
                  " last spline exist: " +                              \
                  (this->splines.size() ?                               \
                   this->splines.back().getRangeStr() :                 \
                   "No spline exist.")));

  return 0.0;
}

double BSpline::get2DerValue(const double x) const
{
  for (auto &spline : this->splines)
  {
    if (spline.isInRange(x) == 0)
    {
      return spline.get2DerValue(x);
    }
  }

  throw(Exception("get2DerValue: Could not find any spline in range for " + \
                  std::to_string(x) +                                   \
                  " last spline exist: " +                              \
                  (this->splines.size() ?                               \
                   this->splines.back().getRangeStr() :                 \
                   "No spline exist.")));

  return 0.0;
}

void BSpline::erase(const double x)
{
  if (x < this->min)
  {
    throw(Exception("erase: Want to erase point " + \
                    std::to_string(x) +                    \
                    " which is less than minimum point " + \
                    std::to_string(this->min) +            \
                    " added to spline."));
    return;
  }

  this->min = x;
  if (this->max < this->min)
  {
    this->max = this->min;
    this->splines.clear();
    return;
  }

  auto it = this->splines.begin();

  while (it != this->splines.end() )
  {
    if (it->isInRange(x) == 1)
    {
      it = this->splines.erase(it);
    }
    else
    {
      // we keep the spline which this x belongs to it, we could
      // change the start_point of it to this given point, but it
      // seems completely unnecessary and inefficient work
      break;
    }
  }
}

void BSpline::eraseUpper(const double x)
{
  if (x > this->max)
  {
    throw(Exception("eraseUpper: Want to erase point " + \
                    std::to_string(x) +                     \
                    " which is upper than minimum point " + \
                    std::to_string(this->max) +             \
                    " added to spline."));
    return;
  }

  this->max = x;
  if (this->max < this->min)
  {
    this->min = this->max;
    this->splines.clear();
    return;
  }

  auto it = this->splines.begin();

  while (it != this->splines.end())
  {
    if (it->isInRange(x) == -1)
    {
      this->splines.erase(it, this->splines.end());
      break;
    }
    ++it;
  }

  if (this->splines.size() == 0)
    return;

  auto last_spline = this->splines.back();
  auto p = BSpline::Point(x, last_spline.getValue(x));
  this->splines.pop_back();
  if (this->splines.size() == 0)
  {
    this->splines.push_back(BSpline::Spline(array<BSpline::Point, 2>({last_spline.points[0], p}),
                                            last_spline.points[0].y));
    return;
  }

  last_spline = this->splines.back();
  this->splines.pop_back();

  if (fabs(last_spline.end_x - x) < EPS)
  {
    this->splines.push_back(BSpline::Spline(array<BSpline::Point, 2>({last_spline.points[0], last_spline.points[1]}),
                                            this->splines.back().control_points[3]));
  }
  else if (this->splines.size() == 0)
  {
    this->splines.push_back(BSpline::Spline(array<BSpline::Point, 2>({last_spline.points[0], p}),
                                            last_spline.points[0].y));
  }
  else
  {
    this->splines.push_back(BSpline::Spline({last_spline.points[0], last_spline.points[1], p},
                                            this->splines.back().control_points[3]));
    this->splines.push_back(BSpline::Spline(array<BSpline::Point, 2>({last_spline.points[1], p}),
                                            this->splines.back().control_points[3]));
  }
}

bool BSpline::isInRange(const double x) const
{
  return this->splines.size() && (this->min - x < EPS &&
                                  x - this->max < EPS);
}

double radianRound(double theta)
{
  while (theta <= -M_PI || theta > M_PI)
  {
    if (theta <= -M_PI)
    {
      theta += 2 * M_PI;
    }
    else if (theta > M_PI)
    {
      theta -= 2 * M_PI;
    }
  }

  return theta;
}

} // namespace tools

} // namespace roboland
