#include <gtest/gtest.h>
#include <ros/console.h>

#include <vector>

using namespace std;

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

#ifdef ROBOLAND_DEBUG
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif

  return RUN_ALL_TESTS();
}
