#include <yandy/Robot.hpp>

int main()
{
    yandy::core::init_logging();

    yandy::Robot robot;
    robot.start();

    return 0;
}
