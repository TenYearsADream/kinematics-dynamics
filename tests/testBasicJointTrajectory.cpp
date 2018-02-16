#include "gtest/gtest.h"

#include <cmath>
#include <vector>

#include "BasicJointTrajectory.hpp"

namespace roboticslab
{

/**
 * @brief Tests \ref KdlTrajectory.
 */
class BasicJointTrajectoryTest : public testing::Test
{
public:
    virtual void SetUp()
    {
        iJointTrajectory = new BasicJointTrajectory;
    }

    virtual void TearDown()
    {
        delete iJointTrajectory;
        iJointTrajectory = 0;
    }

protected:
    IJointTrajectory* iJointTrajectory;
};

TEST_F(BasicJointTrajectoryTest, BasicJointTrajectoryLine)
{
    //-- Create line trajectory
    ASSERT_TRUE( iJointTrajectory->setDuration(20.0) );  // Must be high, or may be expanded due to defaults
    std::vector<double> x(6);
    x[0] = 0;  // x
    x[1] = 0;  // y
    x[2] = 0;  // z
    x[3] = 0;  // o(x)
    x[4] = 0;  // o(y)
    x[5] = 0;  // o(z)
    std::vector<double> xd(6);
    xd[0] = 1;  // x
    xd[1] = 0;  // y
    xd[2] = 0;  // z
    xd[3] = 0;  // o(x)
    xd[4] = 0;  // o(y)
    xd[5] = 0;  // o(z)
    ASSERT_TRUE( iJointTrajectory->addWaypoint(x) );
    ASSERT_TRUE( iJointTrajectory->addWaypoint(xd) );
    ASSERT_TRUE( iJointTrajectory->configurePath( IJointTrajectory::LINE ) );
    ASSERT_TRUE( iJointTrajectory->configureVelocityProfile( IJointTrajectory::TRAPEZOIDAL ) );
    ASSERT_TRUE( iJointTrajectory->create() );

    //-- Use line
    double duration;
    ASSERT_TRUE( iJointTrajectory->getDuration(&duration) );
    ASSERT_EQ(duration, 20.0);

    //-- Destroy line
    ASSERT_TRUE( iJointTrajectory->destroy() );
}

}  // namespace roboticslab
