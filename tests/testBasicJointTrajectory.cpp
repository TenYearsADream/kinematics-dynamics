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
    std::vector<double> q(2);
    q[0] = 0;
    q[1] = 0;
    std::vector<double> qd(2);
    qd[0] = 90;
    qd[1] = 180;
    ASSERT_TRUE( iJointTrajectory->addWaypoint(q) );
    ASSERT_TRUE( iJointTrajectory->addWaypoint(qd) );
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
