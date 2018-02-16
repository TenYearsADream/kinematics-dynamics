// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __BASIC_JOINT_TRAJECTORY_HPP__
#define __BASIC_JOINT_TRAJECTORY_HPP__

#include <vector>

#include "IJointTrajectory.hpp"

namespace roboticslab
{

/**
 * @ingroup TrajectoryLib
 * @brief Base class for a Joint trajectory.
 */
class BasicJointTrajectory : public IJointTrajectory
{
public:

    BasicJointTrajectory();

    /**
     * @brief Get trajectory total duration in seconds
     *
     * @param duration Trajectory total duration in seconds.
     *
     * @return true on success, false otherwise
     */
    virtual bool getDuration(double* duration) const;

    /**
     * @brief Joint position of the trajectory at a specific instant in time
     *
     * @param movementTime Time in seconds since start.
     * @param position n-element vector describing a position in Joint space;
     * where n is the number of joints. Rotational joint position in (degrees),
     * Prismatical joint position in (meters).
     *
     * @return true on success, false otherwise
     */
    virtual bool getPosition(const double movementTime, std::vector<double>& position);

    /**
     * @brief Joint velocity of the trajectory at a specific instant in time
     *
     * @param movementTime Time in seconds since start.
     * @param velocity n-element vector describing a velocity in Joint space;
     * where n is the number of joints. Rotational joint velocity in (degrees/second),
     * Prismatical joint velocity in (meters/second).
     *
     * @return true on success, false otherwise
     */
    virtual bool getVelocity(const double movementTime, std::vector<double>& velocity);

    /**
     * @brief Joint acceleration of the trajectory at a specific instant in time
     *
     * @param movementTime Time in seconds since start.
     * @param acceleration n-element vector describing a acceleration in Joint space;
     * where n is the number of joints. Rotational joint acceleration in (degrees/second^2),
     * Prismatical joint acceleration in (meters/second^2).
     *
     * @return true on success, false otherwise
     */
    virtual bool getAcceleration(const double movementTime, std::vector<double>& acceleration);

    /**
     * @brief Set trajectory total duration in seconds
     *
     * @param duration Trajectory total duration in seconds.
     *
     * @return true on success, false otherwise
     */
    virtual bool setDuration(const double duration);

    /**
     * @brief Add a waypoint to the trajectory
     *
     * @param waypoint Position information of a Joint waypoint, n-element vector describing a
     * position in Joint space; where n is the number of joints. Rotational joint position in (degrees),
     * Prismatical joint position in (meters).
     * @param waypointVelocity Velocity information of a Joint waypoint, n-element vector describing a
     * velocity in Joint space; where n is the number of joints. Rotational joint velocity in (degrees/second),
     * Prismatical joint velocity in (meters/second).
     * @param waypointAcceleration Acceleration information of a Joint waypoint, n-element vector describing a
     * acceleration in Joint space; where n is the number of joints. Rotational joint acceleration in (degrees/second^2),
     * Prismatical joint acceleration in (meters/second^2).
     *
     * @return true on success, false otherwise
     */
    virtual bool addWaypoint(const std::vector<double>& waypoint,
                             const std::vector<double>& waypointVelocity = std::vector<double>(),
                             const std::vector<double>& waypointAcceleration = std::vector<double>());

    /**
     * @brief Configure the type of Joint path upon creation
     *
     * @param pathType Use a \ref joint_path to define the type of Joint path
     *
     * @return true on success, false otherwise
     */
    virtual bool configurePath(const int pathType);

    /**
     * @brief Configure the type of Joint velocity profile upon creation
     *
     * @param velocityProfileType Use a \ref joint_velocity_profile to define the type of Joint velocity profile
     *
     * @return true on success, false otherwise
     */
    virtual bool configureVelocityProfile(const int velocityProfileType);

    /** @brief Create the trajectory
     *
     * @return true on success, false otherwise
     */
    virtual bool create();

    /** @brief Destroy the trajectory
     *
     * @return true on success, false otherwise
     */
    virtual bool destroy();

    /** Destructor */
    virtual ~BasicJointTrajectory() {}

private:

    double duration;

};

}  // namespace roboticslab

#endif  // __BASIC_JOINT_TRAJECTORY_HPP__
