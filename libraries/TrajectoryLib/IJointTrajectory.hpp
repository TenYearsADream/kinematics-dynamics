// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_JOINT_TRAJECTORY_HPP__
#define __I_JOINT_TRAJECTORY_HPP__

#include <vector>

#include "ITrajectory.hpp"

namespace roboticslab
{

/**
 * @ingroup TrajectoryLib
 * @brief Base class for a Joint trajectory.
 */
class IJointTrajectory : public ITrajectory
{
public:
    //! Lists available Joint paths.
    enum joint_path
    {
        LINE        ///< A straight line
    };
    //! Lists available Joint velocity profiles.
    enum joint_velocity_profile
    {
        TRAPEZOIDAL        ///< A trapezoidal velocity profile
    };

    /**
     * @brief Get trajectory total duration in seconds
     *
     * @param duration Trajectory total duration in seconds.
     *
     * @return true on success, false otherwise
     */
    virtual bool getDuration(double* duration) const = 0;

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
    virtual bool getPosition(double movementTime, std::vector<double>& position) = 0;

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
    virtual bool getVelocity(double movementTime, std::vector<double>& velocity) = 0;

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
    virtual bool getAcceleration(double movementTime, std::vector<double>& acceleration) = 0;

    /**
     * @brief Set trajectory total duration in seconds
     *
     * @param duration Trajectory total duration in seconds.
     *
     * @return true on success, false otherwise
     */
    virtual bool setDuration(double duration) = 0;

    /**
     * @brief Set maximum velocity of the trajectory
     *
     * @param maxVelocity The maximum velocity permitted (degrees/second).
     *
     * @return true on success, false otherwise
     */
    virtual bool setMaxVelocity(double maxVelocity) = 0;

    /**
     * @brief Set maximum acceleration of the trajectory
     *
     * @param maxAcceleration The maximum acceleration permitted (degrees/second^2).
     *
     * @return true on success, false otherwise
     */
    virtual bool setMaxAcceleration(double maxAcceleration) = 0;

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
                             const std::vector<double>& waypointAcceleration = std::vector<double>()) = 0;

    /**
     * @brief Configure the type of Joint path upon creation
     *
     * @param pathType Use a \ref joint_path to define the type of Joint path
     *
     * @return true on success, false otherwise
     */
    virtual bool configurePath(int pathType) = 0;

    /**
     * @brief Configure the type of Joint velocity profile upon creation
     *
     * @param velocityProfileType Use a \ref joint_velocity_profile to define the type of Joint velocity profile
     *
     * @return true on success, false otherwise
     */
    virtual bool configureVelocityProfile(int velocityProfileType) = 0;

    /** @brief Create the trajectory
     *
     * @return true on success, false otherwise
     */
    virtual bool create() = 0;

    /** @brief Destroy the trajectory
     *
     * @return true on success, false otherwise
     */
    virtual bool destroy() = 0;

    /** Destructor */
    virtual ~IJointTrajectory() {}
};

}  // namespace roboticslab

#endif  // __I_JOINT_TRAJECTORY_HPP__
