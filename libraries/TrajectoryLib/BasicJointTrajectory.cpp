// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicJointTrajectory.hpp"

#include <ColorDebug.hpp>

// -----------------------------------------------------------------------------

roboticslab::BasicJointTrajectory::BasicJointTrajectory()
{}

// -----------------------------------------------------------------------------

bool roboticslab::BasicJointTrajectory::getDuration(double* duration) const
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicJointTrajectory::getPosition(const double movementTime, std::vector<double>& position)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicJointTrajectory::getVelocity(const double movementTime, std::vector<double>& velocity)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicJointTrajectory::getAcceleration(const double movementTime, std::vector<double>& acceleration)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicJointTrajectory::setDuration(const double duration)
{
    this->duration = duration;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicJointTrajectory::addWaypoint(const std::vector<double>& waypoint,
                         const std::vector<double>& waypointVelocity,
                         const std::vector<double>& waypointAcceleration)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicJointTrajectory::configurePath(const int pathType)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicJointTrajectory::configureVelocityProfile(const int velocityProfileType)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicJointTrajectory::create()
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicJointTrajectory::destroy()
{
    return true;
}

// -----------------------------------------------------------------------------
