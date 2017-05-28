// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::AmorCartesianControl::open(yarp::os::Searchable& config)
{
    CD_DEBUG("AmorCartesianControl config: %s.\n", config.toString().c_str());

    yarp::os::Value vHandle = config.find("handle");

    if (vHandle.isNull())
    {
        CD_INFO("Creating own AMOR handle.\n");
        ownsHandle = true;
        handle = amor_connect((char *)DEFAULT_CAN_LIBRARY, DEFAULT_CAN_PORT);
    }
    else
    {
        CD_INFO("Using external AMOR handle.\n");
        ownsHandle = false;
        handle = *reinterpret_cast<AMOR_HANDLE *>(vHandle.asBlob());
    }

    if (handle == AMOR_INVALID_HANDLE)
    {
        CD_ERROR("Could not get AMOR handle (%s)\n", amor_error());
        return false;
    }

    CD_SUCCESS("Acquired AMOR handle!\n");

    yarp::os::Property robotOptions;
    yarp::os::Value vHandle(handle, sizeof handle);

    robotOptions.put("device", "AmorControlboard");
    //robotOptions.put("handle", vHandle); // FIXME

    robotDevice.open(robotOptions);

    if (!robotDevice.isValid())
    {
        CD_ERROR("Robot device not valid.\n");
        return false;
    }

    if (!robotDevice.view(iEncoders))
    {
        CD_ERROR("Could not view iEncoders.\n");
        return false;
    }

    if (!robotDevice.view(iPositionControl))
    {
        CD_ERROR("Could not view iPositionControl.\n");
        return false;
    }

    if (!robotDevice.view(iVelocityControl))
    {
        CD_ERROR("Could not view iVelocityControl.\n");
        return false;
    }

    if (!robotDevice.view(iControlLimits))
    {
        CD_ERROR("Could not view iControlLimits.\n");
        return false;
    }

    if (!robotDevice.view(iTorqueControl))
    {
        CD_ERROR("Could not view iTorqueControl.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::close()
{
    CD_INFO("Closing...\n");

    if (handle != AMOR_INVALID_HANDLE)
    {
        amor_emergency_stop(handle);
    }

    if (ownsHandle)
    {
        amor_release(handle);
    }

    return true;
}

// -----------------------------------------------------------------------------
