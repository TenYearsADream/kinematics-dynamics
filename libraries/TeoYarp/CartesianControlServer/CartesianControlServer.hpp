// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_SERVER_HPP__
#define __CARTESIAN_CONTROL_SERVER_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <iostream> // only windows

#include "ICartesianControl.h"

namespace roboticslab
{

/**
 * @ingroup TeoYarp
 * \defgroup CartesianControlServer
 *
 * @brief Contains roboticslab::CartesianControlServer.
 */

class RpcResponder;
class StreamResponder;

/**
 * @ingroup CartesianControlServer
 * @brief The CartesianControlServer class implements ICartesianControl server side.
 */

class CartesianControlServer : public yarp::dev::DeviceDriver
{
public:

    CartesianControlServer() : iCartesianControl(NULL), rpcResponder(NULL), streamResponder(NULL)
    {}

    // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------

    /**
     * Open the DeviceDriver.
     * @param config is a list of parameters for the device.
     * Which parameters are effective for your device can vary.
     * See \ref dev_examples "device invocation examples".
     * If there is no example for your device,
     * you can run the "yarpdev" program with the verbose flag
     * set to probe what parameters the device is checking.
     * If that fails too,
     * you'll need to read the source code (please nag one of the
     * yarp developers to add documentation for your device).
     * @return true/false upon success/failure
     */
    virtual bool open(yarp::os::Searchable& config);

    /**
     * Close the DeviceDriver.
     * @return true/false on success/failure.
     */
    virtual bool close();

protected:

    yarp::os::RpcServer rpcServer;

    yarp::os::BufferedPort<yarp::os::Bottle> commandPort;

    yarp::dev::PolyDriver cartesianControlDevice;

    roboticslab::ICartesianControl *iCartesianControl;

    RpcResponder *rpcResponder;

    StreamResponder *streamResponder;
};

/**
 * @ingroup CartesianControlServer
 * @brief Responds to RPC command messages.
 */
class RpcResponder : public yarp::dev::DeviceResponder
{
public:

    RpcResponder(roboticslab::ICartesianControl *iCartesianControl)
    {
        this->iCartesianControl = iCartesianControl;
        this->makeUsage();
    }

    /**
     * Respond to a message.
     * @param in the message
     * @param out the response
     * @return true if there was no critical failure
     */
    virtual bool respond(const yarp::os::Bottle& in, yarp::os::Bottle& out);

    /**
     * Generate command usage information.
     */
    virtual void makeUsage();

protected:

    typedef bool (ICartesianControl::*RunnableFun)();
    typedef bool (ICartesianControl::*ConsumerFun)(const std::vector<double>&);
    typedef bool (ICartesianControl::*FunctionFun)(const std::vector<double>&, std::vector<double>&);

    bool handleStatMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out);

    bool handleRunnableCmdMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out, RunnableFun cmd);
    bool handleConsumerCmdMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out, ConsumerFun cmd);
    bool handleFunctionCmdMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out, FunctionFun cmd);

    roboticslab::ICartesianControl *iCartesianControl;
};

/**
 * @ingroup CartesianControlServer
 * @brief Responds to streaming command messages.
 */
class StreamResponder : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:

    StreamResponder(roboticslab::ICartesianControl *iCartesianControl)
    {
        this->iCartesianControl = iCartesianControl;
    }

    void onRead(yarp::os::Bottle& b);

protected:

    roboticslab::ICartesianControl *iCartesianControl;
};

}  // namespace roboticslab

#endif  // __CARTESIAN_CONTROL_SERVER_HPP__
