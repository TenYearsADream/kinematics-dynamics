// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotSolver.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>

#include <yarp/math/Math.h>

#include <ColorDebug.hpp>

#include "AsibotSolverImpl.hpp"

// ------------------- AsibotSolver Related ------------------------------------

roboticslab::AsibotSolver::AsibotSolver()
    : impl(NULL)
{}

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::AsibotSolver::open(yarp::os::Searchable& config)
{
    CD_DEBUG("config: %s.\n", config.toString().c_str());

    double A0 = config.check("A0", yarp::os::Value(DEFAULT_A0), "length of link 1 (meters)").asDouble();
    double A1 = config.check("A1", yarp::os::Value(DEFAULT_A1), "length of link 2 (meters)").asDouble();
    double A2 = config.check("A2", yarp::os::Value(DEFAULT_A2), "length of link 3 (meters)").asDouble();
    double A3 = config.check("A3", yarp::os::Value(DEFAULT_A3), "length of link 4 (meters)").asDouble();

    CD_INFO("AsibotSolver using A0: %f, A1: %f, A2: %f, A3: %f.\n", A0, A1, A2, A3);

    if (!config.check("mins") || !config.check("maxs"))
    {
        CD_ERROR("Missing 'mins' and/or 'maxs' option(s).\n");
        return false;
    }

    yarp::os::Bottle *mins = config.findGroup("mins", "joint lower limits (meters or degrees)").get(1).asList();
    yarp::os::Bottle *maxs = config.findGroup("maxs", "joint upper limits (meters or degrees)").get(1).asList();

    if (mins == YARP_NULLPTR || maxs == YARP_NULLPTR)
    {
        CD_ERROR("Empty 'mins' and/or 'maxs' option(s)\n");
        return false;
    }

    if (mins->size() != NUM_MOTORS || maxs->size() != NUM_MOTORS)
    {
        CD_ERROR("mins.size(), maxs.size() (%d, %d) != NUM_MOTORS (%d)\n", mins->size(), maxs->size(), NUM_MOTORS);
        return false;
    }

    std::vector<double> qMin(NUM_MOTORS), qMax(NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        qMin[i] = mins->get(i).asDouble();
        qMax[i] = maxs->get(i).asDouble();

        if (qMin[i] >= qMax[i])
        {
            CD_ERROR("qMin >= qMax (%f >= %f) at joint %d\n", qMin[i], qMax[i], i);
            return false;
        }
    }

    impl = new AsibotSolverImpl(A0, A1, A2, A3, qMin, qMax);

    std::string strategy = config.check("invKinStrategy", yarp::os::Value(DEFAULT_STRATEGY), "IK configuration strategy").asString();

    if (!impl->buildStrategyFactory(strategy))
    {
        CD_ERROR("Unsupported IK configuration strategy: %s.\n", strategy.c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::close()
{
    if (impl != NULL)
    {
        delete impl;
        impl = NULL;
    }

    return true;
}

// ------------------- ICartesianSolver Related ------------------------------------

bool roboticslab::AsibotSolver::getNumJoints(int* numJoints)
{
    return impl->getNumJoints(numJoints);
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::appendLink(const std::vector<double> &x)
{
    return impl->appendLink(x);
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::restoreOriginalChain()
{
    return impl->restoreOriginalChain();
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::changeOrigin(const std::vector<double> &x_old_obj, const std::vector<double> &x_new_old, std::vector<double> &x_new_obj)
{
    return impl->changeOrigin(x_old_obj, x_new_old, x_new_obj);
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::fwdKin(const std::vector<double> &q, std::vector<double> &x)
{
    return impl->fwdKin(q, x);
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::poseDiff(const std::vector<double> &xLhs, const std::vector<double> &xRhs, std::vector<double> &xOut)
{
    return impl->poseDiff(xLhs, xRhs, xOut);
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q, const reference_frame frame)
{
    return impl->invKin(xd, qGuess, q, frame);
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot, const reference_frame frame)
{
    return impl->diffInvKin(q, xdot, qdot, frame);
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::invDyn(const std::vector<double> &q, std::vector<double> &t)
{
    return impl->invDyn(q, t);
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::invDyn(const std::vector<double> &q, const std::vector<double> &qdot, const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t)
{
    return impl->invDyn(q, qdot, qdotdot, fexts, t);
}

// -----------------------------------------------------------------------------
