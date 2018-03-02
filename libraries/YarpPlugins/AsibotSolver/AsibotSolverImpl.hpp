// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ASIBOT_SOLVER_IMPL_HPP__
#define __ASIBOT_SOLVER_IMPL_HPP__

#include <string>
#include <vector>

#include <yarp/os/Semaphore.h>
#include <yarp/sig/Matrix.h>

#include "AsibotConfiguration.hpp"
#include "ICartesianSolver.h"

// duplicates from AsibotSolver.hpp
#define NUM_MOTORS 5
#define DEFAULT_STRATEGY "leastOverallAngularDisplacement"

namespace roboticslab
{

/**
 * @ingroup AsibotSolver
 * @brief The AsibotSolverImpl class implements ICartesianSolver.
 */
class AsibotSolverImpl
{
public:

    AsibotSolverImpl(double A0, double A1, double A2, double A3, const std::vector<double> & qMin, const std::vector<double> & qMax);
    virtual ~AsibotSolverImpl();

    bool getNumJoints(int* numJoints);
    bool appendLink(const std::vector<double> &x);
    bool restoreOriginalChain();
    bool changeOrigin(const std::vector<double> &x_old_obj, const std::vector<double> &x_new_old, std::vector<double> &x_new_obj);
    bool fwdKin(const std::vector<double> &q, std::vector<double> &x);
    bool poseDiff(const std::vector<double> &xLhs, const std::vector<double> &xRhs, std::vector<double> &xOut);
    bool invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q, const ICartesianSolver::reference_frame frame);
    bool diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot, const ICartesianSolver::reference_frame frame);
    bool invDyn(const std::vector<double> &q, std::vector<double> &t);
    bool invDyn(const std::vector<double> &q, const std::vector<double> &qdot, const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t);

    bool buildStrategyFactory(const std::string & strategy);

private:

    struct AsibotTcpFrame
    {
        bool hasFrame;
        yarp::sig::Matrix frameTcp;
    };

    AsibotConfiguration * getConfiguration() const;

    AsibotTcpFrame getTcpFrame() const;
    void setTcpFrame(const AsibotTcpFrame & tcpFrameStruct);

    double A0, A1, A2, A3;  // link lengths

    std::vector<double> qMin, qMax;

    AsibotConfigurationFactory * confFactory;

    AsibotTcpFrame tcpFrameStruct;

    mutable yarp::os::Semaphore mutex;
};

}  // namespace roboticslab

#endif  // __ASIBOT_SOLVER_IMPL_HPP__
