// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KDL_SOLVER_IMPL_HPP__
#define __KDL_SOLVER_IMPL_HPP__

#include <yarp/os/Semaphore.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainidsolver.hpp>

#include "ICartesianSolver.h"

namespace roboticslab
{

/**
 * @ingroup KdlSolver
 * @brief The KdlSolverImpl class implements ICartesianSolver.
 */
class KdlSolverImpl
{
public:

    KdlSolverImpl(const KDL::Chain & chain, KDL::ChainFkSolverPos * fkSolverPos, KDL::ChainIkSolverPos * ikSolverPos, KDL::ChainIkSolverVel * ikSolverVel, KDL::ChainIdSolver * idSolver);

    // Get number of joints for which the solver has been configured.
    bool getNumJoints(int* numJoints);

    // Append an additional link.
    bool appendLink(const std::vector<double>& x);

    // Restore original kinematic chain.
    bool restoreOriginalChain();

    // Change reference frame.
    bool changeOrigin(const std::vector<double> &x_old_obj, const std::vector<double> &x_new_old, std::vector<double> &x_new_obj);

    // Perform forward kinematics.
    bool fwdKin(const std::vector<double> &q, std::vector<double> &x);

    // Obtain difference between supplied pose inputs.
    bool poseDiff(const std::vector<double> &xLhs, const std::vector<double> &xRhs, std::vector<double> &xOut);

    // Perform inverse kinematics.
    bool invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q, const ICartesianSolver::reference_frame frame);

    // Perform differential inverse kinematics.
    bool diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot, const ICartesianSolver::reference_frame frame);

    // Perform inverse dynamics.
    bool invDyn(const std::vector<double> &q, std::vector<double> &t);

    // Perform inverse dynamics.
    bool invDyn(const std::vector<double> &q, const std::vector<double> &qdot, const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t);

private:

    mutable yarp::os::Semaphore mutex;

    /** The chain. **/
    KDL::Chain chain;

    /** To store a copy of the original chain. **/
    KDL::Chain originalChain;

    KDL::ChainFkSolverPos * fkSolverPos;
    KDL::ChainIkSolverPos * ikSolverPos;
    KDL::ChainIkSolverVel * ikSolverVel;
    KDL::ChainIdSolver * idSolver;
};

}  // namespace roboticslab

#endif  // __KDL_SOLVER_IMPL_HPP__
