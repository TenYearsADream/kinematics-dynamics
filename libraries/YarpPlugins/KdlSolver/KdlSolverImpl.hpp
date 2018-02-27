// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KDL_SOLVER_IMPL_HPP__
#define __KDL_SOLVER_IMPL_HPP__

#include <yarp/os/Semaphore.h>

#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include "ICartesianSolver.h"

namespace roboticslab
{

/**
 * @ingroup KdlSolver
 * @brief The KdlSolverImpl class implements ICartesianSolver.
 */
class KdlSolverImpl : public ICartesianSolver
{
public:

    KdlSolverImpl(const KDL::Chain & chain, const KDL::Vector & gravity, const KDL::JntArray & qMin, const KDL::JntArray & qMax, double eps, int maxIter);

    // Get number of joints for which the solver has been configured.
    virtual bool getNumJoints(int* numJoints);

    // Append an additional link.
    virtual bool appendLink(const std::vector<double>& x);

    // Restore original kinematic chain.
    virtual bool restoreOriginalChain();

    // Change reference frame.
    virtual bool changeOrigin(const std::vector<double> &x_old_obj, const std::vector<double> &x_new_old, std::vector<double> &x_new_obj);

    // Perform forward kinematics.
    virtual bool fwdKin(const std::vector<double> &q, std::vector<double> &x);

    // Obtain difference between supplied pose inputs.
    virtual bool poseDiff(const std::vector<double> &xLhs, const std::vector<double> &xRhs, std::vector<double> &xOut);

    // Perform inverse kinematics.
    virtual bool invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q, const reference_frame frame);

    // Perform differential inverse kinematics.
    virtual bool diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot, const reference_frame frame);

    // Perform inverse dynamics.
    virtual bool invDyn(const std::vector<double> &q, std::vector<double> &t);

    // Perform inverse dynamics.
    virtual bool invDyn(const std::vector<double> &q, const std::vector<double> &qdot, const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t);

private:

    KDL::Chain getChain() const;
    void setChain(const KDL::Chain & chain);

    mutable yarp::os::Semaphore mutex;

    /** The chain. **/
    KDL::Chain chain;

    /** To store a copy of the original chain. **/
    KDL::Chain originalChain;

    /** Define used gravity for the chain, important to think of DH. **/
    KDL::Vector gravity;

    /** Minimum joint limits. **/
    KDL::JntArray qMin;

    /** Maximum joint limits. **/
    KDL::JntArray qMax;

    /** Precision value used by the IK solver. **/
    double eps;

    /** Maximum number of iterations to calculate inverse kinematics. **/
    unsigned int maxIter;
};

}  // namespace roboticslab

#endif  // __KDL_SOLVER_IMPL_HPP__
