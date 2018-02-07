// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

#include <kdl/segment.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#ifdef _USE_LMA_
    #include <Eigen/Core> // Eigen::Matrix
    #include <kdl/chainiksolverpos_lma.hpp>
#else //_USE_LMA_
    #include <kdl/chainiksolverpos_nr_jl.hpp>
#endif //_USE_LMA_

#include <ColorDebug.hpp>

#include "KdlVectorConverter.hpp"
#include "KinematicRepresentation.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::getNumJoints(int* numJoints)
{
    *numJoints = getChain().getNrOfJoints();
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::appendLink(const std::vector<double>& x)
{
    KDL::Frame frameX = KdlVectorConverter::vectorToFrame(x);

    KDL::Chain chain = getChain();
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), frameX));
    setChain(chain);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::restoreOriginalChain()
{
    setChain(originalChain);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::changeReferenceFrame(const std::vector<double>& x_in, const std::vector<double>& currentQ,
        std::vector<double>& x_out, reference_frame currentFrame, reference_frame newFrame)
{
    if (newFrame == currentFrame)
    {
        CD_WARNING("New frame same as current frame.\n");
        x_out = x_in;
        return true;
    }

    std::vector<double> currentX;

    if (!fwdKin(currentQ, currentX))
    {
        CD_ERROR("fwdKin failed.\n");
        return false;
    }

    KDL::Frame H_0_tcp = KdlVectorConverter::vectorToFrame(currentX);
    //KDL::Frame H_W_0; // world-to-robot_base frame, not used (yet)

    KDL::Frame H_0_ref, H_tcp_ref; //, H_W_ref;

    switch (currentFrame)
    {
    case BASE_FRAME:
        H_0_ref = KdlVectorConverter::vectorToFrame(x_in);
        H_tcp_ref = H_0_tcp.Inverse() * H_0_ref;
        //H_W_ref = H_W_0 * H_0_ref;
        break;
    case TCP_FRAME:
        H_tcp_ref = KdlVectorConverter::vectorToFrame(x_in);
        H_0_ref = H_0_tcp * H_tcp_ref;
        //H_W_ref = H_W_0 * H_0_tcp * H_tcp_ref;
        break;
    /*case WORLD_FRAME:
        H_W_ref = KdlVectorConverter::vectorToFrame(x_in);
        H_0_ref = H_W_0.Inverse() * H_W_ref;
        H_tcp_ref = H_0_tcp.Inverse() * H_W_0.Inverse() * H_W_ref;
        break;*/
    default:
        CD_ERROR("Unsupported input reference frame.\n");
        return false;
    }

    switch (newFrame)
    {
    case BASE_FRAME:
        x_out = KdlVectorConverter::frameToVector(H_0_ref);
        break;
    case TCP_FRAME:
        x_out = KdlVectorConverter::frameToVector(H_tcp_ref);
        break;
    /*case WORLD_FRAME:
        x_out = KdlVectorConverter::frameToVector(H_W_ref);
        break;*/
    default:
        CD_ERROR("Unsupported target reference frame.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::fwdKin(const std::vector<double> &q, std::vector<double> &x)
{
    const KDL::Chain & chain = getChain();
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);
    }

    //-- Main fwdKin (pos) solver lines
    KDL::ChainFkSolverPos_recursive fksolver(chain);
    KDL::Frame fOutCart;
    fksolver.JntToCart(qInRad, fOutCart);

    x = KdlVectorConverter::frameToVector(fOutCart);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::fwdKinError(const std::vector<double> &xd, const std::vector<double> &q, std::vector<double> &x)
{
    KDL::Frame frameXd = KdlVectorConverter::vectorToFrame(xd);

    const KDL::Chain & chain = getChain();
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);
    }

    //-- Main fwdKin (pos) solver lines
    KDL::ChainFkSolverPos_recursive fksolver(chain);
    KDL::Frame fOutCart;
    fksolver.JntToCart(qInRad, fOutCart);

    KDL::Twist diff = KDL::diff(fOutCart, frameXd);
    x = KdlVectorConverter::twistToVector(diff);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q,
        reference_frame frame)
{
    KDL::Frame frameXd = KdlVectorConverter::vectorToFrame(xd);

    const KDL::Chain & chain = getChain();
    KDL::JntArray qGuessInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qGuessInRad(motor) = KinRepresentation::degToRad(qGuess[motor]);
    }

    KDL::JntArray kdlq(chain.getNrOfJoints());

#ifdef _USE_LMA_

    Eigen::Matrix<double, 6, 1> L;
    L(0) = 1; L(1) = 1; L(2) = 1;
    L(3) = 0.1; L(4) = 0.1; L(5) = 0.1;

    //-- Main invKin (pos) solver lines
    KDL::ChainIkSolverPos_LMA iksolver_pos(chain, L);

#else //_USE_LMA_

    //-- Forward solvers, needed by the geometric solver
    KDL::ChainFkSolverPos_recursive fksolver(chain);
    KDL::ChainIkSolverVel_pinv iksolver(chain);  // _givens

    //-- Geometric solver definition (with joint limits)
    KDL::ChainIkSolverPos_NR_JL iksolver_pos(chain, qMin, qMax, fksolver, iksolver, maxIter, eps);

#endif //_USE_LMA_

    if (frame == TCP_FRAME)
    {
        KDL::ChainFkSolverPos_recursive fksolver(chain);
        KDL::Frame fOutCart;
        fksolver.JntToCart(qGuessInRad, fOutCart);
        frameXd = fOutCart * frameXd;
    }
    else if (frame != BASE_FRAME)
    {
        CD_WARNING("Unsupported frame.\n");
        return false;
    }

    int ret = iksolver_pos.CartToJnt(qGuessInRad, frameXd, kdlq);

    if (ret < 0)
    {
        CD_ERROR("%d: %s\n", ret, iksolver_pos.strError(ret));
        return false;
    }
    else if (ret > 0)
    {
        CD_WARNING("%d: %s\n", ret, iksolver_pos.strError(ret));
    }

    q.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        q[motor] = KinRepresentation::radToDeg(kdlq(motor));
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot,
        reference_frame frame)
{
    const KDL::Chain & chain = getChain();
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);
    }

    KDL::Twist kdlxdot = KdlVectorConverter::vectorToTwist(xdot);

    if (frame == TCP_FRAME)
    {
        KDL::ChainFkSolverPos_recursive fksolver(chain);
        KDL::Frame fOutCart;
        fksolver.JntToCart(qInRad, fOutCart);

        //-- Transform the basis to which the twist is expressed, but leave the reference point intact
        //-- "Twist and Wrench transformations" @ http://docs.ros.org/latest/api/orocos_kdl/html/geomprim.html
        kdlxdot = fOutCart.M * kdlxdot;
    }
    else if (frame != BASE_FRAME)
    {
        CD_WARNING("Unsupported frame.\n");
        return false;
    }

    KDL::ChainIkSolverVel_pinv iksolverv(chain);
    KDL::JntArray qDotOutRadS(chain.getNrOfJoints());

    int ret = iksolverv.CartToJnt(qInRad, kdlxdot, qDotOutRadS);

    if (ret < 0)
    {
        CD_ERROR("%d: %s\n", ret, iksolverv.strError(ret));
        return false;
    }
    else if (ret > 0)
    {
        CD_WARNING("%d: %s\n", ret, iksolverv.strError(ret));
    }

    qdot.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qdot[motor] = KinRepresentation::radToDeg(qDotOutRadS(motor));
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::invDyn(const std::vector<double> &q,std::vector<double> &t)
{
    const KDL::Chain & chain = getChain();
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);
    }

    KDL::JntArray qdotInRad(chain.getNrOfJoints());
    KDL::JntArray qdotdotInRad(chain.getNrOfJoints());

    KDL::Wrenches wrenches(chain.getNrOfSegments(), KDL::Wrench::Zero());
    KDL::JntArray kdlt(chain.getNrOfJoints());

    //-- Main invDyn solver lines
    KDL::ChainIdSolver_RNE idsolver(chain, gravity);
    int ret = idsolver.CartToJnt(qInRad, qdotInRad, qdotdotInRad, wrenches, kdlt);

    if (ret < 0)
    {
        CD_ERROR("%d: %s\n", ret, idsolver.strError(ret));
        return false;
    }
    else if (ret > 0)
    {
        CD_WARNING("%d: %s\n", ret, idsolver.strError(ret));
    }

    t.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        t[motor] = kdlt(motor);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::invDyn(const std::vector<double> &q,const std::vector<double> &qdot,const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t)
{
    const KDL::Chain & chain = getChain();
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);
    }

    KDL::JntArray qdotInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qdotInRad(motor) = KinRepresentation::degToRad(qdot[motor]);
    }

    KDL::JntArray qdotdotInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qdotdotInRad(motor) = KinRepresentation::degToRad(qdotdot[motor]);
    }

    KDL::Wrenches wrenches(chain.getNrOfSegments(), KDL::Wrench::Zero());

    for (int i = 0; i < fexts.size(); i++)
    {
        wrenches[i] = KDL::Wrench(
            KDL::Vector(fexts[i][0], fexts[i][1], fexts[i][2]),
            KDL::Vector(fexts[i][3], fexts[i][4], fexts[i][5])
        );
    }

    KDL::JntArray kdlt(chain.getNrOfJoints());

    //-- Main invDyn solver lines
    KDL::ChainIdSolver_RNE idsolver(chain, gravity);
    int ret = idsolver.CartToJnt(qInRad, qdotInRad, qdotdotInRad, wrenches, kdlt);

    if (ret < 0)
    {
        CD_ERROR("%d: %s\n", ret, idsolver.strError(ret));
        return false;
    }
    else if (ret > 0)
    {
        CD_WARNING("%d: %s\n", ret, idsolver.strError(ret));
    }

    t.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        t[motor] = kdlt(motor);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::setLimits(const std::vector<double> &qMin, const std::vector<double> &qMax)
{
    for (int motor = 0; motor < getChain().getNrOfJoints(); motor++)
    {
        this->qMax(motor) = KinRepresentation::degToRad(qMax[motor]);
        this->qMin(motor) = KinRepresentation::degToRad(qMin[motor]);
    }
    return true;
}

// -----------------------------------------------------------------------------
