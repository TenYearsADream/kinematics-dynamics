// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KDL_SOLVER_HPP__
#define __KDL_SOLVER_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/all.h>

#include <kdl/segment.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>

#include <iostream> // only windows
#include <stdlib.h> // for exit()

#include "ColorDebug.hpp"
#include "../ICartesianSolver.h"

#define DEFAULT_ANGLE_REPR "RPY"  // string
#define DEFAULT_NUM_LINKS 1  // int

#define DEFAULT_EPSILON 0.005     // Precision tolerance
#define DEFAULT_DURATION 20     // For Trajectory
#define DEFAULT_MAXVEL 7.5      // unit/s
#define DEFAULT_MAXACC 0.2      // unit/s^2

//using namespace yarp::math;

namespace teo
{

/**
 * @ingroup TeoYarp
 * \defgroup KdlSolver
 *
 * @brief Contains teo::KdlSolver.
 */

/**
 * @ingroup KdlSolver
 * @brief The KdlSolver class exposes a YARP_dev cartesian interface (implements
 * <a href="http://eris.liralab.it/yarpdoc/classyarp_1_1dev_1_1ICartesianControl.html">ICartesianControl</a>).
 */

class KdlSolver : public yarp::dev::DeviceDriver, public ICartesianSolver {

    public:

        KdlSolver() {}

        // -- ICartesianSolver declarations. Implementation in ICartesianSolverImpl.cpp--
        /** Get number of links for which the solver has been configured. */
        virtual bool getNumLinks(int* numLinks);

        /** Perform forward kinematics. */
        virtual bool fwdKin(const std::vector<double> &q, std::vector<double> &x, std::vector<double> &o);

        /** Perform inverse kinematics. */
        virtual bool invKin(const std::vector<double> &xd, const std::vector<double> &od, const std::vector<double> &qGuess, std::vector<double> &q);

        /** Perform inverse dynamics. */
        virtual bool invDyn(const std::vector<double> &q, std::vector<double> &t);
        virtual bool invDyn(const std::vector<double> &q,const std::vector<double> &qdot,const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t);

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

        /** The chain. **/
        KDL::Chain chain;

        /** Number of links of the chain. **/
        int numLinks;

        /** Define used gravity for the chain, important to think of DH. **/
        KDL::Vector gravity;

        /**
        * Simple function to pass from radians to degrees.
        * @param inRad angle value in radians.
        * @return angle value in degrees.
        */
        double toDeg(const double inRad) {
            return (inRad * 180.0 / M_PI);  // return (inRad * 180.0 / 3.14159265);
        }

        /**
        * Simple function to pass from degrees to radians.
        * @param inDeg angle value in degrees.
        * @return angle value in radians.
        */
        double toRad(const double inDeg) {
            return (inDeg * M_PI / 180.0);  // return (inDeg * 3.14159265 / 180.0);
        }

    private:

        bool withOri;

        KDL::Trajectory_Segment* currentTrajectory;
        KDL::RotationalInterpolation_SingleAxis* _orient;
        double _eqradius;
        bool _aggregate;

        yarp::sig::Vector isPrismatic;
        KDL::Frame targetF;
        yarp::sig::Vector targetO;

        double startTime;

        std::string angleRepr;
        double epsilon, duration, maxVel, maxAcc, cmcMs;

};

}  // namespace teo

#endif  // __KDL_SOLVER_HPP__
