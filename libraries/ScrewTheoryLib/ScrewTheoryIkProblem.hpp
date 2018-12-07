// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SCREW_THEORY_IK_PROBLEM_HPP__
#define __SCREW_THEORY_IK_PROBLEM_HPP__

#include <utility>
#include <vector>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "ProductOfExponentials.hpp"

namespace roboticslab
{

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class ScrewTheoryIkSubproblem
{
public:

    typedef std::pair<int, double> JointIdToSolution;
    typedef std::vector<JointIdToSolution> JointIdsToSolutionsVector;
    typedef std::vector<JointIdsToSolutionsVector> SolutionsVector;

    virtual ~ScrewTheoryIkSubproblem() {}

    virtual SolutionsVector solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform = KDL::Frame::Identity()) = 0;
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class ScrewTheoryIkProblem
{
public:

    ~ScrewTheoryIkProblem();

    bool solve(const KDL::Frame & H_S_T, std::vector<KDL::JntArray> & solutions);

    static ScrewTheoryIkProblem * create(const PoeExpression & poe, const std::vector<ScrewTheoryIkSubproblem *> & steps, bool reversed = false);

private:

    enum poe_term
    {
        EXP_KNOWN,
        EXP_COMPUTED,
        EXP_UNKNOWN
    };

    // disable instantiation, force users to call builder class
    ScrewTheoryIkProblem(const PoeExpression & poe, const std::vector<ScrewTheoryIkSubproblem *> & steps, bool reversed);

    // disable these too, avoid issues related to dynamic alloc
    ScrewTheoryIkProblem(const ScrewTheoryIkProblem &);
    ScrewTheoryIkProblem & operator=(const ScrewTheoryIkProblem &);

    void recalculateFrames(const std::vector<KDL::JntArray> & solutions);

    KDL::Frame transformPoint(const KDL::JntArray & jointValues);

    PoeExpression poe;

    // we own these, resources freed in destructor
    std::vector<ScrewTheoryIkSubproblem *> steps;

    std::vector<KDL::Frame> rhsFrames;
    std::vector<poe_term> poeTerms;

    bool reversed;
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class ScrewTheoryIkProblemBuilder
{
public:

    struct PoeTerm
    {
        PoeTerm() : known(false), simplified(false) {}
        bool known, simplified;
    };

    ScrewTheoryIkProblemBuilder(const PoeExpression & poe);

    ScrewTheoryIkProblem * build();

private:

    static std::vector<KDL::Vector> searchPoints(const PoeExpression & poe);

    std::vector<ScrewTheoryIkSubproblem *> searchSolutions();

    void simplify(int depth);
    void simplifyWithPadenKahanOne(const KDL::Vector & point);
    void simplifyWithPadenKahanThree(const KDL::Vector & point);
    void simplifyWithPardosOne();

    ScrewTheoryIkSubproblem * trySolve(int depth);

    PoeExpression poe;

    std::vector<KDL::Vector> points;
    std::vector<KDL::Vector> testPoints;

    std::vector<PoeTerm> poeTerms;

    static const int MAX_SIMPLIFICATION_DEPTH = 2;

    // TODO: addStep()
};

}  // namespace roboticslab

#endif  // __SCREW_THEORY_IK_PROBLEM_HPP__
