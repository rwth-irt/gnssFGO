// /*********************************************************************
//   *
//   * Software License Agreement (BSD License)
//  *
//   *  Copyright (c) 2021
//   *  RWTH Aachen University - Institute of Automatic Control.
//   *  All rights reserved.
//   *
//   *  Redistribution and use in source and binary forms, with or without
//   *  modification, are permitted provided that the following conditions
//   *  are met:
//   *
//   *   * Redistributions of source code must retain the above copyright
//   *     notice, this list of conditions and the following disclaimer.
//   *   * Redistributions in binary form must reproduce the above
//   *     copyright notice, this list of conditions and the following
//   *     disclaimer in the documentation and/or other materials provided
//   *     with the distribution.
//   *   * Neither the name of the institute nor the names of its
//   *     contributors may be used to endorse or promote products derived
//   *     from this software without specific prior written permission.
//   *
//   *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//   *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//   *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//   *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   *  POSSIBILITY OF SUCH DAMAGE.
//   *
//   * Author:  Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//   *********************************************************************


#include "solver/BatchFixedLagSmoother.h"

using namespace std;

namespace fgo::solvers {

/* ************************************************************************* */
    void BatchFixedLagSmoother::print(const string& s,
                                      const gtsam::KeyFormatter& keyFormatter) const {
        FixedLagSmoother::print(s, keyFormatter);
    }

/* ************************************************************************* */
    bool BatchFixedLagSmoother::equals(const FixedLagSmoother& rhs,
                                       double tol) const {
        const auto* e = dynamic_cast<const BatchFixedLagSmoother*>(&rhs);
        return e != nullptr && FixedLagSmoother::equals(*e, tol)
               && factors_.equals(e->factors_, tol) && theta_.equals(e->theta_, tol);
    }

/* ************************************************************************* */
    gtsam::Matrix BatchFixedLagSmoother::marginalCovariance(gtsam::Key key) const {
        throw runtime_error(
                "BatchFixedLagSmoother::marginalCovariance not implemented");
    }

/* ************************************************************************* */
    FixedLagSmoother::Result BatchFixedLagSmoother::update(
            const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta,
            const KeyTimestampMap& timestamps, const gtsam::FactorIndices& factorsToRemove,
            const gtsam::KeyVector& relatedKeys) {
        // Update all of the internal variables with the new information
        gttic(augment_system);
        // Add the new variables to theta
        theta_.insert(newTheta);
        // Add new variables to the end of the ordering
        for (const auto& key_value : newTheta) {
            ordering_.push_back(key_value.key);
        }
        // Augment Delta
        delta_.insert(newTheta.zeroVectors());

        // Add the new factor to the graph, updating the variable index
        insertFactors(newFactors);
        gttoc(augment_system);
        // remove factor in factorToRemove
        for(const size_t i : factorsToRemove){
            if(factors_[i])
                factors_[i].reset();
        }

        // Update the Timestamps associated with the factor keys
        updateKeyTimestampMap(timestamps);
        // Get current timestamp
        double current_timestamp = getCurrentTimestamp();

       // std::cout << std::fixed <<"Time current_timestamp: " << current_timestamp <<  " smootherLag_: " << smootherLag_<<std::endl;
        // Find the set of variables to be marginalized out
        gtsam::KeyVector marginalizableKeys = findKeysBefore(
                current_timestamp - smootherLag_);

        //std::vector<gtsam::FactorIndex> relatedKeyIds;
        gtsam::FactorIndex smallestIndex = std::numeric_limits<gtsam::FactorIndex>::max();
        for(const auto& key: relatedKeys) {
          //std::cout << gtsam::DefaultKeyFormatter(key) << " index: "  << gtsam::symbolIndex(key);
          const auto keyID = gtsam::symbolIndex(key);
          //relatedKeyIds.emplace_back(keyID);
          if(keyID < smallestIndex)
            smallestIndex = keyID;

          smallestIndex -= 3;  // ToDo: this is a dirty fix
        }

        //std::sort(relatedKeyIds.begin(), relatedKeyIds.end());
        //auto redundantIDs = std::unique(relatedKeyIds.begin(), relatedKeyIds.end());
        //relatedKeyIds.erase(redundantIDs, relatedKeyIds.end());

        auto keyIter = marginalizableKeys.begin();
        while(keyIter != marginalizableKeys.end())
        {
          const auto keyID = gtsam::symbolIndex(*keyIter);
          //auto foundKey = std::find(relatedKeyIds.begin(), relatedKeyIds.end(), gtsam::symbolIndex(*keyIter));
          if(keyID >= smallestIndex)
          {
            keyIter = marginalizableKeys.erase(keyIter);
            continue;
          }
          keyIter++;
        }

        // Reorder
        gttic(reorder);
        reorder(marginalizableKeys);
        gttoc(reorder);
        // Optimize
        gttic(optimize);
        Result result;
        if (!factors_.empty()) {
            result = optimize();
        }
        gttoc(optimize);
        // Marginalize out old variables.
        gttic(marginalize);
        if (!marginalizableKeys.empty()) {
            marginalize(marginalizableKeys);
        }
        gttoc(marginalize);
        return result;
    }

/* ************************************************************************* */
    void BatchFixedLagSmoother::insertFactors(
            const gtsam::NonlinearFactorGraph& newFactors) {
        for(const auto& factor: newFactors) {
            gtsam::Key index;
            // Insert the factor into an existing hole in the factor graph, if possible
            if (!availableSlots_.empty()) {
                index = availableSlots_.front();
                availableSlots_.pop();
                factors_.replace(index, factor);
            } else {
                index = factors_.size();
                factors_.push_back(factor);
            }
            // Update the FactorIndex
            for(gtsam::Key key: *factor) {
                factorIndex_[key].insert(index);
            }
        }
    }

/* ************************************************************************* */
    void BatchFixedLagSmoother::removeFactors(
            const set<size_t>& deleteFactors) {
        for(size_t slot: deleteFactors) {
            if (factors_.at(slot)) {
                // Remove references to this factor from the FactorIndex
                for(gtsam::Key key: *(factors_.at(slot))) {
                    factorIndex_[key].erase(slot);
                }
                // Remove the factor from the factor graph
                factors_.remove(slot);
                // Add the factor's old slot to the list of available slots
                availableSlots_.push(slot);
            } else {
                cout << "Attempting to remove a factor from slot " << slot
                     << ", but it is already nullptr." << endl;
            }
        }
    }

/* ************************************************************************* */
    void BatchFixedLagSmoother::eraseKeys(const gtsam::KeyVector& keys) {

        for(gtsam::Key key: keys) {
            // Erase the key from the values
            theta_.erase(key);

            // Erase the key from the factor index
            factorIndex_.erase(key);

            // Erase the key from the set of linearized keys
            if (linearKeys_.exists(key)) {
                linearKeys_.erase(key);
            }
        }

        eraseKeyTimestampMap(keys);

        // Remove marginalized keys from the ordering and delta
        for(gtsam::Key key: keys) {
            ordering_.erase(find(ordering_.begin(), ordering_.end(), key));
            delta_.erase(key);
        }
    }

/* ************************************************************************* */
    void BatchFixedLagSmoother::reorder(const gtsam::KeyVector& marginalizeKeys) {
        // COLAMD groups will be used to place marginalize keys in Group 0, and everything else in Group 1
        ordering_ = gtsam::Ordering::ColamdConstrainedFirst(factors_, marginalizeKeys);
    }

/* ************************************************************************* */
    FixedLagSmoother::Result BatchFixedLagSmoother::optimize() {
        // Create output result structure
        Result result;
        result.nonlinearVariables = theta_.size() - linearKeys_.size();
        result.linearVariables = linearKeys_.size();

        // Set optimization parameters
        double lambda = parameters_.lambdaInitial;
        double lambdaFactor = parameters_.lambdaFactor;
        double lambdaUpperBound = parameters_.lambdaUpperBound;
        double lambdaLowerBound = 1.0e-10;
        size_t maxIterations = parameters_.maxIterations;
        double relativeErrorTol = parameters_.relativeErrorTol;
        double absoluteErrorTol = parameters_.absoluteErrorTol;
        double errorTol = parameters_.errorTol;

        // Create a Values that holds the current evaluation point
        gtsam::Values evalpoint = theta_.retract(delta_);
        result.error = factors_.error(evalpoint);
        // check if we're already close enough
        if (result.error <= errorTol) {
            return result;
        }
        // Use a custom optimization loop so the linearization points can be controlled
        double previousError;
        gtsam::VectorValues newDelta;
        do {
            previousError = result.error;

            // Do next iteration
            gttic(optimizer_iteration);
            {
                // Linearize graph around the linearization point
                gtsam::GaussianFactorGraph linearFactorGraph = *factors_.linearize(theta_);

                // Keep increasing lambda until we make make progress
                while (true) {

                    // Add prior factor at the current solution
                    gttic(damp);
                    gtsam::GaussianFactorGraph dampedFactorGraph(linearFactorGraph);
                    dampedFactorGraph.reserve(linearFactorGraph.size() + delta_.size());
                    {
                        // for each of the variables, add a prior at the current solution
                        double sigma = 1.0 / sqrt(lambda);
                        for(const auto& key_value: delta_) {
                            size_t dim = key_value.second.size();
                            gtsam::Matrix A = gtsam::Matrix::Identity(dim, dim);
                            gtsam::Vector b = key_value.second;
                            gtsam::SharedDiagonal model = gtsam::noiseModel::Isotropic::Sigma(dim, sigma);
                            gtsam::GaussianFactor::shared_ptr prior(
                                    new gtsam::JacobianFactor(key_value.first, A, b, model));
                            dampedFactorGraph.push_back(prior);
                        }
                    }
                    gttoc(damp);
                    result.intermediateSteps++;

                    gttic(solve);
                    // Solve Damped Gaussian Factor GraphBase
                    newDelta = dampedFactorGraph.optimize(ordering_,
                                                          parameters_.getEliminationFunction());
                    // update the evalpoint with the new delta
                    evalpoint = theta_.retract(newDelta);
                    gttoc(solve);

                    // Evaluate the new error
                    gttic(compute_error);
                    double error = factors_.error(evalpoint);
                    gttoc(compute_error);

                    if (error < result.error) {
                        // Keep this change
                        // Update the error value
                        result.error = error;
                        // Update the linearization point
                        theta_ = evalpoint;
                        // Reset the deltas to zeros
                        delta_.setZero();
                        // Put the linearization points and deltas back for specific variables
                        if (enforceConsistency_ && (!linearKeys_.empty())) {
                            theta_.update(linearKeys_);
                            for(const auto& key_value: linearKeys_) {
                                delta_.at(key_value.key) = newDelta.at(key_value.key);
                            }
                        }
                        // Decrease lambda for next time
                        lambda /= lambdaFactor;
                        if (lambda < lambdaLowerBound) {
                            lambda = lambdaLowerBound;
                        }
                        // End this lambda search iteration
                        break;
                    } else {
                        // Reject this change
                        if (lambda >= lambdaUpperBound) {
                            // The maximum lambda has been used. Print a warning and end the search.
                            cout
                                    << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda"
                                    << endl;
                            break;
                        } else {
                            // Increase lambda and continue searching
                            lambda *= lambdaFactor;
                        }
                    }
                } // end while
            }
            gttoc(optimizer_iteration);

            result.iterations++;
        } while (result.iterations < maxIterations
                 && !checkConvergence(relativeErrorTol, absoluteErrorTol, errorTol,
                                      previousError, result.error, gtsam::NonlinearOptimizerParams::SILENT));
        return result;
    }

/* ************************************************************************* */
    void BatchFixedLagSmoother::marginalize(const gtsam::KeyVector& marginalizeKeys) {
        // In order to marginalize out the selected variables, the factor involved in those variables
        // must be identified and removed. Also, the effect of those removed factor on the
        // remaining variables needs to be accounted for. This will be done with linear container factor
        // from the result of a partial elimination. This function removes the marginalized factor and
        // adds the linearized factor back in.

        // Identify all of the factor involving any marginalized variable. These must be removed.
        set<size_t> removedFactorSlots;
        const gtsam::VariableIndex variableIndex(factors_);
        for(gtsam::Key key: marginalizeKeys) {
            const auto& slots = variableIndex[key];
            removedFactorSlots.insert(slots.begin(), slots.end());
        }

        // Add the removed factor to a factor graph
        gtsam::NonlinearFactorGraph removedFactors;
        for(size_t slot: removedFactorSlots) {
            if (factors_.at(slot)) {
                removedFactors.push_back(factors_.at(slot));
            }
        }

        // Calculate marginal factor on the remaining keys
        gtsam::NonlinearFactorGraph marginalFactors = CalculateMarginalFactors(
                removedFactors, theta_, marginalizeKeys, parameters_.getEliminationFunction());

        // Remove marginalized factor from the factor graph
        removeFactors(removedFactorSlots);

        // Remove marginalized keys from the system
        eraseKeys(marginalizeKeys);

        // Insert the new marginal factor
        insertFactors(marginalFactors);
    }

/* ************************************************************************* */
    void BatchFixedLagSmoother::PrintKeySet(const set<gtsam::Key>& keys,
                                            const string& label) {
        cout << label;
        for(gtsam::Key key: keys) {
            cout << " " << gtsam::DefaultKeyFormatter(key);
        }
        cout << endl;
    }

/* ************************************************************************* */
    void BatchFixedLagSmoother::PrintKeySet(const gtsam::KeySet& keys,
                                            const string& label) {
        cout << label;
        for(gtsam::Key key: keys) {
            cout << " " << gtsam::DefaultKeyFormatter(key);
        }
        cout << endl;
    }

/* ************************************************************************* */
    void BatchFixedLagSmoother::PrintSymbolicFactor(
            const gtsam::NonlinearFactor::shared_ptr& factor) {
        cout << "f(";
        if (factor) {
            for(gtsam::Key key: factor->keys()) {
                cout << " " << gtsam::DefaultKeyFormatter(key);
            }
        } else {
            cout << " nullptr";
        }
        cout << " )" << endl;
    }

/* ************************************************************************* */
    void BatchFixedLagSmoother::PrintSymbolicFactor(
            const gtsam::GaussianFactor::shared_ptr& factor) {
        cout << "f(";
        for(gtsam::Key key: factor->keys()) {
            cout << " " << gtsam::DefaultKeyFormatter(key);
        }
        cout << " )" << endl;
    }

/* ************************************************************************* */
    void BatchFixedLagSmoother::PrintSymbolicGraph(
            const gtsam::NonlinearFactorGraph& graph, const string& label) {
        cout << label << endl;
        for(const auto& factor: graph) {
            PrintSymbolicFactor(factor);
        }
    }

/* ************************************************************************* */
    void BatchFixedLagSmoother::PrintSymbolicGraph(const gtsam::GaussianFactorGraph& graph,
                                                   const string& label) {
        cout << label << endl;
        for(const auto& factor: graph) {
            PrintSymbolicFactor(factor);
        }
    }

/* ************************************************************************* */
    gtsam::GaussianFactorGraph BatchFixedLagSmoother::CalculateMarginalFactors(
            const gtsam::GaussianFactorGraph& graph, const gtsam::KeyVector& keys,
            const gtsam::GaussianFactorGraph::Eliminate& eliminateFunction) {
        if (keys.empty()) {
            // There are no keys to marginalize. Simply return the input factor
            return graph;
        } else {
            // .first is the eliminated Bayes tree, while .second is the remaining factor graph
            return *graph.eliminatePartialMultifrontal(keys, eliminateFunction).second;
        }
    }

/* ************************************************************************* */
    gtsam::NonlinearFactorGraph BatchFixedLagSmoother::CalculateMarginalFactors(
            const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& theta, const gtsam::KeyVector& keys,
            const gtsam::GaussianFactorGraph::Eliminate& eliminateFunction) {
        if (keys.empty()) {
            // There are no keys to marginalize. Simply return the input factor
            return graph;
        } else {
            // Create the linear factor graph
            const auto linearFactorGraph = graph.linearize(theta);

            const auto marginalLinearFactors =
                    CalculateMarginalFactors(*linearFactorGraph, keys, eliminateFunction);

            // Wrap in nonlinear container factor
            return gtsam::LinearContainerFactor::ConvertLinearGraph(marginalLinearFactors, theta);
        }
    }

/* ************************************************************************* */
} /// namespace fgonav