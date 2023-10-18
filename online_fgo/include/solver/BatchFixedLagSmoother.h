/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */


//  Copyright 2022 Institute of Automatic Control RWTH Aachen University
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//
//  Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//
//

/**
 * @file    BatchFixedLagSmoother.h
 * @brief   An LM-based fixed-lag smoother.
 *
 * @author  Michael Kaess, Stephen Williams
 * @author  Haoming Zhang, adapted from above
 */

// \callgraph
#pragma once

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <queue>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianFactor.h>
#include "solver/FixedLagSmoother.h"

namespace fgo::solvers {
class  BatchFixedLagSmoother : public FixedLagSmoother {

public:

  /// Typedef for a shared pointer to an Incremental Fixed-Lag Smoother
  typedef FixedLagSmoother Base;
  typedef boost::shared_ptr<BatchFixedLagSmoother> shared_ptr;

  /** default constructor */
   explicit BatchFixedLagSmoother(double smootherLag = 0.0,
                                  const gtsam::LevenbergMarquardtParams& parameters = gtsam::LevenbergMarquardtParams(),
                                  bool enforceConsistency = true) :
       Base(smootherLag), parameters_(parameters), enforceConsistency_(enforceConsistency) { }

  /** destructor */
  ~BatchFixedLagSmoother() override = default;

  /** Print the factor for debugging and testing (implementing Testable) */
  void print(const std::string& s = "BatchFixedLagSmoother:\n", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

  /** Check if two IncrementalFixedLagSmoother Objects are equal */
  [[nodiscard]] bool equals(const FixedLagSmoother& rhs, double tol = 1e-9) const override;

  /** Add new factor, updating the solution and relinearizing as needed. */
  Result update(const gtsam::NonlinearFactorGraph& newFactors = gtsam::NonlinearFactorGraph(),
                const gtsam::Values& newTheta = gtsam::Values(),
                const KeyTimestampMap& timestamps = KeyTimestampMap(),
                const gtsam::FactorIndices& factorsToRemove = gtsam::FactorIndices(),
                const gtsam::KeyVector& relatedKeys = gtsam::KeyVector()) override;

  /** Compute an estimate from the incomplete linear delta computed during the last update.
   * This delta is incomplete because it was not updated below wildfire_threshold.  If only
   * a single variable is needed, it is faster to call calculateEstimate(const KEY&).
   */
  [[nodiscard]] gtsam::Values calculateEstimate() const override {
    return theta_.retract(delta_);
  }

  /** Compute an estimate for a single variable using its incomplete linear delta computed
   * during the last update.  This is faster than calling the no-argument version of
   * calculateEstimate, which operates on all variables.
   * @param key
   * @return
   */
  template<class VALUE>
  VALUE calculateEstimate(gtsam::Key key) const {
    const gtsam::Vector delta = delta_.at(key);
    return gtsam::traits<VALUE>::Retract(theta_.at<VALUE>(key), delta);
  }

  /** read the current set of optimizer parameters */
  [[nodiscard]] const gtsam::LevenbergMarquardtParams& params() const {
    return parameters_;
  }

  /** update the current set of optimizer parameters */
  gtsam::LevenbergMarquardtParams& params() {
    return parameters_;
  }

  /** Access the current set of factor */
  [[nodiscard]] const gtsam::NonlinearFactorGraph& getFactors() const override {
    return factors_;
  }

  /** Access the current linearization point */
  [[nodiscard]] const gtsam::Values& getLinearizationPoint() const {
    return theta_;
  }

  /** Access the current ordering */
  [[nodiscard]] const gtsam::Ordering& getOrdering() const {
    return ordering_;
  }

  /** Access the current set of deltas to the linearization point */
  [[nodiscard]] const gtsam::VectorValues& getDelta() const {
    return delta_;
  }

  /// Calculate marginal covariance on given variable
  [[nodiscard]] gtsam::Matrix marginalCovariance(gtsam::Key key) const;

  /// Marginalize specific keys from a linear graph.
  /// Does not check whether keys actually exist in graph.
  /// In that case will fail somewhere deep within elimination
  static gtsam::GaussianFactorGraph CalculateMarginalFactors(
          const gtsam::GaussianFactorGraph& graph, const gtsam::KeyVector& keys,
          const gtsam::GaussianFactorGraph::Eliminate& eliminateFunction = gtsam::EliminatePreferCholesky);

  /// Marginalize specific keys from a nonlinear graph, wrap in LinearContainers
  static gtsam::NonlinearFactorGraph CalculateMarginalFactors(
          const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& theta, const gtsam::KeyVector& keys,
          const gtsam::GaussianFactorGraph::Eliminate& eliminateFunction = gtsam::EliminatePreferCholesky);

  [[nodiscard]] gtsam::Marginals getMarginals(const gtsam::Values& values) const override {
    return {getFactors(), values};
  }

protected:

  /** A typedef defining an Key-Factor mapping **/
  typedef std::map<gtsam::Key, std::set<gtsam::Key> > FactorIndex;

  /** The L-M optimization parameters **/
  gtsam::LevenbergMarquardtParams parameters_;

  /** A flag indicating if the optimizer should enforce probabilistic consistency by maintaining the
   * linearization point of all variables involved in linearized/marginal factor at the edge of the
   * smoothing window. This idea is from ??? TODO: Look up paper reference **/
  bool enforceConsistency_;

  /** The nonlinear factor **/
  gtsam::NonlinearFactorGraph factors_{};

  /** The current linearization point **/
  gtsam::Values theta_;

  /** The set of keys involved in current linear factor. These keys should not be relinearized. **/
  gtsam::Values linearKeys_;

  /** The current ordering */
  gtsam::Ordering ordering_;

  /** The current set of linear deltas */
  gtsam::VectorValues delta_;

  /** The set of available factor graph slots. These occur because we are constantly deleting factor, leaving holes. **/
  std::queue<size_t> availableSlots_;

  /** A cross-reference structure to allow efficient factor lookups by key **/
  FactorIndex factorIndex_;

  /** Augment the list of factor with a set of new factor */
  void insertFactors(const gtsam::NonlinearFactorGraph& newFactors);

  /** Remove factor from the list of factor by slot index */
  void removeFactors(const std::set<size_t>& deleteFactors);

  /** Erase any keys associated with timestamps before the provided time */
  void eraseKeys(const gtsam::KeyVector& keys);

  /** Use colamd to update into an efficient ordering */
  void reorder(const gtsam::KeyVector& marginalizeKeys = gtsam::KeyVector());

  /** Optimize the current graph using a modified version of L-M */
  Result optimize();

  /** Marginalize out selected variables */
  void marginalize(const gtsam::KeyVector& marginalizableKeys);

private:
  /** Private methods for printing debug information */
  static void PrintKeySet(const std::set<gtsam::Key>& keys, const std::string& label);
  static void PrintKeySet(const gtsam::KeySet& keys, const std::string& label);
  static void PrintSymbolicFactor(const gtsam::NonlinearFactor::shared_ptr& factor);
  static void PrintSymbolicFactor(const gtsam::GaussianFactor::shared_ptr& factor);
  static void PrintSymbolicGraph(const gtsam::NonlinearFactorGraph& graph, const std::string& label);
  static void PrintSymbolicGraph(const gtsam::GaussianFactorGraph& graph, const std::string& label);
}; // BatchFixedLagSmoother
} /// namespace fgonav

