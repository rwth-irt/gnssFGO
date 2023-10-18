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
 * @file    IncrementalFixedLagSmoother.h
 * @brief   An iSAM2-based fixed-lag smoother.
 *
 * @author  Michael Kaess, Stephen Williams, Haoming Zhang
 */

// \callgraph
#pragma once

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/base/debug.h>
#include "solver/FixedLagSmoother.h"


namespace fgo::solvers {

/**
 * This is a base class for the various HMF2 implementations. The HMF2 eliminates the factor graph
 * such that the active states are placed in/near the root. This base class implements a function
 * to calculate the ordering, and an update function to incorporate new factor into the HMF.
 */
    class IncrementalFixedLagSmoother : public FixedLagSmoother {
    public:
        /// Typedef for a shared pointer to an Incremental Fixed-Lag Smoother
        typedef FixedLagSmoother Base;
        typedef boost::shared_ptr<IncrementalFixedLagSmoother> shared_ptr;

        /** default constructor */
         explicit IncrementalFixedLagSmoother(double smootherLag = 0.0,
                                    const gtsam::ISAM2Params &parameters = DefaultISAM2Params()) :
                Base(smootherLag) {
          isam_ = gtsam::ISAM2(parameters);
        }

        /** destructor */
        ~IncrementalFixedLagSmoother() override = default;

        /** Print the factor for debugging and testing (implementing Testable) */
        void print(const std::string &s = "IncrementalFixedLagSmoother:\n",
                           const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override;

        /** Check if two IncrementalFixedLagSmoother Objects are equal */
        bool equals(const FixedLagSmoother &rhs, double tol = 1e-9) const override;

        /**
         * Add new factor, updating the solution and re-linearizing as needed.
         * @param newFactors new factor on old and/or new variables
         * @param newTheta new values for new variables only
         * @param timestamps an (optional) map from keys to real time stamps
         * @param factorsToRemove an (optional) list of factor to remove.
         */
        Result update(const gtsam::NonlinearFactorGraph &newFactors = gtsam::NonlinearFactorGraph(),
                      const gtsam::Values &newTheta = gtsam::Values(), //
                      const KeyTimestampMap &timestamps = KeyTimestampMap(),
                      const gtsam::FactorIndices &factorsToRemove = gtsam::FactorIndices(),
                      const gtsam::KeyVector& relatedKeys = gtsam::KeyVector()) override;

        /** Compute an estimate from the incomplete linear delta computed during the last update.
         * This delta is incomplete because it was not updated below wildfire_threshold.  If only
         * a single variable is needed, it is faster to call calculateEstimate(const KEY&).
         */
        gtsam::Values calculateEstimate() const override {

          std::cout << "calculateEstimate " <<std::endl;
            return isam_.calculateEstimate();
        }

        /** Compute an estimate for a single variable using its incomplete linear delta computed
         * during the last update.  This is faster than calling the no-argument version of
         * calculateEstimate, which operates on all variables.
         * @param key
         * @return
         */
        template<class VALUE>
        VALUE calculateEstimate(gtsam::Key key) const {
            return isam_.calculateEstimate<VALUE>(key);
        }

        /** return the current set of iSAM2 parameters */
        const gtsam::ISAM2Params &params() const {
            return isam_.params();
        }

        /** Access the current set of factor */
        const gtsam::NonlinearFactorGraph &getFactors() const override {
            return isam_.getFactorsUnsafe();
        }

        /** Access the current linearization point */
        const gtsam::Values &getLinearizationPoint() const {
            return isam_.getLinearizationPoint();
        }

        /** Access the current set of deltas to the linearization point */
        const gtsam::VectorValues &getDelta() const {
            return isam_.getDelta();
        }

        /// Calculate marginal covariance on given variable
        gtsam::Matrix marginalCovariance(gtsam::Key key) const {
            return isam_.marginalCovariance(key);
        }

        gtsam::Marginals getMarginals(const gtsam::Values& values) const override {
          return gtsam::Marginals(getFactors(), values);
        }

      /// Get results of latest isam2 update
        const gtsam::ISAM2Result &getISAM2Result() const { return isamResult_; }

    protected:

        /** Create default parameters */
        static gtsam::ISAM2Params DefaultISAM2Params() {
            gtsam::ISAM2Params params;
            params.findUnusedFactorSlots = true;
            return params;
        }

        /** An iSAM2 object used to perform inference. The smoother lag is controlled
         * by what factor are removed each iteration */
        gtsam::ISAM2 isam_;

        /** Store results of latest isam2 update */
        gtsam::ISAM2Result isamResult_;

        /** Erase any keys associated with timestamps before the provided time */
        void eraseKeysBefore(double timestamp);

        /** Fill in an iSAM2 ConstrainedKeys structure such that the provided keys are eliminated before all others */
        void createOrderingConstraints(const gtsam::KeyVector &marginalizableKeys,
                                       boost::optional<gtsam::FastMap<gtsam::Key, int> > &constrainedKeys) const;

    private:
        /** Private methods for printing debug information */
        static void PrintKeySet(const std::set<gtsam::Key> &keys, const std::string &label =
        "Keys:");

        static void PrintSymbolicFactor(const gtsam::GaussianFactor::shared_ptr &factor);

        static void PrintSymbolicGraph(const gtsam::GaussianFactorGraph &graph,
                                       const std::string &label = "Factor GraphBase:");

        static void PrintSymbolicTree(const gtsam::ISAM2 &isam,
                                      const std::string &label = "Bayes Tree:");

        static void PrintSymbolicTreeHelper(
                const gtsam::ISAM2Clique::shared_ptr &clique, const std::string& indent =
        "");

    };
// IncrementalFixedLagSmoother
}/// namespace fgonav

