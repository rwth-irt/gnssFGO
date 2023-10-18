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
 * @file    FixedLagSmoother.h
 * @brief   Base class for a fixed-lag smoother. This mimics the basic interface to iSAM2.
 * @author  Stephen Williams, Haoming ZHang
 *
 */

// \callgraph
#pragma once

#include <map>
#include <vector>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/dllexport.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include "gtsam/nonlinear/Marginals.h"


namespace fgo::solvers {

    class  FixedLagSmoother {
    public:
        /// Typedef for a shared pointer to an Incremental Fixed-Lag Smoother
        typedef boost::shared_ptr<FixedLagSmoother> shared_ptr;

        /// Typedef for a Key-Timestamp map/database
        typedef std::map<gtsam::Key, double> KeyTimestampMap;
        typedef std::map<size_t, double> KeyIndexTimestampMap;
        typedef std::multimap<double, gtsam::Key> TimestampKeyMap;

        /**
         * Meta information returned about the update
         */
        // TODO: Think of some more things to put here
        struct Result {
            size_t iterations; ///< The number of optimizer iterations performed
            size_t intermediateSteps; ///< The number of intermediate steps performed within the optimization. For L-M, this is the number of lambdas tried.
            size_t nonlinearVariables; ///< The number of variables that can be relinearized
            size_t linearVariables; ///< The number of variables that must keep a constant linearization point
            double error; ///< The final factor graph error
            Result() : iterations(0), intermediateSteps(0), nonlinearVariables(0), linearVariables(0), error(0) {};

            /// Getter methods
            [[nodiscard]] size_t getIterations() const { return iterations; }
            [[nodiscard]] size_t getIntermediateSteps() const { return intermediateSteps; }
            [[nodiscard]] size_t getNonlinearVariables() const { return nonlinearVariables; }
            [[nodiscard]] size_t getLinearVariables() const { return linearVariables; }
            [[nodiscard]] double getError() const { return error; }
            void print() const;
        };

        /** default constructor */
        explicit FixedLagSmoother(double smootherLag = 0.0) : smootherLag_(smootherLag) {}
        //FixedLagSmoother() :smootherLag_(0.0) {}
        /** destructor */
        virtual ~FixedLagSmoother() = default;

        /** Print the factor for debugging and testing (implementing Testable) */
        virtual void print(const std::string& s = "FixedLagSmoother:\n", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;

        /** Check if two IncrementalFixedLagSmoother Objects are equal */
        [[nodiscard]] virtual bool equals(const FixedLagSmoother& rhs, double tol = 1e-9) const;

        /** read the current smoother lag */
        [[nodiscard]] double smootherLag() const {
            return smootherLag_;
        }

        void setNotMarginalizing()
        {
          notMarginalizing_ = true;
        }

        void setMarginalizing()
        {
          notMarginalizing_ = false;
        }

        void setSmootherLagInflation(double newInflaction)
        {
          smootherLagInflation_ = newInflaction;
        }

        void resetSmootherLagInflation()
        {
          smootherLagInflation_ = 0.;
        }


        /** write to the current smoother lag */
        double& smootherLag() {
            return smootherLag_;
        }

        /** Access the current set of timestamps associated with each variable */
        [[nodiscard]] const KeyTimestampMap& timestamps() const {
            return keyTimestampMap_;
        }

        [[nodiscard]] KeyIndexTimestampMap keyIndexTimestamps() const {
          KeyIndexTimestampMap thisMap;
          for(const auto& p : keyTimestampMap_)
          {
            if(gtsam::symbolChr(p.first) != 'x')
              continue;
            thisMap.insert(std::make_pair(gtsam::symbolIndex(p.first), p.second));
          }
          return thisMap;
        }


        /** Add new factor, updating the solution and relinearizing as needed. */
        virtual Result update(const gtsam::NonlinearFactorGraph& newFactors = gtsam::NonlinearFactorGraph(),
                              const gtsam::Values& newTheta = gtsam::Values(),
                              const KeyTimestampMap& timestamps = KeyTimestampMap(),
                              const gtsam::FactorIndices& factorsToRemove = gtsam::FactorIndices(),
                              const gtsam::KeyVector& relatedKeys = gtsam::KeyVector()) = 0;

        /** Compute an estimate from the incomplete linear delta computed during the last update.
         * This delta is incomplete because it was not updated below wildfire_threshold.  If only
         * a single variable is needed, it is faster to call calculateEstimate(const KEY&).
         */
        [[nodiscard]] virtual gtsam::Values calculateEstimate() const  = 0;

        [[nodiscard]] virtual gtsam::Marginals getMarginals(const gtsam::Values& values) const = 0;

        [[nodiscard]] virtual  const gtsam::NonlinearFactorGraph &getFactors() const = 0;

    protected:

        /** The length of the smoother lag. Any variable older than this amount will be marginalized out. */
        double smootherLag_;

        double smootherLagInflation_ = 0.;

        std::atomic_bool notMarginalizing_ = false;

        /** The current timestamp associated with each tracked key */
        TimestampKeyMap timestampKeyMap_;
        KeyTimestampMap keyTimestampMap_;

        /** Update the Timestamps associated with the keys */
        void updateKeyTimestampMap(const KeyTimestampMap& newTimestamps);

        /** Erase keys from the Key-Timestamps database */
        void eraseKeyTimestampMap(const gtsam::KeyVector& keys);

        /** Find the most recent timestamp of the system */
        [[nodiscard]] double getCurrentTimestamp() const;

        /** Find all of the keys associated with timestamps before the provided time */
        [[nodiscard]] gtsam::KeyVector findKeysBefore(double timestamp);

        /** Find all of the keys associated with timestamps before the provided time */
        [[nodiscard]] gtsam::KeyVector findKeysAfter(double timestamp) const;

    }; // FixedLagSmoother

/// Typedef for matlab wrapping
    typedef FixedLagSmoother::KeyTimestampMap FixedLagSmootherKeyTimestampMap;
    typedef FixedLagSmootherKeyTimestampMap::value_type FixedLagSmootherKeyTimestampMapValue;
    typedef FixedLagSmoother::Result FixedLagSmootherResult;

} /// namespace fgonav

