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

#include "solver/IncrementalFixedLagSmoother.h"

namespace fgo::solvers {

/* ************************************************************************* */
    void recursiveMarkAffectedKeys(const gtsam::Key& key,
                                   const gtsam::ISAM2Clique::shared_ptr& clique, std::set<gtsam::Key>& additionalKeys) {

        // Check if the separator keys of the current clique contain the specified key
        if (std::find(clique->conditional()->beginParents(),
                      clique->conditional()->endParents(), key)
            != clique->conditional()->endParents()) {

            // Mark the frontal keys of the current clique
            for(gtsam::Key i: clique->conditional()->frontals()) {
                additionalKeys.insert(i);
            }

            // Recursively mark all of the children
            for(const gtsam::ISAM2Clique::shared_ptr& child: clique->children) {
                recursiveMarkAffectedKeys(key, child, additionalKeys);
            }
        }
        // If the key was not found in the separator/parents, then none of its children can have it either
    }

/* ************************************************************************* */
    void IncrementalFixedLagSmoother::print(const std::string& s,
                                            const gtsam::KeyFormatter& keyFormatter) const {
        FixedLagSmoother::print(s, keyFormatter);
    }

/* ************************************************************************* */
    bool IncrementalFixedLagSmoother::equals(const FixedLagSmoother& rhs,
                                             double tol) const {
        const auto* e =
                dynamic_cast<const IncrementalFixedLagSmoother*>(&rhs);
        return e != nullptr && FixedLagSmoother::equals(*e, tol)
               && isam_.equals(e->isam_, tol);
    }

/* ************************************************************************* */
    FixedLagSmoother::Result IncrementalFixedLagSmoother::update(
            const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta,
            const KeyTimestampMap& timestamps, const gtsam::FactorIndices& factorsToRemove,
            const gtsam::KeyVector& relatedKeys) {

        const bool debug = ISDEBUG("IncrementalFixedLagSmoother update");

        if (debug) {
            std::cout << "IncrementalFixedLagSmoother::update() Start" << std::endl;
            PrintSymbolicTree(isam_, "Bayes Tree Before Update:");
            std::cout << "END" << std::endl;
        }

        gtsam::FastVector<size_t> removedFactors;
        boost::optional<gtsam::FastMap<gtsam::Key, int> > constrainedKeys = boost::none;

        // Update the Timestamps associated with the factor keys
        updateKeyTimestampMap(timestamps);

        // Get current timestamp
        double current_timestamp = getCurrentTimestamp();

        if (debug)
            std::cout << "Current Timestamp: " << current_timestamp << " Lag: " << smootherLag_ <<std::endl;

        // Find the set of variables to be marginalized out
        gtsam::KeyVector marginalizableKeys = findKeysBefore(
                current_timestamp - smootherLag_);

       //std::vector<gtsam::FactorIndex> relatedKeyIds;

       if(!relatedKeys.empty())
       {
         gtsam::FactorIndex smallestIndex = std::numeric_limits<gtsam::FactorIndex>::max();

         for(const auto& key: relatedKeys) {
          // std::cout << gtsam::DefaultKeyFormatter(key) << " index: "  << gtsam::symbolIndex(key);
           const auto keyID = gtsam::symbolIndex(key);
           //relatedKeyIds.emplace_back(keyID);
           if(keyID < smallestIndex)
             smallestIndex = keyID;

           //smallestIndex -= 3;  // ToDo: this is a dirty fix
         }
         //std::cout << std::endl;

         //std::sort(relatedKeyIds.begin(), relatedKeyIds.end());

         //auto redundantIDs = std::unique(relatedKeyIds.begin(), relatedKeyIds.end());
         //relatedKeyIds.erase(redundantIDs, relatedKeyIds.end());

         if (debug) {
           std::cout << "Marginalizable Keys: before";
           for(gtsam::Key key: marginalizableKeys) {
             std::cout << gtsam::DefaultKeyFormatter(key);
           }
           std::cout << std::endl;
         }

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
       }

        if (debug) {
            std::cout << "Marginalizable Keys: ";
            for(gtsam::Key key: marginalizableKeys) {
                std::cout << gtsam::DefaultKeyFormatter(key);
            }
            std::cout << std::endl;
        }
        //std::cout << "solve 1" <<std::endl;
        // Force iSAM2 to put the marginalizable variables at the beginning
        createOrderingConstraints(marginalizableKeys, constrainedKeys);

        if (debug) {
            std::cout << "Constrained Keys: ";
            if (constrainedKeys) {
                for (auto & iter : *constrainedKeys) {
                    std::cout << gtsam::DefaultKeyFormatter(iter.first) << "(" << iter.second
                              << ")  ";
                }
            }
            std::cout << std::endl;
        }
      //std::cout << "solve 2" <<std::endl;

        // Mark additional keys between the marginalized keys and the leaves
        std::set<gtsam::Key> additionalKeys;
        for(gtsam::Key key: marginalizableKeys) {
          gtsam::ISAM2Clique::shared_ptr clique = isam_[key];
            for(const gtsam::ISAM2Clique::shared_ptr& child: clique->children) {
                recursiveMarkAffectedKeys(key, child, additionalKeys);
            }
        }
      gtsam::KeyList additionalMarkedKeys(additionalKeys.begin(), additionalKeys.end());

        // Update iSAM2
        isamResult_ = isam_.update(newFactors, newTheta,
                                   factorsToRemove, constrainedKeys, boost::none, additionalMarkedKeys);
        if (debug) {
            PrintSymbolicTree(isam_,
                              "Bayes Tree After Update, Before Marginalization:");
            std::cout << "END" << std::endl;
        }

        // Marginalize out any needed variables
        if (!marginalizableKeys.empty()) {
            gtsam::FastList<gtsam::Key> leafKeys(marginalizableKeys.begin(),
                                   marginalizableKeys.end());
            isam_.marginalizeLeaves(leafKeys);
        }

        // Remove marginalized keys from the KeyTimestampMap
        eraseKeyTimestampMap(marginalizableKeys);

        if (debug) {
            PrintSymbolicTree(isam_, "Final Bayes Tree:");
            std::cout << "END" << std::endl;
        }

        Result result;
        result.iterations = 1;
        result.linearVariables = 0;
        result.nonlinearVariables = 0;
        result.error = 0;

        if (debug)
            std::cout << "IncrementalFixedLagSmoother::update() Finish" << std::endl;

        return result;
    }

/* ************************************************************************* */
    void IncrementalFixedLagSmoother::eraseKeysBefore(double timestamp) {
        auto end = timestampKeyMap_.lower_bound(timestamp);
        auto iter = timestampKeyMap_.begin();
        while (iter != end) {
            keyTimestampMap_.erase(iter->second);
            timestampKeyMap_.erase(iter++);
        }
    }

/* ************************************************************************* */
    void IncrementalFixedLagSmoother::createOrderingConstraints(
            const gtsam::KeyVector& marginalizableKeys,
            boost::optional<gtsam::FastMap<gtsam::Key, int> >& constrainedKeys) const {
        if (!marginalizableKeys.empty()) {
            constrainedKeys = gtsam::FastMap<gtsam::Key, int>();
            // Generate ordering constraints so that the marginalizable variables will be eliminated first
            // Set all variables to Group1
            for(const TimestampKeyMap::value_type& timestamp_key: timestampKeyMap_) {
                constrainedKeys->operator[](timestamp_key.second) = 1;
            }
            // Set marginalizable variables to Group0
            for(gtsam::Key key: marginalizableKeys) {
                constrainedKeys->operator[](key) = 0;
            }
        }
    }

/* ************************************************************************* */
    void IncrementalFixedLagSmoother::PrintKeySet(const std::set<gtsam::Key>& keys,
                                                  const std::string& label) {
        std::cout << label;
        for(gtsam::Key key: keys) {
            std::cout << " " << gtsam::DefaultKeyFormatter(key);
        }
        std::cout << std::endl;
    }

/* ************************************************************************* */
    void IncrementalFixedLagSmoother::PrintSymbolicFactor(
            const gtsam::GaussianFactor::shared_ptr& factor) {
        std::cout << "f(";
        for(gtsam::Key key: factor->keys()) {
            std::cout << " " << gtsam::DefaultKeyFormatter(key);
        }
        std::cout << " )" << std::endl;
    }

/* ************************************************************************* */
    void IncrementalFixedLagSmoother::PrintSymbolicGraph(
            const gtsam::GaussianFactorGraph& graph, const std::string& label) {
        std::cout << label << std::endl;
        for(const gtsam::GaussianFactor::shared_ptr& factor: graph) {
            PrintSymbolicFactor(factor);
        }
    }

/* ************************************************************************* */
    void IncrementalFixedLagSmoother::PrintSymbolicTree(const gtsam::ISAM2& isam,
                                                        const std::string& label) {
        std::cout << label << std::endl;
        if (!isam.roots().empty()) {
            for(const gtsam::ISAM2::sharedClique& root: isam.roots()) {
                PrintSymbolicTreeHelper(root);
            }
        } else
            std::cout << "{Empty Tree}" << std::endl;
    }

/* ************************************************************************* */
    void IncrementalFixedLagSmoother::PrintSymbolicTreeHelper(
            const gtsam::ISAM2Clique::shared_ptr& clique, const std::string& indent) {

        // Print the current clique
        std::cout << indent << "P( ";
        for(gtsam::Key key: clique->conditional()->frontals()) {
            std::cout << gtsam::DefaultKeyFormatter(key) << " ";
        }
        if (clique->conditional()->nrParents() > 0)
            std::cout << "| ";
        for(gtsam::Key key: clique->conditional()->parents()) {
            std::cout << gtsam::DefaultKeyFormatter(key) << " ";
        }
        std::cout << ")" << std::endl;

        // Recursively print all of the children
        for(const gtsam::ISAM2Clique::shared_ptr& child: clique->children) {
            PrintSymbolicTreeHelper(child, indent + " ");
        }
    }

/* ************************************************************************* */
} /// namespace fgonav

