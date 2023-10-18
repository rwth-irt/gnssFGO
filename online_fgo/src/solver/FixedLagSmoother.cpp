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


#include "solver/FixedLagSmoother.h"

namespace fgo::solvers {
/* ************************************************************************* */
    /* ************************************************************************* */
    void FixedLagSmoother::Result::print() const {
        std::cout << "Nr iterations: " << iterations << '\n'
                  << "Nr intermediateSteps: " << intermediateSteps << '\n'
                  << "Nr nonlinear variables: " << nonlinearVariables << '\n'
                  << "Nr linear variables: " << linearVariables << '\n'
                  << "error: " << error << std::endl;
    }

/* ************************************************************************* */
    void FixedLagSmoother::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
        std::cout << s;
        std::cout << "  smoother lag: " << smootherLag_ << std::endl;
    }

/* ************************************************************************* */
    bool FixedLagSmoother::equals(const FixedLagSmoother& rhs, double tol) const {
        return std::abs(smootherLag_ - rhs.smootherLag_) < tol
               && std::equal(timestampKeyMap_.begin(), timestampKeyMap_.end(), rhs.timestampKeyMap_.begin());
    }

/* ************************************************************************* */
    void FixedLagSmoother::updateKeyTimestampMap(const KeyTimestampMap& timestamps) {
        // Loop through each key and add/update it in the map
        for(const auto& key_timestamp: timestamps) {
            // Check to see if this key already exists in the database
            auto keyIter = keyTimestampMap_.find(key_timestamp.first);

            // If the key already exists
            if(keyIter != keyTimestampMap_.end()) {
                // Find the entry in the Timestamp-Key database
                std::pair<TimestampKeyMap::iterator,TimestampKeyMap::iterator> range = timestampKeyMap_.equal_range(keyIter->second);
                auto timeIter = range.first;
                while(timeIter->second != key_timestamp.first) {
                    ++timeIter;
                }
                // remove the entry in the Timestamp-Key database
                timestampKeyMap_.erase(timeIter);
                // insert an entry at the new time
                timestampKeyMap_.insert(TimestampKeyMap::value_type(key_timestamp.second, key_timestamp.first));
                // update the Key-Timestamp database
                keyIter->second = key_timestamp.second;
            } else {
                // Add the Key-Timestamp database
                keyTimestampMap_.insert(key_timestamp);
                // Add the key to the Timestamp-Key database
                timestampKeyMap_.insert(TimestampKeyMap::value_type(key_timestamp.second, key_timestamp.first));
            }
        }
    }

/* ************************************************************************* */
    void FixedLagSmoother::eraseKeyTimestampMap(const gtsam::KeyVector& keys) {
        for(gtsam::Key key: keys) {
            // Erase the key from the Timestamp->Key map
            double timestamp = keyTimestampMap_.at(key);

            auto iter = timestampKeyMap_.lower_bound(timestamp);
            while(iter != timestampKeyMap_.end() && iter->first == timestamp) {
                if(iter->second == key) {
                    timestampKeyMap_.erase(iter++);
                } else {
                    ++iter;
                }
            }
            // Erase the key from the Key->Timestamp map
            keyTimestampMap_.erase(key);
        }
    }

/* ************************************************************************* */
    double FixedLagSmoother::getCurrentTimestamp() const {
        if(!timestampKeyMap_.empty()) {
            return timestampKeyMap_.rbegin()->first;
        } else {
            return -std::numeric_limits<double>::max();
        }
    }

/* ************************************************************************* */
    gtsam::KeyVector FixedLagSmoother::findKeysBefore(double timestamp) {
      static double last_timestamp = timestamp;
        gtsam::KeyVector keys;
      //std::cout << "notMarginalizing_: " << notMarginalizing_ << std::endl;
        if(!notMarginalizing_)
        {
          const auto inflatedTimestamp = timestamp - smootherLagInflation_;
          auto end = timestampKeyMap_.lower_bound(inflatedTimestamp);
          //std::cout << std::fixed <<"Time CutOff: " << inflatedTimestamp << " with inflation: " << smootherLagInflation_<< std::endl;
          for(auto iter = timestampKeyMap_.begin(); iter != end; ++iter) {
            keys.push_back(iter->second);
            //std::cout << std::fixed <<"Key: " << iter->second << " at: " << iter->first << std::endl;
          }
          notMarginalizing_ = false;
        }
      last_timestamp = timestamp;
        return keys;
    }

/* ************************************************************************* */
    gtsam::KeyVector FixedLagSmoother::findKeysAfter(double timestamp) const {
        gtsam::KeyVector keys;
        auto begin = timestampKeyMap_.upper_bound(timestamp);
        for(auto iter = begin; iter != timestampKeyMap_.end(); ++iter) {
            keys.push_back(iter->second);
        }
        return keys;
    }
/* ************************************************************************* */
} /// namespace fgonav
