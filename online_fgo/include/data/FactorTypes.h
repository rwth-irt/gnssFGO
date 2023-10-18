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

//
// Created by haoming on 17.02.22.
//

#ifndef ONLINE_FGO_FACTORTYPES_H
#define ONLINE_FGO_FACTORTYPES_H

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <boost/align.hpp>

namespace fgo::factor{
    enum MeasurementFrame
    {
        ECEF = 0,
        NED  = 1,
        ENU  = 2,
        BODY = 3
    };

    enum VelocityType
    {
        VELZ,
        VELX,
        VELY,
        VEL2D,
        VEL3D
    };

    enum AttitudeType
    {
        ROLL,
        PITCH,
        YAW,
        YAWROLL,
        YAWPITCH,
        RPY
    };

}

namespace fgo{

    template<class VALUE1,  class VALUE2,  class VALUE3,  class VALUE4,
             class VALUE5,  class VALUE6,  class VALUE7,  class VALUE8,
             class VALUE9,  class VALUE10, class VALUE11, class VALUE12,
             class VALUE13, class VALUE14, class VALUE15, class VALUE16>
    class NoiseModelFactor16: public gtsam::NoiseModelFactor {
    public:
        // typedefs for value types pulled from keys
        typedef VALUE1 X1;
        typedef VALUE2 X2;
        typedef VALUE3 X3;
        typedef VALUE4 X4;
        typedef VALUE5 X5;
        typedef VALUE6 X6;
        typedef VALUE7 X7;
        typedef VALUE8 X8;
        typedef VALUE9 X9;
        typedef VALUE10 X10;
        typedef VALUE11 X11;
        typedef VALUE12 X12;
        typedef VALUE13 X13;
        typedef VALUE14 X14;
        typedef VALUE15 X15;
        typedef VALUE16 X16;

    protected:

        typedef NoiseModelFactor Base;
        typedef NoiseModelFactor16<VALUE1, VALUE2,  VALUE3,  VALUE4,  VALUE5,  VALUE6,  VALUE7,  VALUE8,
                                   VALUE9, VALUE10, VALUE11, VALUE12, VALUE13, VALUE14, VALUE15, VALUE16> This;

    public:

        /**
         * Default Constructor for I/O
         */
        NoiseModelFactor16() = default;

        /**
         * Constructor
         * @param noiseModel shared pointer to noise model
         * @param j1 key of the first variable
         * @param j2 key of the second variable
         * @param j3 key of the third variable
         * @param j4 key of the fourth variable
         * @param j5 key of the fifth variable
         * @param j6 key of the sixth variable
         * @param j7 key of the seventh variable
         * @param j8 key of the eighth variable
         * @param j9 key of the ninth variable
         * @param j10 key of the tenth variable
         * @param j11 key of the eleventh variable
         * @param j12 key of the twelfth variable
         */
        NoiseModelFactor16(const gtsam::SharedNoiseModel& noiseModel, gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4,
                           gtsam::Key j5, gtsam::Key j6, gtsam::Key j7, gtsam::Key j8, gtsam::Key j9, gtsam::Key j10, gtsam::Key j11,
                           gtsam::Key j12, gtsam::Key j13, gtsam::Key j14, gtsam::Key j15, gtsam::Key j16) :
            Base(noiseModel, boost::assign::cref_list_of<16>(j1)(j2)(j3)(j4)(j5)(j6)(j7)(j8)(j9)(j10)(j11)(j12)(j13)(j14)(j15)(j16)) {}
        ~NoiseModelFactor16() override = default;

        /** methods to retrieve keys */
        [[nodiscard]] inline gtsam::Key key1() const { return keys_[0]; }
        [[nodiscard]] inline gtsam::Key key2() const { return keys_[1]; }
        [[nodiscard]] inline gtsam::Key key3() const { return keys_[2]; }
        [[nodiscard]] inline gtsam::Key key4() const { return keys_[3]; }
        [[nodiscard]] inline gtsam::Key key5() const { return keys_[4]; }
        [[nodiscard]] inline gtsam::Key key6() const { return keys_[5]; }
        [[nodiscard]] inline gtsam::Key key7() const { return keys_[6]; }
        [[nodiscard]] inline gtsam::Key key8() const { return keys_[7]; }
        [[nodiscard]] inline gtsam::Key key9() const { return keys_[8]; }
        [[nodiscard]] inline gtsam::Key key10() const { return keys_[9]; }
        [[nodiscard]] inline gtsam::Key key11() const { return keys_[10]; }
        [[nodiscard]] inline gtsam::Key key12() const { return keys_[11]; }
        [[nodiscard]] inline gtsam::Key key13() const { return keys_[12]; }
        [[nodiscard]] inline gtsam::Key key14() const { return keys_[13]; }
        [[nodiscard]] inline gtsam::Key key15() const { return keys_[14]; }
        [[nodiscard]] inline gtsam::Key key16() const { return keys_[15]; }

        /** Calls the 8-key specific version of evaluateError, which is pure virtual
      * so must be implemented in the derived class. */
        [[nodiscard]] gtsam::Vector unwhitenedError(const gtsam::Values& x, boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const override {
          if(this->active(x)) {
            if(H)
              return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                                   x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]),
                                   x.at<X9>(keys_[8]), x.at<X10>(keys_[9]), x.at<X11>(keys_[10]), x.at<X12>(keys_[11]),
                                   x.at<X13>(keys_[12]), x.at<X14>(keys_[13]), x.at<X15>(keys_[14]), x.at<X16>(keys_[15]),
                                   (*H)[0], (*H)[1], (*H)[2], (*H)[3],
                                   (*H)[4], (*H)[5], (*H)[6],  (*H)[7],
                                   (*H)[8], (*H)[9], (*H)[10], (*H)[11],
                                   (*H)[12], (*H)[13], (*H)[14], (*H)[15]);
            else
              return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                                   x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]),
                                   x.at<X9>(keys_[8]), x.at<X10>(keys_[9]), x.at<X11>(keys_[10]), x.at<X12>(keys_[11]),
                                   x.at<X13>(keys_[12]), x.at<X14>(keys_[13]), x.at<X15>(keys_[14]), x.at<X16>(keys_[15]));
          } else {
            return gtsam::Vector::Zero(this->dim());
          }
        }

        /**
     *  Override this method to finish implementing a 8-way factor.
     *  If any of the optional Matrix reference arguments are specified, it should compute
     *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
     */
        virtual gtsam::Vector
        evaluateError(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&,
                      const X9&, const X10&, const X11&, const X12&, const X13&, const X14&, const X15&, const X16&,
                      boost::optional<gtsam::Matrix&> H1 = boost::none,
                      boost::optional<gtsam::Matrix&> H2 = boost::none,
                      boost::optional<gtsam::Matrix&> H3 = boost::none,
                      boost::optional<gtsam::Matrix&> H4 = boost::none,
                      boost::optional<gtsam::Matrix&> H5 = boost::none,
                      boost::optional<gtsam::Matrix&> H6 = boost::none,
                      boost::optional<gtsam::Matrix&> H7 = boost::none,
                      boost::optional<gtsam::Matrix&> H8 = boost::none,
                      boost::optional<gtsam::Matrix&> H9 = boost::none,
                      boost::optional<gtsam::Matrix&> H10 = boost::none,
                      boost::optional<gtsam::Matrix&> H11 = boost::none,
                      boost::optional<gtsam::Matrix&> H12 = boost::none,
                      boost::optional<gtsam::Matrix&> H13 = boost::none,
                      boost::optional<gtsam::Matrix&> H14 = boost::none,
                      boost::optional<gtsam::Matrix&> H15 = boost::none,
                      boost::optional<gtsam::Matrix&> H16 = boost::none) const = 0;

    private:

        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & boost::serialization::make_nvp("NoiseModelFactor",
                                              boost::serialization::base_object<Base>(*this));
        }
    }; // \class NoiseModelFactor16

    template<class VALUE1, class VALUE2, class VALUE3,class VALUE4,
        class VALUE5, class VALUE6,class VALUE7, class VALUE8,
        class VALUE9, class VALUE10, class VALUE11, class VALUE12,
        class VALUE13, class VALUE14, class VALUE15>
    class NoiseModelFactor15: public gtsam::NoiseModelFactor {
    public:
        // typedefs for value types pulled from keys
        typedef VALUE1 X1;
        typedef VALUE2 X2;
        typedef VALUE3 X3;
        typedef VALUE4 X4;
        typedef VALUE5 X5;
        typedef VALUE6 X6;
        typedef VALUE7 X7;
        typedef VALUE8 X8;
        typedef VALUE9 X9;
        typedef VALUE10 X10;
        typedef VALUE11 X11;
        typedef VALUE12 X12;
        typedef VALUE13 X13;
        typedef VALUE14 X14;
        typedef VALUE15 X15;

    protected:

        typedef NoiseModelFactor Base;
        typedef NoiseModelFactor15<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6,VALUE7,VALUE8, VALUE9, VALUE10, VALUE11, VALUE12, VALUE13, VALUE14, VALUE15> This;

    public:

        /**
         * Default Constructor for I/O
         */
        NoiseModelFactor15() = default;

        /**
         * Constructor
         * @param noiseModel shared pointer to noise model
         * @param j1 key of the first variable
         * @param j2 key of the second variable
         * @param j3 key of the third variable
         * @param j4 key of the fourth variable
         * @param j5 key of the fifth variable
         * @param j6 key of the sixth variable
         * @param j7 key of the seventh variable
         * @param j8 key of the eighth variable
         * @param j9 key of the ninth variable
         * @param j10 key of the tenth variable
         * @param j11 key of the eleventh variable
         * @param j12 key of the twelfth variable
         */
        NoiseModelFactor15(const gtsam::SharedNoiseModel& noiseModel, gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4,
                           gtsam::Key j5, gtsam::Key j6, gtsam::Key j7, gtsam::Key j8, gtsam::Key j9, gtsam::Key j10, gtsam::Key j11,
                           gtsam::Key j12, gtsam::Key j13, gtsam::Key j14, gtsam::Key j15) :
            Base(noiseModel, boost::assign::cref_list_of<15>(j1)(j2)(j3)(j4)(j5)(j6)(j7)(j8)(j9)(j10)(j11)(j12)(j13)(j14)(j15)) {}
        ~NoiseModelFactor15() override = default;

        /** methods to retrieve keys */
        [[nodiscard]] inline gtsam::Key key1() const { return keys_[0]; }
        [[nodiscard]] inline gtsam::Key key2() const { return keys_[1]; }
        [[nodiscard]] inline gtsam::Key key3() const { return keys_[2]; }
        [[nodiscard]] inline gtsam::Key key4() const { return keys_[3]; }
        [[nodiscard]] inline gtsam::Key key5() const { return keys_[4]; }
        [[nodiscard]] inline gtsam::Key key6() const { return keys_[5]; }
        [[nodiscard]] inline gtsam::Key key7() const { return keys_[6]; }
        [[nodiscard]] inline gtsam::Key key8() const { return keys_[7]; }
        [[nodiscard]] inline gtsam::Key key9() const { return keys_[8]; }
        [[nodiscard]] inline gtsam::Key key10() const { return keys_[9]; }
        [[nodiscard]] inline gtsam::Key key11() const { return keys_[10]; }
        [[nodiscard]] inline gtsam::Key key12() const { return keys_[11]; }
        [[nodiscard]] inline gtsam::Key key13() const { return keys_[12]; }
        [[nodiscard]] inline gtsam::Key key14() const { return keys_[13]; }
        [[nodiscard]] inline gtsam::Key key15() const { return keys_[14]; }

        /** Calls the 8-key specific version of evaluateError, which is pure virtual
      * so must be implemented in the derived class. */
        [[nodiscard]] gtsam::Vector unwhitenedError(const gtsam::Values& x, boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const override {
          if(this->active(x)) {
            if(H)
              return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                                   x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]),
                                   x.at<X9>(keys_[8]), x.at<X10>(keys_[9]), x.at<X11>(keys_[10]), x.at<X12>(keys_[11]),
                                   x.at<X13>(keys_[12]), x.at<X14>(keys_[13]), x.at<X15>(keys_[14]),
                                   (*H)[0], (*H)[1], (*H)[2], (*H)[3],
                                   (*H)[4], (*H)[5], (*H)[6],  (*H)[7],
                                   (*H)[8], (*H)[9], (*H)[10], (*H)[11],
                                   (*H)[12], (*H)[13], (*H)[14]);
            else
              return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                                   x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]),
                                   x.at<X9>(keys_[8]), x.at<X10>(keys_[9]), x.at<X11>(keys_[10]), x.at<X12>(keys_[11]),
                                   x.at<X13>(keys_[12]), x.at<X14>(keys_[13]), x.at<X15>(keys_[14]));
          } else {
            return gtsam::Vector::Zero(this->dim());
          }
        }

        /**
     *  Override this method to finish implementing a 8-way factor.
     *  If any of the optional Matrix reference arguments are specified, it should compute
     *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
     */
        virtual gtsam::Vector
        evaluateError(const X1&, const X2&, const X3&, const X4&,
                      const X5&, const X6&, const X7&, const X8&,
                      const X9&, const X10&, const X11&, const X12&, const X13&, const X14&, const X15&,
                      boost::optional<gtsam::Matrix&> H1 = boost::none,
                      boost::optional<gtsam::Matrix&> H2 = boost::none,
                      boost::optional<gtsam::Matrix&> H3 = boost::none,
                      boost::optional<gtsam::Matrix&> H4 = boost::none,
                      boost::optional<gtsam::Matrix&> H5 = boost::none,
                      boost::optional<gtsam::Matrix&> H6 = boost::none,
                      boost::optional<gtsam::Matrix&> H7 = boost::none,
                      boost::optional<gtsam::Matrix&> H8 = boost::none,
                      boost::optional<gtsam::Matrix&> H9 = boost::none,
                      boost::optional<gtsam::Matrix&> H10 = boost::none,
                      boost::optional<gtsam::Matrix&> H11 = boost::none,
                      boost::optional<gtsam::Matrix&> H12 = boost::none,
                      boost::optional<gtsam::Matrix&> H13 = boost::none,
                      boost::optional<gtsam::Matrix&> H14 = boost::none,
                      boost::optional<gtsam::Matrix&> H15 = boost::none) const = 0;

    private:

        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & boost::serialization::make_nvp("NoiseModelFactor",
                                              boost::serialization::base_object<Base>(*this));
        }
    }; // \class NoiseModelFactor15

  template<class VALUE1, class VALUE2, class VALUE3,class VALUE4,
          class VALUE5, class VALUE6,class VALUE7, class VALUE8,
          class VALUE9, class VALUE10, class VALUE11, class VALUE12>
  class NoiseModelFactor12: public gtsam::NoiseModelFactor {
  public:
    // typedefs for value types pulled from keys
    typedef VALUE1 X1;
    typedef VALUE2 X2;
    typedef VALUE3 X3;
    typedef VALUE4 X4;
    typedef VALUE5 X5;
    typedef VALUE6 X6;
    typedef VALUE7 X7;
    typedef VALUE8 X8;
    typedef VALUE9 X9;
    typedef VALUE10 X10;
    typedef VALUE11 X11;
    typedef VALUE12 X12;

  protected:

    typedef NoiseModelFactor Base;
    typedef NoiseModelFactor12<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6,VALUE7,VALUE8, VALUE9, VALUE10, VALUE11, VALUE12> This;

  public:

    /**
     * Default Constructor for I/O
     */
    NoiseModelFactor12() = default;

    /**
     * Constructor
     * @param noiseModel shared pointer to noise model
     * @param j1 key of the first variable
     * @param j2 key of the second variable
     * @param j3 key of the third variable
     * @param j4 key of the fourth variable
     * @param j5 key of the fifth variable
     * @param j6 key of the sixth variable
     * @param j7 key of the seventh variable
     * @param j8 key of the eighth variable
     * @param j9 key of the ninth variable
     * @param j10 key of the tenth variable
     * @param j11 key of the eleventh variable
     * @param j12 key of the twelfth variable
     */
    NoiseModelFactor12(const gtsam::SharedNoiseModel& noiseModel, gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4,
                       gtsam::Key j5, gtsam::Key j6, gtsam::Key j7, gtsam::Key j8, gtsam::Key j9, gtsam::Key j10, gtsam::Key j11,
                       gtsam::Key j12) :
            Base(noiseModel, boost::assign::cref_list_of<12>(j1)(j2)(j3)(j4)(j5)(j6)(j7)(j8)(j9)(j10)(j11)(j12)) {}
    ~NoiseModelFactor12() override = default;

    /** methods to retrieve keys */
    [[nodiscard]] inline gtsam::Key key1() const { return keys_[0]; }
    [[nodiscard]] inline gtsam::Key key2() const { return keys_[1]; }
    [[nodiscard]] inline gtsam::Key key3() const { return keys_[2]; }
    [[nodiscard]] inline gtsam::Key key4() const { return keys_[3]; }
    [[nodiscard]] inline gtsam::Key key5() const { return keys_[4]; }
    [[nodiscard]] inline gtsam::Key key6() const { return keys_[5]; }
    [[nodiscard]] inline gtsam::Key key7() const { return keys_[6]; }
    [[nodiscard]] inline gtsam::Key key8() const { return keys_[7]; }
    [[nodiscard]] inline gtsam::Key key9() const { return keys_[8]; }
    [[nodiscard]] inline gtsam::Key key10() const { return keys_[9]; }
    [[nodiscard]] inline gtsam::Key key11() const { return keys_[10]; }
    [[nodiscard]] inline gtsam::Key key12() const { return keys_[11]; }

    /** Calls the 8-key specific version of evaluateError, which is pure virtual
  * so must be implemented in the derived class. */
    [[nodiscard]] gtsam::Vector unwhitenedError(const gtsam::Values& x, boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const override {
      if(this->active(x)) {
        if(H)
          return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                               x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]),
                               x.at<X9>(keys_[8]), x.at<X10>(keys_[9]), x.at<X11>(keys_[10]), x.at<X12>(keys_[11]),
                               (*H)[0], (*H)[1], (*H)[2], (*H)[3],
                               (*H)[4], (*H)[5], (*H)[6],  (*H)[7],
                               (*H)[8], (*H)[9], (*H)[10], (*H)[11]);
        else
          return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                               x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]),
                               x.at<X9>(keys_[8]), x.at<X10>(keys_[9]), x.at<X11>(keys_[10]), x.at<X12>(keys_[11]));
      } else {
        return gtsam::Vector::Zero(this->dim());
      }
    }

    /**
 *  Override this method to finish implementing a 8-way factor.
 *  If any of the optional Matrix reference arguments are specified, it should compute
 *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
 */
    virtual gtsam::Vector
    evaluateError(const X1&, const X2&, const X3&, const X4&,
                  const X5&, const X6&, const X7&, const X8&,
                  const X9&, const X10&, const X11&, const X12&,
                  boost::optional<gtsam::Matrix&> H1 = boost::none,
                  boost::optional<gtsam::Matrix&> H2 = boost::none,
                  boost::optional<gtsam::Matrix&> H3 = boost::none,
                  boost::optional<gtsam::Matrix&> H4 = boost::none,
                  boost::optional<gtsam::Matrix&> H5 = boost::none,
                  boost::optional<gtsam::Matrix&> H6 = boost::none,
                  boost::optional<gtsam::Matrix&> H7 = boost::none,
                  boost::optional<gtsam::Matrix&> H8 = boost::none,
                  boost::optional<gtsam::Matrix&> H9 = boost::none,
                  boost::optional<gtsam::Matrix&> H10 = boost::none,
                  boost::optional<gtsam::Matrix&> H11 = boost::none,
                  boost::optional<gtsam::Matrix&> H12 = boost::none) const = 0;

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor",
                                          boost::serialization::base_object<Base>(*this));
    }
  }; // \class NoiseModelFactor12

  template<class VALUE1, class VALUE2, class VALUE3,class VALUE4,
          class VALUE5, class VALUE6,class VALUE7, class VALUE8,
                  class VALUE9, class VALUE10, class VALUE11>
  class NoiseModelFactor11: public gtsam::NoiseModelFactor {
  public:
    // typedefs for value types pulled from keys
    typedef VALUE1 X1;
    typedef VALUE2 X2;
    typedef VALUE3 X3;
    typedef VALUE4 X4;
    typedef VALUE5 X5;
    typedef VALUE6 X6;
    typedef VALUE7 X7;
    typedef VALUE8 X8;
    typedef VALUE9 X9;
    typedef VALUE10 X10;
    typedef VALUE11 X11;

  protected:

    typedef NoiseModelFactor Base;
    typedef NoiseModelFactor11<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6,VALUE7,VALUE8, VALUE9, VALUE10, VALUE11> This;

  public:

    /**
     * Default Constructor for I/O
     */
    NoiseModelFactor11() = default;

    /**
     * Constructor
     * @param noiseModel shared pointer to noise model
     * @param j1 key of the first variable
     * @param j2 key of the second variable
     * @param j3 key of the third variable
     * @param j4 key of the fourth variable
     * @param j5 key of the fifth variable
     * @param j6 key of the sixth variable
     * @param j7 key of the seventh variable
     * @param j8 key of the eighth variable
     * @param j9 key of the ninth variable
     * @param j10 key of the tenth variable
     * @param j11 key of the eleventh variable
     */
    NoiseModelFactor11(const gtsam::SharedNoiseModel& noiseModel, gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4,
                      gtsam::Key j5, gtsam::Key j6, gtsam::Key j7, gtsam::Key j8, gtsam::Key j9, gtsam::Key j10, gtsam::Key j11) :
            Base(noiseModel, boost::assign::cref_list_of<11>(j1)(j2)(j3)(j4)(j5)(j6)(j7)(j8)(j9)(j10)(j11)) {}
    ~NoiseModelFactor11() override = default;

    /** methods to retrieve keys */
    [[nodiscard]] inline gtsam::Key key1() const { return keys_[0]; }
    [[nodiscard]] inline gtsam::Key key2() const { return keys_[1]; }
    [[nodiscard]] inline gtsam::Key key3() const { return keys_[2]; }
    [[nodiscard]] inline gtsam::Key key4() const { return keys_[3]; }
    [[nodiscard]] inline gtsam::Key key5() const { return keys_[4]; }
    [[nodiscard]] inline gtsam::Key key6() const { return keys_[5]; }
    [[nodiscard]] inline gtsam::Key key7() const { return keys_[6]; }
    [[nodiscard]] inline gtsam::Key key8() const { return keys_[7]; }
    [[nodiscard]] inline gtsam::Key key9() const { return keys_[8]; }
    [[nodiscard]] inline gtsam::Key key10() const { return keys_[9]; }
    [[nodiscard]] inline gtsam::Key key11() const { return keys_[10]; }

    /** Calls the 8-key specific version of evaluateError, which is pure virtual
  * so must be implemented in the derived class. */
    [[nodiscard]] gtsam::Vector unwhitenedError(const gtsam::Values& x, boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const override {
      if(this->active(x)) {
        if(H)
          return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                               x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]),
                               x.at<X9>(keys_[8]), x.at<X10>(keys_[9]), x.at<X11>(keys_[10]),
                               (*H)[0], (*H)[1], (*H)[2], (*H)[3],
                               (*H)[4], (*H)[5], (*H)[6],  (*H)[7],
                               (*H)[8], (*H)[9], (*H)[10]);
        else
          return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                               x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]),
                                       x.at<X9>(keys_[8]), x.at<X10>(keys_[9]), x.at<X11>(keys_[10]));
      } else {
        return gtsam::Vector::Zero(this->dim());
      }
    }

    /**
 *  Override this method to finish implementing a 8-way factor.
 *  If any of the optional Matrix reference arguments are specified, it should compute
 *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
 */
    virtual gtsam::Vector
    evaluateError(const X1&, const X2&, const X3&, const X4&,
                  const X5&, const X6&, const X7&, const X8&,
                  const X9&, const X10&, const X11&,
                  boost::optional<gtsam::Matrix&> H1 = boost::none,
                  boost::optional<gtsam::Matrix&> H2 = boost::none,
                  boost::optional<gtsam::Matrix&> H3 = boost::none,
                  boost::optional<gtsam::Matrix&> H4 = boost::none,
                  boost::optional<gtsam::Matrix&> H5 = boost::none,
                  boost::optional<gtsam::Matrix&> H6 = boost::none,
                  boost::optional<gtsam::Matrix&> H7 = boost::none,
                  boost::optional<gtsam::Matrix&> H8 = boost::none,
                  boost::optional<gtsam::Matrix&> H9 = boost::none,
                  boost::optional<gtsam::Matrix&> H10 = boost::none,
                  boost::optional<gtsam::Matrix&> H11 = boost::none) const = 0;

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor",
                                          boost::serialization::base_object<Base>(*this));
    }
  }; // \class NoiseModelFactor11


    template<class VALUE1, class VALUE2, class VALUE3,class VALUE4,
        class VALUE5, class VALUE6,class VALUE7, class VALUE8,
        class VALUE9, class VALUE10>
    class NoiseModelFactor10: public gtsam::NoiseModelFactor {
    public:
        // typedefs for value types pulled from keys
        typedef VALUE1 X1;
        typedef VALUE2 X2;
        typedef VALUE3 X3;
        typedef VALUE4 X4;
        typedef VALUE5 X5;
        typedef VALUE6 X6;
        typedef VALUE7 X7;
        typedef VALUE8 X8;
        typedef VALUE9 X9;
        typedef VALUE10 X10;

    protected:

        typedef NoiseModelFactor Base;
        typedef NoiseModelFactor10<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6, VALUE7, VALUE8, VALUE9, VALUE10> This;

    public:

        /**
         * Default Constructor for I/O
         */
        NoiseModelFactor10() = default;

        /**
         * Constructor
         * @param noiseModel shared pointer to noise model
         * @param j1 key of the first variable
         * @param j2 key of the second variable
         * @param j3 key of the third variable
         * @param j4 key of the fourth variable
         * @param j5 key of the fifth variable
         * @param j6 key of the sixth variable
         * @param j7 key of the seventh variable
         * @param j8 key of the eighth variable
         * @param j9 key of the ninth variable
         * @param j10 key of the tenth variable
         * @param j11 key of the eleventh variable
         */
        NoiseModelFactor10(const gtsam::SharedNoiseModel& noiseModel, gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4,
                           gtsam::Key j5, gtsam::Key j6, gtsam::Key j7, gtsam::Key j8, gtsam::Key j9, gtsam::Key j10) :
            Base(noiseModel, boost::assign::cref_list_of<10>(j1)(j2)(j3)(j4)(j5)(j6)(j7)(j8)(j9)(j10)) {}
        ~NoiseModelFactor10() override = default;

        /** methods to retrieve keys */
        [[nodiscard]] inline gtsam::Key key1() const { return keys_[0]; }
        [[nodiscard]] inline gtsam::Key key2() const { return keys_[1]; }
        [[nodiscard]] inline gtsam::Key key3() const { return keys_[2]; }
        [[nodiscard]] inline gtsam::Key key4() const { return keys_[3]; }
        [[nodiscard]] inline gtsam::Key key5() const { return keys_[4]; }
        [[nodiscard]] inline gtsam::Key key6() const { return keys_[5]; }
        [[nodiscard]] inline gtsam::Key key7() const { return keys_[6]; }
        [[nodiscard]] inline gtsam::Key key8() const { return keys_[7]; }
        [[nodiscard]] inline gtsam::Key key9() const { return keys_[8]; }
        [[nodiscard]] inline gtsam::Key key10() const { return keys_[9]; }

        /** Calls the 8-key specific version of evaluateError, which is pure virtual
      * so must be implemented in the derived class. */
        [[nodiscard]] gtsam::Vector unwhitenedError(const gtsam::Values& x, boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const override {
          if(this->active(x)) {
            if(H)
              return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                                   x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]),
                                   x.at<X9>(keys_[8]), x.at<X10>(keys_[9]),
                                   (*H)[0], (*H)[1], (*H)[2], (*H)[3],
                                   (*H)[4], (*H)[5], (*H)[6],  (*H)[7],
                                   (*H)[8], (*H)[9]);
            else
              return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                                   x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]),
                                   x.at<X9>(keys_[8]), x.at<X10>(keys_[9]));
          } else {
            return gtsam::Vector::Zero(this->dim());
          }
        }

        /**
     *  Override this method to finish implementing a 8-way factor.
     *  If any of the optional Matrix reference arguments are specified, it should compute
     *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
     */
        virtual gtsam::Vector
        evaluateError(const X1&, const X2&, const X3&, const X4&,
                      const X5&, const X6&, const X7&, const X8&,
                      const X9&, const X10&,
                      boost::optional<gtsam::Matrix&> H1 = boost::none,
                      boost::optional<gtsam::Matrix&> H2 = boost::none,
                      boost::optional<gtsam::Matrix&> H3 = boost::none,
                      boost::optional<gtsam::Matrix&> H4 = boost::none,
                      boost::optional<gtsam::Matrix&> H5 = boost::none,
                      boost::optional<gtsam::Matrix&> H6 = boost::none,
                      boost::optional<gtsam::Matrix&> H7 = boost::none,
                      boost::optional<gtsam::Matrix&> H8 = boost::none,
                      boost::optional<gtsam::Matrix&> H9 = boost::none,
                      boost::optional<gtsam::Matrix&> H10 = boost::none) const = 0;

    private:

        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & boost::serialization::make_nvp("NoiseModelFactor",
                                              boost::serialization::base_object<Base>(*this));
        }
    }; // \class NoiseModelFactor10


  template<class VALUE1, class VALUE2, class VALUE3,class VALUE4,
          class VALUE5, class VALUE6,class VALUE7, class VALUE8, class VALUE9>
  class NoiseModelFactor9: public gtsam::NoiseModelFactor {
  public:
    // typedefs for value types pulled from keys
    typedef VALUE1 X1;
    typedef VALUE2 X2;
    typedef VALUE3 X3;
    typedef VALUE4 X4;
    typedef VALUE5 X5;
    typedef VALUE6 X6;
    typedef VALUE7 X7;
    typedef VALUE8 X8;
    typedef VALUE9 X9;

  protected:

    typedef NoiseModelFactor Base;
    typedef NoiseModelFactor9<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6,VALUE7,VALUE8, VALUE9> This;

  public:

    /**
     * Default Constructor for I/O
     */
    NoiseModelFactor9() = default;

    /**
     * Constructor
     * @param noiseModel shared pointer to noise model
     * @param j1 key of the first variable
     * @param j2 key of the second variable
     * @param j3 key of the third variable
     * @param j4 key of the fourth variable
     * @param j5 key of the fifth variable
     * @param j6 key of the sixth variable
     * @param j7 key of the seventh variable
     * @param j8 key of the eighth variable
     * @param j9 key of the nineth variable
     */
    NoiseModelFactor9(const gtsam::SharedNoiseModel& noiseModel, gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4,
                      gtsam::Key j5, gtsam::Key j6, gtsam::Key j7, gtsam::Key j8, gtsam::Key j9) :
            Base(noiseModel, boost::assign::cref_list_of<9>(j1)(j2)(j3)(j4)(j5)(j6)(j7)(j8)(j9)) {}

    ~NoiseModelFactor9() override = default;

    /** methods to retrieve keys */
    [[nodiscard]] inline gtsam::Key key1() const { return keys_[0]; }
    [[nodiscard]] inline gtsam::Key key2() const { return keys_[1]; }
    [[nodiscard]] inline gtsam::Key key3() const { return keys_[2]; }
    [[nodiscard]] inline gtsam::Key key4() const { return keys_[3]; }
    [[nodiscard]] inline gtsam::Key key5() const { return keys_[4]; }
    [[nodiscard]] inline gtsam::Key key6() const { return keys_[5]; }
    [[nodiscard]] inline gtsam::Key key7() const { return keys_[6]; }
    [[nodiscard]] inline gtsam::Key key8() const { return keys_[7]; }
    [[nodiscard]] inline gtsam::Key key9() const { return keys_[8]; }

    /** Calls the 8-key specific version of evaluateError, which is pure virtual
  * so must be implemented in the derived class. */
    [[nodiscard]] gtsam::Vector unwhitenedError(const gtsam::Values& x, boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const override {
      if(this->active(x)) {
        if(H)
          return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                               x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]),
                               x.at<X9>(keys_[8]),
                               (*H)[0], (*H)[1], (*H)[2], (*H)[3],(*H)[4], (*H)[5], (*H)[6],
                               (*H)[7], (*H)[8]);
        else
          return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                               x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]),
                               x.at<X9>(keys_[8]));
      } else {
        return gtsam::Vector::Zero(this->dim());
      }
    }

    /**
 *  Override this method to finish implementing a 8-way factor.
 *  If any of the optional Matrix reference arguments are specified, it should compute
 *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
 */
    virtual gtsam::Vector
    evaluateError(const X1&, const X2&, const X3&, const X4&,
                  const X5&, const X6&, const X7&, const X8&,
                  const X9&,
                  boost::optional<gtsam::Matrix&> H1 = boost::none,
                  boost::optional<gtsam::Matrix&> H2 = boost::none,
                  boost::optional<gtsam::Matrix&> H3 = boost::none,
                  boost::optional<gtsam::Matrix&> H4 = boost::none,
                  boost::optional<gtsam::Matrix&> H5 = boost::none,
                  boost::optional<gtsam::Matrix&> H6 = boost::none,
                  boost::optional<gtsam::Matrix&> H7 = boost::none,
                  boost::optional<gtsam::Matrix&> H8 = boost::none,
                  boost::optional<gtsam::Matrix&> H9 = boost::none) const = 0;

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor",
                                          boost::serialization::base_object<Base>(*this));
    }
  }; // \class NoiseModelFactor9

    template<class VALUE1, class VALUE2, class VALUE3,class VALUE4,
             class VALUE5, class VALUE6,class VALUE7, class VALUE8>
    class NoiseModelFactor8: public gtsam::NoiseModelFactor {
    public:
        // typedefs for value types pulled from keys
        typedef VALUE1 X1;
        typedef VALUE2 X2;
        typedef VALUE3 X3;
        typedef VALUE4 X4;
        typedef VALUE5 X5;
        typedef VALUE6 X6;
        typedef VALUE7 X7;
        typedef VALUE8 X8;

    protected:

        typedef NoiseModelFactor Base;
        typedef NoiseModelFactor8<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6,VALUE7,VALUE8> This;

    public:

        /**
         * Default Constructor for I/O
         */
        NoiseModelFactor8() = default;

        /**
         * Constructor
         * @param noiseModel shared pointer to noise model
         * @param j1 key of the first variable
         * @param j2 key of the second variable
         * @param j3 key of the third variable
         * @param j4 key of the fourth variable
         * @param j5 key of the fifth variable
         * @param j6 key of the sixth variable
         * @param j7 key of the seventh variable
         * @param j8 key of the eighth variable
         */
        NoiseModelFactor8(const gtsam::SharedNoiseModel& noiseModel, gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4,
                          gtsam::Key j5, gtsam::Key j6, gtsam::Key j7, gtsam::Key j8) :
            Base(noiseModel, boost::assign::cref_list_of<8>(j1)(j2)(j3)(j4)(j5)(j6)(j7)(j8)) {}
        ~NoiseModelFactor8() override = default;

        /** methods to retrieve keys */
        [[nodiscard]] inline gtsam::Key key1() const { return keys_[0]; }
        [[nodiscard]] inline gtsam::Key key2() const { return keys_[1]; }
        [[nodiscard]] inline gtsam::Key key3() const { return keys_[2]; }
        [[nodiscard]] inline gtsam::Key key4() const { return keys_[3]; }
        [[nodiscard]] inline gtsam::Key key5() const { return keys_[4]; }
        [[nodiscard]] inline gtsam::Key key6() const { return keys_[5]; }
        [[nodiscard]] inline gtsam::Key key7() const { return keys_[6]; }
        [[nodiscard]] inline gtsam::Key key8() const { return keys_[7]; }

        /** Calls the 8-key specific version of evaluateError, which is pure virtual
      * so must be implemented in the derived class. */
        [[nodiscard]] gtsam::Vector unwhitenedError(const gtsam::Values& x, boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const override {
          if(this->active(x)) {
            if(H)
              return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                                   x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]),
                                   (*H)[0], (*H)[1], (*H)[2], (*H)[3],
                                   (*H)[4], (*H)[5], (*H)[6],  (*H)[7]);
            else
              return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]),
                                   x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]));
          } else {
            return gtsam::Vector::Zero(this->dim());
          }
        }

        /**
     *  Override this method to finish implementing a 8-way factor.
     *  If any of the optional Matrix reference arguments are specified, it should compute
     *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
     */
        virtual gtsam::Vector
        evaluateError(const X1&, const X2&, const X3&, const X4&,
                      const X5&, const X6&, const X7&, const X8&,
                      boost::optional<gtsam::Matrix&> H1 = boost::none,
                      boost::optional<gtsam::Matrix&> H2 = boost::none,
                      boost::optional<gtsam::Matrix&> H3 = boost::none,
                      boost::optional<gtsam::Matrix&> H4 = boost::none,
                      boost::optional<gtsam::Matrix&> H5 = boost::none,
                      boost::optional<gtsam::Matrix&> H6 = boost::none,
                      boost::optional<gtsam::Matrix&> H7 = boost::none,
                      boost::optional<gtsam::Matrix&> H8 = boost::none) const = 0;

    private:

        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & boost::serialization::make_nvp("NoiseModelFactor",
                                              boost::serialization::base_object<Base>(*this));
        }
    }; // \class NoiseModelFactor8

  template<class VALUE1, class VALUE2, class VALUE3, class VALUE4,
          class VALUE5, class VALUE6, class VALUE7>
  class NoiseModelFactor7 : public gtsam::NoiseModelFactor {
  public:
    // typedefs for value types pulled from keys
    typedef VALUE1 X1;
    typedef VALUE2 X2;
    typedef VALUE3 X3;
    typedef VALUE4 X4;
    typedef VALUE5 X5;
    typedef VALUE6 X6;
    typedef VALUE7 X7;

  protected:
    typedef NoiseModelFactor Base;
    typedef NoiseModelFactor7<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6, VALUE7> This;

  public:

    /**
     * Default Constructor for I/O
     */
    NoiseModelFactor7() = default;

    /**
     * Constructor
     * @param noiseModel shared pointer to noise model
     * @param j1 key of the first variable
     * @param j2 key of the second variable
     * @param j3 key of the third variable
     * @param j4 key of the fourth variable
     * @param j5 key of the fifth variable
     * @param j6 key of the sixth variable
     * @param j7 key of the seventh variable
     * @param j8 key of the eighth variable
     */
    NoiseModelFactor7(const gtsam::SharedNoiseModel &noiseModel, gtsam::Key j1, gtsam::Key j2, gtsam::Key j3,
                      gtsam::Key j4, gtsam::Key j5, gtsam::Key j6, gtsam::Key j7) :
            Base(noiseModel, boost::assign::cref_list_of<7>(j1)(j2)(j3)(j4)(j5)(j6)(j7)) {}

    ~NoiseModelFactor7() override = default;

    /** methods to retrieve keys */
    [[nodiscard]] inline gtsam::Key key1() const { return keys_[0]; }

    [[nodiscard]] inline gtsam::Key key2() const { return keys_[1]; }

    [[nodiscard]] inline gtsam::Key key3() const { return keys_[2]; }

    [[nodiscard]] inline gtsam::Key key4() const { return keys_[3]; }

    [[nodiscard]] inline gtsam::Key key5() const { return keys_[4]; }

    [[nodiscard]] inline gtsam::Key key6() const { return keys_[5]; }

    [[nodiscard]] inline gtsam::Key key7() const { return keys_[6]; }

    /** Calls the 8-key specific version of evaluateError, which is pure virtual
  * so must be implemented in the derived class. */
    [[nodiscard]] gtsam::Vector unwhitenedError(const gtsam::Values &x,
                                               boost::optional<std::vector<gtsam::Matrix> &> H = boost::none) const override {
      if (this->active(x)) {
        if (H)
          return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]),
                               x.at<X4>(keys_[3]),
                               x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]),
                               (*H)[0], (*H)[1], (*H)[2], (*H)[3],
                               (*H)[4], (*H)[5], (*H)[6]);
        else
          return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]),
                               x.at<X4>(keys_[3]),
                               x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]));
      } else {
        return gtsam::Vector::Zero(this->dim());
      }
    }

    /**
  *  Override this method to finish implementing a 8-way factor.
  *  If any of the optional Matrix reference arguments are specified, it should compute
  *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
  */
    virtual gtsam::Vector
    evaluateError(const X1 &, const X2 &, const X3 &, const X4 &,
                  const X5 &, const X6 &, const X7 &,
                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                  boost::optional<gtsam::Matrix &> H4 = boost::none,
                  boost::optional<gtsam::Matrix &> H5 = boost::none,
                  boost::optional<gtsam::Matrix &> H6 = boost::none,
                  boost::optional<gtsam::Matrix &> H7 = boost::none) const = 0;

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor",
                                          boost::serialization::base_object<Base>(*this));
    }
  }; // \class NoiseModelFactor7

}
#endif //ONLINE_FGO_FACTORTYPES_H
