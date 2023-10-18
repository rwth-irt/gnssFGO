//
// Created by lars on 06.12.21.
//NOT MAINTAINED

#ifndef ONLINE_FGO_CPFACTOR_H
#define ONLINE_FGO_CPFACTOR_H

#pragma once //?

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/numericalDerivative.h>

#include <cmath>
#include <fstream>
#include <iostream>

// Position in ECEF, Rotation in Body and Velocity in NED
/*Inputs:
 * Keys: pose of time i X(i), integer ambiguity of time i N(i), clockbiasdrift of time i C(i)
 * CarrierPhase measurement: carPha
 * Position of the satellite: pointSat
 * Integer to find the right ambiguity in the vector: nAmbiguity (maybe it can be done in a more elegant way)
 * Position of sensor with respect to the Body: lb
 * Wavelength: lambda
 * Covariance Matrix of the measurement/s: model*/
/* measurement equation used:
 * Wavelength * CarrierPhase = Distance of Satellite and Receiver + Range through clock bias -(+) lambda * AmbiguityCycles + noise
 * somtimes the AmbiguityCycles get added, but its just a definition, here AmbiguityCycles are defined as positive integer*/
/*Jacobian: (e_RS * R_eb * skrew(lb_), e_RS)
 * */
//Informations to Carrierphase starts at p. 295 in Farrell - Aided Navigation

namespace fgo {
    namespace factors {
        class CarrierPhaseFactor : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector, gtsam::Vector2> {
        protected:
            double phi_; //CPMeasurement Master - Receiver
            double lambda_; //wavelength
            gtsam::Point3 pointSat_; //point Satellite
            gtsam::Point3 lb_; //Point of Receiver in body frame
            int nAmbiguity_; //Tells us which ambiguity of vector is used

            typedef CarrierPhaseFactor This;
            typedef gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector, gtsam::Vector2> Base;

        public:
            CarrierPhaseFactor() = default;
            //Constructor gtsam::Key point_i, gtsam::Key ambiguity_i, gtsam::Key bias_i
            CarrierPhaseFactor(gtsam::Key point_i, gtsam::Key ambiguity_i, gtsam::Key cbd_i, const double &carPha,
                                 const gtsam::Point3 &pointSat, const int nAmbiguity, const gtsam::Point3 &lb, const double &lambda,
                                 const gtsam::SharedNoiseModel &model) :
                    Base(model, point_i, ambiguity_i, cbd_i), phi_(carPha), lambda_(lambda), pointSat_(pointSat), lb_(lb),
                    nAmbiguity_(nAmbiguity){}

            ~CarrierPhaseFactor() = default;

            gtsam::NonlinearFactor::shared_ptr clone() const override {
                return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
            }

            gtsam::Vector evaluateError(const gtsam::Pose3 &poseBody, const gtsam::Vector &ambiguity,
                                        const gtsam::Vector2 &cbd,
                                        boost::optional<gtsam::Matrix &> H1 = boost::none,
                                        boost::optional<gtsam::Matrix &> H2 = boost::none,
                                        boost::optional<gtsam::Matrix &> H3 = boost::none) const override {
                //numerical Derivative
                /*if (H1)
                    *H1 = gtsam::numericalDerivative21<gtsam::Pose3, gtsam::Vector1>(
                            boost::bind(&This::evaluateError_, this, _1, _2), poseReceiver, ambiguity, 1e-5);
                if (H2)
                    *H2 = gtsam::numericalDerivative22<gtsam::Pose3, gtsam::Vector1>(
                            boost::bind(&This::evaluateError_, this, _1, _2), poseReceiver, ambiguity, 1e-5); */

                //calculate Jacobian
                gtsam::Matrix13 e_RS; //unity vector pointing from receiver to satellite (1x3)
                gtsam::Pose3 body_P_sensor = gtsam::Pose3(gtsam::Rot3::Ypr(0, 0, 0), lb_); //pose_Antenna wrt to Body
                gtsam::Point3 positionReceiver = poseBody.compose(body_P_sensor).translation(); //pose_Antenna wrt to Earth
                double range = gtsam::distance3(positionReceiver, pointSat_, e_RS); //calculate range

                if (H1 || H2 || H3){
                    gtsam::Matrix rotation = poseBody.rotation().matrix(); // eRb (3x3)
                    gtsam::Matrix jacobian_w = e_RS * rotation * gtsam::skewSymmetric(-lb_); //calculate Jacobian for Rotation
                    if (H1)
                        *H1 = (gtsam::Matrix16() << jacobian_w, e_RS).finished(); // derivation pose
                    if (H2)
                        *H2 = (gtsam::Matrix11() << -lambda_).finished(); // derivation n
                    if (H3)
                        *H3 = (gtsam::Matrix12() << 1,0).finished(); // derivation cbd
                }

                return (gtsam::Vector1() << range + cbd(0) - lambda_ * (ambiguity(nAmbiguity_) + phi_) ).finished();
            }

            /** return the measured */
            double measuredDDphi() const {
                return phi_;
            }

            /** number of variables attached to this factor */
            size_t size() const {
                return 3;
            }

            /** equals specialized to this factor */
            bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const override {
                const This *e =  dynamic_cast<const This*> (&expected);
                return e != NULL && Base::equals(*e, tol)
                       && gtsam::equal_with_abs_tol((gtsam::Vector1() << this->phi_).finished(), (gtsam::Vector2() << e->phi_).finished(), tol);
            }

            /** print contents */
            void print(const std::string &s = "",
                       const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
                std::cout << s << "CarrierphaseFactor:" << std::endl;
                Base::print("", keyFormatter);
            }

        private:

            /** Serialization function */
            friend class boost::serialization::access;

            template<class ARCHIVE>
            void serialize(ARCHIVE &ar, const unsigned int version) {
                ar & boost::serialization::make_nvp("NoiseModelFactor3",
                                                    boost::serialization::base_object<Base>(*this));
                ar & BOOST_SERIALIZATION_NVP(phi_);
            }

        };
    }
}

namespace gtsam {
    template<>
    struct traits<fgo::factors::CarrierPhaseFactor> : public Testable<fgo::factors::CarrierPhaseFactor> {
    };
}














#endif //ONLINE_FGO_CPFACTOR_H
