//
// Created by lars on 15.12.21.
//NOT MAINTAINED

#ifndef ONLINE_FGO_GPINTERPOLATEDCARPHAFACTOR_H
#define ONLINE_FGO_GPINTERPOLATEDCARPHAFACTOR_H

# define M_PI           3.14159265358979323846  /* pi */

#pragma once

//#include "model/gp_interpolator/GPInterpolatorPose3WNOJ.h"
//#include "model/gp_interpolator/GPInterpolatorPose3WNOA.h"
/*Inputs:
 * Keys: pose of time i&j X(i)&X(j), velocity of time i&j V(i)&V(j), integer ambiguity of time i N(i), clockbiasdrift of time i&j C(i)&C(j)
 * CarrierPhase measurement: carPha
 * Position of the satellite: pointSat
 * Integer to find the right ambiguity in the vector: nAmbiguity (maybe it can be done in a more elegant way)
 * Position of sensor with respect to the Body: lb
 * Wavelength: lambda
 * delta_t: timedifference of state i & j tau: timedifference of state i & time of measurement
 * omega1&2: angularvelocity of time i and j
 * Covariance Matrix of the measurement/s: model & QcModel*/
/* measurement equation used:
 * Wavelength * CarrierPhase = Distance of Satellite and Receiver + Range through clock bias -(+) lambda * AmbiguityCycles + noise
 * somtimes the AmbiguityCycles get added, but its just a definition, here AmbiguityCycles are defined as positive integer*/
/*Jacobian: (e_RS * R_eb * skrew(lb_), e_RS) * derivation from gpslam
 * */
//Informations to Carrierphase starts at p. 295 in Farrell - Aided Navigation
namespace fgo::factors {
    class GPInterpolatedCarPhaFactor : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector2 ,
            gtsam::Pose3, gtsam::Vector3, gtsam::Vector> {
    private:
        //typedef
        typedef GPInterpolatedCarPhaFactor This;
        typedef gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector2 ,
                gtsam::Pose3, gtsam::Vector3, gtsam::Vector> Base;
        typedef fgo::models::GaussianProcessInterpolatorPose3WNOA GPbase;

        //interpolator
        GPbase gpBase_;

        double carPha_;
        gtsam::Point3 pointSat_;

        //parameters
        int nAmbiguity_;
        gtsam::Point3 lb_;
        double lambda_;

        double tau_;

        gtsam::Vector3 omega1_; //angular Rate of time i already corrected
        gtsam::Vector3 omega2_; // angular rate of time j already corrected

    public:

        /// shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<This> shared_ptr;

        GPInterpolatedCarPhaFactor() {}

        GPInterpolatedCarPhaFactor(gtsam::Key point_i, gtsam::Key vel_i, gtsam::Key cbd_i,gtsam::Key point_j,
                                   gtsam::Key vel_j, gtsam::Key ambiguity_i, const double &carPha,
                                   const gtsam::Point3 &pointSat, const int nAmbiguity,
                                   const gtsam::Point3 &lb, const double& lambda,
                                   const double& tau, const double& delta_t, const gtsam::Vector3& omega1,
                                   const gtsam::Vector3& omega2,
                                   const gtsam::SharedNoiseModel &model,
                                   const gtsam::SharedNoiseModel& Qc_model):
                                   Base(model, point_i, vel_i, cbd_i, point_j, vel_j, ambiguity_i),
                                   gpBase_(Qc_model, delta_t, tau),
                                   carPha_(carPha), pointSat_(pointSat), nAmbiguity_(nAmbiguity),
                                   lb_(lb), lambda_(lambda), tau_(tau), omega1_(omega1), omega2_(omega2){}

        virtual ~GPInterpolatedCarPhaFactor() {}

        virtual gtsam::NonlinearFactor::shared_ptr clone() const override {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        gtsam::Vector evaluateError(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector2 &cbd1,
                                    const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2,
                                    const gtsam::Vector &ambiguity2,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none,
                                    boost::optional<gtsam::Matrix &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix &> H3 = boost::none,
                                    boost::optional<gtsam::Matrix &> H4 = boost::none,
                                    boost::optional<gtsam::Matrix &> H5 = boost::none,
                                    boost::optional<gtsam::Matrix &> H6 = boost::none) const override {

            gtsam::Pose3 poseBody; //interpolated position
            gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P; //matrices for interpolate position
            if (H1 || H2 || H4 || H5) {
                poseBody = gpBase_.interpolatePose(pose1, vel1, omega1_, pose2, vel2, omega2_,
                                                   Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P);
            } else {
                poseBody = gpBase_.interpolatePose(pose1, vel1, omega1_, pose2, vel2, omega2_);
            }
            gtsam::Matrix13 e_RS; //unit vector receiver satellite
            gtsam::Pose3 body_P_sensor = gtsam::Pose3(gtsam::Rot3::Ypr(0, 0, 0), lb_); //pose_Antenna wrt to Body
            gtsam::Point3 positionReceiver = poseBody.compose(body_P_sensor).translation(); //pose_Antenna wrt to Earth

            double range = gtsam::distance3(positionReceiver, pointSat_, e_RS); //calculate range

            if (H1 || H2 || H4 || H5) {
                gtsam::Matrix rotation = poseBody.rotation().matrix(); // eRb
                gtsam::Matrix16 jacobian =
                        (gtsam::Matrix16() << (e_RS) * rotation * gtsam::skewSymmetric(-lb_), e_RS).finished(); //calculate Jacobian for poseReceiver
                if (H1) *H1 = jacobian * Hint1_P;//pose1
                if (H2) *H2 = jacobian * Hint2_P;//vel1
                if (H4) *H4 = jacobian * Hint4_P;//pose2
                if (H5) *H5 = jacobian * Hint5_P;//vel2
            }

            if (H3) *H3 = (gtsam::Matrix12() << 1,tau_).finished(); //bias
            if (H6) *H6 = (gtsam::Matrix11() << -lambda_).finished(); //ambiguity2

            int estAmbiguityError = 0;
            //estimate the ambiguity we lost or gained
            /*gtsam::Vector1 deltaDistance = e_RS * (pose2.translation() - poseBody.translation());
            for (int n = 0; n < 10; n++){
                if( carPha_/(2*M_PI) + deltaDistance.value() > n + 1 ){
                    estAmbiguityError = n;
                }
                else{
                    if ( carPha_/(2*M_PI) + deltaDistance.value() < -n ){
                        estAmbiguityError = -n;
                    }
                }
            }*/
            return (gtsam::Vector1() << range + cbd1(0) + tau_ * cbd1(1) - lambda_ *
                (ambiguity2(nAmbiguity_) + estAmbiguityError + carPha_ )).finished();
        } // end error fct

        /** return the measured */

        const double measuredDDphi() const {
            return carPha_;
        }

        /** number of variables attached to this factor */
        size_t size() const {
            return 5;
        }

        /** equals specialized to this factor */
        bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const override {
            const This *e =  dynamic_cast<const This*> (&expected);
            return e != NULL && Base::equals(*e, tol)
                   && gtsam::equal_with_abs_tol((gtsam::Vector1() << this->carPha_).finished(),
                                                (gtsam::Vector1() << e->carPha_).finished(), tol);
        }

        /** print contents */
        void print(const std::string &s = "",
                   const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
            std::cout << s << "GPInterDDPseudorange:" << std::endl;
            Base::print("", keyFormatter);
        }

    private:

        /** Serialization function */
        friend class boost::serialization::access;

        template<class ARCHIVE>
        void serialize(ARCHIVE &ar, const unsigned int version) {
            ar & boost::serialization::make_nvp("NoiseModelFactor6",
                                                boost::serialization::base_object<Base>(*this));
            ar & BOOST_SERIALIZATION_NVP(carPha_);
        }
    };
}


#endif //ONLINE_FGO_GPINTERPOLATEDCARPHAFACTOR_H

namespace gtsam {
    template<>
    struct traits<fgo::factors::GPInterpolatedCarPhaFactor> :
            public Testable<fgo::factors::GPInterpolatedCarPhaFactor> {
    };
}