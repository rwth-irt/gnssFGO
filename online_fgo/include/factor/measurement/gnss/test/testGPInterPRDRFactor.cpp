//
// Created by lars on 31.01.22.
// NT

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <factors/measurement/gnss/GPInterpolatedPRDRFactor.h>

#include <iostream>

const double N0=4.0, E0=6.0, D0=0.0, Cb0=0;
gtsam::Point3 nomXYZ(4517590.87884893,5.04144017217502e-14,4487348.40886592);

TEST(CarrierPhaseFactor, Constructor){
//init param
double pseuRa = 25062948.043;
double dPseuRa = 598.3567;
gtsam::Vector3 angRate(0.02,0,0);
gtsam::Point3 pointSat(11322896.8543,23963601.5727,1733488.2027);
gtsam::Point3 velSat(-177.457,312.672,-3163.235);
gtsam::Point3 lb (0,0,0);
double tau = 0.05;
double deltat = 0.1;
gtsam::Vector3 omega1(0,0,0.03);
gtsam::Vector3 omega2(0,0,0.06);
gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(2.0));
    gtsam::noiseModel::Gaussian::shared_ptr qcModel =
            gtsam::noiseModel::Gaussian::Covariance(0.5 * gtsam::I_6x6);
//create factor
fgo::factors::GPInterpolatedPRDRFactor factor(1,2,3,4,5, pseuRa, dPseuRa, pointSat, velSat, lb, omega1, omega2, deltat,
                                              tau, angRate, model, qcModel);
//check error
    gtsam::Pose3 poseReci(gtsam::Rot3::Ypr(0,0,0),gtsam::Point3(0,0,0));
    gtsam::Pose3 poseRecj(gtsam::Rot3::Ypr(0.003,0,0),gtsam::Point3(0,0,0));
    gtsam::Vector3 velReci;
    gtsam::Vector3 velRecj;
gtsam::Vector2 cbd (0,0);
gtsam::Matrix H1, H2, H3, H4, H5;
double error = factor.evaluateError(poseReci, velReci, cbd, poseRecj, velRecj, H1, H2, H3, H4, H5).value();
gtsam::Matrix Hexpected /*= gtsam::numericalDerivative11<gtsam::Vector,gtsam::Pose3>(
        boost::bind(&fgo::factor::GPInterpolatedPRDRFactor::evaluateError, &factor, boost::placeholders::_1, velReci,
                    cbd, poseRecj, velRecj, boost::none, boost::none, boost::none, boost::none, boost::none), poseReci)*/; //TODO boost dont work with this many arguments
gtsam::Matrix66 E2B = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, poseReci.rotation().matrix()).finished();
H1 = H1 * E2B; //H1 position in ECEF and Hexpected position in Body, so have to rotate, when pose has rotation
//EXPECT_DOUBLES_EQUAL(error,pointSat.norm()-pseuRa,1e-2);
//EXPECT(gtsam::assert_equal(Hexpected,H1,1e-8));

}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}