//
// Created by lars on 26.01.22.
// NT

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <factors/measurement/gnss/GPInterpolatedDDPRDRFactor.h>

#include <iostream>

const double N0=4.0, E0=6.0, D0=0.0, Cb0=0;
gtsam::Point3 nomXYZ(4517590.87884893,5.04144017217502e-14,4487348.40886592);
gtsam::Point3 sat_vel_ecef(-177.457,312.672,-3163.235);
double pseudorange = 25062948.043;
double deltarange = 598.3567;

TEST(CarrierPhaseFactor, Constructor){
//init param
    double ddPseuRa = 0.37;
//TODO find a realistic measurement
    gtsam::Point3 pointSatM(11322896.8543,23963601.5727,1733488.2027);
    gtsam::Point3 pointSatI;
    gtsam::Point3 pointBase;
    gtsam::Point3 lb (0,0,0);
    double tau = 0.05;
    double deltat = 0.1;
    gtsam::Vector3 omega1(0,0,0.03);
    gtsam::Vector3 omega2(0,0,0.06);
    gtsam::noiseModel::Gaussian::shared_ptr qcModel =
            gtsam::noiseModel::Gaussian::Covariance(0.5 * gtsam::I_6x6);
    gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(2.0));
//create factor
    fgo::factors::GPInterpolatedDDPseuRaFactor factor(1,2,3,4,ddPseuRa,pointSatM,
                                             pointSatI, pointBase, lb, deltat, tau, omega1, omega2, model, qcModel);
//check error
    gtsam::Pose3 poseReci(gtsam::Rot3::Ypr(0,0,0),gtsam::Point3(0,0,0));
    gtsam::Pose3 poseRecj(gtsam::Rot3::Ypr(0.003,0,0),gtsam::Point3(0,0,0));
    gtsam::Vector3 velReci;
    gtsam::Vector3 velRecj;
    gtsam::Matrix H1,H2,H3,H4;
    double error = factor.evaluateError(poseReci,velReci, poseRecj, velRecj, H1, H2, H3, H4).value();
    gtsam::Matrix Hexpected = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Pose3>(
            boost::bind(&fgo::factors::GPInterpolatedDDPseuRaFactor::evaluateError, &factor, boost::placeholders::_1,
                        velReci, poseRecj, velRecj, boost::none, boost::none, boost::none, boost::none), poseReci);
    gtsam::Matrix66 E2B = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, poseReci.rotation().matrix()).finished();
    H1 = H1 * E2B; //H1 position in ECEF and Hexpected position in Body, so have to rotate, when pose has rotation
    //EXPECT_DOUBLES_EQUAL(error,pointSatM.norm()-ddPseuRa_,1e-2); //TODO change this
    //EXPECT(gtsam::assert_equal(Hexpected,H1,1e-8));

}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}