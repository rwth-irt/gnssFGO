//
// Created by lars on 26.01.22.
//

#include <CppUnitLite/TestHarness.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <factors/measurement/gnss/CPFactor.h>

#include <cmath>
#include <fstream>
#include <iostream>






const double N0=4.0, E0=6.0, D0=0.0, Cb0=0;
gtsam::Point3 nomXYZ(4517590.87884893,5.04144017217502e-14,4487348.40886592);
gtsam::Point3 sat_pos_ecef(11322896.8543,23963601.5727,1733488.2027);
gtsam::Point3 sat_vel_ecef(-177.457,312.672,-3163.235);
double pseudorange = 25062948.043;
double deltarange = 598.3567;

TEST(CarrierPhaseFactor, Constructor){
    //init param
    double carPha = 0.37;
    gtsam::Point3 pointSat(11322896.8543,23963601.5727,1733488.2027);
    int nAmbiguity = 0;
    gtsam::Point3 lb (0,0,0);
    double lambda = 0.19;
    gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(2.0));
    //create factor
    fgo::factors::CarrierPhaseFactor factor(1,2,3,carPha,pointSat,nAmbiguity,lb,lambda,model);
    //check error
    gtsam::Pose3 poseRec(gtsam::Rot3::Ypr(0,0,0),gtsam::Point3(0,0,0));
    gtsam::Vector ambiguity(0);
    gtsam::Vector2 cbd (0,0);
    gtsam::Matrix H1,H2,H3;
    double error = factor.evaluateError(poseRec,ambiguity,cbd,H1,H2,H3).value();
    gtsam::Matrix Hexpected = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Pose3>(
            boost::bind(&fgo::factors::CarrierPhaseFactor::evaluateError, &factor, boost::placeholders::_1, ambiguity, cbd, boost::none, boost::none, boost::none), poseRec);
    gtsam::Matrix66 E2B = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, poseRec.rotation().matrix()).finished();
    H1 = H1 * E2B; //H1 position in ECEF and Hexpected position in Body, so have to rotate, when pose has rotation
    EXPECT_DOUBLES_EQUAL(error,pointSat.norm()-carPha*lambda,1e-2);
    EXPECT(gtsam::assert_equal(Hexpected,H1,1e-8));

}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}