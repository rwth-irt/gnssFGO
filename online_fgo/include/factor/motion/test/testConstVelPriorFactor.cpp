//
// Created by lars on 16.02.22.
//

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <factors/motion/ConstVelPriorFactor.h>

#include <iostream>

TEST(MotionModelFactor, Constructor) {
//INIT PARAMS
double deltat = 4.5;
gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(2.0, 2.0, 2.0));
//CREATE FACTORS
fgo::factors::MotionModelFactor factor(1, 2, 3, 4, deltat, model);
//CHECK ERROR
gtsam::Pose3 posei(gtsam::Rot3::Ypr(0.6, 0, 0.3), gtsam::Point3(5, 5, 5));
gtsam::Pose3 posej(gtsam::Rot3::Ypr(0.5, 0.3, 0.6), gtsam::Point3(10, 10, 10));
gtsam::Vector3 veli(1, 1, 1);
gtsam::Vector3 velj(2, 2, 2);
gtsam::Matrix H1 = gtsam::Matrix66(), H2 = gtsam::Matrix63(), H3 = gtsam::Matrix66(), H4 = gtsam::Matrix63();
gtsam::Vector6 error = factor.evaluateError(posei, veli, posej, velj, H1, H2, H3, H4);
//CREATE ERROR
gtsam::Matrix66 H1e = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
        boost::bind(&fgo::factors::MotionModelFactor::evaluateError, &factor, boost::placeholders::_1, veli, posej,
                    velj,
                    boost::none, boost::none, boost::none, boost::none), posei);
gtsam::Matrix63 H2e = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
        boost::bind(&fgo::factors::MotionModelFactor::evaluateError, &factor, posei, boost::placeholders::_1, posej,
                    velj,
                    boost::none, boost::none, boost::none, boost::none), veli);
gtsam::Matrix66 H3e = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
        boost::bind(&fgo::factors::MotionModelFactor::evaluateError, &factor, posei, veli, boost::placeholders::_1,
                    velj,
                    boost::none, boost::none, boost::none, boost::none), posej);
gtsam::Matrix63 H4e = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
        boost::bind(&fgo::factors::MotionModelFactor::evaluateError, &factor, posei, veli, posej,
                    boost::placeholders::_1,
                    boost::none, boost::none, boost::none, boost::none), velj);
gtsam::Matrix66 E2B = (gtsam::Matrix66()
        << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, posei.rotation().matrix()).finished();
gtsam::Matrix66 E2B2 = (gtsam::Matrix66()
        << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, posej.rotation().matrix()).finished();
H1e = H1e * E2B.transpose(); //H1 position in ECEF and Hexpected position in Body, so have to rotate, when pose has rotation
H2e = H2e;
H3e = H3e * E2B2.transpose();
H4e = H4e;
gtsam::Vector6 errore = (gtsam::Vector6() << posej.translation() - posei.translation() - deltat * velj, velj - velj).finished();
//PRINT
/*
std::cout << "H1e :" << H1e << std::endl;
std::cout << "H1 :" << H1 << std::endl;
std::cout << "H2e :" << H2e << std::endl;
std::cout << "H2 :" << H2 << std::endl;
std::cout << "H3e :" << H3e << std::endl;
std::cout << "H3 :" << H3 << std::endl;
std::cout << "H4e :" << H4e << std::endl;
std::cout << "H4 :" << H4 << std::endl;
std::cout << "errore :" << errore << std::endl;
std::cout << "error :" << error << std::endl;
*/
EXPECT(gtsam::assert_equal(error,errore,1e-2));
EXPECT(gtsam::assert_equal(H1,H1e,1e-2));
EXPECT(gtsam::assert_equal(H2,H2e,1e-2));
EXPECT(gtsam::assert_equal(H3,H3e,1e-2));
EXPECT(gtsam::assert_equal(H4,H4e,1e-2));

}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}