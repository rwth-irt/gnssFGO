//
// Created by lars on 17.02.22.
//

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <factors/motion/GPWNOJPriorPose3.h>

#include "utils/NavigationTools.h"

#include <iostream>

TEST(GPPriorPose3WNOJ, Constructor) {
//INIT PARAMS
gtsam::Vector3 omegai(0.0,0.0,0.05);
gtsam::Vector3 omegaj(0.0,0.0,0.3);
gtsam::Vector3 accai(0.0,0.2,0.0);
gtsam::Vector3 accgi(0.0,0.0,0.05);
gtsam::Vector3 accaj(0.0,0.2,0.0);
gtsam::Vector3 accgj(0.0,0.0,0.05);
double deltat = 5;
gtsam::noiseModel::Gaussian::shared_ptr model = gtsam::noiseModel::Gaussian::Covariance(0.5 * gtsam::I_6x6);
//CREATE FACTORS
fgo::factors::GaussianProcessPriorPose3WNOJ factor(1, 2, 3, 4, omegai, omegaj, accai, accgi, accaj, accgj, deltat, model);
//CHECK ERROR
gtsam::Pose3 posei(gtsam::Rot3::Ypr(0,0,0),gtsam::Point3(0,0,0));
gtsam::Pose3 posej(gtsam::Rot3::Ypr(0.25,0,0),gtsam::Point3(0,5,0));
gtsam::Vector3 veli(0,1,0);
gtsam::Vector3 velj(0,2,0);
gtsam::Matrix H1 = gtsam::Matrix96(),H2 = gtsam::Matrix93(),H3 = gtsam::Matrix96(),H4 = gtsam::Matrix93();
gtsam::Vector9 error = factor.evaluateError(posei, veli, posej, velj, H1, H2, H3, H4);
//CREATE ERROR
gtsam::Matrix96 H1e = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Pose3>(
        boost::bind(&fgo::factors::GaussianProcessPriorPose3WNOJ::evaluateError, &factor, boost::placeholders::_1, veli, posej, velj,
                    boost::none, boost::none,  boost::none, boost::none), posei);
gtsam::Matrix93 H2e = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Vector3>(
        boost::bind(&fgo::factors::GaussianProcessPriorPose3WNOJ::evaluateError, &factor, posei, boost::placeholders::_1, posej, velj,
                    boost::none, boost::none,  boost::none, boost::none), veli);
gtsam::Matrix96 H3e = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Pose3>(
        boost::bind(&fgo::factors::GaussianProcessPriorPose3WNOJ::evaluateError, &factor, posei, veli, boost::placeholders::_1, velj,
                    boost::none, boost::none,  boost::none, boost::none), posej);
gtsam::Matrix93 H4e = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Vector3>(
        boost::bind(&fgo::factors::GaussianProcessPriorPose3WNOJ::evaluateError, &factor, posei, veli, posej, boost::placeholders::_1,
                    boost::none, boost::none,  boost::none, boost::none), velj);
gtsam::Matrix66 E2B = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, posei.rotation().matrix()).finished();
gtsam::Matrix66 E2B2= (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, posej.rotation().matrix()).finished();
H1e = H1e * E2B.transpose(); //H1 position in ECEF and Hexpected position in Body, so have to rotate, when pose has rotation
H3e = H3e * E2B2.transpose(); //because of auto diff correction already in factor
gtsam::Vector6 vel1 = (gtsam::Vector6() << omegai,veli).finished();
gtsam::Vector6 vel2 = (gtsam::Vector6() << omegaj,velj).finished();
gtsam::Vector6 acc1 = fgo::utils::convertGbAbtoGbAw(accgi,accai,posei);
gtsam::Vector6 acc2 = fgo::utils::convertGbAbtoGbAw(accgj,accaj,posej);
gtsam::Vector6 r = gtsam::Pose3::Logmap(gtsam::Pose3(posei.rotation().inverse() * posej.rotation(),
                                                     - posei.translation() + posej.translation()));
gtsam::Vector9 errore = (gtsam::Vector9() << (r - vel1 * deltat - acc1/2 * pow(deltat,2)),
        (posej.rotation().transpose() * vel2.block<3,1>(3,0) - posei.rotation().transpose() * (vel1.block<3,1>(3,0) + accai*deltat))).finished();
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
EXPECT(gtsam::assert_equal(error,errore,1e-2))
EXPECT(gtsam::assert_equal(H1,H1e,1e-2))
EXPECT(gtsam::assert_equal(H2,H2e,1e-2))
EXPECT(gtsam::assert_equal(H3,H3e,1e-2))
EXPECT(gtsam::assert_equal(H4,H4e,1e-2))
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
