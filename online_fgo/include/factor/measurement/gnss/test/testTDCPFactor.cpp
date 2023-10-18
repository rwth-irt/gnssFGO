//
// Created by lars on 16.02.22.
//

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <factors/measurement/gnss/TripDCpFactor.h>

#include <iostream>
TEST(TripleDiffCPFactor, Constructor) {
//INIT PARAMS
  double ddphi_i = 41424.043;
  double ddphi_j = 23424.34;
  gtsam::Point3 pointMaster_i(3322896.8543, 2963601.5727, 133488.2027);
  gtsam::Point3 pointSatI_i(11322896.8543, 23963601.5727, 1733488.2027);
  gtsam::Point3 pointMaster_j(3322996.8543, 2963501.5727, 133988.2027);
  gtsam::Point3 pointSatI_j(11322596.8543, 23963401.5727, 1733688.2027);
  gtsam::Point3 pointBase(-177.457, 312.672, -3163.235);
  gtsam::Point3 lb(0, 5, 0);
  double lambda = 0.19;
  gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(2.0, 2.0));
//CREATE FACTORS
  fgo::factors::TripleDiffCPFactor factor(1, 2, ddphi_i, ddphi_j, pointMaster_i, pointSatI_i, pointMaster_j,
                                          pointSatI_j, pointBase, lb, lambda, model);
//CHECK ERROR
  gtsam::Pose3 posei(gtsam::Rot3::Ypr(0.6, 0, 0.3), gtsam::Point3(4, 7, 9));
  gtsam::Pose3 posej(gtsam::Rot3::Ypr(0.5, 0.3, 0.6), gtsam::Point3(14, -7, 1));
  gtsam::Matrix H1 = gtsam::Matrix16(), H2 = gtsam::Matrix16();
  double error = factor.evaluateError(posei, posej, H1, H2).value();
//CREATE ERROR
  gtsam::Matrix H1e = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
          boost::bind(&fgo::factors::TripleDiffCPFactor::evaluateError, &factor, boost::placeholders::_1, posej,
                      boost::none, boost::none), posei);
  gtsam::Matrix H2e = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
          boost::bind(&fgo::factors::TripleDiffCPFactor::evaluateError, &factor, posei, boost::placeholders::_1,
                      boost::none, boost::none), posej);
  gtsam::Matrix66 E2B = (gtsam::Matrix66()
          << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, posei.rotation().matrix()).finished();
  gtsam::Matrix66 E2B2 = (gtsam::Matrix66()
          << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, posej.rotation().matrix()).finished();
  H1e = H1e *
        E2B.transpose(); //H1 position in ECEF and Hexpected position in Body, so have to rotate, when pose has rotation
  H2e = H2e * E2B2.transpose();
  gtsam::Vector3 traReci = posei.translation() + posei.rotation() * lb;
  gtsam::Vector3 traRecj = posej.translation() + posej.rotation() * lb;
  double rangei = gtsam::distance3(traReci, pointMaster_i) - gtsam::distance3(pointBase, pointMaster_i) -
                  (gtsam::distance3(traReci, pointSatI_i) - gtsam::distance3(pointBase, pointSatI_i));
  double rangej = gtsam::distance3(traRecj, pointMaster_j) - gtsam::distance3(pointBase, pointMaster_j) -
                  (gtsam::distance3(traRecj, pointSatI_j) - gtsam::distance3(pointBase, pointSatI_j));
  double errore = rangej - rangei - lambda * (ddphi_j - ddphi_i);
//PRINT
/*
std::cout << "H1e :" << H1e << std::endl;
std::cout << "H1 :" << H1 << std::endl;
std::cout << "H2e :" << H2e << std::endl;
std::cout << "H2 :" << H2 << std::endl;
std::cout << "errore :" << errore << std::endl;
std::cout << "error :" << error << std::endl;
*/
  EXPECT_DOUBLES_EQUAL(error, errore, 1e-2)
  EXPECT(gtsam::assert_equal(H1, H1e, 1e-2))
  EXPECT(gtsam::assert_equal(H2, H2e, 1e-2))
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}