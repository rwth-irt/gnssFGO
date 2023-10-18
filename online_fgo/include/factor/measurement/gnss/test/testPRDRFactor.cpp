//
// Created by lars on 31.01.22.
//

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <factors/measurement/gnss/PrDrFactor.h>

#include <iostream>


TEST(CarrierPhaseFactor, Constructor){
  //INIT PARAMS
  double pseuRa = 25062948.043;
  double dPseuRa = 598.3567;
  gtsam::Vector3 angRate(0.02,0,0);
  gtsam::Point3 pointSat(11322896.8543,23963601.5727,1733488.2027);
  gtsam::Point3 velSat(-177.457,312.672,-3163.235);
  gtsam::Point3 lb (0,5,0);
  gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(2.0, 2.0));
  //CREATE FACTORS
  fgo::factors::GnssFactor factor(1,2,3, pseuRa, dPseuRa, angRate, pointSat, velSat, lb, model);
  //CHECK ERROR
  gtsam::Pose3 poseRec(gtsam::Rot3::Ypr(0.6,0,0.3),gtsam::Point3(4,7,9));
  gtsam::Vector3 vel(0,3,7);
  gtsam::Vector2 cbd (1,0.1);
  gtsam::Matrix H1 = gtsam::Matrix26(),H2,H3;
  gtsam::Vector2 error = factor.evaluateError(poseRec, vel, cbd,H1,H2,H3);
  //CREATE ERROR
  gtsam::Matrix H1e = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Pose3>(
          boost::bind(&fgo::factors::GnssFactor::evaluateError, &factor, boost::placeholders::_1, vel, cbd, boost::none, boost::none, boost::none), poseRec);
  gtsam::Matrix H2e = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Vector3>(
          boost::bind(&fgo::factors::GnssFactor::evaluateError, &factor, poseRec, boost::placeholders::_1, cbd, boost::none, boost::none, boost::none), vel);
  gtsam::Matrix H3e = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Vector2>(
          boost::bind(&fgo::factors::GnssFactor::evaluateError, &factor, poseRec, vel, boost::placeholders::_1, boost::none, boost::none, boost::none), cbd);
  gtsam::Matrix66 E2B = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, poseRec.rotation().matrix()).finished();
  H1e = H1e * E2B.transpose(); //H1 position in ECEF and Hexpected position in Body, so have to rotate, when pose has rotation
  gtsam::Vector3 traRec = poseRec.translation() + poseRec.rotation() * lb;
  gtsam::Matrix13 e;
  double range = gtsam::distance3(traRec, pointSat, e);
  gtsam::Vector2 errore = gtsam::Vector2 (range - pseuRa + cbd(0), e * (vel - velSat) + cbd(1) - dPseuRa);
  //PRINT
  /*
  std::cout << "H1e :" << H1e << std::endl;
  std::cout << "H1 :" << H1 << std::endl;
  //std::cout << "H2e :" << H2e << std::endl;
  //std::cout << "H2 :" << H2 << std::endl;
  //std::cout << "H3e :" << H3e << std::endl;
  //std::cout << "H3 :" << H3 << std::endl;
  std::cout << "errore :" << errore << std::endl;
  std::cout << "error :" << error << std::endl;
  */
  EXPECT(gtsam::assert_equal(error,errore,1e-2))
  EXPECT(gtsam::assert_equal(H1,H1e,1e-2))
  EXPECT(gtsam::assert_equal(H2,H2e,1e-2))
  EXPECT(gtsam::assert_equal(H3,H3e,1e-2))

}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}