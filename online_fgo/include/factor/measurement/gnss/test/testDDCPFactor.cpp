//
// Created by lars on 26.01.22.
//

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>

#include <factors/measurement/gnss/DDCpFactor.h>
#include <iostream>

TEST(CarrierPhaseFactor, Constructor){
  //TEST DDCARPHA FACTOR
  //INIT PARAMS
  double ddCarPha = 0.37;
  gtsam::Point3 pointSatM(11322896.8543,23963601.5727,1733488.2027);
  gtsam::Point3 pointSatI(23963601.5727,11322896.8543,11322896.8543);
  gtsam::Point3 pointBase(100, 100, 0);
  int nAmbiguity = 0;
  gtsam::Point3 lb (0,0,2);
  double lambda = 0.19;
  gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(2.0));
  //CREATE FACTOR
  fgo::factors::DDCarrierPhaseFactor factor(1,2,ddCarPha,pointSatM,
                                            pointSatI, pointBase, nAmbiguity, lb, lambda,model);
  //CREATE ESTIMATES
  gtsam::Pose3 poseBody(gtsam::Rot3::Ypr(0, 0, 0), gtsam::Point3(0, 0, 0));
  gtsam::Vector2 ambiguity(200,100);

  //CREATE FACTOR
  gtsam::Matrix H1 = gtsam::Matrix16(), H2;
  double error = factor.evaluateError(poseBody, ambiguity, H1).value();

  //EXPECTED ERRORS
  gtsam::Matrix H1e = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Pose3>(
          boost::bind(&fgo::factors::DDCarrierPhaseFactor::evaluateError, &factor, boost::placeholders::_1, ambiguity, boost::none, boost::none), poseBody);
  gtsam::Matrix66 E2B = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, poseBody.rotation().matrix()).finished();
  H1e = H1e * E2B.transpose(); //H1 position in ECEF and Hexpected position in Body, so have to rotate, when pose has rotation

  gtsam::Vector3 traRec = poseBody.translation() + poseBody.rotation() * lb;
  double range = gtsam::distance3(traRec, pointSatM) - gtsam::distance3(traRec, pointSatI)
                 - (gtsam::distance3(pointBase, pointSatM) - gtsam::distance3(pointBase, pointSatI));
  double errore = range-ddCarPha*lambda - lambda*ambiguity(nAmbiguity);
  EXPECT_DOUBLES_EQUAL(error,errore,1e-2)
  EXPECT(gtsam::assert_equal(H1e,H1,1e-2))

  //PRINTS
  /*
  std::cout << error << std::endl;
  std::cout << errore << std::endl;
  std::cout << H1 << std::endl;
  std::cout << H1e  << std::endl;
  */

}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}

