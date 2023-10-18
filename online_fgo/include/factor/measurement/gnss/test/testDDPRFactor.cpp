//
// Created by lars on 26.01.22.
//

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <factors/measurement/gnss/DDPrDrFactor.h>

#include <iostream>

TEST(CarrierPhaseFactor, Constructor){
  //INIT PARAMS
  double ddPseuRa = 100.23;
  double ddDoppRa = 0.4;
  gtsam::Point3 pointSatM(11322896.8543,23963601.5727,1733488.2027);
  gtsam::Point3 pointSatI(23963601.5727,11322896.8543,1733488.2027);
  gtsam::Point3 pointBase(1132.09,239.5727,1733.2027);
  gtsam::Vector3 lb (0,0,5);
  gtsam::Vector3 velSatM(123,342,324);
  gtsam::Vector3 velSatI(13,32,-324);
  gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(2.0));
  //CREATE FACTORS
  fgo::factors::DDPRDRFactor factor(1, 2, 3, ddPseuRa, ddDoppRa, pointSatM, velSatM, pointSatI, velSatI, pointBase, lb, model);
  //CREATE ESTIMATED STATES
  gtsam::Pose3 poseBody(gtsam::Rot3::Ypr(1,0.5,0),gtsam::Point3(4,1,0));
  gtsam::Vector3 vel(12,2,4);
  gtsam::Vector3 omega(0.3,0.7,1.0);
  gtsam::Matrix H1 = gtsam::Matrix26(), H2 = gtsam::Matrix23(), H3 =  gtsam::Matrix23();
  //CALCULATE ERRORS
  gtsam::Vector2 error = factor.evaluateError(poseBody, vel, omega, H1, H2, H3);
  //ESTIMATE ERRORS
  gtsam::Matrix H1e = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Pose3>(
          boost::bind(&fgo::factors::DDPRDRFactor::evaluateError, &factor,
                      boost::placeholders::_1, vel, omega, boost::none, boost::none, boost::none), poseBody);

  gtsam::Matrix66 E2B = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, poseBody.rotation().matrix()).finished();

  H1e = H1e * E2B.transpose(); //H1 position in ECEF and Hexpected position in Body, so have to rotate, when pose has rotation

  gtsam::Vector3 traRec = poseBody.translation() + poseBody.rotation() * lb;
  gtsam::Matrix13 e_RM, e_RI, e_BM, e_BI;
  double range = gtsam::distance3(traRec, pointSatM, e_RM) - gtsam::distance3(traRec, pointSatI, e_RI)
                 - (gtsam::distance3(pointBase, pointSatM, e_BM) - gtsam::distance3(pointBase, pointSatI, e_BI));
  double drange = e_RM * (vel - velSatM); double drange2 = - e_RI * (vel - velSatI);
  double drange3 = e_BM * (-velSatM); double drange4 = e_BI * (-velSatI);
  gtsam::Vector2 errore(range - ddPseuRa,  (drange - drange2) - (drange3 - drange4) - ddDoppRa);

  //PRINTS
  /*
  std::cout << "H1e :" << H1e << std::endl;
  std::cout << "H1 :" << H1 << std::endl;
  std::cout << "errore :" << errore << std::endl;
  std::cout << "error :" << error << std::endl;
  */

  EXPECT(gtsam::assert_equal(errore,error,1e-2));
  EXPECT(gtsam::assert_equal(H1e,H1,1e-2));

}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}