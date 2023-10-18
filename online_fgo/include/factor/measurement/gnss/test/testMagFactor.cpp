//
// Created by lars on 31.01.22.
//

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <factors/measurement/gnss/MagFactor_eRb.h>

#include <iostream>

TEST(CarrierPhaseFactor, Constructor){
  //INIT PARAMS
  gtsam::Vector3 measFieldb(19496.2,902.2,45142.3);
  GeographicLib::MagneticModel magMod("wmm2020");
  gtsam::Vector3 magField_enu;
  gtsam::Vector3 posllh(51.233334,6.783333,0);
  magMod(2021.0 - 1900.0 + 45.0/265, posllh.x(), posllh.y(), posllh.z(), magField_enu.x(), magField_enu.y(), magField_enu.z());
  std::cout << magField_enu << std::endl;
  gtsam::Matrix enuConv = (gtsam::Matrix33() << 0, 1, 0,
          1, 0, 0,
          0, 0, -1).finished();
  gtsam::Vector3 magField_xyz = fgo::utils::earthToNavTrans(fgo::utils::llh2xyz(posllh)).transpose() * enuConv.transpose() * magField_enu;
  std::cout << magField_xyz << std::endl;
  gtsam::Point3 bias(1,3,4);
  gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(2.0));

  //CREATE FACTOR
  fgo::factors::MagFactor_eRb factor(1, measFieldb, magField_xyz, bias, model);
  //MAKE ERROR
  gtsam::Pose3 poseRec(gtsam::Rot3::Ypr(0,0,0),gtsam::Point3(0,0,0));
  gtsam::Matrix H1 = gtsam::Matrix16();
  gtsam::Vector error = factor.evaluateError(poseRec,H1);
  gtsam::Matrix H1e = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Pose3>(
          boost::bind(&fgo::factors::MagFactor_eRb::evaluateError, &factor, boost::placeholders::_1, boost::none), poseRec);
  //gtsam::Matrix66 E2B = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, poseRec.rotation().matrix()).finished();
  //H1 = H1 * E2B; //H1 position in ECEF and Hexpected position in Body, so have to rotate, when pose has rotation

  gtsam::Vector errore = poseRec.rotation().unrotate(magField_xyz) - measFieldb;

  //PRINT
  /*
  std::cout << "H1e :" << H1e << std::endl;
  std::cout << "H1 :" << H1 << std::endl;
  std::cout << "errore :" << errore << std::endl;
  std::cout << "error :" << error << std::endl;
   */
    EXPECT(gtsam::assert_equal(error,errore,1e-2)); //TODO
    EXPECT(gtsam::assert_equal(H1,H1e,1e-2));

}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}

