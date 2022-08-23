#include <iostream>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/base/serializationTestHelpers.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;


/* ************************************************************************* */
// Create GUIDs for Noisemodels
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Base , "gtsam_noiseModel_mEstimator_Base")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Null , "gtsam_noiseModel_mEstimator_Null")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Fair , "gtsam_noiseModel_mEstimator_Fair")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Huber, "gtsam_noiseModel_mEstimator_Huber")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Tukey, "gtsam_noiseModel_mEstimator_Tukey")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic,"gtsam_noiseModel_Isotropic")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Robust, "gtsam_noiseModel_Robust")
BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel")
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal")

/* ************************************************************************* */
// Create GUIDs for factors
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Pose3>, "gtsam::PriorFactor<gtsam::Pose3>")
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Pose3>, "gtsam::BetweenFactor<gtsam::Pose3>")
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsam::JacobianFactor")
BOOST_CLASS_EXPORT_GUID(gtsam::HessianFactor , "gtsam::HessianFactor")
BOOST_CLASS_EXPORT_GUID(gtsam::GaussianConditional , "gtsam::GaussianConditional")


/* ************************************************************************* */
// Export all classes derived from Value
GTSAM_VALUE_EXPORT(gtsam::Cal3_S2)
GTSAM_VALUE_EXPORT(gtsam::Cal3Bundler)
GTSAM_VALUE_EXPORT(gtsam::Point3)
GTSAM_VALUE_EXPORT(gtsam::Pose3)
GTSAM_VALUE_EXPORT(gtsam::Rot3)
GTSAM_VALUE_EXPORT(gtsam::PinholeCamera<Cal3_S2>)
GTSAM_VALUE_EXPORT(gtsam::PinholeCamera<Cal3DS2>)
GTSAM_VALUE_EXPORT(gtsam::PinholeCamera<Cal3Bundler>)



/* ************************************************************************* */
int main() { 
  gtsam::ISAM2Params parameters;
  gtsam::ISAM2 solver(parameters);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initialValues;
  initialValues.clear();

  gtsam::Vector6 temp6;
  for (int i = 0; i < 6; ++i) {
    temp6[i] = 0.0001;
  }
  gtsam::noiseModel::Diagonal::shared_ptr noiseModel = gtsam::noiseModel::Diagonal::Sigmas(temp6);

  gtsam::Pose3 pose0(gtsam::Rot3(), gtsam::Point3(0, 0, 0));
  gtsam::Symbol symbol0('x', 0);
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(symbol0, pose0, noiseModel));
  initialValues.insert(symbol0, pose0);

  gtsam::Pose3 pose_b(gtsam::Rot3(), gtsam::Point3(0, 0, 1));
  int num = 10000;
  for (int i = 1; i < num; ++i) {
    gtsam::Pose3 posei(gtsam::Rot3(), gtsam::Point3(0, 0, i));
    gtsam::Symbol symboli('x', i);
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(symboli, posei, noiseModel));
    initialValues.insert(symboli, posei);

    gtsam::Symbol symbolk('x', i-1);
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(symbolk, symboli, pose_b));
  }  
  solver.update(graph, initialValues,
                gtsam::FastVector<gtsam::FactorIndex>());

  std::string binaryPath = "saved_solver.dat";
  try {
    std::ofstream outputStream(binaryPath);
    boost::archive::binary_oarchive outputArchive(outputStream);
    outputArchive << solver;
  } catch(...) {
    std::cerr << "error saving solver" << std::endl;
  }

  gtsam::ISAM2 solverFromDisk;
  try {
    std::ifstream ifs(binaryPath);
    boost::archive::binary_iarchive inputArchive(ifs);
    inputArchive >> solverFromDisk;
  } catch(...) {
    std::cerr << "error loading solver" << std::endl;
  }

  gtsam::Pose3 p1, p2;
  try {
    p1 = solver.calculateEstimate<gtsam::Pose3>(symbol0);
  } catch(std::exception &e) {
    std::cerr << "error calculating estimates" << std::endl;
  }

  try {
    p2 = solverFromDisk.calculateEstimate<gtsam::Pose3>(symbol0);
  } catch(std::exception &e) {
    std::cerr << "error calculating estimates from loaded solver" << std::endl;
  }
  return 0;
}
/* ************************************************************************* */
