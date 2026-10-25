/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// ---- scan3d patch (2026-10-25, branch ovopt-20261025) ----
// Patched copy of open_vins ov_msckf/src/update/UpdaterMSCKF.cpp (upstream commit 69488123), built into ov_bridge
// instead of the upstream file. All changes behind ovpatch/ovopt.h flags, default off = upstream code path:
//  - par_tri (exact): step 3 triangulation + Gauss-Newton of each feature on the OpenCV pool. Each call only reads the
//    clone poses and writes its own feature; failures are then removed in the original order (same calls as upstream,
//    GN also runs when triangulation failed, as upstream does).
//  - par_jac (exact): step 4 per-feature Jacobian + nullspace projection + chi2 test on the pool (reads the state only),
//    then the accepted features are stacked into Hx_big / res_big serially in the original order.
//  - givens_rm (exact): step 5 measurement compression = the same Givens loop on a row-major copy (see there).
//  - qr_compress (near-exact): step 5 measurement compression with Householder QR (Q^T applied to H and r) instead of
//    the Givens loop: the same projection up to row signs / rounding (R = sigma^2 I is invariant).
//  - ovopt::Lap profiling (timing only).
#include "ovopt.h"

#include "update/UpdaterMSCKF.h"  // scan3d: dir prefix (copy lives in ovpatch/)

#include "update/UpdaterHelper.h"  // scan3d: dir prefix (copy lives in ovpatch/)

#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/LandmarkRepresentation.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

UpdaterMSCKF::UpdaterMSCKF(UpdaterOptions &options, ov_core::FeatureInitializerOptions &feat_init_options) : _options(options) {

  // Save our raw pixel noise squared
  _options.sigma_pix_sq = std::pow(_options.sigma_pix, 2);

  // Save our feature initializer
  initializer_feat = std::shared_ptr<ov_core::FeatureInitializer>(new ov_core::FeatureInitializer(feat_init_options));

  // Initialize the chi squared test table with confidence level 0.95
  // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
  for (int i = 1; i < 500; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

void UpdaterMSCKF::update(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec) {

  // Return if no features
  if (feature_vec.empty())
    return;

  // Start timing
  boost::posix_time::ptime rT0, rT1, rT2, rT3, rT4, rT5;
  rT0 = boost::posix_time::microsec_clock::local_time();
  ovopt::Lap prof_lap;  // scan3d: timing only
  ovopt::count(ovopt::C_MSCKF_FEATS, (long)feature_vec.size());
  ovopt::h.msckf_in += (long)feature_vec.size();  // scan3d health (ovrobust)

  // 0. Get all timestamps our clones are at (and thus valid measurement times)
  std::vector<double> clonetimes;
  for (const auto &clone_imu : state->_clones_IMU) {
    clonetimes.emplace_back(clone_imu.first);
  }

  // 1. Clean all feature measurements and make sure they all have valid clone times
  auto it0 = feature_vec.begin();
  while (it0 != feature_vec.end()) {

    // Clean the feature
    (*it0)->clean_old_measurements(clonetimes);

    // Count how many measurements
    int ct_meas = 0;
    for (const auto &pair : (*it0)->timestamps) {
      ct_meas += (*it0)->timestamps[pair.first].size();
    }

    // Remove if we don't have enough
    if (ct_meas < 2) {
      (*it0)->to_delete = true;
      it0 = feature_vec.erase(it0);
    } else {
      it0++;
    }
  }
  rT1 = boost::posix_time::microsec_clock::local_time();

  // 2. Create vector of cloned *CAMERA* poses at each of our clone timesteps
  std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>> clones_cam;
  for (const auto &clone_calib : state->_calib_IMUtoCAM) {

    // For this camera, create the vector of camera poses
    std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
    for (const auto &clone_imu : state->_clones_IMU) {

      // Get current camera pose
      Eigen::Matrix<double, 3, 3> R_GtoCi = clone_calib.second->Rot() * clone_imu.second->Rot();
      Eigen::Matrix<double, 3, 1> p_CioinG = clone_imu.second->pos() - R_GtoCi.transpose() * clone_calib.second->pos();

      // Append to our map
      clones_cami.insert({clone_imu.first, FeatureInitializer::ClonePose(R_GtoCi, p_CioinG)});
    }

    // Append to our map
    clones_cam.insert({clone_calib.first, clones_cami});
  }
  prof_lap.to(ovopt::S_MSCKF_CLEAN);

  // 3. Try to triangulate all MSCKF or new SLAM features that have measurements
  if (ovopt::g.par_tri) {
    // scan3d par_tri: the same per-feature calls as the loop below, on the OpenCV pool; removal in the original order
    const bool tri_1d = initializer_feat->config().triangulate_1d;
    const bool refine = initializer_feat->config().refine_features;
    const int n_feat = (int)feature_vec.size();
    std::vector<char> ok(n_feat, 0);
    ovopt::par_for(true, n_feat, [&](int i) {
      bool success_tri = true;
      if (tri_1d) {
        success_tri = initializer_feat->single_triangulation_1d(feature_vec[i], clones_cam);
      } else {
        success_tri = initializer_feat->single_triangulation(feature_vec[i], clones_cam);
      }
      bool success_refine = true;
      if (refine) {
        success_refine = initializer_feat->single_gaussnewton(feature_vec[i], clones_cam);
      }
      ok[i] = (success_tri && success_refine) ? 1 : 0;
    });
    std::vector<std::shared_ptr<Feature>> kept;
    kept.reserve(feature_vec.size());
    for (int i = 0; i < n_feat; i++) {
      if (ok[i]) {
        kept.push_back(feature_vec[i]);
      } else {
        feature_vec[i]->to_delete = true;
      }
    }
    feature_vec.swap(kept);
  } else {
    auto it1 = feature_vec.begin();
    while (it1 != feature_vec.end()) {

      // Triangulate the feature and remove if it fails
      bool success_tri = true;
      if (initializer_feat->config().triangulate_1d) {
        success_tri = initializer_feat->single_triangulation_1d(*it1, clones_cam);
      } else {
        success_tri = initializer_feat->single_triangulation(*it1, clones_cam);
      }

      // Gauss-newton refine the feature
      bool success_refine = true;
      if (initializer_feat->config().refine_features) {
        success_refine = initializer_feat->single_gaussnewton(*it1, clones_cam);
      }

      // Remove the feature if not a success
      if (!success_tri || !success_refine) {
        (*it1)->to_delete = true;
        it1 = feature_vec.erase(it1);
        continue;
      }
      it1++;
    }
  }
  rT2 = boost::posix_time::microsec_clock::local_time();
  prof_lap.to(ovopt::S_MSCKF_TRI);
  ovopt::h.msckf_tri += (long)feature_vec.size();  // scan3d health (ovrobust)

  // Calculate the max possible measurement size
  size_t max_meas_size = 0;
  for (size_t i = 0; i < feature_vec.size(); i++) {
    for (const auto &pair : feature_vec.at(i)->timestamps) {
      max_meas_size += 2 * feature_vec.at(i)->timestamps[pair.first].size();
    }
  }

  // Calculate max possible state size (i.e. the size of our covariance)
  // NOTE: that when we have the single inverse depth representations, those are only 1dof in size
  size_t max_hx_size = state->max_covariance_size();
  for (auto &landmark : state->_features_SLAM) {
    max_hx_size -= landmark.second->size();
  }

  // Large Jacobian and residual of *all* features for this update
  Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
  std::unordered_map<std::shared_ptr<Type>, size_t> Hx_mapping;
  std::vector<std::shared_ptr<Type>> Hx_order_big;
  size_t ct_jacob = 0;
  size_t ct_meas = 0;

  // 4. Compute linear system for each feature, nullspace project, and reject
  if (ovopt::g.par_jac) {
    // scan3d par_jac: per-feature part of the loop below on the OpenCV pool (state is only read), then the same
    // stacking as below, serially and in the original feature order
    struct FeatSys {
      Eigen::MatrixXd H_x;
      Eigen::VectorXd res;
      std::vector<std::shared_ptr<Type>> Hx_order;
      bool keep = false;
    };
    const int n_feat = (int)feature_vec.size();
    std::vector<FeatSys> fsys(n_feat);
    ovopt::par_for(true, n_feat, [&](int i) {
      const std::shared_ptr<Feature> &fp = feature_vec[i];

      // Convert our feature into our current format
      UpdaterHelper::UpdaterHelperFeature feat;
      feat.featid = fp->featid;
      feat.uvs = fp->uvs;
      feat.uvs_norm = fp->uvs_norm;
      feat.timestamps = fp->timestamps;

      // If we are using single inverse depth, then it is equivalent to using the msckf inverse depth
      feat.feat_representation = state->_options.feat_rep_msckf;
      if (state->_options.feat_rep_msckf == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {
        feat.feat_representation = LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
      }

      // Save the position and its fej value
      if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
        feat.anchor_cam_id = fp->anchor_cam_id;
        feat.anchor_clone_timestamp = fp->anchor_clone_timestamp;
        feat.p_FinA = fp->p_FinA;
        feat.p_FinA_fej = fp->p_FinA;
      } else {
        feat.p_FinG = fp->p_FinG;
        feat.p_FinG_fej = fp->p_FinG;
      }

      // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
      Eigen::MatrixXd H_f;
      Eigen::MatrixXd &H_x = fsys[i].H_x;
      Eigen::VectorXd &res = fsys[i].res;
      std::vector<std::shared_ptr<Type>> &Hx_order = fsys[i].Hx_order;

      // Get the Jacobian for this feature
      UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

      // Nullspace project
      UpdaterHelper::nullspace_project_inplace(H_f, H_x, res);

      /// Chi2 distance check
      Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
      Eigen::MatrixXd S = H_x * P_marg * H_x.transpose();
      S.diagonal() += _options.sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
      double chi2 = res.dot(S.llt().solve(res));

      // Get our threshold (we precompute up to 500 but handle the case that it is more)
      double chi2_check;
      if (res.rows() < 500) {
        chi2_check = chi_squared_table.at((int)res.rows());  // .at: const lookup (table filled 1..499 in the ctor)
      } else {
        boost::math::chi_squared chi_squared_dist(res.rows());
        chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
        PRINT_WARNING(YELLOW "chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
      }
      fsys[i].keep = !(chi2 > _options.chi2_multipler * chi2_check);
    });
    std::vector<std::shared_ptr<Feature>> kept;
    kept.reserve(feature_vec.size());
    for (int i = 0; i < n_feat; i++) {
      if (!fsys[i].keep) {
        feature_vec[i]->to_delete = true;
        continue;
      }
      const Eigen::MatrixXd &H_x = fsys[i].H_x;
      const Eigen::VectorXd &res = fsys[i].res;

      // We are good!!! Append to our large H vector
      size_t ct_hx = 0;
      for (const auto &var : fsys[i].Hx_order) {

        // Ensure that this variable is in our Jacobian
        if (Hx_mapping.find(var) == Hx_mapping.end()) {
          Hx_mapping.insert({var, ct_jacob});
          Hx_order_big.push_back(var);
          ct_jacob += var->size();
        }

        // Append to our large Jacobian
        Hx_big.block(ct_meas, Hx_mapping[var], H_x.rows(), var->size()) = H_x.block(0, ct_hx, H_x.rows(), var->size());
        ct_hx += var->size();
      }

      // Append our residual and move forward
      res_big.block(ct_meas, 0, res.rows(), 1) = res;
      ct_meas += res.rows();
      kept.push_back(feature_vec[i]);
    }
    feature_vec.swap(kept);
  }
  auto it2 = feature_vec.begin();
  while (!ovopt::g.par_jac && it2 != feature_vec.end()) {

    // Convert our feature into our current format
    UpdaterHelper::UpdaterHelperFeature feat;
    feat.featid = (*it2)->featid;
    feat.uvs = (*it2)->uvs;
    feat.uvs_norm = (*it2)->uvs_norm;
    feat.timestamps = (*it2)->timestamps;

    // If we are using single inverse depth, then it is equivalent to using the msckf inverse depth
    feat.feat_representation = state->_options.feat_rep_msckf;
    if (state->_options.feat_rep_msckf == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {
      feat.feat_representation = LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
    }

    // Save the position and its fej value
    if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
      feat.anchor_cam_id = (*it2)->anchor_cam_id;
      feat.anchor_clone_timestamp = (*it2)->anchor_clone_timestamp;
      feat.p_FinA = (*it2)->p_FinA;
      feat.p_FinA_fej = (*it2)->p_FinA;
    } else {
      feat.p_FinG = (*it2)->p_FinG;
      feat.p_FinG_fej = (*it2)->p_FinG;
    }

    // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
    Eigen::MatrixXd H_f;
    Eigen::MatrixXd H_x;
    Eigen::VectorXd res;
    std::vector<std::shared_ptr<Type>> Hx_order;

    // Get the Jacobian for this feature
    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

    // Nullspace project
    UpdaterHelper::nullspace_project_inplace(H_f, H_x, res);

    /// Chi2 distance check
    Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
    Eigen::MatrixXd S = H_x * P_marg * H_x.transpose();
    S.diagonal() += _options.sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
    double chi2 = res.dot(S.llt().solve(res));

    // Get our threshold (we precompute up to 500 but handle the case that it is more)
    double chi2_check;
    if (res.rows() < 500) {
      chi2_check = chi_squared_table[res.rows()];
    } else {
      boost::math::chi_squared chi_squared_dist(res.rows());
      chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
      PRINT_WARNING(YELLOW "chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
    }

    // Check if we should delete or not
    if (chi2 > _options.chi2_multipler * chi2_check) {
      (*it2)->to_delete = true;
      it2 = feature_vec.erase(it2);
      // PRINT_DEBUG("featid = %d\n", feat.featid);
      // PRINT_DEBUG("chi2 = %f > %f\n", chi2, _options.chi2_multipler*chi2_check);
      // std::stringstream ss;
      // ss << "res = " << std::endl << res.transpose() << std::endl;
      // PRINT_DEBUG(ss.str().c_str());
      continue;
    }

    // We are good!!! Append to our large H vector
    size_t ct_hx = 0;
    for (const auto &var : Hx_order) {

      // Ensure that this variable is in our Jacobian
      if (Hx_mapping.find(var) == Hx_mapping.end()) {
        Hx_mapping.insert({var, ct_jacob});
        Hx_order_big.push_back(var);
        ct_jacob += var->size();
      }

      // Append to our large Jacobian
      Hx_big.block(ct_meas, Hx_mapping[var], H_x.rows(), var->size()) = H_x.block(0, ct_hx, H_x.rows(), var->size());
      ct_hx += var->size();
    }

    // Append our residual and move forward
    res_big.block(ct_meas, 0, res.rows(), 1) = res;
    ct_meas += res.rows();
    it2++;
  }
  rT3 = boost::posix_time::microsec_clock::local_time();
  prof_lap.to(ovopt::S_MSCKF_JAC);

  ovopt::h.msckf_used += (long)feature_vec.size();  // scan3d health (ovrobust): after the chi2 test
  // We have appended all features to our Hx_big, res_big
  // Delete it so we do not reuse information
  for (size_t f = 0; f < feature_vec.size(); f++) {
    feature_vec[f]->to_delete = true;
  }

  // Return if we don't have anything and resize our matrices
  if (ct_meas < 1) {
    return;
  }
  assert(ct_meas <= max_meas_size);
  assert(ct_jacob <= max_hx_size);
  res_big.conservativeResize(ct_meas, 1);
  Hx_big.conservativeResize(ct_meas, ct_jacob);
  ovopt::count(ovopt::C_MSCKF_ROWS, (long)ct_meas);
  ovopt::count(ovopt::C_MSCKF_COLS, (long)ct_jacob);

  // 5. Perform measurement compression
  if (ovopt::g.givens_rm && !ovopt::g.qr_compress && Hx_big.rows() > Hx_big.cols()) {
    // scan3d givens_rm (exact): the loop of UpdaterHelper::measurement_compress_inplace, same makeGivens / applyOnTheLeft
    // calls in the same order, on a row-major copy viewed through a dynamic-stride Map. Eigen applies a rotation to two
    // rows with its scalar loop in both cases (a column-major row and a dynamic-stride row have no packet access), so
    // every element gets the same arithmetic; only the memory walk changes (rows contiguous instead of rows() apart).
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMat;
    RowMat Hr = Hx_big;
    Eigen::Map<RowMat, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> H_x(Hr.data(), Hr.rows(), Hr.cols(),
                                                                          Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>(Hr.cols(), 1));
    Eigen::VectorXd &res = res_big;
    Eigen::JacobiRotation<double> tempHo_GR;
    for (int n = 0; n < H_x.cols(); n++) {
      for (int m = (int)H_x.rows() - 1; m > n; m--) {
        // Givens matrix G
        tempHo_GR.makeGivens(H_x(m - 1, n), H_x(m, n));
        // Multiply G to the corresponding lines (m-1,m) in each matrix
        (H_x.block(m - 1, n, 2, H_x.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
        (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      }
    }
    int r = std::min(H_x.rows(), H_x.cols());
    Hx_big = H_x.topRows(r);
    res.conservativeResize(r, res.cols());
  } else if (ovopt::g.qr_compress && Hx_big.rows() > Hx_big.cols()) {
    // scan3d qr_compress (near-exact): Householder QR, H <- R (upper, cols x cols), r <- (Q^T r)(0:cols)
    const int r = (int)Hx_big.cols();
    Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixXd>> qr(Hx_big);  // in place: Hx_big holds R + reflectors
    res_big.applyOnTheLeft(qr.householderQ().adjoint());
    Hx_big.conservativeResize(r, Hx_big.cols());
    Hx_big.triangularView<Eigen::StrictlyLower>().setZero();
    res_big.conservativeResize(r, 1);
  } else {
    UpdaterHelper::measurement_compress_inplace(Hx_big, res_big);
  }
  if (Hx_big.rows() < 1) {
    return;
  }
  rT4 = boost::posix_time::microsec_clock::local_time();
  prof_lap.to(ovopt::S_MSCKF_COMP);

  // Our noise is isotropic, so make it here after our compression
  Eigen::MatrixXd R_big = _options.sigma_pix_sq * Eigen::MatrixXd::Identity(res_big.rows(), res_big.rows());

  // 6. With all good features update the state
  StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);
  rT5 = boost::posix_time::microsec_clock::local_time();
  prof_lap.to(ovopt::S_MSCKF_EKF);

  // Debug print timing information
  PRINT_ALL("[MSCKF-UP]: %.4f seconds to clean\n", (rT1 - rT0).total_microseconds() * 1e-6);
  PRINT_ALL("[MSCKF-UP]: %.4f seconds to triangulate\n", (rT2 - rT1).total_microseconds() * 1e-6);
  PRINT_ALL("[MSCKF-UP]: %.4f seconds create system (%d features)\n", (rT3 - rT2).total_microseconds() * 1e-6, (int)feature_vec.size());
  PRINT_ALL("[MSCKF-UP]: %.4f seconds compress system\n", (rT4 - rT3).total_microseconds() * 1e-6);
  PRINT_ALL("[MSCKF-UP]: %.4f seconds update state (%d size)\n", (rT5 - rT4).total_microseconds() * 1e-6, (int)res_big.rows());
  PRINT_ALL("[MSCKF-UP]: %.4f seconds total\n", (rT5 - rT1).total_microseconds() * 1e-6);
}
