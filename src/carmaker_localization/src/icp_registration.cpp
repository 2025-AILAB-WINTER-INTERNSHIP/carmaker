#include "carmaker_localization/icp_registration.h"
#include "carmaker_localization/nanoflann.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <cmath>
#include <chrono>
#include <limits>
#include <utility>
#include <Eigen/Eigenvalues>

namespace carmaker_localization {

// ─────────────────────────────────────────────────────────────────────────────
// nanoflann point-cloud adapter for LandmarkFeature (2D: x, y)
// ─────────────────────────────────────────────────────────────────────────────
struct LandmarkFeatureCloud {
    const std::vector<LandmarkFeature>& pts;
    explicit LandmarkFeatureCloud(const std::vector<LandmarkFeature>& p) : pts(p) {}

    // nanoflann interface
    inline size_t kdtree_get_point_count() const { return pts.size(); }
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return (dim == 0) ? pts[idx].x : pts[idx].y;
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

using KDTree2D = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, LandmarkFeatureCloud>,
    LandmarkFeatureCloud,
    2 /* dims */>;

// ─────────────────────────────────────────────────────────────────────────────
// Helper: 2×2 대칭 행렬 [[a, b], [b, c]] 의 최대 고유값
// 고유값 계산: det(M - lambda I) = 0
//   → Euclidean 보수 탐색 반경 r² = χ²_threshold × λ_max(C_map) 계산에 사용
// ─────────────────────────────────────────────────────────────────────────────
static double maxEigenvalue2x2(double a, double b, double c) {
    double half_trace = 0.5 * (a + c);
    double disc       = std::sqrt(0.25 * (a - c) * (a - c) + b * b);
    return half_trace + disc;
}

// ─────────────────────────────────────────────────────────────────────────────
// Thread-local cache for KD-Tree index reuse
// ─────────────────────────────────────────────────────────────────────────────
struct KDTreeCache {
    const LandmarkFeature* cached_landmarks_data = nullptr;
    size_t cached_landmarks_size = 0;
    double cached_first_x = 0.0;

    std::map<int, std::vector<LandmarkFeature>> landmarks_by_class;
    std::map<int, std::unique_ptr<LandmarkFeatureCloud>> clouds;
    std::map<int, std::unique_ptr<KDTree2D>> kdtrees;

    void clear() {
        kdtrees.clear();
        clouds.clear();
        landmarks_by_class.clear();
        cached_landmarks_data = nullptr;
        cached_landmarks_size = 0;
        cached_first_x = 0.0;
    }
};

static thread_local KDTreeCache t_tree_cache;


// ─────────────────────────────────────────────────────────────────────────────
// IcpRegistration
// ─────────────────────────────────────────────────────────────────────────────
IcpRegistration::IcpRegistration(double fitness_threshold, int max_iterations, double vision_base_std, double vision_base_yaw_std, double min_search_radius, double max_covariance)
    : fitness_threshold_(fitness_threshold),
      max_iterations_(max_iterations),
      vision_base_std_(vision_base_std),
      vision_base_yaw_std_(vision_base_yaw_std),
      min_search_radius_(min_search_radius),
      max_covariance_(max_covariance) {}

RegistrationResult IcpRegistration::align(
    const std::vector<LocalFeature>& observed,
    const std::vector<LandmarkFeature>& landmarks,
    const Eigen::Isometry2d& initial_guess,
    bool collect_debug_info,
    bool collect_associations,
    bool collect_iteration_trace) {

    auto start_time = std::chrono::high_resolution_clock::now();
    RegistrationResult result;
    result.transform = initial_guess;
    result.num_observed = observed.size();
    result.num_landmarks = landmarks.size();
    if (collect_debug_info || collect_associations || collect_iteration_trace) {
        result.debug.emplace();
    }

    auto finish_debug_timing = [&]() {
        if (!result.debug) return;
        auto end_time = std::chrono::high_resolution_clock::now();
        result.debug->latency_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    };

    if (observed.empty() || landmarks.size() < 3) {
        finish_debug_timing();
        return result;
    }

    // Uniform step downsampling to prevent CPU saturation while preserving spatial distribution
    std::vector<LocalFeature> observed_used;
    if (observed.size() > 400) {
        size_t step = observed.size() / 400;
        observed_used.reserve(400);
        for (size_t i = 0; i < observed.size(); i += step)
            observed_used.push_back(observed[i]);
    } else {
        observed_used = observed;
    }

    // ── KDTree 캐싱 검사 및 지연 빌드 (Lazy Building) ──────────────────────────
    const LandmarkFeature* current_data = landmarks.empty() ? nullptr : landmarks.data();
    size_t current_size = landmarks.size();
    double current_first_x = landmarks.empty() ? 0.0 : landmarks.front().x;

    bool cache_miss = (t_tree_cache.cached_landmarks_data != current_data) ||
                      (t_tree_cache.cached_landmarks_size != current_size) ||
                      (t_tree_cache.cached_first_x != current_first_x);

    if (cache_miss) {
        t_tree_cache.clear();
        t_tree_cache.cached_landmarks_data = current_data;
        t_tree_cache.cached_landmarks_size = current_size;
        t_tree_cache.cached_first_x = current_first_x;

        if (!landmarks.empty()) {
            for (const auto& landmark : landmarks) {
                t_tree_cache.landmarks_by_class[landmark.class_id].push_back(landmark);
            }

            for (const auto& kv : t_tree_cache.landmarks_by_class) {
                int cid = kv.first;
                auto cloud = std::make_unique<LandmarkFeatureCloud>(kv.second);
                auto tree = std::make_unique<KDTree2D>(
                                 2, *cloud,
                                 nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf size */));
                tree->buildIndex();
                t_tree_cache.clouds.emplace(cid, std::move(cloud));
                t_tree_cache.kdtrees.emplace(cid, std::move(tree));
            }
        }
    }

    auto& landmarks_by_class = t_tree_cache.landmarks_by_class;
    auto& kdtrees = t_tree_cache.kdtrees;


    Eigen::Isometry2d transform  = initial_guess;
    double            prev_error = 1e9;
    int actual_iterations = 0;
    if (collect_iteration_trace && result.debug) {
        result.debug->iteration_traces.reserve(max_iterations_ + 1);
        IterationTrace initial_trace;
        initial_trace.iteration = -1;
        initial_trace.estimated_x = transform.translation().x();
        initial_trace.estimated_y = transform.translation().y();
        initial_trace.estimated_yaw = Eigen::Rotation2Dd(transform.linear()).angle();
        initial_trace.elapsed_ms = 0.0;
        result.debug->iteration_traces.push_back(std::move(initial_trace));
    }

    // ── ICP 반복 ──────────────────────────────────────────────────────────────
    for (int iter = 0; iter < max_iterations_; ++iter) {
        actual_iterations = iter + 1;
        std::vector<Eigen::Vector2d> dst_pts;
        double current_error      = 0.0;
        double total_error_weight = 0.0;

        // 1:1 유니크 정합 테이블: class_id → (landmark_idx → {LocalFeature, min_dist²})
        std::map<int, std::map<size_t, std::pair<LocalFeature, double>>> best_matches;
        Eigen::Matrix2d R_curr = transform.linear();

        for (const auto& obs : observed_used) {
            if (kdtrees.find(obs.class_id) == kdtrees.end()) continue;

            Eigen::Vector2d pt_obs(obs.x, obs.y);
            Eigen::Vector2d pt_map_guess = transform * pt_obs;

            // 1. 공분산을 Map 좌표계로 회전 후 정보 행렬(Information Matrix) 도출
            Eigen::Matrix2d C_obs;
            C_obs << obs.cov_xx, obs.cov_xy,
                     obs.cov_xy, obs.cov_yy;
            Eigen::Matrix2d C_map = R_curr * C_obs * R_curr.transpose();

            // 최소 탐색 반경(min_search_radius)을 고려하여 공분산에 최소 불확실성 마진 추가
            // 이를 통해 KD-tree 탐색 범위가 지나치게 좁아지는 것을 막고, Mahalanobis 거리 평가 시 게이트를 완화
            double min_var = (min_search_radius_ * min_search_radius_) / 9.0;
            C_map(0,0) += std::max(1e-3, min_var);
            C_map(1,1) += std::max(1e-3, min_var);
            Eigen::Matrix2d Info_map = C_map.inverse();

            // 2. KD-tree 반경 탐색
            //    마할라노비스 3σ 타원의 Euclidean 외접 반경: r² = χ²_th × λ_max(C_map)
            //    이 반경 내에 있는 점만 마할라노비스 조건을 통과할 수 있음(보수적 bound)
            double lambda_max       = maxEigenvalue2x2(C_map(0,0), C_map(0,1), C_map(1,1));
            double search_radius_sq = 9.0 * lambda_max;  // χ²_threshold = 9.0 (≈3σ)

            double query_pt[2] = { pt_map_guess.x(), pt_map_guess.y() };
            std::vector<nanoflann::ResultItem<uint32_t, double>> ret_matches;
            nanoflann::SearchParameters params;
            params.sorted = false;  // 순서 불필요 → 약간의 추가 절약
            const auto num_matches = kdtrees[obs.class_id]->radiusSearch(query_pt, search_radius_sq, ret_matches, params);
            if (num_matches == 0) continue;

            // 3. 반경 내 후보들 중 최소 마할라노비스 거리 선택
            double min_dist_sq = 9.0;  // χ² threshold
            int    best_landmark_idx = -1;

            for (const auto& m : ret_matches) {
                const auto& landmark = landmarks_by_class[obs.class_id][m.first];
                Eigen::Vector2d dp(landmark.x - pt_map_guess.x(), landmark.y - pt_map_guess.y());
                double d2 = dp.transpose() * Info_map * dp;
                if (d2 < min_dist_sq) {
                    min_dist_sq  = d2;
                    best_landmark_idx = static_cast<int>(m.first);
                }
            }

            // 4. 1:1 유니크 정합 강제 (이미 할당된 맵 포인트가 있다면 더 가까운 것만 생존)
            if (best_landmark_idx != -1) {
                auto& class_matches = best_matches[obs.class_id];
                if (class_matches.find(best_landmark_idx) == class_matches.end() ||
                    min_dist_sq < class_matches[best_landmark_idx].second) {
                    class_matches[best_landmark_idx] = {obs, min_dist_sq};
                }
            }
        }

        // SVD 연산 배열에 1:1 대응 관계 수집
        std::vector<LocalFeature> src_feats;
        for (const auto& class_pair : best_matches) {
            int class_id = class_pair.first;
            for (const auto& landmark_pair : class_pair.second) {
                size_t landmark_idx  = landmark_pair.first;
                const auto& feat = landmark_pair.second.first;
                double w = 1.0 / (feat.cov_xx + feat.cov_yy + 1e-4);
                src_feats.push_back(feat);
                dst_pts.push_back(Eigen::Vector2d(landmarks_by_class[class_id][landmark_idx].x,
                                                  landmarks_by_class[class_id][landmark_idx].y));
                current_error       += w * landmark_pair.second.second;
                total_error_weight  += w;
            }
        }

        if (src_feats.size() < 3) break;

        // Weighted Procrustes Analysis (Point-to-Point, closed-form, weighted by inv-cov trace)
        std::vector<double> weights;
        weights.reserve(src_feats.size());
        double total_weight = 0.0;
        for (const auto& feat : src_feats) {
            double w = 1.0 / (feat.cov_xx + feat.cov_yy + 1e-4);
            weights.push_back(w);
            total_weight += w;
        }

        if (total_weight < 1e-6) break;

        Eigen::Vector2d src_mean = Eigen::Vector2d::Zero();
        Eigen::Vector2d dst_mean = Eigen::Vector2d::Zero();
        for (size_t i = 0; i < src_feats.size(); ++i) {
            src_mean += weights[i] * Eigen::Vector2d(src_feats[i].x, src_feats[i].y);
            dst_mean += weights[i] * dst_pts[i];
        }
        src_mean /= total_weight;
        dst_mean /= total_weight;

        Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < src_feats.size(); ++i) {
            Eigen::Vector2d src_centered(src_feats[i].x - src_mean.x(), src_feats[i].y - src_mean.y());
            Eigen::Vector2d dst_centered = dst_pts[i] - dst_mean;
            W += weights[i] * dst_centered * src_centered.transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d R = svd.matrixU() * svd.matrixV().transpose();
        if (R.determinant() < 0) {
            Eigen::Matrix2d V = svd.matrixV();
            V.col(1) *= -1;
            R = svd.matrixU() * V.transpose();
        }

        Eigen::Vector2d t       = dst_mean - R * src_mean;
        transform.linear()      = R;
        transform.translation() = t;

        if (total_error_weight > 1e-6) current_error /= total_error_weight;
        if (collect_iteration_trace && result.debug) {
            IterationTrace trace;
            trace.iteration = iter;
            trace.estimated_x = transform.translation().x();
            trace.estimated_y = transform.translation().y();
            trace.estimated_yaw = Eigen::Rotation2Dd(transform.linear()).angle();
            trace.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_time).count();

            size_t trace_size = 0;
            for (const auto& class_pair : best_matches) trace_size += class_pair.second.size();
            trace.associations.reserve(trace_size);

            for (const auto& class_pair : best_matches) {
                const int class_id = class_pair.first;
                const auto& landmark_list = landmarks_by_class[class_id];
                for (const auto& landmark_pair : class_pair.second) {
                    const size_t landmark_idx = landmark_pair.first;
                    const auto& feat = landmark_pair.second.first;
                    const auto& landmark = landmark_list[landmark_idx];

                    Association assoc;
                    assoc.obs_x = feat.x;
                    assoc.obs_y = feat.y;
                    assoc.landmark_x = landmark.x;
                    assoc.landmark_y = landmark.y;
                    assoc.dist_mahalanobis = std::sqrt(landmark_pair.second.second);
                    assoc.class_id = static_cast<uint8_t>(class_id);
                    assoc.is_inlier = true;
                    trace.associations.push_back(assoc);
                }
            }
            result.debug->iteration_traces.push_back(std::move(trace));
        }
        if (std::abs(prev_error - current_error) < 1e-4) break;
        prev_error = current_error;
    }

    // ── Final Verification: KD-tree 가속 인라이어 카운팅 ─────────────────────
    int inliers             = 0;
    int valid_observed_count = 0;
    Eigen::Matrix2d R_final = transform.linear();

    // 3x3 2D Pose (x, y, yaw)에 대한 정보 행렬(Information Matrix, Hessian) 누적 변수
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

    std::vector<std::pair<int, size_t>> added_landmarks;

    if (collect_associations && result.debug) {
        result.debug->associations.reserve(observed_used.size());
        result.debug->query_landmarks.reserve(landmarks.size());
        added_landmarks.reserve(landmarks.size());
    }

    for (const auto& obs : observed_used) {
        if (kdtrees.find(obs.class_id) == kdtrees.end()) continue;
        valid_observed_count++;

        Eigen::Vector2d p_obs(obs.x, obs.y);
        Eigen::Vector2d p_rot  = R_final * p_obs;          // 회전만 적용된 좌표 (Jacobian 계산용)
        Eigen::Vector2d pt_map = p_rot + transform.translation(); // 최종 맵 좌표

        // 정합 루프와 동일한 마할라노비스 메트릭 재사용 (일관성 보장)
        Eigen::Matrix2d C_obs;
        C_obs << obs.cov_xx, obs.cov_xy, obs.cov_xy, obs.cov_yy;
        Eigen::Matrix2d C_map = R_final * C_obs * R_final.transpose();
        C_map(0,0) += 1e-3; C_map(1,1) += 1e-3;
        Eigen::Matrix2d Info_map = C_map.inverse(); // 이 정합점의 가중치 행렬(W) 역할

        double lambda_max       = maxEigenvalue2x2(C_map(0,0), C_map(0,1), C_map(1,1));
        double search_radius_sq = 9.0 * lambda_max;

        double query_pt[2] = { pt_map.x(), pt_map.y() };
        std::vector<nanoflann::ResultItem<uint32_t, double>> ret_matches;
        nanoflann::SearchParameters params;
        params.sorted = false;
        const auto num_matches = kdtrees[obs.class_id]->radiusSearch(query_pt, search_radius_sq, ret_matches, params);

        double min_dist_sq = 1e9;
        int best_landmark_idx = -1;

        for (const auto& m : ret_matches) {
            const auto& landmark = landmarks_by_class[obs.class_id][m.first];

            if (collect_associations && result.debug) {
                const auto key = std::make_pair(obs.class_id, static_cast<size_t>(m.first));
                const auto it = std::find(added_landmarks.begin(), added_landmarks.end(), key);
                if (it == added_landmarks.end()) {
                    added_landmarks.push_back(key);
                    result.debug->query_landmarks.push_back(landmark);
                }
            }

            Eigen::Vector2d dp(landmark.x - pt_map.x(), landmark.y - pt_map.y());
            double d2 = dp.transpose() * Info_map * dp;
            if (d2 < min_dist_sq) {
                min_dist_sq  = d2;
                best_landmark_idx = static_cast<int>(m.first);
            }
        }

        if (best_landmark_idx != -1) {
            const auto& landmark = landmarks_by_class[obs.class_id][best_landmark_idx];
            bool is_inlier = (min_dist_sq < 9.0);

            if (is_inlier) {
                inliers++;

                // 정합 쌍의 기하학적 제약 조건을 Hessian에 누적
                // 상태 벡터: x = [tx, ty, theta]^T
                // 자코비안 J (2x3)
                // J = [ I_2x2 | dR/d(theta) * p_obs ]
                Eigen::Matrix<double, 2, 3> J;
                J << 1.0, 0.0, -p_rot.y(),
                     0.0, 1.0,  p_rot.x();

                // H += J^T * W * J (가중치 W로 Info_map 사용)
                H += J.transpose() * Info_map * J;
            }

            // Record association
            if (collect_associations && result.debug) {
                Association assoc;
                assoc.obs_x = obs.x;
                assoc.obs_y = obs.y;
                assoc.landmark_x = landmark.x;
                assoc.landmark_y = landmark.y;
                assoc.dist_mahalanobis = std::sqrt(min_dist_sq);
                assoc.class_id = static_cast<uint8_t>(obs.class_id);
                assoc.is_inlier = is_inlier;
                result.debug->associations.push_back(assoc);
            }
        } else {
            // No matches inside search radius, mark as outlier with NaN landmarks
            if (collect_associations && result.debug) {
                Association assoc;
                assoc.obs_x = obs.x;
                assoc.obs_y = obs.y;
                assoc.landmark_x = std::numeric_limits<double>::quiet_NaN();
                assoc.landmark_y = std::numeric_limits<double>::quiet_NaN();
                assoc.dist_mahalanobis = 1e9;
                assoc.class_id = static_cast<uint8_t>(obs.class_id);
                assoc.is_inlier = false;
                result.debug->associations.push_back(assoc);
            }
        }
    }

    result.fitness_score = (valid_observed_count == 0) ? 0.0
                         : static_cast<double>(inliers) / valid_observed_count;

    // Always fill final transform for debug/visualization
    result.transform = transform;

    if (result.fitness_score >= fitness_threshold_) {
        result.success   = true;

        // 정합 스코어에 의한 공분산 인플레이션 계수 계산 (이중 스케일링 방지)
        double scale      = 1.0 / std::max(result.fitness_score, 1e-3);

        H += Eigen::Matrix3d::Identity() * 1e-6;

        // SVD 기반 pseudo-inverse 및 unconstrained 방향에 대한 분산 상한 제한 (Max Variance Capping)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(H);
        if (eigensolver.info() == Eigen::Success) {
            Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
            Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors();
            Eigen::Vector3d capped_cov_eigenvalues = Eigen::Vector3d::Zero();

            const double max_variance = max_covariance_; // unconstrained 방향에 대한 최대 분산 값 (m^2 또는 rad^2)
            for (int i = 0; i < 3; ++i) {
                if (eigenvalues(i) > 1e-9) {
                    capped_cov_eigenvalues(i) = std::min(scale / eigenvalues(i), max_variance);
                } else {
                    capped_cov_eigenvalues(i) = max_variance;
                }
            }
            result.covariance = eigenvectors * capped_cov_eigenvalues.asDiagonal() * eigenvectors.transpose();

            // EKF 과신(Overconfidence) 방지를 위해 최소 바닥 노이즈(Floor Noise) 추가
            // 정합 결과가 완벽하더라도 센서 자체의 해상도/왜곡 한계 오차를 공분산에 보장
            double min_pos_var = std::pow(vision_base_std_, 2);
            double min_yaw_var = std::pow(vision_base_yaw_std_, 2);
            result.covariance(0, 0) += min_pos_var;
            result.covariance(1, 1) += min_pos_var;
            result.covariance(2, 2) += min_yaw_var;
        } else {
            result.covariance = Eigen::Matrix3d::Identity() * max_covariance_;
        }
    }

    if (result.debug) {
        result.debug->iterations = actual_iterations;
    }
    finish_debug_timing();

    return result;
}

} // namespace carmaker_localization
