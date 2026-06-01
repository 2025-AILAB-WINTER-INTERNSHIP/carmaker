#include "carmaker_localization/icp_matcher.h"
#include "carmaker_localization/nanoflann.hpp"

#include <map>
#include <memory>
#include <cmath>

namespace carmaker_localization {

// ─────────────────────────────────────────────────────────────────────────────
// nanoflann point-cloud adapter for MapFeature (2D: x, y)
// ─────────────────────────────────────────────────────────────────────────────
struct MapFeatureCloud {
    const std::vector<MapFeature>& pts;
    explicit MapFeatureCloud(const std::vector<MapFeature>& p) : pts(p) {}

    // nanoflann interface
    inline size_t kdtree_get_point_count() const { return pts.size(); }
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return (dim == 0) ? pts[idx].x : pts[idx].y;
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

using KDTree2D = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, MapFeatureCloud>,
    MapFeatureCloud,
    2 /* dims */>;

// ─────────────────────────────────────────────────────────────────────────────
// Helper: 2×2 대칭 행렬 [[a, b], [b, c]] 의 최대 고유값
//   → Euclidean 보수 탐색 반경 r² = χ²_threshold × λ_max(C_map) 계산에 사용
// ─────────────────────────────────────────────────────────────────────────────
static double maxEigenvalue2x2(double a, double b, double c) {
    double half_trace = 0.5 * (a + c);
    double disc       = std::sqrt(0.25 * (a - c) * (a - c) + b * b);
    return half_trace + disc;
}

// ─────────────────────────────────────────────────────────────────────────────
// IcpMatcher
// ─────────────────────────────────────────────────────────────────────────────
IcpMatcher::IcpMatcher(double fitness_threshold, int max_iterations, double vision_base_std, double min_search_radius)
    : fitness_threshold_(fitness_threshold),
      max_iterations_(max_iterations),
      vision_base_std_(vision_base_std),
      min_search_radius_(min_search_radius) {}

MatchResult IcpMatcher::match(
    const std::vector<LocalFeature>& observed,
    const std::vector<MapFeature>&   reference,
    const Eigen::Isometry2d&         initial_guess) {

    MatchResult result;
    result.success = false;
    if (observed.empty() || reference.size() < 3) return result;

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

    // Group map features by class for class-specific matching
    std::map<int, std::vector<MapFeature>> map_by_class;
    for (const auto& ref : reference)
        map_by_class[ref.class_id].push_back(ref);

    // ── 클래스별 KD-tree 구축 (ICP 반복 루프 시작 전 1회) ─────────────────────
    //   KDTree2D 는 참조(reference)를 보관하므로 map_by_class 의 수명 내에서만 사용
    std::map<int, std::unique_ptr<MapFeatureCloud>> clouds;
    std::map<int, std::unique_ptr<KDTree2D>>        kdtrees;

    for (const auto& kv : map_by_class) {
        int cid   = kv.first;
        auto cloud = std::make_unique<MapFeatureCloud>(kv.second);
        auto tree  = std::make_unique<KDTree2D>(
                         2, *cloud,
                         nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf size */));
        tree->buildIndex();
        clouds .emplace(cid, std::move(cloud));
        kdtrees.emplace(cid, std::move(tree));
    }

    Eigen::Isometry2d transform  = initial_guess;
    double            prev_error = 1e9;

    // ── ICP 반복 ──────────────────────────────────────────────────────────────
    for (int iter = 0; iter < max_iterations_; ++iter) {
        std::vector<Eigen::Vector2d> dst_pts;
        double current_error      = 0.0;
        double total_error_weight = 0.0;

        // 1:1 유니크 매칭 테이블: class_id → (ref_idx → {LocalFeature, min_dist²})
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
            kdtrees[obs.class_id]->radiusSearch(query_pt, search_radius_sq, ret_matches, params);

            // 3. 반경 내 후보들 중 최소 마할라노비스 거리 선택
            double min_dist_sq = 9.0;  // χ² threshold
            int    best_ref_idx = -1;

            for (const auto& m : ret_matches) {
                const auto& ref = map_by_class[obs.class_id][m.first];
                Eigen::Vector2d dp(ref.x - pt_map_guess.x(), ref.y - pt_map_guess.y());
                double d2 = dp.transpose() * Info_map * dp;
                if (d2 < min_dist_sq) {
                    min_dist_sq  = d2;
                    best_ref_idx = static_cast<int>(m.first);
                }
            }

            // 4. 1:1 유니크 매칭 강제 (이미 할당된 맵 포인트가 있다면 더 가까운 것만 생존)
            if (best_ref_idx != -1) {
                auto& class_matches = best_matches[obs.class_id];
                if (class_matches.find(best_ref_idx) == class_matches.end() ||
                    min_dist_sq < class_matches[best_ref_idx].second) {
                    class_matches[best_ref_idx] = {obs, min_dist_sq};
                }
            }
        }

        // 생존한 1:1 매칭 쌍들만 SVD 연산 배열에 수집
        std::vector<LocalFeature> src_feats;
        for (const auto& class_pair : best_matches) {
            int class_id = class_pair.first;
            for (const auto& ref_pair : class_pair.second) {
                size_t ref_idx  = ref_pair.first;
                const auto& feat = ref_pair.second.first;
                double w = 1.0 / (feat.cov_xx + feat.cov_yy + 1e-4);
                src_feats.push_back(feat);
                dst_pts.push_back(Eigen::Vector2d(map_by_class[class_id][ref_idx].x,
                                                  map_by_class[class_id][ref_idx].y));
                current_error       += w * ref_pair.second.second;
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
        if (std::abs(prev_error - current_error) < 1e-4) break;
        prev_error = current_error;
    }

    // ── Final Verification: KD-tree 가속 인라이어 카운팅 ─────────────────────
    int inliers             = 0;
    int valid_observed_count = 0;
    Eigen::Matrix2d R_final = transform.linear();

    // 3x3 2D Pose (x, y, yaw)에 대한 정보 행렬(Information Matrix, Hessian) 누적 변수
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

    for (const auto& obs : observed_used) {
        if (kdtrees.find(obs.class_id) == kdtrees.end()) continue;
        valid_observed_count++;

        Eigen::Vector2d p_obs(obs.x, obs.y);
        Eigen::Vector2d p_rot  = R_final * p_obs;          // 회전만 적용된 좌표 (Jacobian 계산용)
        Eigen::Vector2d pt_map = p_rot + transform.translation(); // 최종 맵 좌표

        // 매칭 루프와 동일한 마할라노비스 메트릭 재사용 (일관성 보장)
        Eigen::Matrix2d C_obs;
        C_obs << obs.cov_xx, obs.cov_xy, obs.cov_xy, obs.cov_yy;
        Eigen::Matrix2d C_map = R_final * C_obs * R_final.transpose();
        C_map(0,0) += 1e-3; C_map(1,1) += 1e-3;
        Eigen::Matrix2d Info_map = C_map.inverse(); // 이 매칭점의 가중치 행렬(W) 역할

        double lambda_max       = maxEigenvalue2x2(C_map(0,0), C_map(0,1), C_map(1,1));
        double search_radius_sq = 9.0 * lambda_max;

        double query_pt[2] = { pt_map.x(), pt_map.y() };
        std::vector<nanoflann::ResultItem<uint32_t, double>> ret_matches;
        nanoflann::SearchParameters params;
        params.sorted = false;
        kdtrees[obs.class_id]->radiusSearch(query_pt, search_radius_sq, ret_matches, params);

        for (const auto& m : ret_matches) {
            const auto& ref = map_by_class[obs.class_id][m.first];
            Eigen::Vector2d dp(ref.x - pt_map.x(), ref.y - pt_map.y());

            // 인라이어(Inlier) 판정
            if (dp.transpose() * Info_map * dp < 9.0) {
                inliers++;

                // 매칭 쌍의 기하학적 제약 조건을 Hessian에 누적
                // 상태 벡터: x = [tx, ty, theta]^T
                // 자코비안 J (2x3): 매칭점의 위치가 x, y, theta 변화에 어떻게 반응하는가?
                // J = [ I_2x2 | dR/d(theta) * p_obs ]
                Eigen::Matrix<double, 2, 3> J;
                J << 1.0, 0.0, -p_rot.y(),
                     0.0, 1.0,  p_rot.x();

                // H += J^T * W * J (가중치 W로 Info_map 사용)
                // 차선과 같이 한쪽 방향의 불확실성이 큰 경우, Info_map이 그 기하학적 특성을 
                // 반영하므로 H 행렬 역시 해당 방향의 정보량을 0에 가깝게 만듭니다.
                H += J.transpose() * Info_map * J;
                break;
            }
        }
    }

    result.num_observed  = observed.size();
    result.num_reference = reference.size();
    result.fitness_score = (valid_observed_count == 0) ? 0.0
                         : static_cast<double>(inliers) / valid_observed_count;

    if (result.fitness_score >= fitness_threshold_) {
        result.success   = true;
        result.transform = transform;

        // 매칭 스코어와 비전 센서의 기본 노이즈를 결합한 스케일 팩터 계산
        double base_var   = vision_base_std_ * vision_base_std_;
        double confidence = static_cast<double>(inliers) * result.fitness_score;
        double scale      = base_var / std::max(confidence, 1.0);

        // 최종 공분산 도출, 등방성(Identity) -> 비등방성(Anisotropic) 타원체
        // 특징점이 부족하거나(직선 구간 등) 한 방향의 제약이 없을 때 역행렬이 발산(Singular)
        // 하는 것을 방지하기 위해 최소한의 안정화(Regularization) 값 추가
        H += Eigen::Matrix3d::Identity() * 1e-6;

        // 최종 공분산 행렬 = 정보 행렬(H)의 역행렬 * 신뢰도 스케일
        // 직선 차선일 경우 종방향 정보량이 0에 가까우므로, 역행렬 계산 시 종방향 공분산(불확실성)이 매우 증가
        result.covariance = H.inverse() * scale;
    }

    return result;
}

} // namespace carmaker_localization
