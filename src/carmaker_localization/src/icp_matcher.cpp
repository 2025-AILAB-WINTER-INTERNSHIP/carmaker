#include "carmaker_localization/icp_matcher.h"

namespace carmaker_localization {

IcpMatcher::IcpMatcher(double fitness_threshold, int max_iterations, double vision_base_std)
    : fitness_threshold_(fitness_threshold), max_iterations_(max_iterations), vision_base_std_(vision_base_std) {}

MatchResult IcpMatcher::match(
    const std::vector<LocalFeature>& observed,
    const std::vector<MapFeature>& reference,
    const Eigen::Isometry2d& initial_guess) {

    MatchResult result;
    result.success = false;
    if (observed.empty() || reference.size() < 3) return result;

    // Uniform step downsampling to prevent CPU saturation while preserving spatial distribution
    std::vector<LocalFeature> observed_used;
    if (observed.size() > 400) {
        size_t step = observed.size() / 400;
        observed_used.reserve(400);
        for (size_t i = 0; i < observed.size(); i += step) {
            observed_used.push_back(observed[i]);
        }
    } else {
        observed_used = observed;
    }

    // Group map features by class for faster filtering
    std::map<int, std::vector<MapFeature>> map_by_class;
    for (const auto& ref : reference) {
        map_by_class[ref.class_id].push_back(ref);
    }

    Eigen::Isometry2d transform = initial_guess;
    double prev_error = 1e9;

    // ICP Iterations
    for (int iter = 0; iter < max_iterations_; ++iter) {
        std::vector<Eigen::Vector2d> src_pts;
        std::vector<Eigen::Vector2d> dst_pts;
        double current_error = 0.0;

        // 1:1 Unique Matching Competition Map: class_id -> (ref_idx -> {pt_obs, min_mahalanobis_dist})
        std::map<int, std::map<size_t, std::pair<Eigen::Vector2d, double>>> best_matches;
        Eigen::Matrix2d R_curr = transform.linear();

        for (const auto& obs : observed_used) {
            if (map_by_class.find(obs.class_id) == map_by_class.end()) continue;

            Eigen::Vector2d pt_obs(obs.x, obs.y);
            Eigen::Vector2d pt_map_guess = transform * pt_obs;

            // 1. 공분산을 Map 좌표계로 회전 후 정보 행렬(Information Matrix) 도출
            Eigen::Matrix2d C_obs;
            C_obs << obs.cov_xx, obs.cov_xy,
                     obs.cov_xy, obs.cov_yy;
            Eigen::Matrix2d C_map = R_curr * C_obs * R_curr.transpose();

            // 특이 행렬 방지(Invertibility 보장)를 위해 대각 성분에 아주 작은 노이즈 추가
            C_map(0,0) += 1e-3;
            C_map(1,1) += 1e-3;
            Eigen::Matrix2d Info_map = C_map.inverse();

            // 2. 마할라노비스 거리 기반 최근접 맵 특징점 탐색
            double min_dist_sq = 9.0; // Chi-square threshold (약 3 Sigma 수준의 통계적 허용치)
            int best_ref_idx = -1;

            for (size_t i = 0; i < map_by_class[obs.class_id].size(); ++i) {
                const auto& ref = map_by_class[obs.class_id][i];
                Eigen::Vector2d dp(ref.x - pt_map_guess.x(), ref.y - pt_map_guess.y());

                // 마할라노비스 거리 제곱: D^2 = dp^T * Info * dp
                double d2_mahalanobis = dp.transpose() * Info_map * dp;

                if (d2_mahalanobis < min_dist_sq) {
                    min_dist_sq = d2_mahalanobis;
                    best_ref_idx = i;
                }
            }

            // 3. 1:1 유니크 매칭 강제 (이미 해당 맵 포인트에 할당된 짝이 있다면 더 가까운 것만 생존)
            if (best_ref_idx != -1) {
                auto& class_matches = best_matches[obs.class_id];
                if (class_matches.find(best_ref_idx) == class_matches.end() ||
                    min_dist_sq < class_matches[best_ref_idx].second) {
                    class_matches[best_ref_idx] = {pt_obs, min_dist_sq};
                }
            }
        }

        // 생존한 1:1 매칭 쌍들만 SVD 연산 배열에 수집
        for (const auto& class_pair : best_matches) {
            int class_id = class_pair.first;
            for (const auto& ref_pair : class_pair.second) {
                size_t ref_idx = ref_pair.first;
                src_pts.push_back(ref_pair.second.first);
                dst_pts.push_back(Eigen::Vector2d(map_by_class[class_id][ref_idx].x,
                                                  map_by_class[class_id][ref_idx].y));
                current_error += ref_pair.second.second;
            }
        }

        if (src_pts.size() < 3) break;

        // Procrustes Analysis (Point-to-Point closed form)
        Eigen::Vector2d src_mean = Eigen::Vector2d::Zero();
        Eigen::Vector2d dst_mean = Eigen::Vector2d::Zero();
        for (size_t i = 0; i < src_pts.size(); ++i) {
            src_mean += src_pts[i];
            dst_mean += dst_pts[i];
        }
        src_mean /= src_pts.size();
        dst_mean /= dst_pts.size();

        Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < src_pts.size(); ++i) {
            W += (dst_pts[i] - dst_mean) * (src_pts[i] - src_mean).transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d R = svd.matrixU() * svd.matrixV().transpose();
        if (R.determinant() < 0) {
            Eigen::Matrix2d V = svd.matrixV();
            V.col(1) *= -1;
            R = svd.matrixU() * V.transpose();
        }

        Eigen::Vector2d t = dst_mean - R * src_mean;
        transform.linear() = R;
        transform.translation() = t;

        current_error /= src_pts.size();
        if (std::abs(prev_error - current_error) < 1e-4) break;
        prev_error = current_error;
    }

    // Final Verification and Covariance Estimation
    int inliers = 0;
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& obs : observed_used) {
        if (map_by_class.find(obs.class_id) == map_by_class.end()) continue;

        Eigen::Vector2d pt_map = transform * Eigen::Vector2d(obs.x, obs.y);
        double min_dist_sq = 1.0; // 1.0m fitness threshold
        bool found = false;
        for (const auto& ref : map_by_class[obs.class_id]) {
            double dx = ref.x - pt_map.x();
            double dy = ref.y - pt_map.y();
            if (dx*dx + dy*dy < min_dist_sq) {
                found = true;
                break;
            }
        }

        if (found) {
            inliers++;
            cov(0,0) += obs.cov_xx;
            cov(1,1) += obs.cov_yy;
            cov(0,1) += obs.cov_xy;
            cov(1,0) += obs.cov_xy;
        }
    }

    int valid_observed_count = 0;
    for (const auto& obs : observed_used) {
        if (map_by_class.find(obs.class_id) != map_by_class.end()) {
            valid_observed_count++;
        }
    }

    result.num_observed = observed.size();
    result.num_reference = reference.size();
    result.fitness_score = (valid_observed_count == 0) ? 0.0 : (double)inliers / valid_observed_count;
    if (result.fitness_score >= fitness_threshold_) {
        result.success = true;
        result.transform = transform;

        // Base noise for rotation and position
        if (inliers > 0) {
            cov /= inliers;
            cov(2,2) = vision_base_std_ * vision_base_std_;
        } else {
            cov = Eigen::Matrix3d::Identity() * vision_base_std_;
        }
        result.covariance = cov;
    }

    return result;
}

} // namespace carmaker_localization
