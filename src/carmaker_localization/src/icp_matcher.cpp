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
        std::vector<Eigen::Vector2d> dst_pts;
        double current_error = 0.0;
        double total_error_weight = 0.0;

        // 1:1 Unique Matching Competition Map: class_id -> (ref_idx -> {LocalFeature, min_mahalanobis_dist})
        std::map<int, std::map<size_t, std::pair<LocalFeature, double>>> best_matches;
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
                    class_matches[best_ref_idx] = {obs, min_dist_sq};
                }
            }
        }

        // 생존한 1:1 매칭 쌍들만 SVD 연산 배열에 수집
        std::vector<LocalFeature> src_feats;
        for (const auto& class_pair : best_matches) {
            int class_id = class_pair.first;
            for (const auto& ref_pair : class_pair.second) {
                size_t ref_idx = ref_pair.first;
                const auto& feat = ref_pair.second.first;
                double w = 1.0 / (feat.cov_xx + feat.cov_yy + 1e-4);
                src_feats.push_back(feat);
                dst_pts.push_back(Eigen::Vector2d(map_by_class[class_id][ref_idx].x,
                                                  map_by_class[class_id][ref_idx].y));
                current_error += w * ref_pair.second.second;
                total_error_weight += w;
            }
        }

        if (src_feats.size() < 3) break;

        // Weighted Procrustes Analysis (Point-to-Point closed form weighted by inverse covariance trace)
        std::vector<double> weights;
        weights.reserve(src_feats.size());
        double total_weight = 0.0;
        for (const auto& feat : src_feats) {
            // 가중치 w_i = 1.0 / (trace(C) + epsilon)
            // 즉, 카메라 최적 투영 부근의 오차가 작고 선명한 매칭 점일수록 더 높은 최적화 가중치를 받게 됩니다.
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

        Eigen::Vector2d t = dst_mean - R * src_mean;
        transform.linear() = R;
        transform.translation() = t;

        if (total_error_weight > 1e-6) current_error /= total_error_weight;
        if (std::abs(prev_error - current_error) < 1e-4) break;
        prev_error = current_error;
    }

    // Final Verification: count inliers using consistent Mahalanobis threshold
    int inliers = 0;
    int valid_observed_count = 0;
    Eigen::Matrix2d R_final = transform.linear();
    for (const auto& obs : observed_used) {
        if (map_by_class.find(obs.class_id) == map_by_class.end()) continue;
        valid_observed_count++;

        Eigen::Vector2d pt_map = transform * Eigen::Vector2d(obs.x, obs.y);

        // Reuse same Mahalanobis metric as NN search for consistency
        Eigen::Matrix2d C_obs;
        C_obs << obs.cov_xx, obs.cov_xy, obs.cov_xy, obs.cov_yy;
        Eigen::Matrix2d C_map = R_final * C_obs * R_final.transpose();
        C_map(0,0) += 1e-3; C_map(1,1) += 1e-3;
        Eigen::Matrix2d Info_map = C_map.inverse();

        for (const auto& ref : map_by_class[obs.class_id]) {
            Eigen::Vector2d dp(ref.x - pt_map.x(), ref.y - pt_map.y());
            if (dp.transpose() * Info_map * dp < 9.0) { // same 3-sigma threshold
                inliers++;
                break;
            }
        }
    }

    result.num_observed = observed.size();
    result.num_reference = reference.size();
    result.fitness_score = (valid_observed_count == 0) ? 0.0 : (double)inliers / valid_observed_count;
    if (result.fitness_score >= fitness_threshold_) {
        result.success = true;
        result.transform = transform;

        // Covariance is inversely proportional to the combined confidence of the match.
        // confidence = inliers * fitness_score represents the "effective quality observation count":
        //   - more inliers → higher confidence → smaller uncertainty
        //   - higher fitness (closer matches) → higher confidence → smaller uncertainty
        // This gives sigma_sq a clear physical unit: base_var / effective_observations.
        double base_var = vision_base_std_ * vision_base_std_;
        double confidence = static_cast<double>(inliers) * result.fitness_score;
        double sigma_sq = base_var / std::max(confidence, 1.0);
        result.covariance = Eigen::Matrix3d::Identity() * sigma_sq;
    }

    return result;
}

} // namespace carmaker_localization
