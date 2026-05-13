#include "carmaker_localization/icp_matcher.h"

namespace carmaker_localization {

IcpMatcher::IcpMatcher(double fitness_threshold, int max_iterations, double vision_base_std)
    : fitness_threshold_(fitness_threshold), max_iterations_(max_iterations), vision_base_std_(vision_base_std) {}

MatchResult IcpMatcher::match(
    const std::vector<carmaker_msgs::LocalFeature>& observed,
    const std::vector<MapFeature>& reference,
    const Eigen::Isometry2d& initial_guess) {

    MatchResult result;
    result.success = false;
    if (observed.empty() || reference.size() < 3) return result;

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

        for (const auto& obs : observed) {
            if (map_by_class.find(obs.class_id) == map_by_class.end()) continue;

            Eigen::Vector2d pt_obs(obs.x, obs.y);
            Eigen::Vector2d pt_map_guess = transform * pt_obs;

            // Simple nearest neighbor within the same class
            double min_dist_sq = 4.0; // 2.0m search threshold
            int best_idx = -1;

            for (size_t i = 0; i < map_by_class[obs.class_id].size(); ++i) {
                const auto& ref = map_by_class[obs.class_id][i];
                double dx = ref.x - pt_map_guess.x();
                double dy = ref.y - pt_map_guess.y();
                double d2 = dx*dx + dy*dy;
                if (d2 < min_dist_sq) {
                    min_dist_sq = d2;
                    best_idx = i;
                }
            }

            if (best_idx != -1) {
                src_pts.push_back(pt_obs);
                dst_pts.push_back(Eigen::Vector2d(map_by_class[obs.class_id][best_idx].x,
                                                 map_by_class[obs.class_id][best_idx].y));
                current_error += min_dist_sq;
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
    for (const auto& obs : observed) {
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

    result.fitness_score = (observed.empty()) ? 0.0 : (double)inliers / observed.size();
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
