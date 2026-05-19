#ifndef CARMAKER_LOCALIZATION_FEATURE_EXTRACTOR_H
#define CARMAKER_LOCALIZATION_FEATURE_EXTRACTOR_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace carmaker_localization {

struct BevConfig {
    double x_min, x_max;
    double y_min, y_max;
    double resolution;
    int width, height;
};

struct LocalFeature {
    double x;
    double y;
    double cov_xx;
    double cov_xy;
    double cov_yy;
    int class_id;
};

class FeatureExtractor {
public:
    explicit FeatureExtractor(const std::string& camera_name);
    ~FeatureExtractor() = default;

    /**
     * @brief Initialize BEV grid configuration and image type
     */
    void initialize(const std::vector<double>& x_range, const std::vector<double>& y_range,
                    const std::string& image_type, double resolution);

    /**
     * @brief Set tuning parameters for feature extraction
     */
    /**
     * @brief Set tuning parameters for feature extraction
     * @param r_max  Maximum valid observation range [m]
     * @param cov_k  Covariance scaling factor
     * @param max_fov Maximum Field of View of camera lens [deg]
     */
    void setExtractionParameters(double r_max, double cov_k, double max_fov = 180.0);

    /**
     * @brief Set vehicle bounding box in Fr1A frame for footprint filtering.
     * Features falling inside the vehicle body are excluded from extraction
     * (e.g., vehicle outline in GT image misdetected as lane lines).
     * @param x_min  Rear edge of vehicle [m]
     * @param x_max  Front edge of vehicle [m]
     * @param y_half Half-width of vehicle (symmetric) [m]
     */
    void setVehicleFootprint(double x_min, double x_max, double y_half, double height, double margin = 0.15);

    /**
     * @brief Check if the extractor has been initialized
     */
    bool isInitialized() const { return is_initialized_; }

    /**
     * @brief Process incoming image using current configuration
     */
    std::vector<LocalFeature> process(
        const cv::Mat& seg_img,
        cv::Mat& out_bev_image);

    /**
     * @brief Update projection look-up table using intrinsic and extrinsic data
     */
    void updateLUT(const std::vector<double>& K, const std::vector<double>& D,
                    const std::string& distortion_model, const cv::Mat& R_base_cam, const cv::Mat& t_base_cam);

private:
    std::string image_type_;
    BevConfig bev_cfg_;
    std::string camera_name_;

    // Tuning Parameters
    double r_max_;
    double cov_k_;
    double max_fov_;

    // Look-Up Table for remap
    cv::Mat map1_, map2_;
    cv::Mat cartesian_lut_x_, cartesian_lut_y_; // Precomputed X,Y in Fr1A

    bool is_initialized_;
    bool lut_initialized_;
    bool has_optimal_point_;
    cv::Point2f optimal_point_;

    // Vehicle footprint filter (Fr1A frame)
    bool has_vehicle_footprint_;
    double veh_x_min_, veh_x_max_, veh_y_half_;
    double phys_x_min_, phys_x_max_, phys_y_half_;
    double veh_height_;
    double cam_origin_x_, cam_origin_y_;
    cv::Mat footprint_mask_;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_FEATURE_EXTRACTOR_H
