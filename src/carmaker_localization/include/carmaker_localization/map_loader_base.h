#ifndef CARMAKER_LOCALIZATION_MAP_LOADER_BASE_H
#define CARMAKER_LOCALIZATION_MAP_LOADER_BASE_H

#include <string>
#include <vector>
#include <carmaker_msgs/LocalFeature.h>

namespace carmaker_localization {

struct MapFeature {
    double x;
    double y;
    uint8_t class_id;
};

/**
 * @brief Abstract interface for map loading.
 * Supports OSM, Lanelet2, PCD, etc.
 */
class MapLoaderBase {
public:
    virtual ~MapLoaderBase() = default;

    /**
     * @brief Load map from file
     * @param path Absolute path to map file
     * @return true if success
     */
    virtual bool load(const std::string& path) = 0;

    /**
     * @brief Query features near a position
     * @param x Query X (m)
     * @param y Query Y (m)
     * @param radius Search radius (m)
     * @return Vector of features within radius
     */
    virtual std::vector<MapFeature> queryNear(double x, double y, double radius) const = 0;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_MAP_LOADER_BASE_H
