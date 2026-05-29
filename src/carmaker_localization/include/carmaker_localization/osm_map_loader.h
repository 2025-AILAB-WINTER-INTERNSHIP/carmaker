#ifndef CARMAKER_LOCALIZATION_OSM_MAP_LOADER_H
#define CARMAKER_LOCALIZATION_OSM_MAP_LOADER_H

#include "carmaker_localization/map_loader_base.h"

namespace carmaker_localization {

class OsmMapLoader : public MapLoaderBase {
public:
    explicit OsmMapLoader(double resolution = 0.05);
    virtual ~OsmMapLoader() = default;

    virtual bool load(const std::string& path) override;
    virtual std::vector<MapFeature> queryNear(double x, double y, double radius) const override;
    virtual nav_msgs::OccupancyGrid getOccupancyGrid() const override;

private:
    std::vector<MapFeature> features_;
    nav_msgs::OccupancyGrid occupancy_grid_;
    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
    bool origin_set_ = false;
    double resolution_ = 0.05;
};

} // namespace carmaker_localization

#endif
