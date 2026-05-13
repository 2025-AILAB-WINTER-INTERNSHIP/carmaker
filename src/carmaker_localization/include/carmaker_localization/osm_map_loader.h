#ifndef CARMAKER_LOCALIZATION_OSM_MAP_LOADER_H
#define CARMAKER_LOCALIZATION_OSM_MAP_LOADER_H

#include "carmaker_localization/map_loader_base.h"

namespace carmaker_localization {

class OsmMapLoader : public MapLoaderBase {
public:
    OsmMapLoader() = default;
    virtual ~OsmMapLoader() = default;

    virtual bool load(const std::string& path) override;
    virtual std::vector<MapFeature> queryNear(double x, double y, double radius) const override;

private:
    std::vector<MapFeature> features_;
    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
    bool origin_set_ = false;
};

} // namespace carmaker_localization

#endif
