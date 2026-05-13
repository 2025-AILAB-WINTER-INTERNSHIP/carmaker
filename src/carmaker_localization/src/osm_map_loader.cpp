#include "carmaker_localization/osm_map_loader.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <ros/ros.h>

namespace carmaker_localization {

bool OsmMapLoader::load(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open OSM map file: %s", path.c_str());
        return false;
    }

    std::string line;
    int current_class = 1; // Default class
    while (std::getline(file, line)) {
        if (line.find("<node") != std::string::npos) {
            auto extract_attr = [](const std::string& l, const std::string& attr) -> std::string {
                size_t pos = l.find(attr + "=\"");
                if (pos == std::string::npos) return "";
                pos += attr.length() + 2;
                size_t end = l.find("\"", pos);
                if (end == std::string::npos) return "";
                return l.substr(pos, end - pos);
            };

            std::string lat_str = extract_attr(line, "lat");
            std::string lon_str = extract_attr(line, "lon");
            if (!lat_str.empty() && !lon_str.empty()) {
                double lat = std::stod(lat_str);
                double lon = std::stod(lon_str);

                if (!origin_set_) {
                    origin_lat_ = lat;
                    origin_lon_ = lon;
                    origin_set_ = true;
                }

                double r_earth = 6378137.0;
                double dlat = (lat - origin_lat_) * M_PI / 180.0;
                double dlon = (lon - origin_lon_) * M_PI / 180.0;
                double x = r_earth * std::cos(origin_lat_ * M_PI / 180.0) * dlon;
                double y = r_earth * dlat;

                MapFeature feat;
                feat.x = x;
                feat.y = y;
                feat.class_id = current_class;
                features_.push_back(feat);
            }
        } else if (line.find("<tag k=\"class\"") != std::string::npos) {
            // Simple tag parsing: if we see a class tag, use it for the NEXT nodes
            // In a real OSM, tags are inside nodes, but for a "simple" loader, 
            // this is a quick way to handle class-separated blocks.
            size_t v_pos = line.find("v=\"");
            if (v_pos != std::string::npos) {
                current_class = std::stoi(line.substr(v_pos + 3, line.find("\"", v_pos + 3) - (v_pos + 3)));
            }
        }
    }
    ROS_INFO("Loaded %zu nodes from OSM map.", features_.size());
    return true;
}

std::vector<MapFeature> OsmMapLoader::queryNear(double x, double y, double radius) const {
    std::vector<MapFeature> result;
    double r_sq = radius * radius;
    for (const auto& feat : features_) {
        double dx = feat.x - x;
        double dy = feat.y - y;
        if (dx*dx + dy*dy <= r_sq) {
            result.push_back(feat);
        }
    }
    return result;
}

} // namespace carmaker_localization
