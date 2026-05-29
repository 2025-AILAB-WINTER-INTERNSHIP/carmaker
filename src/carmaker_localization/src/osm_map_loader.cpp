#include "carmaker_localization/osm_map_loader.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <map>
#include <set>
#include <iostream>
#include <limits>

namespace carmaker_localization {

OsmMapLoader::OsmMapLoader(double resolution)
    : resolution_(resolution) {
}

bool OsmMapLoader::load(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[OsmMapLoader] Failed to open OSM map file: " << path << std::endl;
        return false;
    }

    // ----------------------------------------------------------------
    // OSM Parsing Structures
    // ----------------------------------------------------------------
    struct TempNode {
        double x;
        double y;
        int id;
    };

    struct Way {
        int id;
        std::vector<int> nodes;
        std::map<std::string, std::string> tags;
    };

    // ----------------------------------------------------------------
    // XML attribute extraction helper
    // ----------------------------------------------------------------
    auto extract_attr = [](const std::string& l, const std::string& attr) -> std::string {
        size_t pos = l.find(attr + "=\"");
        if (pos == std::string::npos) return "";
        pos += attr.length() + 2;
        size_t end = l.find("\"", pos);
        if (end == std::string::npos) return "";
        return l.substr(pos, end - pos);
    };

    // ----------------------------------------------------------------
    // Parse nodes and ways from OSM XML
    // ----------------------------------------------------------------
    std::map<int, TempNode> node_map;
    std::vector<Way> ways;
    Way current_way;
    bool in_way = false;
    std::string line;

    while (std::getline(file, line)) {
        if (line.find("<node") != std::string::npos) {
            std::string id_str  = extract_attr(line, "id");
            std::string lat_str = extract_attr(line, "lat");
            std::string lon_str = extract_attr(line, "lon");
            if (id_str.empty() || lat_str.empty() || lon_str.empty()) continue;

            int id       = std::stoi(id_str);
            double lat   = std::stod(lat_str);
            double lon   = std::stod(lon_str);

            TempNode n{0.0, 0.0, id};
            double local_x = 0.0, local_y = 0.0;
            bool has_local = false;

            if (line.find("/>") == std::string::npos) {
                std::string node_line;
                while (std::getline(file, node_line)) {
                    if (node_line.find("</node>") != std::string::npos) break;
                    if (node_line.find("<tag") != std::string::npos) {
                        std::string k = extract_attr(node_line, "k");
                        std::string v = extract_attr(node_line, "v");
                        if (k == "local_x") { local_x = std::stod(v); has_local = true; }
                        else if (k == "local_y") { local_y = std::stod(v); has_local = true; }
                    }
                }
            }

            if (has_local) {
                n.x = local_x;
                n.y = local_y;
                if (!origin_set_) { origin_lat_ = lat; origin_lon_ = lon; origin_set_ = true; }
            } else {
                if (!origin_set_) { origin_lat_ = lat; origin_lon_ = lon; origin_set_ = true; }
                double lat_rad  = lat * M_PI / 180.0;
                double lon_rad  = lon * M_PI / 180.0;
                double lat0_rad = origin_lat_ * M_PI / 180.0;
                double lon0_rad = origin_lon_ * M_PI / 180.0;
                double R = 6378137.0; // WGS-84 Semimajor axis
                n.x = R * (lon_rad - lon0_rad) * std::cos(lat0_rad);
                n.y = R * (lat_rad - lat0_rad);
            }
            node_map[id] = n;

        } else if (line.find("<way") != std::string::npos) {
            current_way = Way();
            std::string id_str = extract_attr(line, "id");
            if (!id_str.empty()) current_way.id = std::stoi(id_str);
            in_way = true;

        } else if (in_way && line.find("<nd") != std::string::npos) {
            std::string ref = extract_attr(line, "ref");
            if (!ref.empty()) current_way.nodes.push_back(std::stoi(ref));

        } else if (in_way && line.find("<tag") != std::string::npos) {
            std::string k = extract_attr(line, "k");
            std::string v = extract_attr(line, "v");
            if (!k.empty() && !v.empty()) current_way.tags[k] = v;

        } else if (in_way && line.find("</way>") != std::string::npos) {
            in_way = false;
            bool is_valid = false;
            if (current_way.tags.count("amenity") && current_way.tags["amenity"] == "ev_charging")
                is_valid = true;
            else if (current_way.tags.count("type")) {
                const auto& t = current_way.tags["type"];
                is_valid = (t == "parking_spot" || t == "ev_charging" || t == "lane" || t == "line" || t == "boundary");
            }
            if (is_valid) ways.push_back(current_way);
        }
    }

    // ----------------------------------------------------------------
    // Voxel Grid Sampling
    // Each way is rasterised onto the resolution_ grid.
    //   Class 1 (lane / parking_spot): polyline with 0.1m physical width
    //   Class 2 (ev_charging):         filled polygon
    //   Class 3 (boundary):            filled polygon (inside is free space)
    // ----------------------------------------------------------------
    features_.clear();

    std::set<std::pair<double,double>> lane_voxels;
    std::set<std::pair<double,double>> landmark_voxels;
    std::set<std::pair<double,double>> boundary_voxels;

    for (const auto& way : ways) {
        // Determine class
        int class_id = 1;
        if ((way.tags.count("amenity") && way.tags.at("amenity") == "ev_charging") ||
            (way.tags.count("type")   && way.tags.at("type")   == "ev_charging"))
            class_id = 2;
        else if (way.tags.count("type") && way.tags.at("type") == "boundary")
            class_id = 3;

        if (class_id == 1) {
            // Lane/parking_spot: sample each segment with 0.05m half-width (total 0.1m)
            for (size_t i = 0; i + 1 < way.nodes.size(); ++i) {
                auto it_a = node_map.find(way.nodes[i]);
                auto it_b = node_map.find(way.nodes[i+1]);
                if (it_a == node_map.end() || it_b == node_map.end()) continue;

                Point2d a{it_a->second.x, it_a->second.y};
                Point2d b{it_b->second.x, it_b->second.y};
                sampleSegment(a, b, 0.05, resolution_, lane_voxels);
            }
        } else if (class_id == 2 && way.nodes.size() >= 3) {
            // Landmark: sample the entire filled polygon
            std::vector<Point2d> poly;
            poly.reserve(way.nodes.size());
            for (int nid : way.nodes) {
                auto it = node_map.find(nid);
                if (it != node_map.end())
                    poly.push_back({it->second.x, it->second.y});
            }
            samplePolygon(poly, resolution_, landmark_voxels);
        } else if (class_id == 3 && way.nodes.size() >= 3) {
            // Boundary: sample the entire filled polygon (inside)
            std::vector<Point2d> poly;
            poly.reserve(way.nodes.size());
            for (int nid : way.nodes) {
                auto it = node_map.find(nid);
                if (it != node_map.end())
                    poly.push_back({it->second.x, it->second.y});
            }
            samplePolygon(poly, resolution_, boundary_voxels);
        }
    }

    features_.reserve(lane_voxels.size() + landmark_voxels.size());
    voxelsToFeatures(lane_voxels,     1, features_);
    voxelsToFeatures(landmark_voxels, 2, features_);

    // ----------------------------------------------------------------
    // Build Occupancy Grid from Boundary Voxels (class 3)
    // ----------------------------------------------------------------
    occupancy_grid_ = nav_msgs::OccupancyGrid();
    if (!boundary_voxels.empty()) {
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();

        for (const auto& v : boundary_voxels) {
            min_x = std::min(min_x, v.first);  max_x = std::max(max_x, v.first);
            min_y = std::min(min_y, v.second); max_y = std::max(max_y, v.second);
        }

        const double res = resolution_;
        const double margin = 1.0;
        double origin_x = min_x - margin;
        double origin_y = min_y - margin;

        // Snap to resolution
        origin_x = std::floor(origin_x / res) * res;
        origin_y = std::floor(origin_y / res) * res;

        int w = static_cast<int>(std::ceil((max_x - origin_x + margin) / res));
        int h = static_cast<int>(std::ceil((max_y - origin_y + margin) / res));

        std::vector<int8_t> grid_data(static_cast<size_t>(w) * h, 100);
        for (const auto& v : boundary_voxels) {
            int ix = static_cast<int>(std::round((v.first - origin_x - 0.5 * res) / res));
            int iy = static_cast<int>(std::round((v.second - origin_y - 0.5 * res) / res));
            if (ix >= 0 && ix < w && iy >= 0 && iy < h) {
                grid_data[static_cast<size_t>(iy) * w + ix] = 0;
            }
        }

        occupancy_grid_.header.frame_id = "map";
        occupancy_grid_.info.resolution = res;
        occupancy_grid_.info.width = w;
        occupancy_grid_.info.height = h;
        occupancy_grid_.info.origin.position.x = origin_x;
        occupancy_grid_.info.origin.position.y = origin_y;
        occupancy_grid_.info.origin.position.z = 0.0;
        occupancy_grid_.info.origin.orientation.w = 1.0;
        occupancy_grid_.data = std::move(grid_data);
    }

    std::cout << "[OsmMapLoader] Loaded " << features_.size()
              << " features, boundary map: " << occupancy_grid_.info.width << "x" << occupancy_grid_.info.height
              << " (voxel resolution: " << resolution_ << " m) from OSM map." << std::endl;
    return true;
}

std::vector<MapFeature> OsmMapLoader::queryNear(double x, double y, double radius) const {
    if (radius < 0.0) return features_;

    std::vector<MapFeature> result;
    result.reserve(features_.size() / 10);

    double r_sq = radius * radius;
    for (const auto& f : features_) {
        double dx = f.x - x, dy = f.y - y;
        if (dx*dx + dy*dy <= r_sq) result.push_back(f);
    }
    return result;
}

nav_msgs::OccupancyGrid OsmMapLoader::getOccupancyGrid() const {
    return occupancy_grid_;
}

} // namespace carmaker_localization
