#include "carmaker_localization/osm_map_loader.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <map>
#include <iostream>

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

    struct TempNode {
        double x;
        double y;
        int id;
    };
    std::map<int, TempNode> node_map;

    struct Way {
        int id;
        std::vector<int> nodes;
        std::map<std::string, std::string> tags;
    };

    std::string line;
    auto extract_attr = [](const std::string& l, const std::string& attr) -> std::string {
        size_t pos = l.find(attr + "=\"");
        if (pos == std::string::npos) return "";
        pos += attr.length() + 2;
        size_t end = l.find("\"", pos);
        if (end == std::string::npos) return "";
        return l.substr(pos, end - pos);
    };

    auto is_point_in_polygon = [](double x, double y, const std::vector<TempNode>& polygon) -> bool {
        bool inside = false;
        size_t n = polygon.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            if (((polygon[i].y > y) != (polygon[j].y > y)) &&
                (x < (polygon[j].x - polygon[i].x) * (y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x)) {
                inside = !inside;
            }
        }
        return inside;
    };

    std::vector<Way> ways;
    Way current_way;
    bool in_way = false;

    while (std::getline(file, line)) {
        if (line.find("<node") != std::string::npos) {
            std::string id_str = extract_attr(line, "id");
            std::string lat_str = extract_attr(line, "lat");
            std::string lon_str = extract_attr(line, "lon");
            if (!id_str.empty() && !lat_str.empty() && !lon_str.empty()) {
                int id = std::stoi(id_str);
                double lat = std::stod(lat_str);
                double lon = std::stod(lon_str);

                TempNode n;
                n.id = id;
                n.x = 0.0;
                n.y = 0.0;

                // local_x, local_y tags present in nodes
                std::string node_line;
                double local_x = 0.0;
                double local_y = 0.0;
                bool has_local = false;

                // Read tags inside the node if it's not a self-closing node
                if (line.find("/>") == std::string::npos) {
                    while (std::getline(file, node_line)) {
                        if (node_line.find("</node>") != std::string::npos) {
                            break;
                        }
                        if (node_line.find("<tag") != std::string::npos) {
                            std::string k = extract_attr(node_line, "k");
                            std::string v = extract_attr(node_line, "v");
                            if (k == "local_x") {
                                local_x = std::stod(v);
                                has_local = true;
                            } else if (k == "local_y") {
                                local_y = std::stod(v);
                                has_local = true;
                            }
                        }
                    }
                }

                if (has_local) {
                    n.x = local_x;
                    n.y = local_y;
                    if (!origin_set_) {
                        origin_lat_ = lat;
                        origin_lon_ = lon;
                        origin_set_ = true;
                    }
                } else {
                    // Geodetic (Lat, Lon) to ENU Cartesian Coordinate Conversion
                    if (!origin_set_) {
                        origin_lat_ = lat;
                        origin_lon_ = lon;
                        origin_set_ = true;
                    }
                    double lat_rad = lat * M_PI / 180.0;
                    double lon_rad = lon * M_PI / 180.0;
                    double lat0_rad = origin_lat_ * M_PI / 180.0;
                    double lon0_rad = origin_lon_ * M_PI / 180.0;

                    double R_earth = 6378137.0; // WGS-84 Semimajor axis
                    n.x = R_earth * (lon_rad - lon0_rad) * std::cos(lat0_rad);
                    n.y = R_earth * (lat_rad - lat0_rad);
                }

                node_map[id] = n;
            }
        } else if (line.find("<way") != std::string::npos) {
            current_way = Way();
            std::string id_str = extract_attr(line, "id");
            if (!id_str.empty()) {
                current_way.id = std::stoi(id_str);
            }
            in_way = true;
        } else if (in_way && line.find("<nd") != std::string::npos) {
            std::string ref_str = extract_attr(line, "ref");
            if (!ref_str.empty()) {
                current_way.nodes.push_back(std::stoi(ref_str));
            }
        } else if (in_way && line.find("<tag") != std::string::npos) {
            std::string k = extract_attr(line, "k");
            std::string v = extract_attr(line, "v");
            if (!k.empty() && !v.empty()) {
                current_way.tags[k] = v;
            }
        } else if (in_way && line.find("</way>") != std::string::npos) {
            in_way = false;
            // Validate way: Lanes, parking spots, or ev charging points (boundary is ignored)
            bool is_valid = false;
            if (current_way.tags.count("amenity") && current_way.tags["amenity"] == "ev_charging") {
                is_valid = true;
            } else if (current_way.tags.count("type") &&
                       (current_way.tags["type"] == "parking_spot" ||
                        current_way.tags["type"] == "ev_charging" ||
                        current_way.tags["type"] == "lane" ||
                        current_way.tags["type"] == "line")) {
                is_valid = true;
            }

            if (is_valid) {
                ways.push_back(current_way);
            }
        }
    }

    features_.clear();

    // Iterate over valid ways, inserting nodes and interpolating segments at resolution
    for (const auto& way : ways) {
        int class_id = 1; // Default to 1 (Lane markings, parking spots)
        if (way.tags.count("amenity") && way.tags.at("amenity") == "ev_charging") {
            class_id = 2; // EV charging
        } else if (way.tags.count("type") && way.tags.at("type") == "ev_charging") {
            class_id = 2; // EV charging
        }

        for (size_t i = 0; i + 1 < way.nodes.size(); ++i) {
            int id_a = way.nodes[i];
            int id_b = way.nodes[i+1];
            if (node_map.find(id_a) == node_map.end() || node_map.find(id_b) == node_map.end()) {
                continue;
            }

            const auto& node_a = node_map[id_a];
            const auto& node_b = node_map[id_b];

            // Add the segment start node
            MapFeature feat_a;
            feat_a.x = node_a.x;
            feat_a.y = node_a.y;
            feat_a.class_id = class_id;
            features_.push_back(feat_a);

            // Interpolate points between node_a and node_b using exact resolution step!
            double dx = node_b.x - node_a.x;
            double dy = node_b.y - node_a.y;
            double len = std::sqrt(dx*dx + dy*dy);

            int steps = std::ceil(len / resolution_);
            for (int s = 1; s < steps; ++s) {
                double t = (double)s / steps;
                MapFeature feat;
                feat.x = node_a.x + t * dx;
                feat.y = node_a.y + t * dy;
                feat.class_id = class_id;
                features_.push_back(feat);
            }
        }

        // Add the closing node of the way to ensure a complete outline
        if (!way.nodes.empty()) {
            int last_id = way.nodes.back();
            if (node_map.find(last_id) != node_map.end()) {
                MapFeature feat_last;
                feat_last.x = node_map[last_id].x;
                feat_last.y = node_map[last_id].y;
                feat_last.class_id = class_id;
                features_.push_back(feat_last);
            }
        }

        // If it's a landmark (Class 2), fill the inner polygon with grid points at resolution
        if (class_id == 2 && way.nodes.size() >= 3) {
            std::vector<TempNode> poly_nodes;
            poly_nodes.reserve(way.nodes.size());
            for (int node_id : way.nodes) {
                if (node_map.find(node_id) != node_map.end()) {
                    poly_nodes.push_back(node_map[node_id]);
                }
            }
            if (poly_nodes.size() >= 3) {
                double min_x = poly_nodes[0].x;
                double max_x = poly_nodes[0].x;
                double min_y = poly_nodes[0].y;
                double max_y = poly_nodes[0].y;
                for (const auto& node : poly_nodes) {
                    if (node.x < min_x) min_x = node.x;
                    if (node.x > max_x) max_x = node.x;
                    if (node.y < min_y) min_y = node.y;
                    if (node.y > max_y) max_y = node.y;
                }
                for (double gx = min_x + resolution_/2.0; gx < max_x; gx += resolution_) {
                    for (double gy = min_y + resolution_/2.0; gy < max_y; gy += resolution_) {
                        if (is_point_in_polygon(gx, gy, poly_nodes)) {
                            MapFeature feat;
                            feat.x = gx;
                            feat.y = gy;
                            feat.class_id = class_id;
                            features_.push_back(feat);
                        }
                    }
                }
            }
        }
    }

    std::cout << "[OsmMapLoader] Loaded " << features_.size()
              << " features selectively (interpolated at dynamic BEV resolution " << resolution_
              << " m) from OSM map." << std::endl;
    return true;
}

std::vector<MapFeature> OsmMapLoader::queryNear(double x, double y, double radius) const {
    if (radius < 0.0) {
        return features_;
    }

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
