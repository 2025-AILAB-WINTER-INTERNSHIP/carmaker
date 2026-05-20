#ifndef CARMAKER_LOCALIZATION_MAP_LOADER_BASE_H
#define CARMAKER_LOCALIZATION_MAP_LOADER_BASE_H

#include <string>
#include <vector>
#include <set>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>
#include <utility>

namespace carmaker_localization {

struct MapFeature {
    double x;
    double y;
    uint8_t class_id;
};

/**
 * @brief 2D point used internally for geometry calculations.
 */
struct Point2d {
    double x;
    double y;
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

protected:
    static double snapToVoxel(double val, double res) {
        return (std::floor(val / res) + 0.5) * res;
    }

    static void sampleSegment(const Point2d& a, const Point2d& b,
                              double half_width, double res,
                              std::set<std::pair<double,double>>& out) {
        // 1. 물리적 마진을 포함한 Bounding Box 계산
        const double eps = 1e-6;
        double margin = half_width + eps;
        double min_x = std::min(a.x, b.x) - margin;
        double max_x = std::max(a.x, b.x) + margin;
        double min_y = std::min(a.y, b.y) - margin;
        double max_y = std::max(a.y, b.y) + margin;

        // 2. 부동소수점 공간을 정수형(Integer) 격자 인덱스로 변환
        int min_ix = static_cast<int>(std::floor(min_x / res));
        int max_ix = static_cast<int>(std::floor(max_x / res));
        int min_iy = static_cast<int>(std::floor(min_y / res));
        int max_iy = static_cast<int>(std::floor(max_y / res));

        // 3. 정수 루프를 통한 탐색 (누적 오차 0%)
        for (int ix = min_ix; ix <= max_ix; ++ix) {
            double gx = (ix + 0.5) * res; // 정확한 Voxel Center 위상 보장

            for (int iy = min_iy; iy <= max_iy; ++iy) {
                double gy = (iy + 0.5) * res;

                if (distToSegment(gx, gy, a, b) <= half_width + eps) {
                    out.insert({gx, gy});
                }
            }
        }
    }

    static void samplePolygon(const std::vector<Point2d>& polygon,
                              double res,
                              std::set<std::pair<double,double>>& out) {
        if (polygon.size() < 3) return;

        // 1. 다각형의 Bounding Box 계산
        double min_x = polygon[0].x, max_x = polygon[0].x;
        double min_y = polygon[0].y, max_y = polygon[0].y;
        for (const auto& p : polygon) {
            min_x = std::min(min_x, p.x);  max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y);  max_y = std::max(max_y, p.y);
        }

        // 2. 경계선 처리를 위해 상하좌우 1칸(res)씩 여유를 두고 정수 인덱스 변환
        int min_ix = static_cast<int>(std::floor(min_x / res)) - 1;
        int max_ix = static_cast<int>(std::floor(max_x / res)) + 1;
        int min_iy = static_cast<int>(std::floor(min_y / res)) - 1;
        int max_iy = static_cast<int>(std::floor(max_y / res)) + 1;

        // 3. 정수 루프로 다각형 내부 및 경계 판별
        for (int ix = min_ix; ix <= max_ix; ++ix) {
            double gx = (ix + 0.5) * res;

            for (int iy = min_iy; iy <= max_iy; ++iy) {
                double gy = (iy + 0.5) * res;

                if (isInsidePolygon(gx, gy, polygon) ||
                    distToPolygonBoundary(gx, gy, polygon) <= res * 0.5) {
                    out.insert({gx, gy});
                }
            }
        }
    }

    static void voxelsToFeatures(const std::set<std::pair<double,double>>& voxels,
                                 uint8_t class_id,
                                 std::vector<MapFeature>& features) {
        for (const auto& pt : voxels) {
            MapFeature f;
            f.x = pt.first;
            f.y = pt.second;
            f.class_id = class_id;
            features.push_back(f);
        }
    }

private:
    static double distToSegment(double px, double py, const Point2d& a, const Point2d& b) {
        double dx = b.x - a.x, dy = b.y - a.y;
        double l2 = dx*dx + dy*dy;
        if (l2 < 1e-12) return std::sqrt((px-a.x)*(px-a.x) + (py-a.y)*(py-a.y));
        double t = std::max(0.0, std::min(1.0, ((px-a.x)*dx + (py-a.y)*dy) / l2));
        double cx = a.x + t*dx, cy = a.y + t*dy;
        return std::sqrt((px-cx)*(px-cx) + (py-cy)*(py-cy));
    }

    static bool isInsidePolygon(double x, double y, const std::vector<Point2d>& poly) {
        bool inside = false;
        size_t n = poly.size();
        for (size_t i = 0, j = n-1; i < n; j = i++) {
            if (((poly[i].y > y) != (poly[j].y > y)) &&
                (x < (poly[j].x - poly[i].x) * (y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x)) {
                inside = !inside;
            }
        }
        return inside;
    }

    static double distToPolygonBoundary(double x, double y, const std::vector<Point2d>& poly) {
        double min_dist = std::numeric_limits<double>::max();
        size_t n = poly.size();
        for (size_t i = 0; i < n; ++i) {
            double d = distToSegment(x, y, poly[i], poly[(i+1) % n]);
            if (d < min_dist) min_dist = d;
        }
        return min_dist;
    }
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_MAP_LOADER_BASE_H
