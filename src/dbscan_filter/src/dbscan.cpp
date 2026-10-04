#include "dbscan_filter/dbscan.hpp"

#include <cmath>
#include <deque>


namespace dbscan_filter {

VoxelIndex::VoxelIndex(const std::vector<Eigen::Vector3f>& points, double cell_size)
    : cell_size_(cell_size > 0.0 ? cell_size : 1.0) {

    cells_.reserve(points.size());

    for (uint32_t index = 0; index < points.size(); ++index) {
        cells_[keyOf(points[index])].push_back(index);
    }
}


VoxelIndex::Key VoxelIndex::keyOf(const Eigen::Vector3f& point) const {
    return Key{
        static_cast<int32_t>(std::floor(point.x() / cell_size_)),
        static_cast<int32_t>(std::floor(point.y() / cell_size_)),
        static_cast<int32_t>(std::floor(point.z() / cell_size_)),
    };
}


void VoxelIndex::candidates(
    const Eigen::Vector3f& point,
    std::vector<uint32_t>& out
) const {
    out.clear();

    const Key center = keyOf(point);

    for (int32_t dx = -1; dx <= 1; ++dx) {
        for (int32_t dy = -1; dy <= 1; ++dy) {
            for (int32_t dz = -1; dz <= 1; ++dz) {
                const auto cell = cells_.find(
                    Key{center.x + dx, center.y + dy, center.z + dz}
                );

                if (cell == cells_.end()) {
                    continue;
                }

                out.insert(out.end(), cell->second.begin(), cell->second.end());
            }
        }
    }
}


size_t cluster(
    const std::vector<Eigen::Vector3f>& points,
    const DbscanParams& params,
    std::vector<int32_t>& labels
) {
    labels.assign(points.size(), kUnvisited);

    if (points.empty()) {
        return 0;
    }

    // Нулевые и слишком близкие точки выводятся из игры до кластеризации:
    // они не должны ни попадать в выход, ни влиять на плотность соседей.
    const float min_range_squared =
        static_cast<float>(params.min_range * params.min_range);
    std::vector<uint32_t> valid;
    valid.reserve(points.size());

    for (uint32_t index = 0; index < points.size(); ++index) {
        if (points[index].squaredNorm() < min_range_squared) {
            labels[index] = kNoise;
        } else {
            valid.push_back(index);
        }
    }

    if (valid.empty()) {
        return 0;
    }

    // Сетка строится только по валидным точкам, иначе отброшенные всё равно
    // находились бы в соседях.
    std::vector<Eigen::Vector3f> valid_points;
    valid_points.reserve(valid.size());
    for (const uint32_t index : valid) {
        valid_points.push_back(points[index]);
    }

    const VoxelIndex index(valid_points, params.eps);
    const float eps_squared = static_cast<float>(params.eps * params.eps);

    // Буферы живут снаружи циклов: на кадре в 20 тыс. точек их переаллокация
    // на каждом соседстве заметно дороже самого поиска.
    std::vector<uint32_t> candidates;
    std::vector<uint32_t> neighbours;
    std::deque<uint32_t> queue;

    const auto neighboursOf = [&](uint32_t point_index, std::vector<uint32_t>& out) {
        index.candidates(valid_points[point_index], candidates);
        out.clear();

        for (const uint32_t candidate : candidates) {
            if ((valid_points[candidate] - valid_points[point_index]).squaredNorm() <= eps_squared) {
                out.push_back(candidate);
            }
        }
    };

    // labels_ хранит метки по индексам входного облака, а кластеризация идёт
    // по valid_points -- отсюда перевод через valid[].
    std::vector<int32_t> valid_labels(valid_points.size(), kUnvisited);

    size_t clusters = 0;

    for (uint32_t seed = 0; seed < valid_points.size(); ++seed) {
        if (valid_labels[seed] != kUnvisited) {
            continue;
        }

        neighboursOf(seed, neighbours);

        if (neighbours.size() < static_cast<size_t>(params.min_points)) {
            // Не ядровая. Может быть позже присоединена как граничная, поэтому
            // метка шума здесь не окончательная.
            valid_labels[seed] = kNoise;
            continue;
        }

        const int32_t label = static_cast<int32_t>(clusters++);
        valid_labels[seed] = label;

        queue.assign(neighbours.begin(), neighbours.end());

        while (!queue.empty()) {
            const uint32_t current = queue.front();
            queue.pop_front();

            if (valid_labels[current] >= 0) {
                continue;
            }

            // Точка, ранее помеченная шумом, становится граничной: в кластер
            // входит, но расширение от неё не идёт.
            const bool was_noise = valid_labels[current] == kNoise;
            valid_labels[current] = label;

            if (was_noise) {
                continue;
            }

            neighboursOf(current, neighbours);

            if (neighbours.size() < static_cast<size_t>(params.min_points)) {
                continue;
            }

            for (const uint32_t neighbour : neighbours) {
                if (valid_labels[neighbour] < 0) {
                    queue.push_back(neighbour);
                }
            }
        }
    }


    // Метки переносятся на индексы входного облака.
    for (uint32_t index = 0; index < valid.size(); ++index) {
        labels[valid[index]] = valid_labels[index];
    }

    return clusters;
}


size_t dropSmallClusters(
    const std::vector<Eigen::Vector3f>& points,
    std::vector<int32_t>& labels,
    size_t clusters,
    const DbscanParams& params
) {
    if (clusters == 0 || params.min_cluster_size <= 1) {
        return 0;
    }

    std::vector<size_t> sizes(clusters, 0);
    std::vector<double> sum_x(clusters, 0.0);
    std::vector<double> sum_y(clusters, 0.0);

    for (size_t index = 0; index < labels.size(); ++index) {
        const int32_t label = labels[index];

        if (label < 0) {
            continue;
        }

        const size_t cluster_index = static_cast<size_t>(label);
        ++sizes[cluster_index];
        sum_x[cluster_index] += points[index].x();
        sum_y[cluster_index] += points[index].y();
    }

    // Радиус 0 отключает ограничение: порог действует на любой дистанции.
    const bool limited = params.min_cluster_size_radius > 0.0;
    const double radius_squared =
        params.min_cluster_size_radius * params.min_cluster_size_radius;

    std::vector<bool> drop(clusters, false);

    for (size_t cluster_index = 0; cluster_index < clusters; ++cluster_index) {
        if (sizes[cluster_index] >= static_cast<size_t>(params.min_cluster_size)) {
            continue;
        }

        if (limited) {
            // Центроид по горизонтали: плотность падает с дальностью, и порог
            // применяется только там, где облако заведомо плотное.
            const double centroid_x = sum_x[cluster_index] / static_cast<double>(sizes[cluster_index]);
            const double centroid_y = sum_y[cluster_index] / static_cast<double>(sizes[cluster_index]);

            if (centroid_x * centroid_x + centroid_y * centroid_y > radius_squared) {
                continue;
            }
        }

        drop[cluster_index] = true;
    }

    size_t dropped = 0;

    for (int32_t& label : labels) {
        if (label >= 0 && drop[static_cast<size_t>(label)]) {
            label = kNoise;
            ++dropped;
        }
    }

    return dropped;
}


}  // namespace dbscan_filter
