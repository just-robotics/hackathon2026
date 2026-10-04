#ifndef DBSCAN_FILTER__DBSCAN_HPP
#define DBSCAN_FILTER__DBSCAN_HPP


#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>


namespace dbscan_filter {

/// Метки точек. Неотрицательное значение -- номер кластера.
constexpr int32_t kNoise = -1;
constexpr int32_t kUnvisited = -2;


struct DbscanParams {
    /// Радиус соседства, м.
    double eps = 0.2;

    /// Сколько точек (включая саму) должно попасть в eps, чтобы точка
    /// считалась ядровой.
    int min_points = 4;

    /// Кластеры мельче этого порога отбрасываются вместе с шумом: одиночные
    /// отражения от пыли собираются в группы по 2-3 точки и ядровой критерий
    /// их не всегда срезает.
    int min_cluster_size = 10;

    /// Минимальная дальность точки от начала координат, м. Ближе этого
    /// радиуса точки отбрасываются до кластеризации.
    ///
    /// Livox отдаёт невалидные измерения как ровно нулевые точки при
    /// is_dense = true, поэтому отсечь их по NaN нельзя: в кадре Mid-360 их
    /// около половины, и в кластеризации они сливаются в один плотный комок в
    /// позиции сенсора.
    double min_range = 0.05;

    /// Радиус действия min_cluster_size, м. Проверяются только кластеры, чей
    /// центроид ближе этого радиуса по горизонтали.
    ///
    /// Плотность облака падает с дистанцией: дальняя стена даёт законные 5-10
    /// точек, и без радиуса порог срезал бы её вместе с шумом. 0 -- проверять
    /// на любой дистанции.
    double min_cluster_size_radius = 3.0;
};


struct DbscanStats {
    size_t points_in = 0;
    size_t clusters = 0;
    size_t points_noise = 0;
    size_t points_small = 0;
    size_t points_out = 0;
};


/// Разреженная воксельная сетка для поиска соседей.
///
/// Нужна, потому что честный перебор пар на облаке Mid-360 (до ~20 тыс. точек
/// на кадр) -- это 4*10^8 сравнений на кадр, что не укладывается в 10 Гц.
/// Шаг сетки равен eps, поэтому все соседи точки лежат в её вокселе и 26
/// прилегающих.
class VoxelIndex {
public:
    VoxelIndex(const std::vector<Eigen::Vector3f>& points, double cell_size);

    /// Складывает в out индексы точек из вокселя точки и соседних вокселей.
    /// Это надмножество настоящих соседей -- расстояние проверяет вызывающий.
    void candidates(const Eigen::Vector3f& point, std::vector<uint32_t>& out) const;

private:
    struct Key {
        int32_t x;
        int32_t y;
        int32_t z;

        bool operator==(const Key& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct KeyHash {
        size_t operator()(const Key& key) const {
            // Перемешивание тремя большими простыми: ключи идут подряд по
            // координатам, и без него корзины вырождаются в цепочки.
            return (static_cast<size_t>(static_cast<uint32_t>(key.x)) * 73856093u)
                 ^ (static_cast<size_t>(static_cast<uint32_t>(key.y)) * 19349663u)
                 ^ (static_cast<size_t>(static_cast<uint32_t>(key.z)) * 83492791u);
        }
    };

    Key keyOf(const Eigen::Vector3f& point) const;

    double cell_size_;
    std::unordered_map<Key, std::vector<uint32_t>, KeyHash> cells_;
};


/// Классический DBSCAN: обход в ширину от каждой непосещённой ядровой точки.
///
/// labels заполняется на каждую точку: номер кластера, kNoise для шума.
/// Возвращает число найденных кластеров.
size_t cluster(
    const std::vector<Eigen::Vector3f>& points,
    const DbscanParams& params,
    std::vector<int32_t>& labels
);


/// Отмечает kNoise точки мелких кластеров, чей центроид лежит ближе
/// min_cluster_size_radius по горизонтали. Возвращает число отброшенных точек.
size_t dropSmallClusters(
    const std::vector<Eigen::Vector3f>& points,
    std::vector<int32_t>& labels,
    size_t clusters,
    const DbscanParams& params
);

}  // namespace dbscan_filter


#endif  // DBSCAN_FILTER__DBSCAN_HPP
