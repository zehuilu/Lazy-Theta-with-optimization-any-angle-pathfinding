// Adapted from Peter Goldsborough's website.
// http://www.goldsborough.me/c++/python/cuda/2017/09/10/20-32-46-exploring_k-means_in_python,_c++_and_cuda/

#ifndef K_MEANS_NEW_HPP
#define K_MEANS_NEW_HPP

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>
#include <tuple>


struct Point {
    double x{0}, y{0};
};


using DataFrame = std::vector<Point>;


inline double square(double value) {
    return value * value;
}


inline double squared_l2_distance(Point first, Point second) {
    return square(first.x - second.x) + square(first.y - second.y);
}


std::tuple<DataFrame, std::vector<size_t>, std::vector<std::vector<size_t>>> k_means_new(const DataFrame& data, const size_t& k, const size_t& number_of_iterations) {
    static std::random_device seed;
    static std::mt19937 random_number_generator(seed());
    std::uniform_int_distribution<size_t> indices(0, data.size() - 1);

    // Pick centroids as random points from the dataset.
    DataFrame means(k);
    for (auto& cluster : means) {
        cluster = data[indices(random_number_generator)];
    }

    std::vector<size_t> assignments(data.size());
    for (size_t iteration = 0; iteration < number_of_iterations; ++iteration) {

        // Sum up and count points for each cluster.
        DataFrame new_means(k);
        std::vector<size_t> counts(k, 0);

        // Find assignments.
        for (size_t point = 0; point < data.size(); ++point) {
            double best_distance = std::numeric_limits<double>::max();
            size_t best_cluster = 0;
            for (size_t cluster = 0; cluster < k; ++cluster) {
                const double distance = squared_l2_distance(data[point], means[cluster]);
                if (distance < best_distance) {
                    best_distance = distance;
                    best_cluster = cluster;
                }
            }
            assignments[point] = best_cluster;

            // Sum up and count points for each cluster.
            const auto cluster = assignments[point];
            new_means[cluster].x += data[point].x;
            new_means[cluster].y += data[point].y;
            counts[cluster] += 1;
        }

        // Divide sums by counts to get new centroids.
        for (size_t cluster = 0; cluster < k; ++cluster) {
            // Turn 0/0 into 0/1 to avoid zero division.
            const auto count = std::max<size_t>(1, counts[cluster]);
            means[cluster].x = new_means[cluster].x / count;
            means[cluster].y = new_means[cluster].y / count;
        }
    }


    // create a 2D vector which contains points in different clusters
    std::vector<size_t> points_index_1d;
    std::vector<std::vector<size_t>> points_index_2d(k, points_index_1d);
    for (size_t point = 0; point < data.size(); ++point) {
        points_index_2d[assignments[point]].push_back(point);
    }


    return {means, assignments, points_index_2d};
}


#endif