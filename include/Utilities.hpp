#pragma once

#include <algorithm>
#include <numeric>
#include <random>
#include <vector>

const double PI = 3.1415926535897932384626433832795;

namespace util
{

// Clamps the value x to lie within min and max
template<typename T>
T clamp(T x, T min, T max)
{
    return x < min ? min : (x > max ? max : x);
}

// Return the closest integer to x, rounding up
template<typename T>
int iround(T x)
{
    return x < 0.0 ? static_cast<int>(x - 0.5) : static_cast<int>(x + 0.5);
}

// Convert radians to degrees
template<typename T>
T rad2deg(T r)
{
    return r * (180.0 / PI);
}

// Convert degrees to radians
template<typename T>
T deg2rad(T d)
{
    return d * (PI / 180.0);
}

// Returns the value of the nth element
template<typename T>
typename T::value_type GetNthElement(int n, const T &container)
{
    T container_copy(container);
    std::nth_element(container_copy.begin(), container_copy.begin() + n, container_copy.end());

    return container_copy[n];
}

// Returns n random integer values ranging from 0 to max
inline std::vector<int> GetNRandomIndices(int n, int max)
{
    std::vector<int> indices(max);
    std::iota(indices.begin(), indices.end(), 0);

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(indices.begin(), indices.end(), g);
    indices.erase(indices.begin() + n, indices.end());

    return indices;
}

template<typename T>
bool SaveContainerToFileBinary(const std::string& filename, const T& descriptors)
{
    std::ofstream file(filename, std::ios::out | std::ios::binary);

    const std::size_t size = descriptors.size();
    file.write(reinterpret_cast<const char*>(&size), sizeof(std::size_t));
    file.write(reinterpret_cast<const char*>(descriptors.data()), size * sizeof(typename T::value_type));

    file.close();
    return file.good();
}

template<typename T>
bool LoadContainerFromFileBinary(const std::string& filename, T& descriptors)
{
    descriptors.clear();
    std::ifstream file(filename, std::ios::in | std::ios::binary);

    std::size_t size;
    file.read(reinterpret_cast<char*>(&size), sizeof(std::size_t));

    descriptors.resize(size);
    file.read(reinterpret_cast<char*>(descriptors.data()), size * sizeof(typename T::value_type));

    file.close();
    return file.good();
}

}
