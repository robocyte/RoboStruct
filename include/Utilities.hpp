#pragma once

#include <algorithm>
#include <numeric>
#include <random>
#include <vector>

const double PI = 3.1415926535897932384626433832795;

namespace util
{

	template <typename T> inline const
		T& min(const T &a, const T &b)
	{
		return (a < b) ? a : b;
	}

	template <typename T> inline const
		T& max(const T &a, const T &b)
	{
		return (a > b) ? a : b;
	}

	// Clamps the value x to lie within min and max
	template <typename T> inline
		T clamp(T x, T min, T max)
	{
		return x < min ? min : (x > max ? max : x);
	}

	// Return the closest integer to x, rounding up
	template <typename T> inline
		int iround(T x)
	{
		return x < 0.0 ? static_cast<int>(x - 0.5) : static_cast<int>(x + 0.5);
	}

	// Convert radians to degrees
	template <typename T> inline
		T rad2deg(T r)
	{
		return r * (180.0 / PI);
	}

	// Convert degrees to radians
	template <typename T> inline
		T deg2rad(T d)
	{
		return d * (PI / 180.0);
	}

	// Returns the value of the nth element
	template <typename T> typename
		T::value_type GetNthElement(int n, const T &container)
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

}
