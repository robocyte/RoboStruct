#pragma once

#include <map>

struct TransformInfo
{
    TransformInfo() = default;

    int     m_num_inliers  = 0;
    double  m_inlier_ratio = 0.0;
};

typedef std::pair<std::size_t, std::size_t>  ImagePair;
typedef std::map<ImagePair, TransformInfo>  TransformData;
typedef std::pair<ImagePair, TransformInfo> TransformsEntry;
