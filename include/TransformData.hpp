#pragma once

#include <map>

struct TransformInfo
{
    TransformInfo() = default;

    int     m_num_inliers = 0;
    double  m_inlier_ratio = 0.0;
};

typedef std::pair<unsigned int, unsigned int>   MatchIndex;
typedef std::map<MatchIndex, TransformInfo>     TransformData;
typedef std::pair<MatchIndex, TransformInfo>    TransformsEntry;
