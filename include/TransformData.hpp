#pragma once

#include <map>

struct TransformInfo
{
    TransformInfo()
        : m_num_inliers(0)
        , m_inlier_ratio(0.0)
    {}

    int     m_num_inliers;
    double  m_inlier_ratio;
};

typedef std::pair<unsigned long, unsigned long> MatchIndex;
typedef std::map<MatchIndex, TransformInfo>     TransformData;
typedef std::pair<MatchIndex, TransformInfo>    TransformsEntry;
