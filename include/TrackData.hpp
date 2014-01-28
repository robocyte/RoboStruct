#pragma once

#include <vector>

typedef std::pair<int, int>     ImageKey;
typedef std::vector<ImageKey>   ImageKeyVector;

struct TrackData
{
    TrackData() = default;

    TrackData(ImageKeyVector views)
        : m_views(views)
    {}

    ImageKeyVector  m_views;
    int             m_extra = -1;
};
