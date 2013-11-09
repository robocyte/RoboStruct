#pragma once

#include <vector>

typedef std::pair<int, int>     ImageKey;
typedef std::vector<ImageKey>   ImageKeyVector;

struct TrackData
{
public:
    TrackData()
        : m_views()
        , m_extra(-1)
    {}

    TrackData(ImageKeyVector views)
        : m_views(views)
        , m_extra(-1)
    {}

    ImageKeyVector  m_views;
    int             m_extra;
};
