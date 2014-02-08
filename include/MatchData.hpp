#pragma once

#include <numeric>
#include <vector>

typedef std::pair<std::size_t, std::size_t>	KeypointMatch;
typedef std::pair<std::size_t, std::size_t>	ImagePair;

struct MatchingImage
{
    MatchingImage() = default;
    MatchingImage(std::size_t index)
        : m_index(index)
    {}

    std::size_t                m_index = 0;
    std::vector<KeypointMatch> m_match_list;
};

typedef std::vector<MatchingImage> NeighborList;

bool operator < (const MatchingImage& lhs, const MatchingImage& rhs);



// Table containing information about which pairs of images match
class MatchTable
{
public:
    MatchTable() = default;
    MatchTable(std::size_t num_images);

    void SetMatch(ImagePair pair);
    void AddMatch(ImagePair pair, KeypointMatch match);
    void ClearMatch(ImagePair pair);
    void RemoveMatch(ImagePair pair);
    void RemoveAll();
    bool Contains(ImagePair pair) const;

    std::size_t                 GetNumMatches(ImagePair pair) const;
    std::size_t                 GetNumNeighbors(std::size_t i) const;
    NeighborList&               GetNeighbors(std::size_t i);
    std::vector<KeypointMatch>& GetMatchList(ImagePair pair);

    NeighborList::iterator      begin(std::size_t i);
    NeighborList::iterator      end(std::size_t i);

private:
    std::vector<NeighborList>   m_match_lists;
};