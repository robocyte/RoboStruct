#include <algorithm>
#include <assert.h>

#include "MatchData.hpp"

bool operator < (const MatchingImage& lhs, const MatchingImage& rhs)
{
    return lhs.m_index < rhs.m_index;
}



MatchTable::MatchTable(std::size_t num_images)
    : m_match_lists(num_images)
{
}

void MatchTable::SetMatch(ImagePair pair)
{
    if (Contains(pair)) return;      // Already set

    auto& list = m_match_lists[pair.first];
    const auto p = lower_bound(list.begin(), list.end(), MatchingImage(pair.second));
    list.insert(p, MatchingImage(pair.second));
}

void MatchTable::AddMatch(ImagePair pair, KeypointMatch match)
{
    assert(Contains(idx));
    GetMatchList(pair).push_back(match);
}

void MatchTable::ClearMatch(ImagePair pair)
{
    // But don't erase!
    if (Contains(pair)) GetMatchList(pair).clear();
}

void MatchTable::RemoveMatch(ImagePair pair)
{
    if (Contains(pair))
    {
        GetMatchList(pair).clear();

        // Remove the neighbor
        auto& list = m_match_lists[pair.first];
        const auto p = equal_range(list.begin(), list.end(), MatchingImage(pair.second));

        list.erase(p.first, p.second);
    }
}

void MatchTable::RemoveAll()
{
    for (auto& list : m_match_lists) list.clear();
}

bool MatchTable::Contains(ImagePair pair) const
{
    const auto& list = m_match_lists[pair.first];
    const auto p = equal_range(list.begin(), list.end(), MatchingImage(pair.second));

    return (p.first != p.second);
}

std::size_t MatchTable::GetNumMatches(ImagePair pair) const
{
    const auto& list = m_match_lists[pair.first];
    const auto p = equal_range(list.begin(), list.end(), MatchingImage(pair.second));

    if (p.first == p.second) return 0;
    return p.first->m_match_list.size();
}

std::size_t MatchTable::GetNumNeighbors(std::size_t i) const
{
    return m_match_lists[i].size();
}

NeighborList& MatchTable::GetNeighbors(std::size_t i)
{
    return m_match_lists[i];
}

std::vector<KeypointMatch>& MatchTable::GetMatchList(ImagePair pair)
{
    auto& list = m_match_lists[pair.first];
    auto p = equal_range(list.begin(), list.end(), MatchingImage(pair.second));

    assert(p.first != p.second);
    return (p.first)->m_match_list;
}

NeighborList::iterator MatchTable::begin(std::size_t i)
{
    return m_match_lists[i].begin();
}

NeighborList::iterator MatchTable::end(std::size_t i)
{
    return m_match_lists[i].end();
}
