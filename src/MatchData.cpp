#include <algorithm>
#include <assert.h>

#include "MatchData.hpp"

bool operator < (const AdjListElem& lhs, const AdjListElem& rhs)
{
    return lhs.m_index < rhs.m_index;
}



MatchTable::MatchTable(std::size_t num_images)
    : m_match_lists(num_images)
{
}

void MatchTable::SetMatch(MatchIndex idx)
{
    if (Contains(idx)) return;      // Already set

    auto& list = m_match_lists[idx.first];
    auto p = lower_bound(list.begin(), list.end(), AdjListElem(idx.second));
    list.insert(p, AdjListElem(idx.second));
}

void MatchTable::AddMatch(MatchIndex idx, KeypointMatch m)
{
    assert(Contains(idx));
    GetMatchList(idx).push_back(m);
}

void MatchTable::ClearMatch(MatchIndex idx)
{
    // But don't erase!
    if (Contains(idx)) GetMatchList(idx).clear();
}

void MatchTable::RemoveMatch(MatchIndex idx)
{
    if (Contains(idx))
    {
        auto &match_list = GetMatchList(idx);
        match_list.clear();

        // Remove the neighbor
        auto& list = m_match_lists[idx.first];
        auto p = equal_range(list.begin(), list.end(), AdjListElem(idx.second));

        list.erase(p.first, p.second);
    }
}

void MatchTable::RemoveAll()
{
    for (auto& list : m_match_lists) list.clear();
}

bool MatchTable::Contains(MatchIndex idx) const
{
    const auto& list = m_match_lists[idx.first];
    auto p = equal_range(list.begin(), list.end(), AdjListElem(idx.second));

    return (p.first != p.second);
}

std::size_t MatchTable::GetNumMatches(MatchIndex idx)
{
    if (!Contains(idx)) return 0;
    return GetMatchList(idx).size();
}

std::size_t MatchTable::GetNumNeighbors(std::size_t i)
{
    return m_match_lists[i].size();
}

MatchAdjList& MatchTable::GetNeighbors(std::size_t i)
{
    return m_match_lists[i];
}

std::vector<KeypointMatch>& MatchTable::GetMatchList(MatchIndex idx)
{
    auto& list = m_match_lists[idx.first];
    auto p = equal_range(list.begin(), list.end(), AdjListElem(idx.second));

    assert(p.first != p.second);
    return (p.first)->m_match_list;
}

MatchAdjList::iterator MatchTable::begin(std::size_t i)
{
    return m_match_lists[i].begin();
}

MatchAdjList::iterator MatchTable::end(std::size_t i)
{
    return m_match_lists[i].end();
}
