#include <algorithm>
#include <assert.h>

#include "MatchData.hpp"

bool operator < (const AdjListElem& lhs, const AdjListElem& rhs)
{
    return lhs.m_index < rhs.m_index;
}



MatchTable::MatchTable(int num_images)
    : m_match_lists(num_images)
{
}

void MatchTable::SetMatch(MatchIndex idx)
{
    if (Contains(idx)) return;      // Already set

    AdjListElem e;
    e.m_index = idx.second;
    auto &l = m_match_lists[idx.first];
    auto p = lower_bound(l.begin(), l.end(), e);
    l.insert(p, e);
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
        AdjListElem e;
        e.m_index = idx.second;
        auto &l = m_match_lists[idx.first];
        auto p = equal_range(l.begin(), l.end(), e);

        l.erase(p.first, p.second);
    }
}

void MatchTable::RemoveAll()
{
    for (auto &list : m_match_lists) list.clear();
}

bool MatchTable::Contains(MatchIndex idx) const
{
    AdjListElem e;
    e.m_index = idx.second;
    const auto &l = m_match_lists[idx.first];
    auto p = equal_range(l.begin(), l.end(), e);

    return (p.first != p.second);
}

unsigned int MatchTable::GetNumMatches(MatchIndex idx)
{
    if (!Contains(idx)) return 0;
    return GetMatchList(idx).size();
}

unsigned int MatchTable::GetNumNeighbors(unsigned int i)
{
    return m_match_lists[i].size();
}

MatchAdjList& MatchTable::GetNeighbors(unsigned int i)
{
    return m_match_lists[i];
}

std::vector<KeypointMatch>& MatchTable::GetMatchList(MatchIndex idx)
{
    AdjListElem e;
    e.m_index = idx.second;
    auto &l = m_match_lists[idx.first];
    auto p = equal_range(l.begin(), l.end(), e);

    assert(p.first != p.second);
    return (p.first)->m_match_list;
}

MatchAdjList::iterator MatchTable::begin(unsigned int i)
{
    return m_match_lists[i].begin();
}

MatchAdjList::iterator MatchTable::end(unsigned int i)
{
    return m_match_lists[i].end();
}
