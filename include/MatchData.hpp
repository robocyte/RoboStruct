#pragma once

#include <algorithm>
#include <assert.h>
#include <vector>

struct KeypointMatch
{
    KeypointMatch() = default;

    KeypointMatch(int idx1, int idx2)
        : m_idx1(idx1)
        , m_idx2(idx2)
    {}

    int m_idx1 = 0, m_idx2 = 0;
};

struct AdjListElem
{
    AdjListElem() = default;

    bool operator< (const AdjListElem &other) const { return m_index < other.m_index; }

    unsigned int                m_index = 0;
    std::vector<KeypointMatch>  m_match_list;
};

typedef std::vector<AdjListElem> MatchAdjList;
typedef std::pair<unsigned long, unsigned long>	MatchIndex;

// Table containing information about which pairs of images match
class MatchTable
{
public:
    MatchTable()
        : m_match_lists(0)
    {}

    MatchTable(int num_images)
        : m_match_lists(num_images)
    {}

    void SetMatch(MatchIndex idx)
    {
        if (Contains(idx)) return;      // Already set

        AdjListElem e;
        e.m_index = idx.second;
        auto &l = m_match_lists[idx.first];
        auto p = lower_bound(l.begin(), l.end(), e);
        l.insert(p, e);
    }

    void AddMatch(MatchIndex idx, KeypointMatch m)
    {
        assert(Contains(idx));
        GetMatchList(idx).push_back(m);
    }

    void ClearMatch(MatchIndex idx)
    {
        // But don't erase!
        if (Contains(idx)) GetMatchList(idx).clear();
    }

    void RemoveMatch(MatchIndex idx)
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

    unsigned int GetNumMatches(MatchIndex idx)
    {
        if (!Contains(idx)) return 0;
        return GetMatchList(idx).size();
    }

    std::vector<KeypointMatch> &GetMatchList(MatchIndex idx)
    {
        AdjListElem e;
        e.m_index = idx.second;
        auto &l = m_match_lists[idx.first];
        auto p = equal_range(l.begin(), l.end(), e);

        assert(p.first != p.second);
        return (p.first)->m_match_list;
    }

    bool Contains(MatchIndex idx) const
    {
        AdjListElem e;
        e.m_index = idx.second;
        const auto &l = m_match_lists[idx.first];
        auto p = equal_range(l.begin(), l.end(), e);

        return (p.first != p.second);
    }

    void RemoveAll()
    {
        for (auto &list : m_match_lists) list.clear();
    }

    unsigned int GetNumNeighbors(unsigned int i)
    {
        return m_match_lists[i].size();
    }

    MatchAdjList &GetNeighbors(unsigned int i)
    {
        return m_match_lists[i];
    }

    MatchAdjList::iterator Begin(unsigned int i)
    {
        return m_match_lists[i].begin();
    }

    MatchAdjList::iterator End(unsigned int i)
    {
        return m_match_lists[i].end();
    }

private:
    std::vector<MatchAdjList> m_match_lists;
};

// Return the match index of a pair of images
inline MatchIndex GetMatchIndex(int i1, int i2)
{
    return MatchIndex((unsigned long) i1, (unsigned long) i2);
}

inline MatchIndex GetMatchIndexUnordered(int i1, int i2)
{
    if (i1 < i2)    return MatchIndex((unsigned long) i1, (unsigned long) i2);
    else            return MatchIndex((unsigned long) i2, (unsigned long) i1);
}
