#pragma once

#include <vector>

struct KeypointMatch
{
    KeypointMatch() = default;

    KeypointMatch(int index1, int index2);

    int m_idx1 = 0, m_idx2 = 0;
};

struct AdjListElem
{
    AdjListElem() = default;

    unsigned int                m_index = 0;
    std::vector<KeypointMatch>  m_match_list;
};

bool operator < (const AdjListElem& lhs, const AdjListElem& rhs);

typedef std::vector<AdjListElem>                MatchAdjList;
typedef std::pair<unsigned int, unsigned int>	MatchIndex;

// Table containing information about which pairs of images match
class MatchTable
{
public:
    MatchTable() = default;
    MatchTable(int num_images);

    void SetMatch(MatchIndex idx);
    void AddMatch(MatchIndex idx, KeypointMatch m);
    void ClearMatch(MatchIndex idx);
    void RemoveMatch(MatchIndex idx);
    void RemoveAll();
    bool Contains(MatchIndex idx) const;

    unsigned int                GetNumMatches(MatchIndex idx);
    unsigned int                GetNumNeighbors(unsigned int i);
    MatchAdjList&               GetNeighbors(unsigned int i);
    std::vector<KeypointMatch>& GetMatchList(MatchIndex idx);

    MatchAdjList::iterator begin(unsigned int i);
    MatchAdjList::iterator end(unsigned int i);

private:
    std::vector<MatchAdjList> m_match_lists;
};

// Return the match index of a pair of images
inline MatchIndex GetMatchIndex(int i1, int i2)
{
    return MatchIndex((unsigned int)i1, (unsigned int)i2);
}