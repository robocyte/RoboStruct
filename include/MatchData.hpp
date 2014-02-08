#pragma once

#include <vector>

typedef std::pair<std::size_t, std::size_t>	KeypointMatch;
typedef std::pair<std::size_t, std::size_t>	MatchIndex;

struct AdjListElem
{
    AdjListElem() = default;
    AdjListElem(std::size_t index)
        : m_index(index)
    {}

    std::size_t                m_index = 0;
    std::vector<KeypointMatch> m_match_list;
};

bool operator < (const AdjListElem& lhs, const AdjListElem& rhs);

typedef std::vector<AdjListElem> MatchAdjList;



// Table containing information about which pairs of images match
class MatchTable
{
public:
    MatchTable() = default;
    MatchTable(std::size_t num_images);

    void SetMatch(MatchIndex idx);
    void AddMatch(MatchIndex idx, KeypointMatch m);
    void ClearMatch(MatchIndex idx);
    void RemoveMatch(MatchIndex idx);
    void RemoveAll();
    bool Contains(MatchIndex idx) const;

    std::size_t                 GetNumMatches(MatchIndex idx);
    std::size_t                 GetNumNeighbors(std::size_t i);
    MatchAdjList&               GetNeighbors(std::size_t i);
    std::vector<KeypointMatch>& GetMatchList(MatchIndex idx);

    MatchAdjList::iterator      begin(std::size_t i);
    MatchAdjList::iterator      end(std::size_t i);

private:
    std::vector<MatchAdjList>   m_match_lists;
};