#pragma once

namespace Viewshed
{
    class VSRelationEdge
    {
    public:

        VSRelationEdge() {};
        VSRelationEdge(int iSource, int jDestination) : iSource(iSource), jDestination(jDestination) {};

        int iSource = -1;
        int jDestination = -1;

        bool intersects = false; //i intersects j
        bool obscures = false;   //i obscures j
        bool obscured = false;   //j obscures i
    };

    class ViewshedRelationGraph
    {
    public:

        ViewshedRelationGraph() {};
        ViewshedRelationGraph(const std::vector<VSRelationEdge>& relationEdges);

        void AddEdges(const std::vector<VSRelationEdge>& relationEdges);
        void AddEdge(const VSRelationEdge& relationEdge);
        void AddNode(const int node);

        void RemoveNode(const int node);
        void RemoveNodes(const std::vector<int> nodes);
        void Trim();

        void RemoveIntersectionEdges();

        std::vector<VSRelationEdge> GetEdges(const int nodeId) const;
        int GetClosestNeighbour(const int nodeId, const std::vector<int>& priorities) const;

        std::vector<int> TopologicalSort() const;
        std::vector<int> GetStartingNodes() const;


        void ViewshedRelationGraph::PrintGraph() const;

    private:
        void TopologicalSortVisit(const int node, std::vector<int>& sortedNodes, std::map<int, bool>& visited) const;

        std::map<int, std::vector<VSRelationEdge>> nodes;
    };
}