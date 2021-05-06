#include "precomp.h"
#include "ViewshedRelationGraph.h"


namespace Viewshed
{
    ViewshedRelationGraph::ViewshedRelationGraph(const std::vector<VSRelationEdge>& relationEdges)
    {
        AddEdges(relationEdges);
    }

    void ViewshedRelationGraph::AddEdges(const std::vector<VSRelationEdge>& relationEdges)
    {
        for (const VSRelationEdge& re : relationEdges)
        {
            AddEdge(re);
        }
    }

    void ViewshedRelationGraph::AddEdge(const VSRelationEdge& relationEdge)
    {
        assert(relationEdge.iSource >= 0);
        assert(relationEdge.jDestination >= 0);
        assert(!(relationEdge.obscured && relationEdge.obscures));

        //Add twin of obscures as well (Handles embedded polys)
        //TODO: Improve this, map of maps instead?
        if (relationEdge.obscures)
        {
            VSRelationEdge twinEdge(relationEdge.jDestination, relationEdge.iSource);
            twinEdge.obscured = true;
            twinEdge.intersects = relationEdge.intersects;
            AddEdge(twinEdge);
        }

        nodes[relationEdge.iSource].push_back(relationEdge);
    }

    void ViewshedRelationGraph::AddNode(const int node)
    {
        nodes.insert(std::pair<int, std::vector<VSRelationEdge>>(node, std::vector<VSRelationEdge>()));
    }

    void ViewshedRelationGraph::RemoveNode(const int node)
    {
        nodes.erase(node);
    }

    void ViewshedRelationGraph::RemoveNodes(const std::vector<int> nodes)
    {
        for (const int n : nodes)
        {
            RemoveNode(n);
        }
    }

    //Removes all edges that contain a destination to a node that does not exist in the graph
    void ViewshedRelationGraph::Trim()
    {
        for (auto& node : nodes)
        {
            node.second.erase(std::remove_if(node.second.begin(), node.second.end(), [&](VSRelationEdge edge) {return (nodes.find(edge.jDestination) == nodes.end()); }), node.second.end());
        }
    }

    //Removes all edges that represent an intersection between two polygons
    void ViewshedRelationGraph::RemoveIntersectionEdges()
    {
        for (auto& node : nodes)
        {
            node.second.erase(std::remove_if(node.second.begin(), node.second.end(), [&](VSRelationEdge edge) {return (edge.intersects); }), node.second.end());
        }
    }

    std::vector<VSRelationEdge> ViewshedRelationGraph::GetEdges(const int nodeId) const
    {
        return nodes.at(nodeId);
    }

    //Returns the closest neighbouring node based on a given priority list (Far -> Close)
    int ViewshedRelationGraph::GetClosestNeighbour(const int nodeId, const std::vector<int>& priorities) const
    {
        int highestPrio = -1;
        int highestindex = -1;
        for (const VSRelationEdge& edge : nodes.at(nodeId))
        {
            std::vector<int>::const_iterator it = std::find(priorities.begin(), priorities.end(), edge.jDestination);

            if (it != priorities.end())
            {
                int index = std::distance(priorities.begin(), it);
                if (index > highestindex)
                {
                    highestPrio = priorities.at(index);
                    highestindex = index;
                }
            }
        }

        return highestPrio;
    }

    std::vector<int> ViewshedRelationGraph::GetStartingNodes() const
    {
        std::vector<int> startNodes;
        for (auto& node : nodes)
        {
            //If node does not have any 'obscures' edges add to start node list
            bool obscures = false;
            for (const VSRelationEdge& edge : node.second)
            {
                obscures = (obscures || edge.obscures);
            }

            if (!obscures)
            {
                startNodes.push_back(node.first);
            }
        }

        return startNodes;
    }

    //Sorts the nodes in the graph from farthest to closest polygons
    std::vector<int> ViewshedRelationGraph::TopologicalSort() const
    {
        std::vector<int> sortedNodes;
        sortedNodes.reserve(nodes.size());

        //For each node set visited to false
        std::map<int, bool> visited;
        for (auto& n : nodes)
        {
            visited[n.first] = false;
        }

        //For each starting node recursively visit the graph depth first
        std::vector<int> visitStack = GetStartingNodes();
        while (!visitStack.empty())
        {
            TopologicalSortVisit(visitStack.back(), sortedNodes, visited);
            visitStack.pop_back();
        }

        //Reverse the order of the sorted vector and return
        std::reverse(sortedNodes.begin(), sortedNodes.end());
        return sortedNodes;
    }

    void ViewshedRelationGraph::TopologicalSortVisit(const int node, std::vector<int>& sortedNodes, std::map<int, bool>& visited) const
    {
        if (visited.at(node))
        {
            return;
        }

        //Recursively visit each child node that obscures this nodes polygon
        for (const VSRelationEdge& edge : nodes.at(node))
        {
            if (edge.obscured)
            {
                TopologicalSortVisit(edge.jDestination, sortedNodes, visited);
            }
        }

        //After visiting each child mark as visited and add this node to the sorted list
        visited.at(node) = true;
        sortedNodes.push_back(node);
    }

    void ViewshedRelationGraph::PrintGraph() const
    {
        std::cout << "Overlap relation graph:" << std::endl;
        for (auto& node : nodes)
        {
            std::cout << node.first << std::endl;


            for (auto& edge : node.second)
            {
                if (edge.obscures)
                {
                    std::cout << "\t" << edge.iSource << "<" << edge.jDestination;
                }

                if (edge.obscured)
                {
                    std::cout << "\t" << edge.iSource << ">" << edge.jDestination;
                }

                if (!edge.intersects)
                {
                    std::cout << "(embedded)";
                }

                std::cout << std::endl;
            }
        }
    }
}