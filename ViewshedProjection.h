#pragma once

namespace Viewshed
{
    class ViewshedRelationGraph;
    class VSRelationEdge;

    class ViewshedProjection
    {
    public:

        ViewshedProjection(const Delaunay& DT, const Point_3& viewpoint, const ViewDirection direction);

        ViewshedRelationGraph OverlapRelation(std::unordered_map<int, Vs_Vertex_handle>& repVertexHandles);

        ViewshedArrangement DetermineVisibility();

        bool IsEmpty() const;

    private:

        bool IntersectViewplaneWithAABB(const Ray_3& viewToPoint, const Plane_3& viewplane, const Point_2& AABBMin, const Point_2& AABBMax, const ViewDirection viewDirection, Point_2& result) const;
        Point_2 IntersectViewplane(const Ray_3& viewToPoint, const Plane_3& viewplane, const ViewDirection viewDirection) const;
        bool PointInSquare(const Point_2& point, const Point_2& squareMin, const Point_2& squareMax) const;


        Point_2 Project3dTo2d(const Point_3& point3d, const ViewDirection direction) const;
        Point_3 Project2dTo3d(const Point_2& point2d, const ViewDirection direction) const;

        void SetTriData(Vs_Arrangement_2& faceArrangement, int currentTriangle, const Vs_Point_2& startingPoint) const;

        std::vector<VSRelationEdge> Overlap(const int faceIndex, const Vs_Vertex_const_handle repVertex, std::set<int>& invisibleFaces) const;

        Vs_Halfedge_const_handle NextHalfedge(const Vs_Vertex_const_handle vertex, const int faceIndex) const;

        bool HalfedgeOnBoundary(const size_t faceIndex, const Vs_Arrangement_2::Halfedge_const_handle halfedge) const;
        bool TriangleIsInfront(const Point_2& viewplanePoint, const int faceIndex, const int faceIndexOther, Kernel::FT& distanceDiff, bool& hit) const;

        bool VisibleFirstExternalHalfedge(const Vs_Vertex_handle vertex, const int faceIndex, Vs_Halfedge_handle& intersectingHalfedge) const;
        bool VisibleLastExternalHalfedge(const Vs_Vertex_handle vertex, const int faceIndex, Vs_Halfedge_handle& intersectingHalfedge) const;
        std::vector<Vs_Halfedge_handle> VisibleInternalHalfedges(const Vs_Vertex_handle vertex, const int faceIndex) const;

        bool RayTriangleIntersectionPoint(const Ray_3& ray, const Triangle_3& triangle, Point_3& intersectionPoint);

        Vs_Arrangement_2 vsArrangement;

        const Delaunay& DT;
        const Point_3 viewpoint;
        const ViewDirection direction;
        Plane_3 viewplane;

        //TODO: Fix this ugliness
        std::vector<Delaunay::Face_handle> faceHandles;

        const Vs_extended_overlay_traits overlay_traits;
    };
}