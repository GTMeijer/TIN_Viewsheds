#include "precomp.h"
#include "ViewshedProjection.h"

namespace Viewshed
{

    ViewshedProjection::ViewshedProjection(const Delaunay& DT, const Point_3& viewpoint, const ViewDirection direction) : DT(DT), viewpoint(viewpoint), direction(direction)
    {
        Vector_3 viewDirection;

        switch (direction)
        {
        case ViewDirection::UP: //(0,0,1) -> (x,y)
            viewDirection = Vector_3(0, 0, 1);
            break;
        case ViewDirection::DOWN: //(0,0,-1) -> (x,-y)
            viewDirection = Vector_3(0, 0, -1);
            break;
        case ViewDirection::LEFT://(-1,0,0) -> (-y,z)
            viewDirection = Vector_3(-1, 0, 0);
            break;
        case ViewDirection::RIGHT://(1,0,0) -> (y,z)
            viewDirection = Vector_3(1, 0, 0);
            break;
        case ViewDirection::FRONT://(0,1,0) -> (x,z)
            viewDirection = Vector_3(0, 1, 0);
            break;
        case ViewDirection::BEHIND://(0,-1,0) -> (-x,z)
            viewDirection = Vector_3(0, -1, 0);
            break;
        }

        Point_3 viewplaneCenter = viewpoint + viewDirection;
        Plane_3 viewplane(viewplaneCenter, viewDirection);
        this->viewplane = viewplane;

        std::vector<Point_3> viewplaneCorners;

        //Yes this is ugly..
        if (direction == ViewDirection::UP || direction == ViewDirection::DOWN)
        {
            viewplaneCorners = {
                (viewplaneCenter + Vector_3(-1.0, -1.0, 0)),
                (viewplaneCenter + Vector_3(1.0,  -1.0, 0)),
                (viewplaneCenter + Vector_3(1.0,   1.0, 0)),
                (viewplaneCenter + Vector_3(-1.0,  1.0, 0))
            };
        }
        else if (direction == ViewDirection::FRONT || direction == ViewDirection::BEHIND)
        {
            viewplaneCorners = {
                (viewplaneCenter + Vector_3(-1.0, 0, -1.0)),
                (viewplaneCenter + Vector_3(1.0,  0, -1.0)),
                (viewplaneCenter + Vector_3(1.0,  0,  1.0)),
                (viewplaneCenter + Vector_3(-1.0, 0,  1.0))
            };
        }
        else //Left/Right
        {
            viewplaneCorners = {
                (viewplaneCenter + Vector_3(0, -1.0, -1.0)),
                (viewplaneCenter + Vector_3(0, 1.0,  -1.0)),
                (viewplaneCenter + Vector_3(0, 1.0,   1.0)),
                (viewplaneCenter + Vector_3(0, -1.0,  1.0))
            };
        }

        std::vector<Plane_3> viewFrustum =
        {
            Plane_3(viewpoint, viewplaneCorners.at(0), viewplaneCorners.at(1)),
            Plane_3(viewpoint, viewplaneCorners.at(1), viewplaneCorners.at(2)),
            Plane_3(viewpoint, viewplaneCorners.at(2), viewplaneCorners.at(3)),
            Plane_3(viewpoint, viewplaneCorners.at(3), viewplaneCorners.at(0))
        };

        Point_2 viewplaneMin = Point_2(-1.0, -1.0);
        Point_2 viewplaneMax = Point_2(1.0, 1.0);

        //for (Point_3& t : viewplaneCorners)
        //{
        //    std::cout << t << std::endl;
        //}

        Vs_Arrangement_2 lastArrangement;

        //std::cout << "ViewPoint: " << viewpoint << std::endl;
        //std::cout << "ViewCenter: " << viewplaneCenter << std::endl;


        //TODO Optimization: During this for loop check with dot and add to multiple viewarrangements at once..
        int overlayCount = 0;
        int currentTriangle = 0;
        const size_t faceCount = DT.number_of_faces();
        //Loop through all the triangles in the delaunay triangulation and project them onto the 2d arrangement plane
        for (auto triFace = DT.finite_faces_begin(); triFace != DT.finite_faces_end(); triFace++)
        {
            //std::cout << "\r\tFace: " << (currentTriangle + 1) << "/" << faceCount;
            std::vector<Point_2> projectedPoints;

            faceHandles.push_back(triFace);

            for (size_t i = 0; i < 3; i++)
            {
                //Construct a line between the viewpoint and the triangle vertex
                Ray_3 viewToPoint(viewpoint, triFace->vertex(i)->point());

                //std::cout << "triangle vertex target \t" << triFace->vertex(i)->point() << std::endl;

                //Test for intersection with the viewplane and store its 2d coordinate
                Point_2 vpIntersectionPoint;
                if (IntersectViewplaneWithAABB(viewToPoint, viewplane, viewplaneMin, viewplaneMax, direction, vpIntersectionPoint))
                {
                    projectedPoints.push_back(vpIntersectionPoint);
                }
            }

            //If tri verts are not contained in the frustum..
            if (projectedPoints.size() < 3)
            {
                //Find segments intersecting with the view frustum..
                for (size_t i = 0; i < 3; i++)
                {
                    //Take tri segment
                    Segment_3 triSeg(triFace->vertex(i)->point(), triFace->vertex(DT.ccw(i))->point());

                    //Test for segment intersection with the view frustum
                    for (size_t vfSide = 0; vfSide < viewFrustum.size(); vfSide++)
                    {
                        auto frustumIntersection = CGAL::intersection(triSeg, viewFrustum.at(vfSide));

                        if (frustumIntersection)
                        {
                            //Point intersection
                            if (Point_3* frustumIntersectionPoint = boost::get<Point_3>(&*frustumIntersection))
                            {
                                //On this viewplanes side? (Note: If it is between the viewpoint and viewplane thats fine because it'll just count as closer when mapped)
                                if (CGAL::scalar_product(*frustumIntersectionPoint - viewpoint, viewplaneCenter - viewpoint) > 0)
                                {
                                    Ray_3 viewToPoint(viewpoint, *frustumIntersectionPoint);

                                    //Intersect with viewplane
                                    Point_2 vpIntersectionPoint;
                                    if (IntersectViewplaneWithAABB(viewToPoint, viewplane, viewplaneMin, viewplaneMax, direction, vpIntersectionPoint))
                                    {
                                        projectedPoints.push_back(vpIntersectionPoint);
                                    }
                                }
                            }
                            //segment intersection with frustum plane (BOTH points are always infront or behind the viewpoint because we float it in the air by 1m)
                            else if (Segment_3* frustumIntersectionSegment = boost::get<Segment_3>(&*frustumIntersection))
                            {
                                Point_3 startPoint = frustumIntersectionSegment->start();
                                Point_3 endPoint = frustumIntersectionSegment->end();

                                bool startInfront = (CGAL::scalar_product(startPoint - viewpoint, viewplaneCenter - viewpoint) > 0);
                                bool endInfront = (CGAL::scalar_product(endPoint - viewpoint, viewplaneCenter - viewpoint) > 0);

                                //Intersect both points with the viewplane
                                //In the ultra-edge case of one/both of them falling outside of it they'll get picked up by the corner rays later
                                if (startInfront)
                                {
                                    Ray_3 viewToPoint(viewpoint, startPoint);

                                    //Intersect with viewplane
                                    Point_2 vpIntersectionPoint;
                                    if (IntersectViewplaneWithAABB(viewToPoint, viewplane, viewplaneMin, viewplaneMax, direction, vpIntersectionPoint))
                                    {
                                        projectedPoints.push_back(vpIntersectionPoint);
                                    }
                                }

                                if (endInfront)
                                {
                                    Ray_3 viewToPoint(viewpoint, endPoint);

                                    //Intersect with viewplane
                                    Point_2 vpIntersectionPoint;
                                    if (IntersectViewplaneWithAABB(viewToPoint, viewplane, viewplaneMin, viewplaneMax, direction, vpIntersectionPoint))
                                    {
                                        projectedPoints.push_back(vpIntersectionPoint);
                                    }
                                }
                            }
                        }
                    }
                }

                //Finally when the vertices and edge are outside of the view frustum but the triangle still intersects it
                //we need to add a point at one or more corners of the 2d viewplane
                Triangle_3 tri3d(triFace->vertex(0)->point(), triFace->vertex(1)->point(), triFace->vertex(2)->point());

                for (Point_3& viewplaneCorner : viewplaneCorners)
                {
                    Ray_3 viewToPoint(viewpoint, viewplaneCorner);

                    //std::cout << "corner" << viewplaneCorner << std::endl;
                    //std::cout << viewToPoint.start() << std::endl;
                    //std::cout << viewToPoint.direction() << std::endl;

                    auto triIntersection = CGAL::intersection(viewToPoint, tri3d);

                    //Check if the ray going through the corner intersects the triangle
                    if (triIntersection)
                    {
                        Point_3* triIntersectionPoint = boost::get<Point_3>(&*triIntersection);
                        Ray_3 viewToIntersectionPoint(viewpoint, *triIntersectionPoint);

                        //Intersect with viewplane
                        //Point_2 vpIntersection2d = Point_2(vpIntersectionPoint->x(), vpIntersectionPoint->y());
                        Point_2 vpIntersection2d = IntersectViewplane(viewToIntersectionPoint, viewplane, direction);
                        projectedPoints.push_back(vpIntersection2d);
                    }
                }
            }


            //If it is still only 2 here, discard -> only edge
            if (projectedPoints.size() < 3)
            {
                currentTriangle++;
                continue;
            }
            else
            {
                //Add to arrangement:

                //std::cout << triFace->vertex(0)->point() << std::endl;
                //std::cout << triFace->vertex(1)->point() << std::endl;
                //std::cout << triFace->vertex(2)->point() << std::endl;

                //Sort them in CCW order, do this with convex hull function :)
                std::vector<std::size_t> toSort(projectedPoints.size()), ccwIndices;
                std::iota(toSort.begin(), toSort.end(), 0);
                CGAL::convex_hull_2(toSort.begin(), toSort.end(), std::back_inserter(ccwIndices), Convex_hull_traits_2(CGAL::make_property_map(projectedPoints)));


                if (ccwIndices.size() < 3)
                {
                    //Points coincide, ignore..
                    //std::cout << std::endl << "Points coincide, ignoring.." << std::endl;
                    currentTriangle++;
                    continue;
                }

                //std::cout << std::endl;
                //for (Point_2& p : projectedPoints)
                //{
                //    std::cout << p << std::endl;
                //}

                //for (Point_3& p : originalPoints)
                //{
                //    std::cout << p << std::endl;
                //}

                //for (std::size_t i : ccwIndices) {
                //    std::cout << "points[" << i << "] = " << projectedPoints[i] << std::endl;
                //}

                //Construct arrangement for this face
                std::vector<Vs_Segment_2> faceSegments;
                for (size_t ccwI = 0; ccwI < ccwIndices.size(); ccwI++)
                {
                    faceSegments.push_back(Vs_Segment_2(projectedPoints.at(ccwIndices.at(ccwI)), projectedPoints.at(ccwIndices.at((ccwI + 1) % ccwIndices.size()))));
                }


                Vs_Arrangement_2 faceArrangement;
                for (Vs_Segment_2& seg : faceSegments)
                {
                    CGAL::insert_non_intersecting_curve(faceArrangement, seg);
                }

                SetTriData(faceArrangement, currentTriangle, projectedPoints.at(ccwIndices.at(0)));

                overlayCount++;
                //Overlay arrangements..
                CGAL::overlay(lastArrangement, faceArrangement, vsArrangement, overlay_traits);
                lastArrangement = vsArrangement;
            }

            currentTriangle++;
        }
        //std::cout << "\n" << overlayCount << " faces overlayed." << std::endl;
        //CGAL::draw(vsArrangement, "2d Projection");
        std::cout << std::endl;
    }

    bool ViewshedProjection::IntersectViewplaneWithAABB(const Ray_3& viewToPoint, const Plane_3& viewplane, const Point_2& AABBMin, const Point_2& AABBMax, const ViewDirection viewDirection, Point_2& result) const
    {
        auto intersection = CGAL::intersection(viewToPoint, viewplane);
        if (intersection)
        {
            Point_3* intersectionPoint = boost::get<Point_3>(&*intersection);
            //std::cout << "intersectionPoint\t" << *intersectionPoint << std::endl;
            Vector_3 viewpointVec = viewToPoint.source() - CGAL::ORIGIN;
            //Move to origin (viewpoint space) and project to 2d plane
            Point_2 intersection2d = Project3dTo2d((*intersectionPoint - viewpointVec), viewDirection);
            //std::cout << "intersection2d\t" << intersection2d << std::endl;
            //std::cout << "AABBMin\t" << AABBMin << std::endl;
            //std::cout << "AABBMax\t" << AABBMax << std::endl;

            //Within frustum?
            if (PointInSquare(intersection2d, AABBMin, AABBMax))
            {
                result = intersection2d;
                return true;
            }
        }

        return false;
    }

    Point_2 ViewshedProjection::IntersectViewplane(const Ray_3& viewToPoint, const Plane_3& viewplane, const ViewDirection viewDirection) const
    {
        //Intersect with viewplane
        auto viewplaneIntersection = CGAL::intersection(viewToPoint, viewplane);
        if (viewplaneIntersection)
        {
            Point_3* vpIntersectionPoint = boost::get<Point_3>(&*viewplaneIntersection);
            Vector_3 viewpointVec = viewToPoint.source() - CGAL::ORIGIN;;

            //Convert the viewplane intersection point to 2d by removing the dimension the plane is on
            Point_2 vpIntersection2d = Project3dTo2d((*vpIntersectionPoint - viewpointVec), viewDirection);

            return vpIntersection2d;
        }
        else
        {
            std::cout << "Shouldn't reach this!" << std::endl;
            return Point_2();
        }
    }

    //Check if a 2d point is within a 2d axis alligned square
    bool ViewshedProjection::PointInSquare(const Point_2& point, const Point_2& squareMin, const Point_2& squareMax) const
    {
        if (point.x() >= squareMin.x() &&
            point.x() <= squareMax.x() &&
            point.y() >= squareMin.y() &&
            point.y() <= squareMax.y())
        {
            return true;
        }
        else
        {
            return false;

        }
    }

    Point_2 ViewshedProjection::Project3dTo2d(const Point_3& point3d, const ViewDirection direction) const
    {
        switch (direction)
        {
        case ViewDirection::UP: //(0,0,1) -> (x,y)
            return Point_2(point3d.x(), point3d.y());
            break;
        case ViewDirection::DOWN: //(0,0,-1) -> (x,-y)
            return Point_2(point3d.x(), -point3d.y());
            break;
        case ViewDirection::LEFT://(-1,0,0) -> (y,z)
            return Point_2(point3d.y(), point3d.z());
            break;
        case ViewDirection::RIGHT://(1,0,0) -> (-y,z)
            return Point_2(-point3d.y(), point3d.z());
            break;
        case ViewDirection::FRONT://(0,1,0) -> (x,z)
            return Point_2(point3d.x(), point3d.z());
            break;
        case ViewDirection::BEHIND://(0,-1,0) -> (-x,z)
            return Point_2(point3d.x(), point3d.z());
            break;
        default:
            throw "Argument to Project3dTo2d out of range!";
        }
    }

    Point_3 ViewshedProjection::Project2dTo3d(const Point_2& point2d, const ViewDirection direction) const
    {
        switch (direction)
        {
        case ViewDirection::UP: //(0,0,1) -> (x,y)
            return Point_3(point2d.x(), point2d.y(), Kernel::FT(1)) + (viewpoint - CGAL::ORIGIN);
            break;
        case ViewDirection::DOWN: //(0,0,-1) -> (x,-y)
            return Point_3(point2d.x(), -point2d.y(), Kernel::FT(-1)) + (viewpoint - CGAL::ORIGIN);
            break;
        case ViewDirection::LEFT://(-1,0,0) -> (y,z)
            return Point_3(Kernel::FT(-1), point2d.x(), point2d.y()) + (viewpoint - CGAL::ORIGIN);
            break;
        case ViewDirection::RIGHT://(1,0,0) -> (-y,z)
            return Point_3(Kernel::FT(1), -point2d.x(), point2d.y()) + (viewpoint - CGAL::ORIGIN);
            break;
        case ViewDirection::FRONT://(0,1,0) -> (x,z)
            return Point_3(point2d.x(), Kernel::FT(1), point2d.y()) + (viewpoint - CGAL::ORIGIN);
            break;
        case ViewDirection::BEHIND://(0,-1,0) -> (-x,z)
            return Point_3(point2d.x(), Kernel::FT(-1), point2d.y()) + (viewpoint - CGAL::ORIGIN);
            break;
        default:
            throw "Argument to Project2dTo3d out of range!";
        }
    }

    void ViewshedProjection::SetTriData(Vs_Arrangement_2& faceArrangement, int currentTriangle, const Vs_Point_2& startingPoint) const
    {
        assert(faceArrangement.number_of_faces() == 2);

        //Get the interior face and set data
        auto faceHandle = faceArrangement.faces_begin();
        if (faceHandle->is_unbounded())
        {
            faceHandle++;
        }

        FaceData faceData;
        faceData.faceIndices.push_back(currentTriangle);
        faceHandle->set_data(faceData);

        //Set edge and vert data
        Vs_Halfedge_circulator edgeCirculator = faceHandle->outer_ccb();
        Vs_Halfedge_circulator startEdgeCirc = edgeCirculator;

        //Find the Rep vertex
        Vs_Halfedge_circulator repVertex = edgeCirculator;
        while ((++edgeCirculator) != startEdgeCirc)
        {
            //If current vertex has lower x, set as rep
            if (edgeCirculator->source()->point().x() < repVertex->source()->point().x())
            {
                repVertex = edgeCirculator;
            }
            //If equal x, take lower y
            else if (edgeCirculator->source()->point().x() == repVertex->source()->point().x())
            {
                if (edgeCirculator->source()->point().y() < repVertex->source()->point().y())
                {
                    repVertex = edgeCirculator;
                }
            }
        }
        repVertex->source()->data().isRepresentative.push_back(true);

        //Start at edge with point equal to starting 3d point at source
        edgeCirculator = startEdgeCirc; //Reset circulator

        //Find starting point
        while (edgeCirculator->source()->point() != startingPoint)
        {
            edgeCirculator = edgeCirculator->next();
        }

        startEdgeCirc = edgeCirculator; //Set start

        //Circle along the edges and set both vertex and edge data
        int edgeIndex = 0;
        do
        {
            edgeCirculator->source()->data().faceIndices.push_back(currentTriangle);

            if (edgeCirculator->source()->data().isRepresentative.size() == 0)
            {
                edgeCirculator->source()->data().isRepresentative.push_back(false);
            }

            edgeCirculator->data().faceIndices.push_back(currentTriangle);
            edgeIndex++;

        } while (++edgeCirculator != startEdgeCirc);
    }

    bool ViewshedProjection::IsEmpty() const
    {
        return vsArrangement.is_empty();
    }

    //Determines the overlap relation between the faces and returns a list of faces sorted from farthest to closest
    ViewshedRelationGraph ViewshedProjection::OverlapRelation(std::unordered_map<int, Vs_Vertex_handle>& repVertexHandles)
    {
        for (Vs_Vertex_handle vertexHandle = vsArrangement.vertices_begin(); vertexHandle != vsArrangement.vertices_end(); vertexHandle++)
        {
            if (vertexHandle->data().faceIndices.size() > 0 && vertexHandle->data().intersectionMap.size() > 0)
            {
                std::cout << "Intersection point is also face vertex?" << std::endl;
            }
        }

        //Building relation graph R:
        //Each face contains a list of overlapping faces
        //Iterate over vertices
        //When at representative for P'_i call overlap (determine which polygons p_i overlaps:
            //Overlap:
            //Copy list of current faces (skipping invisible faces) and check if they are adjacent to every halfedge
            //If embedded of fully overlapping a face should be adjacent to all halfedges
            //Walk around boundary for P'_i
            //At intersection: 
            //  Add relation to intersecting poly P'_j 
            //  Set P'_j Int to true
            //  set to obscures true if P_j is infront of intersection point (3d) (Intersect triangle plane to get 3d point? check distance between 3d two points)
            //  set to obscured true if P_j is behind intersection point (3d)
            //When back at rep point check if obscured and obscure are not both set to true.. warning if so.. (shouldnt happend because no intersections between 3d triangles)
            //Finally check if any face is fully obscuring or P'_i is fully embedded, if fully obscuring record P'_i as invisible else set obscures to true
        //Insert the node and its obscured/obscures edges into R, set Int bool in R edge as well(?)
        //Remove invisible polygons from the relation graph

        //There should only be one unbounded face because we add closed faces with finite edges to the arrangement
        assert(vsArrangement.number_of_unbounded_faces() == 1);

        ViewshedRelationGraph relationGraph;
        //TODO: Convert to unordered_set for O(1) search?
        std::set<int> invisibleFaces;

        //Loop through vertices and find representative vertex
        //when encountering a representative vertex find the overlap relation for the face and add them to the relation graph
        for (Vs_Vertex_handle vertexHandle = vsArrangement.vertices_begin(); vertexHandle != vsArrangement.vertices_end(); vertexHandle++)
        {
            auto vd = vertexHandle->data();
            for (size_t i = 0; i < vd.isRepresentative.size(); i++)
            {
                if (vd.isRepresentative.at(i))
                {
                    //Store representative vertex handles
                    repVertexHandles[vd.faceIndices.at(i)] = vertexHandle;

                    //Find the overlap relations of this face and add them to the graph.
                    //Result is empty if the face is completely obscured or isolated.
                    std::vector<VSRelationEdge> faceRelations = Overlap(vd.faceIndices.at(i), vertexHandle, invisibleFaces);

                    relationGraph.AddNode(vd.faceIndices.at(i));
                    relationGraph.AddEdges(faceRelations);
                }
            }
        }


        //relationGraph.PrintGraph();

        for (const int node : invisibleFaces)
        {
            relationGraph.RemoveNode(node);
            repVertexHandles.erase(node);
        }
        //relationGraph.PrintGraph();

        relationGraph.Trim();


        return relationGraph;
    }

    std::vector<VSRelationEdge> ViewshedProjection::Overlap(const int faceIndex, const Vs_Vertex_const_handle repVertex, std::set<int>& invisibleFaces) const
    {
        Vs_Arrangement_2::Halfedge_around_vertex_const_circulator edgeCirculator, firstHalfedge;
        edgeCirculator = repVertex->incident_halfedges();
        firstHalfedge = repVertex->incident_halfedges();

        //Circulate through the halfedges around repVertex and find the start halfedge
        //The halfedges in the circulator all point towards the vertex:
        //target() == repVertex and twin()->source == repVertex
        Vs_Halfedge_const_handle currentTriHalfedge;
        do
        {
            //Check if start
            if (HalfedgeOnBoundary(faceIndex, edgeCirculator->twin()))
            {
                currentTriHalfedge = edgeCirculator->twin();
                break;
            }
        } while (++edgeCirculator != firstHalfedge);

        assert(currentTriHalfedge != Vs_Halfedge_const_handle());

        //Dictionary that stores the obscure(s/d) relation between the faces in relation to the current face
        std::map<int, VSRelationEdge> relationMap;
        std::vector<int> fullOverlapSet;

        //Add overlapping faces at current half edge, fully obscured/obscuring should be in all faces of the triangle
        //and have no intersections at the end.
        for (const int& overlapFaceIndex : currentTriHalfedge->face()->data().faceIndices)
        {
            //skip self and invisible faces
            if (overlapFaceIndex != faceIndex && invisibleFaces.find(overlapFaceIndex) == invisibleFaces.end())
            {
                fullOverlapSet.push_back(overlapFaceIndex);
                //relationMap.insert(std::pair<int, VSRelationEdge>(overlapFaceIndex, VSRelationEdge(faceIndex, overlapFaceIndex)));
            }
        }

        assert(std::is_sorted(fullOverlapSet.begin(), fullOverlapSet.end()));

        Vs_Halfedge_const_handle startHalfEdge = currentTriHalfedge;
        currentTriHalfedge = NextHalfedge(currentTriHalfedge->target(), faceIndex);

        //std::cout << "Overlap for: " << faceIndex << std::endl;


        //Store 3 triangular points along the current face so we can use their centroid to test if faces are fully obscured later
        std::vector<Point_2> obscureTri{ startHalfEdge->source()->point() };

        //Walk around the face, at the intersection vertices check the order of triangles from the viewpoint
        std::vector<Vs_Halfedge_const_handle> edges;
        while (currentTriHalfedge != startHalfEdge)
        {
            //Remove the face indices from the full overlap list that are missing from this face
            std::vector<int> overlapIntersection(fullOverlapSet.size());
            auto oit = std::set_intersection(fullOverlapSet.begin(), fullOverlapSet.end(), currentTriHalfedge->face()->data().faceIndices.begin(), currentTriHalfedge->face()->data().faceIndices.end(), overlapIntersection.begin());
            overlapIntersection.resize(oit - overlapIntersection.begin());
            fullOverlapSet = overlapIntersection;

            //Add a point to the triangle that we use for full obscure testing
            if (obscureTri.size() < 3)
            {
                if (obscureTri.size() == 2)
                {
                    //Make sure the 3rd point is not collinear with the other 2
                    if (!CGAL::collinear(obscureTri.at(0), obscureTri.at(1), currentTriHalfedge->source()->point()))
                    {
                        obscureTri.push_back(currentTriHalfedge->source()->point());
                    }
                }
                else
                {
                    obscureTri.push_back(currentTriHalfedge->source()->point());
                }
            }

            //Is the current vertex an intersection point? Do obscure(s/d) test
            if (currentTriHalfedge->source()->data().intersectionMap.size() > 0)
            {
                //For each intersecting face evaluate order and add the relationship data to the map
                for (const int& intersectingFace : currentTriHalfedge->source()->data().intersectionMap.at(faceIndex))
                {
                    //skip self and invisible faces
                    if (intersectingFace != faceIndex && invisibleFaces.find(intersectingFace) == invisibleFaces.end())
                    {
                        //Add relation to the map, set intersect, and determine order
                        VSRelationEdge relationEdge(faceIndex, intersectingFace);
                        relationEdge.intersects = true;

                        Kernel::FT distanceDiff;
                        bool hit;
                        bool triangleInfront = TriangleIsInfront(currentTriHalfedge->source()->point(), faceIndex, intersectingFace, distanceDiff, hit);
                        assert(hit);

                        //Handle edge case where an adjacent face got added to the intersection list (see Arr_extended_overlay_traits.h for more details)
                        if (distanceDiff == 0)
                        {
                            //std::cout << "adjacent.. skipped" << std::endl;
                            continue;
                        }

                        if (triangleInfront)
                        {
                            relationEdge.obscures = true;
                        }
                        else
                        {
                            relationEdge.obscured = true;
                        }

                        relationMap.insert(std::pair<int, VSRelationEdge>(intersectingFace, relationEdge));
                    }
                }
            }
            edges.push_back(currentTriHalfedge);
            currentTriHalfedge = NextHalfedge(currentTriHalfedge->target(), faceIndex);
        }

#ifndef NDEBUG
        for (auto& r : relationMap)
        {
            assert(!(r.second.obscured && r.second.obscures));
        }
#endif // !NDEBUG

        for (const int& foFace : fullOverlapSet)
        {
            relationMap[foFace] = VSRelationEdge(faceIndex, foFace);
        }

        //Loop through relation map, if intersect == false test obscures/d
        //If obscured == true this triangle is completely obscured and we can exit.
        //TODO: Can't we just loop through the fullOverlapset instead?
        Point_2 pointInFace = CGAL::centroid(obscureTri.at(0), obscureTri.at(1), obscureTri.at(2));
        std::vector<int> toRemove;
        for (auto& r : relationMap)
        {
            if (!r.second.intersects)
            {
                assert(!(r.second.obscured || r.second.obscures));

                Kernel::FT distanceDiff;
                bool hit;
                bool obscures = TriangleIsInfront(pointInFace, faceIndex, r.first, distanceDiff, hit);

                if (!hit)
                {
                    //If we didn't hit one of the faces the current face is not completely contained in the other face
                    //We can skip it for now and resolve it later in the other face's overlap() call
                    toRemove.push_back(r.first);
                    continue;
                }

                if (obscures)
                {
                    //Current face covers (a part of) the other face
                    assert(distanceDiff != 0);
                    r.second.obscures = true;

                    //Ultra-edge case: If edges overlap we need to also count this as an intersection to prevent the mislabeling of the background later..
                    //Loop through the edges and check if any edges overlap, if so mark as intersecting face
                    //TODO: We can probably prevent this from ever occuring with backface culling
                    for (Vs_Halfedge_const_handle edge : edges)
                    {
                        if (std::find(edge->data().faceIndices.begin(), edge->data().faceIndices.end(), r.second.jDestination) != edge->data().faceIndices.end())
                        {
                            r.second.intersects = true;
                        }
                    }
                }
                else
                {
                    //Current face is completely obscured by the other face, record and return empty map
                    assert(distanceDiff != 0);
                    //#ifndef NDEBUG
                    //                    std::cout << faceIndex << " is hidden by " << r.second.jDestination << std::endl;
                    //#endif //!NDEBUG
                    invisibleFaces.insert(faceIndex);
                    return std::vector<VSRelationEdge>();
                }
            }
        }

        //Remove unresolved overlaps (these will be resolved in their respective overlap() call)
        for (int r : toRemove)
        {
            //std::cout << "Removing: " << r << std::endl;
            relationMap.erase(r);
        }

        std::vector<VSRelationEdge> edgeRelations;
        for (const auto& r : relationMap)
        {
            edgeRelations.push_back(r.second);
        }


        return edgeRelations;
    }

    //Checks if the given halfedge is on the boundary of the given face
    bool ViewshedProjection::HalfedgeOnBoundary(const size_t faceIndex, const Vs_Arrangement_2::Halfedge_const_handle halfedge) const
    {
        auto found = std::find(halfedge->data().faceIndices.begin(), halfedge->data().faceIndices.end(), faceIndex);

        if (found != halfedge->data().faceIndices.end())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    //Finds the halfedge on the boundary of the given face and outgoing from the given vertex
    Vs_Halfedge_const_handle ViewshedProjection::NextHalfedge(const Vs_Vertex_const_handle vertex, const int faceIndex) const
    {
        Vs_Arrangement_2::Halfedge_around_vertex_const_circulator edgeCirculator, firstHE;
        edgeCirculator = vertex->incident_halfedges();
        firstHE = vertex->incident_halfedges();

        Vs_Halfedge_const_handle currentTriHalfedge;

        //Twins are outgoing, so loop around until twin contains the face index
        do
        {
            size_t foundIndex;
            if (HalfedgeOnBoundary(faceIndex, edgeCirculator->twin()))
            {
                currentTriHalfedge = edgeCirculator->twin();
                break;
            }
        } while (++edgeCirculator != firstHE);

        assert(currentTriHalfedge != Vs_Halfedge_const_handle());

        return currentTriHalfedge;
    }

    //Determine if the ray going through the viewplane point intersects the first or second face first
    bool ViewshedProjection::TriangleIsInfront(const Point_2& viewplanePoint, const int faceIndex, const int faceIndexOther, Kernel::FT& distanceDiff, bool& hit) const
    {
        //std::cout << "2d plane point: " << viewplanePoint << std::endl;

        Point_3 viewplanePoint3d = Project2dTo3d(viewplanePoint, direction);
        //std::cout << "3d plane point: " << viewplanePoint3d << std::endl;
        Del_finite_face_iterator faceIt = std::next(DT.finite_faces_begin(), faceIndex);
        Del_finite_face_iterator otherFaceIt = std::next(DT.finite_faces_begin(), faceIndexOther);

        Triangle_3 faceTriangle = DT.triangle(faceIt);
        Triangle_3 otherFaceTriangle = DT.triangle(otherFaceIt);

        //std::cout << "3d faceTriangle: " << faceTriangle.vertex(0) << faceTriangle.vertex(1) << faceTriangle.vertex(2) << std::endl;
        //std::cout << "3d otherFaceTriangle: " << otherFaceTriangle.vertex(0) << otherFaceTriangle.vertex(1) << otherFaceTriangle.vertex(2) << std::endl;


        Ray_3 viewToPoint(viewpoint, viewplanePoint3d);
        //std::cout << "viewplanePoint3d: " << viewplanePoint3d << std::endl;
        //std::cout << "viewpoint: " << viewpoint << std::endl;


        //Check if the ray going through the viewplane point intersects the triangles
        auto triIntersection = CGAL::intersection(viewToPoint, faceTriangle);
        auto otherTriIntersection = CGAL::intersection(viewToPoint, otherFaceTriangle);

        if (triIntersection && otherTriIntersection)
        {
            hit = true;
            Point_3* triIntersectionPoint = boost::get<Point_3>(&*triIntersection);
            Point_3* otherTriIntersectionPoint = boost::get<Point_3>(&*otherTriIntersection);

            Vector_3 viewpointToTri = *triIntersectionPoint - viewpoint;
            Vector_3 viewpointToOtherTri = *otherTriIntersectionPoint - viewpoint;

            //std::cout << "triIntersectionPoint: " << *triIntersectionPoint << std::endl;
            //std::cout << "otherTriIntersectionPoint: " << *otherTriIntersectionPoint << std::endl;

            //std::cout << "viewpointToTri: " << viewpointToTri << std::endl;
            //std::cout << "viewpointToOtherTri: " << viewpointToOtherTri << std::endl;
            //std::cout << "lengths:\n" << viewpointToTri.squared_length() << " & " << viewpointToOtherTri.squared_length() << std::endl;

            distanceDiff = viewpointToTri.squared_length() - viewpointToOtherTri.squared_length();

            return(viewpointToTri.squared_length() < viewpointToOtherTri.squared_length());
        }
        else
        {
            hit = false;
            return false;
        }
    }

    //For each poly in sortedPolys:
    //  1. Travel through embedRelations to find background poly
    //        If none, -inf (-1 face?)
    //  2. Walk around poly and add to new final arrangement
    //        Init currentRight with BG face from 1.
    //        Set edges vis to TRUE (Needed?)
    //        Set left to P_j
    //        Set right to currentRight from 1
    //        When encountering intersection with visible edge:
    //            If bg of intersecting poly is not currentRight, back track to start and set to bg of intersecting
    //            Set currentRight to intersecting poly
    //  3. Walk around poly again when encountering intersection with vis is true, make edges inside P_i invisible with DFS
    ViewshedArrangement ViewshedProjection::DetermineVisibility()
    {
        std::unordered_map<int, Vs_Vertex_handle> repVertexHandles;
        //Get the relation graph detailing overlap relations between the polygons
        ViewshedRelationGraph relationGraph = OverlapRelation(repVertexHandles);

        //relationGraph.PrintGraph();

        //Copy the relation graph and remove all edges that represent poly intersections so we are only left with fully embed edges
        ViewshedRelationGraph embedRelationGraph = relationGraph;
        embedRelationGraph.RemoveIntersectionEdges();

        std::vector<int> sortedPolys = relationGraph.TopologicalSort();

        //std::cout << "Sorted polys from farthest to closest:" << std::endl;
        //for (const int n : sortedPolys)
        //{
        //    std::cout << n << std::endl;
        //}
        //std::cout << std::endl;

        std::unordered_map<int, Vs_Halfedge_handle> repEdgeHandles;
        std::vector<int> fartherPolygons; //Contains the polygons that have been visited and by definition are further of the current
        for (const int polygonIndex : sortedPolys)
        {
            //Init current right with the closest polygon we are embedded in
            int currentRight = embedRelationGraph.GetClosestNeighbour(polygonIndex, fartherPolygons);
            fartherPolygons.push_back(polygonIndex);
            Vs_Vertex_handle currentVertex = repVertexHandles[polygonIndex];

            //Circulate through the halfedges around repVertex and find the start halfedge
            //The halfedges in the circulator all point towards the vertex:
            //target() == repVertex and twin()->source == repVertex
            Vs_Arrangement_2::Halfedge_around_vertex_circulator edgeCirculator = currentVertex->incident_halfedges();
            Vs_Arrangement_2::Halfedge_around_vertex_circulator firstHalfedge = edgeCirculator;
            Vs_Halfedge_handle currentHalfedge;

            do
            {
                //Check if start
                if (HalfedgeOnBoundary(polygonIndex, edgeCirculator->twin()))
                {
                    currentHalfedge = edgeCirculator->twin();
                    repEdgeHandles[polygonIndex] = currentHalfedge;
                    break;
                }
            } while (++edgeCirculator != firstHalfedge);

            assert(currentHalfedge != Vs_Halfedge_handle());

            std::vector<Vs_Halfedge_handle> visitedBoundaryEdges;

            //1st walk around the polygon, mark the boundary as visible and set the right background polygon
            bool firstIntersection = true;
            firstHalfedge = currentHalfedge;
            do
            {
                Vs_Halfedge_handle adjacentVisibleHalfedge;

                //For the first intersection we need to check if our current right background polygon is correct
                if (firstIntersection && VisibleFirstExternalHalfedge(currentHalfedge->target(), polygonIndex, adjacentVisibleHalfedge))
                {
                    //If the current right is not equal to the left poly of the first CCW incoming visible edge outside of the boundary, backtrack and correct rights
                    if (currentRight != adjacentVisibleHalfedge->data().left)
                    {
                        currentRight = adjacentVisibleHalfedge->data().left;

                        for (auto prevEdge : visitedBoundaryEdges)
                        {
                            prevEdge->data().right = currentRight;
                            prevEdge->twin()->data().left = currentRight;
                        }
                    }

                    firstIntersection = false;
                }

                //Set data, reverse left/right for twin
                currentHalfedge->data().visible = true;
                currentHalfedge->data().left = polygonIndex;
                currentHalfedge->data().right = currentRight;

                currentHalfedge->twin()->data().visible = true;
                currentHalfedge->twin()->data().left = currentRight;
                currentHalfedge->twin()->data().right = polygonIndex;

                //Get the CCW last outgoing visible edge on the outside of the boundary
                if (VisibleLastExternalHalfedge(currentHalfedge->target(), polygonIndex, adjacentVisibleHalfedge))
                {
                    //We crossed an intersection, update currentRight
                    currentRight = adjacentVisibleHalfedge->data().left;
                }

                visitedBoundaryEdges.push_back(currentHalfedge);

                //Continue boundary walk
                currentHalfedge = vsArrangement.non_const_handle(NextHalfedge(currentHalfedge->target(), polygonIndex));

            } while (currentHalfedge != firstHalfedge);

            //CGAL::draw(vsArrangement, true, false, "Visible edges");

            //2nd walk around the polygon, remove inner edges with a depth-first search (we'll reuse the visited boundary vector to remove the cost of finding the next boundary edge)
            for (Vs_Halfedge_handle boundaryEdge : visitedBoundaryEdges)
            {
                //Take any visible internal edges connected to this boundary vertex
                std::vector<Vs_Halfedge_handle> internalHalfedges = VisibleInternalHalfedges(boundaryEdge->target(), polygonIndex);

                //Push them on the stack
                std::stack<Vs_Halfedge_handle> internalEdgeStack;
                for (Vs_Halfedge_handle internalHalfedge : internalHalfedges)
                {
                    internalEdgeStack.push(internalHalfedge);
                }

                //While there are visible internal edges on the stack
                while (!internalEdgeStack.empty())
                {
                    //Set to invisible, remove left/right (also for twin)
                    internalEdgeStack.top()->data().visible = false;
                    internalEdgeStack.top()->data().left = -1;
                    internalEdgeStack.top()->data().right = -1;

                    internalEdgeStack.top()->twin()->data().visible = false;
                    internalEdgeStack.top()->twin()->data().left = -1;
                    internalEdgeStack.top()->twin()->data().right = -1;

                    //Find visible internal edges connected to the next vertex and push them on the stack
                    internalHalfedges = VisibleInternalHalfedges(internalEdgeStack.top()->target(), polygonIndex);
                    internalEdgeStack.pop(); //Remove current before adding new
                    for (Vs_Halfedge_handle internalHalfedge : internalHalfedges)
                    {
                        internalEdgeStack.push(internalHalfedge);
                    }
                }
            }

            //CGAL::draw(vsArrangement, true, false, "Visible edges");
        }

        //CGAL::DrawViewshedProjection(vsArrangement, false, true, "Viewshed projection with invis");

        //Report points and edges
        std::vector<Point_3> reported3dPoints;

        //CGAL::DrawViewshedProjection(vsArrangement, false, false, "All edges", true);

        //Remove invisible edges
        Vs_Arrangement_2::Edge_iterator edgeit;
        std::vector< Vs_Halfedge_handle> invisEdges;
        for (edgeit = vsArrangement.edges_begin(); edgeit != vsArrangement.edges_end(); ++edgeit)
        {
            Vs_Halfedge_handle hEdge = edgeit;

            if (!hEdge->data().visible)
            {
                invisEdges.push_back(hEdge);
            }
        }

        for (Vs_Halfedge_handle hEdge : invisEdges)
        {
            vsArrangement.remove_edge(hEdge);
        }
        //CGAL::DrawViewshedProjection(vsArrangement, false, false, "invis removed", true);

        //Cycle through every visible face and report its edges
        ViewshedArrangement viewshed;
        ViewshedArrangement prevViewshedOverlay;
        prevViewshedOverlay.unbounded_face()->data() = false;
        ViewshedOverlayTraits viewshedOverlayTraits;

        Vs_Arrangement_2::Face_const_iterator faceIt;
        for (faceIt = vsArrangement.faces_begin(); faceIt != vsArrangement.faces_end(); ++faceIt)
        {
            if (faceIt->is_unbounded())
            {
                continue;
            }

            Vs_Arrangement_2::Ccb_halfedge_const_circulator  firstEdge, currentEdge;
            firstEdge = currentEdge = faceIt->outer_ccb();


            std::vector<Segment_2> viewshedFaceEdges;
            do
            {
                //Report edge
                Point_2 source = currentEdge->source()->point();
                Point_2 target = currentEdge->target()->point();

                //Convert 2d viewplane points to 3d
                Point_3 source3d = Project2dTo3d(source, direction);
                Point_3 target3d = Project2dTo3d(target, direction);

                //Shoot ray through viewplane points and intersect the foreground
                Ray_3 viewToSource3d(viewpoint, source3d);
                Ray_3 viewToTarget3d(viewpoint, target3d);

                //Find foreground points and construct edge
                Point_3 foregroundSource;
                Point_3 foregroundTarget;

                //TODO: move this out of this loop to test and speedup
                if (currentEdge->data().left != -1)
                {

                    //std::cout << currentEdge->data().left << std::endl;
                    Triangle_3 foregroundTri = DT.triangle(faceHandles.at(currentEdge->data().left));
                    if (RayTriangleIntersectionPoint(viewToSource3d, foregroundTri, foregroundSource) && RayTriangleIntersectionPoint(viewToTarget3d, foregroundTri, foregroundTarget))
                    {
                        Segment_2 foregroundEdge(Point_2(foregroundSource.x(), foregroundSource.y()), Point_2(foregroundTarget.x(), foregroundTarget.y()));
                        viewshedFaceEdges.push_back(foregroundEdge);

                        reported3dPoints.push_back(foregroundSource);
                        reported3dPoints.push_back(foregroundTarget);

                    }
                    else
                    {
                        std::cout << "Triangle index: " << currentEdge->data().left << std::endl;
                        std::cout << "Foreground tri: " << foregroundTri << std::endl;
                        std::cout << "2d source: " << source << std::endl;
                        std::cout << "2d target: " << target << std::endl;

                        throw(std::runtime_error("Critical miss exception"));
                    }

                }
                else
                {
                    std::cout << "Empty" << std::endl;
                }
                ++currentEdge;
            } while (currentEdge != firstEdge);

            if (viewshedFaceEdges.size() == 0)
            {
                continue;
            }

            //Construct face and insert into the viewshed
            ViewshedArrangement currFaceViewshed;

            for (Segment_2 edge : viewshedFaceEdges)
            {
                CGAL::insert(currFaceViewshed, edge);
            }
            //CGAL::DrawViewshed(currFaceViewshed, false, "Viewshed face");

            if (currFaceViewshed.number_of_faces() != 2)
            {
                throw(std::runtime_error("Missing edge in face"));

            }

            //Mark the viewshed face 'visible' and the outer face 'invisible'
            for (ViewshedArrangement::Face_handle faceHandle : currFaceViewshed.face_handles())
            {
                faceHandle->data() = !faceHandle->is_unbounded();
            }

            //Overlay the face into the viewshed, keeping holes invisible
            CGAL::overlay(prevViewshedOverlay, currFaceViewshed, viewshed, viewshedOverlayTraits);
            prevViewshedOverlay = viewshed;

            //CGAL::DrawViewshed(viewshed, false, "Viewshed segment wip");

        }

        //CGAL::DrawViewshedProjection(vsArrangement, true, false, "Viewshed projection invis removed");
        //CGAL::DrawViewshed(viewshed, false, "Viewshed segment", false);

        return viewshed;
    }

    //Check if there is a visible incoming edge in between the incoming and outgoing edge, CCW from the incoming boundary edge
    bool ViewshedProjection::VisibleFirstExternalHalfedge(const Vs_Vertex_handle vertex, const int faceIndex, Vs_Halfedge_handle& intersectingHalfedge) const
    {
        Vs_Arrangement_2::Halfedge_around_vertex_circulator edgeCirculator, firstHalfedge;
        edgeCirculator = firstHalfedge = vertex->incident_halfedges();

        //Find start
        do
        {
            //Check if incoming boundary edge
            if (HalfedgeOnBoundary(faceIndex, edgeCirculator))
            {
                firstHalfedge = edgeCirculator; //Reset first half edge so we don't early out
                break;
            }

        } while (++edgeCirculator != firstHalfedge);

        //CCW circulate until we find the polygon's outgoing edge or a visible edge
        while (--edgeCirculator != firstHalfedge)
        {
            //Check if outgoing boundary edge (the halfedges in the circulater are incoming, so twin)
            if (HalfedgeOnBoundary(faceIndex, edgeCirculator->twin()))
            {
                return false;
            }

            if (edgeCirculator->data().visible)
            {
                intersectingHalfedge = edgeCirculator;
                return true;
            }
        }

        return false;
    }

    //Check if there is a visible outgoing edge in between the incoming and outgoing edge, CW from the outgoing boundary edge
    bool ViewshedProjection::VisibleLastExternalHalfedge(const Vs_Vertex_handle vertex, const int faceIndex, Vs_Halfedge_handle& intersectingHalfedge) const
    {
        Vs_Arrangement_2::Halfedge_around_vertex_circulator edgeCirculator, firstHalfedge;
        edgeCirculator = firstHalfedge = vertex->incident_halfedges();

        //Find start
        do
        {
            //Check if outgoing boundary edge (the halfedges in the circulater are incoming, so twin)
            if (HalfedgeOnBoundary(faceIndex, edgeCirculator->twin()))
            {
                firstHalfedge = edgeCirculator; //Reset first half edge so we don't early out
                break;
            }

        } while (++edgeCirculator != firstHalfedge);

        //CW circulate until we find the polygon's incoming edge or a visible edge
        while (++edgeCirculator != firstHalfedge)
        {
            //Check if incoming boundary edge
            if (HalfedgeOnBoundary(faceIndex, edgeCirculator))
            {
                return false;
            }

            if (edgeCirculator->twin()->data().visible)
            {
                intersectingHalfedge = edgeCirculator->twin();
                return true;
            }
        }

        return false;
    }

    //Reports all visible edges within the boundary of a given face
    std::vector<Vs_Halfedge_handle> ViewshedProjection::VisibleInternalHalfedges(const Vs_Vertex_handle vertex, const int faceIndex) const
    {
        Vs_Arrangement_2::Halfedge_around_vertex_circulator edgeCirculator, firstHalfedge;
        edgeCirculator = firstHalfedge = vertex->incident_halfedges();

        bool boundaryVert = false;

        //Find start
        do
        {
            //Check if this vertex has an outgoing boundary edge (the halfedges in the circulater are incoming, so twin)
            if (HalfedgeOnBoundary(faceIndex, edgeCirculator->twin()))
            {
                firstHalfedge = edgeCirculator; //Reset first half edge so we don't early out
                --edgeCirculator; //CCW rotate into the polygon so we don't report the boundary edge
                boundaryVert = true;
                break;
            }

        } while (++edgeCirculator != firstHalfedge);

        //CCW circulate around the vertex until we went full circle or,
        //in the case of a boundary vertex, we find the polygon's incoming edge, 
        //reporting visible edges along the way.
        std::vector<Vs_Halfedge_handle> intersectingHalfedges;
        do
        {
            //TODO: Optimize by checking for left face?
            //Check if incoming boundary edge
            if (boundaryVert && HalfedgeOnBoundary(faceIndex, edgeCirculator))
            {
                //Interior scan is done, report
                return intersectingHalfedges;
            }

            if (edgeCirculator->twin()->data().visible)
            {
                //Report visible interior edge
                intersectingHalfedges.push_back(edgeCirculator->twin());
            }

        } while (--edgeCirculator != firstHalfedge);

        return intersectingHalfedges;
    }

    bool ViewshedProjection::RayTriangleIntersectionPoint(const Ray_3& ray, const Triangle_3& triangle, Point_3& intersectionPoint)
    {
        auto intersection = CGAL::intersection(ray, triangle);

        if (intersection)
        {
            if (Point_3* intersectionPP = boost::get<Point_3>(&*intersection))
            {
                intersectionPoint = *intersectionPP;
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
}