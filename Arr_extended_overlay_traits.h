#pragma once

namespace Viewshed
{
    template <class ArrangementA, class ArrangementB, class ArrangementR>
    class Arr_extended_overlay_traits
    {
    public:

        typedef typename ArrangementA::Vertex_const_handle    Vertex_handle_A;
        typedef typename ArrangementA::Halfedge_const_handle  Halfedge_handle_A;
        typedef typename ArrangementA::Face_const_handle      Face_handle_A;

        typedef typename ArrangementB::Vertex_const_handle    Vertex_handle_B;
        typedef typename ArrangementB::Halfedge_const_handle  Halfedge_handle_B;
        typedef typename ArrangementB::Face_const_handle      Face_handle_B;

        typedef typename ArrangementR::Vertex_handle          Vertex_handle_R;
        typedef typename ArrangementR::Halfedge_handle        Halfedge_handle_R;
        typedef typename ArrangementR::Face_handle            Face_handle_R;

        /*! Destructor. */
        virtual ~Arr_extended_overlay_traits()
        {}

        /*!
         * Create a vertex v that corresponds to the coinciding vertices v1 and v2.
         */
        virtual void create_vertex(Vertex_handle_A v1,
            Vertex_handle_B v2,
            Vertex_handle_R v) const
        {
            //std::cout << "new coinciding vertex" << std::endl;

            //Combine the face indices and representative boolean vectors while making sure the pairs keep the same order.
            //TODO Note: Convert to an actual pair?
            v->data().faceIndices.insert(v->data().faceIndices.end(), v1->data().faceIndices.begin(), v1->data().faceIndices.end());
            v->data().faceIndices.insert(v->data().faceIndices.end(), v2->data().faceIndices.begin(), v2->data().faceIndices.end());

            v->data().isRepresentative.insert(v->data().isRepresentative.end(), v1->data().isRepresentative.begin(), v1->data().isRepresentative.end());
            v->data().isRepresentative.insert(v->data().isRepresentative.end(), v2->data().isRepresentative.begin(), v2->data().isRepresentative.end());

            //Combine intersection maps into result vertex intersection map
            v->data().intersectionMap = v1->data().intersectionMap;
            for (const auto& kv : v2->data().intersectionMap)
            {
                for (const int faceIndex : kv.second)
                {
                    v->data().intersectionMap[kv.first].push_back(faceIndex);
                }
            }
        }

        /*!
         * Create a vertex v that matches v1, contained in the face f2.
         */
        virtual void create_vertex(Vertex_handle_A v1,
            Face_handle_B f2,
            Vertex_handle_R v) const
        {
            //std::cout << "new vertex in face 2" << std::endl;

            v->set_data(v1->data());
        }

        /*!
         * Create a vertex v that matches v1, which lies on the edge e2.
         */
        virtual void create_vertex(Vertex_handle_A v1,
            Halfedge_handle_B e2,
            Vertex_handle_R v) const
        {
            //std::cout << "new vertex on edge 2 inter map size" << v1->data().intersectionMap.size() << std::endl;
            //std::cout << "Edge faces: " << std::endl;
            //for (auto q : e2->data().faceIndices)
                //std::cout << q << std::endl;
            //for (auto q : e2->twin()->data().faceIndices)
                //std::cout << q << std::endl;

            v->set_data(v1->data());

            if (v->data().intersectionMap.size() > 0)
            {
                //This is an edge case where the original vertex that lies on the newly added edge is an intersection vertex.
                //Because it doesn't get handled as a new intersection but as a vertex in edge we cannot determine if one of the faces is adjacent to the face adjacent to the new edge.
                //So we just pair it with all known faces at this intersection point and we have to watch out that the triangles do not have a distance of 0 at the intersection point during the overlap procedure.
                //std::cout << "Adding intersection faces" << std::endl;
                std::vector<int> e2Faces;
                e2Faces.insert(e2Faces.end(), e2->data().faceIndices.begin(), e2->data().faceIndices.end());
                e2Faces.insert(e2Faces.end(), e2->twin()->data().faceIndices.begin(), e2->twin()->data().faceIndices.end());

                for (int faceIndex : e2Faces)
                {
                    for (auto& kv : v->data().intersectionMap)
                    {
                        v->data().intersectionMap[kv.first].push_back(faceIndex);
                        v->data().intersectionMap[faceIndex].push_back(kv.first);
                    }
                }
            }
        }

        /*!
         * Create a vertex v that matches v2, which lies on the edge e1.
         */
        virtual void create_vertex(Halfedge_handle_A e1,
            Vertex_handle_B v2,
            Vertex_handle_R v) const
        {
            //std::cout << "new vertex on edge 1 inter map size" << v2->data().intersectionMap.size() << std::endl;
            //std::cout << "Edge faces: " << std::endl;
            //for (auto q : e1->data().faceIndices)
                //std::cout << q << std::endl;
            //for (auto q : e1->twin()->data().faceIndices)
                //std::cout << q << std::endl;

            v->set_data(v2->data());

            if (v->data().intersectionMap.size() > 0)
            {
                //This is an edge case where the original vertex that lies on the newly added edge is an intersection vertex.
                //Because it doesn't get handled as a new intersection but as a vertex in edge we cannot determine if one of the faces is adjacent to the face adjacent to the new edge.
                //So we just pair it with all known faces at this intersection point and we have to watch out that the triangles do not have a distance of 0 at the intersection point during the overlap procedure.
                //std::cout << "Adding intersection faces" << std::endl;

                std::vector<int> e1Faces;
                e1Faces.insert(e1Faces.end(), e1->data().faceIndices.begin(), e1->data().faceIndices.end());
                e1Faces.insert(e1Faces.end(), e1->twin()->data().faceIndices.begin(), e1->twin()->data().faceIndices.end());

                for (int faceIndex : e1Faces)
                {
                    for (auto& kv : v->data().intersectionMap)
                    {
                        v->data().intersectionMap[kv.first].push_back(faceIndex);
                        v->data().intersectionMap[faceIndex].push_back(kv.first);
                    }
                }
            }
        }

        /*!
         * Create a vertex v that matches v2, contained in the face f1.
         */
        virtual void create_vertex(Face_handle_A f1,
            Vertex_handle_B v2,
            Vertex_handle_R v) const
        {
            //std::cout << "new vertex in face 1" << std::endl;

            v->set_data(v2->data());
        }

        /*!
         * Create a vertex v that matches the intersection on the edges e1 and e2.
         */
        virtual void create_vertex(Halfedge_handle_A e1,
            Halfedge_handle_B e2,
            Vertex_handle_R v) const
        {
            //std::cout << "new intersection vertex" << std::endl;

            //To reduce the amount of adjacent (but not intersecting) faces that will be handled as intersections later we add the intersection pairs in a dictionary.
            std::vector<int> e1Faces;
            e1Faces.insert(e1Faces.end(), e1->data().faceIndices.begin(), e1->data().faceIndices.end());
            e1Faces.insert(e1Faces.end(), e1->twin()->data().faceIndices.begin(), e1->twin()->data().faceIndices.end());

            std::vector<int> e2Faces;
            e2Faces.insert(e2Faces.end(), e2->data().faceIndices.begin(), e2->data().faceIndices.end());
            e2Faces.insert(e2Faces.end(), e2->twin()->data().faceIndices.begin(), e2->twin()->data().faceIndices.end());


            for (int faceIndexE1 : e1Faces)
            {
                for (int faceIndexE2 : e2Faces)
                {
                    v->data().intersectionMap[faceIndexE1].push_back(faceIndexE2);
                    v->data().intersectionMap[faceIndexE2].push_back(faceIndexE1);
                }
            }
        }

        /*!
         * Create an edge e that matches the overlap between e1 and e2.
         */
        virtual void create_edge(Halfedge_handle_A  e1,
            Halfedge_handle_B  e2,
            Halfedge_handle_R  e) const
        {
            //std::cout << "new overlap edge" << std::endl;

            e->data().faceIndices.insert(e->data().faceIndices.end(), e1->data().faceIndices.begin(), e1->data().faceIndices.end());
            e->data().faceIndices.insert(e->data().faceIndices.end(), e2->data().faceIndices.begin(), e2->data().faceIndices.end());

            e->twin()->data().faceIndices.insert(e->twin()->data().faceIndices.end(), e1->twin()->data().faceIndices.begin(), e1->twin()->data().faceIndices.end());
            e->twin()->data().faceIndices.insert(e->twin()->data().faceIndices.end(), e2->twin()->data().faceIndices.begin(), e2->twin()->data().faceIndices.end());
        }

        /*!
         * Create an edge e that matches the edge e1, contained in the face f2.
         */
        virtual void create_edge(Halfedge_handle_A  e1,
            Face_handle_B  f2,
            Halfedge_handle_R  e) const
        {
            //std::cout << "new edge in face 2" << std::endl;

            //Copy data from the old edge into the new edge
            e->set_data(e1->data());
            e->twin()->set_data(e1->twin()->data());
        }

        /*!
         * Create an edge e that matches the edge e2, contained in the face f1.
         */
        virtual void create_edge(Face_handle_A  f1,
            Halfedge_handle_B  e2,
            Halfedge_handle_R  e) const
        {
            //std::cout << "new edge in face 1" << std::endl;
            //Copy data from the old edge into the new edge
            e->set_data(e2->data());
            e->twin()->set_data(e2->twin()->data());

        }

        /*!
         * Create a face f that matches the overlapping region between f1 and f2.
         */
        virtual void create_face(Face_handle_A  f1,
            Face_handle_B  f2,
            Face_handle_R  f) const
        {
            //std::cout << "new face" << std::endl;

            //Copy face data from both faces into the new face
            f->data().faceIndices.insert(f->data().faceIndices.end(), f1->data().faceIndices.begin(), f1->data().faceIndices.end());
            f->data().faceIndices.insert(f->data().faceIndices.end(), f2->data().faceIndices.begin(), f2->data().faceIndices.end());

            std::sort(f->data().faceIndices.begin(), f->data().faceIndices.end());
        }

    };
}