#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/number_utils.h>

//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h> //Projects 3d points to the 2d XY plane
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/draw_triangulation_2.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Arr_linear_traits_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_extended_dcel.h>
#include <CGAL/Arrangement_with_history_2.h>
#include <CGAL/Arr_default_overlay_traits.h>
#include <CGAL/IO/Arr_iostream.h>

#include <vector>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <sstream>
#include <queue>

#include <thread>
#include <mutex>

#include "Arr_extended_overlay_traits.h"

//#include "boost/variant.hpp"

//Typedef for number (precision) representation
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
//typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

//Typedefs for common types
typedef unsigned int uint;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Segment_3 Segment_3;
typedef Kernel::Vector_2 Vector_2;
typedef Kernel::Vector_3 Vector_3;
typedef Kernel::Direction_2 Direction_2;
typedef Kernel::Direction_3 Direction_3;
typedef Kernel::Line_2 Line_2;
typedef Kernel::Line_3 Line_3;
typedef Kernel::Plane_3 Plane_3;
typedef Kernel::Ray_3 Ray_3;
typedef Kernel::Triangle_3 Triangle_3;

//Typedefs for triangulation implementation
typedef CGAL::Projection_traits_xy_3<Kernel>  Projection_traits_xy;
typedef CGAL::Triangulation_2<Projection_traits_xy> Triangulation;
typedef CGAL::Triangulation_vertex_base_2<Projection_traits_xy> Vb;
typedef CGAL::Triangulation_face_base_with_info_2<CGAL::Color, Projection_traits_xy> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Delaunay_triangulation_2<Projection_traits_xy, Tds> Delaunay;

//Typedefs for iterators and circulators
typedef Delaunay::Finite_edges_iterator Del_finite_edges_iterator;
typedef Delaunay::Vertex_handle Del_vertex_handle;
typedef Delaunay::Finite_vertex_handles Del_finite_vertex_handles;
typedef Delaunay::Finite_vertices_iterator Del_finite_vertices_iterator;
typedef Delaunay::Edge_circulator Del_edge_circulator;
typedef Delaunay::Vertex_circulator Del_vertex_circulator;
typedef Delaunay::Face_circulator Del_face_circulator;
typedef Delaunay::Face_handle Del_face_handle;
typedef Delaunay::Finite_faces_iterator Del_finite_face_iterator;

typedef Del_finite_vertex_handles::iterator Del_finite_vertex_handles_iterator;

//For rotation etc.
typedef CGAL::Aff_transformation_2<Kernel> transform2d;

//Viewshed stuff
namespace Viewshed
{
    struct VertexData
    {
        //TODO: rep and face are always used together, can consolidate these into one vector
        std::vector<bool> isRepresentative;
        std::vector<int> faceIndices;
        std::map<int, std::vector<int>> intersectionMap;
    };

    struct EdgeData
    {
        std::vector<int> faceIndices;

        //Visibility data
        bool visible = false;
        int right = -1;
        int left = -1;
    };

    struct FaceData
    {
        std::vector<int> faceIndices;
    };

    typedef CGAL::Arr_segment_traits_2<Kernel> Traits_2;
    typedef Traits_2::Point_2 Vs_Point_2;
    typedef Traits_2::X_monotone_curve_2 Vs_Segment_2;
    typedef CGAL::Arr_extended_dcel<Traits_2, VertexData, EdgeData, FaceData> Vs_Arrangement_DCEL;
    typedef CGAL::Arrangement_2<Traits_2, Vs_Arrangement_DCEL> Vs_Arrangement_2;
    typedef Vs_Arrangement_2::Halfedge_handle Vs_Halfedge_handle;
    typedef Vs_Arrangement_2::Vertex_handle Vs_Vertex_handle;
    typedef Vs_Arrangement_2::Vertex_const_handle Vs_Vertex_const_handle;
    typedef Vs_Arrangement_2::Ccb_halfedge_circulator Vs_Halfedge_circulator;
    typedef Vs_Arrangement_2::Halfedge_const_handle Vs_Halfedge_const_handle;
    typedef Arr_extended_overlay_traits<Vs_Arrangement_2, Vs_Arrangement_2, Vs_Arrangement_2> Vs_extended_overlay_traits;

    typedef CGAL::Convex_hull_traits_adapter_2<Kernel, CGAL::Pointer_property_map<Point_2>::type> Convex_hull_traits_2;

    typedef CGAL::Arr_face_extended_dcel<Traits_2, bool> ViewshedDCEL;
    typedef CGAL::Arrangement_2<Traits_2, ViewshedDCEL> ViewshedArrangement;
    typedef CGAL::Arr_face_overlay_traits<ViewshedArrangement, ViewshedArrangement, ViewshedArrangement, std::logical_or<bool>> ViewshedOverlayTraits;
    typedef CGAL::Arr_face_extended_text_formatter<ViewshedArrangement> ViewshedIOFormatter;

    enum class ViewDirection
    {
        UP,
        DOWN,
        LEFT,
        RIGHT,
        FRONT,
        BEHIND
    };
}


#include "ViewshedRelationGraph.h"
#include "ViewshedProjection.h"
#include "Viewshed.h"

#include "draw_arrangement_2.h"
#include "DrawViewshed.h"
#include "DrawViewshedProjection.h"
