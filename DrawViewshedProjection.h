#ifndef CGAL_DRAW_VIEWSHED_PROJECTION_H
#define CGAL_DRAW_VIEWSHED_PROJECTION_H

#include <CGAL/Qt/Basic_viewer_qt.h>

#ifdef CGAL_USE_BASIC_VIEWER

#include <CGAL/Arrangement_2.h>
#include <CGAL/Random.h>

namespace CGAL
{

    class SimpleProjectionViewerQt : public Basic_viewer_qt
    {
        typedef Basic_viewer_qt Base;

    public:
        /// Construct the viewer.
        /// @param a_a the arrangement to view
        /// @param title the title of the window
        /// @param anofaces if true, do not draw faces (faces are not computed; this can be
        ///        usefull for very big object where this time could be long)
        SimpleProjectionViewerQt(QWidget* parent,
            const Viewshed::Vs_Arrangement_2& a_arr,
            bool onlyVisible = false,
            bool hideVerts = false,
            const char* title = "Basic Arrangement Viewer",
            bool anofaces = false) :
            // First draw: vertices; edges, faces; multi-color; inverse normal
            Base(parent, title, true, true, true, true, true),
            arr(a_arr),
            m_nofaces(anofaces)
        {
            compute_elements(onlyVisible, hideVerts);
        }

    protected:

        void compute_face(Viewshed::Vs_Arrangement_2::Halfedge_const_handle fh)
        {
            if (fh->is_fictitious())
            {
                return;
            }

            //CGAL::Random random((unsigned long)(&*fh));
            CGAL::Color c = CGAL::Color(30, 30, 220);

            face_begin(c);

            auto edgeCirculator = fh->ccb();
            auto startEdgeCirc = fh->ccb();

            //Circle along the edges and add the vertices to the draw face
            do
            {
                add_point_in_face(edgeCirculator->source()->point());
                //std::cout << edgeCirculator->source()->point() << std::endl;
            } while (++edgeCirculator != startEdgeCirc);


            face_end();
        }
        void compute_elements(bool onlyVisible, bool hideVerts)
        {
            clear();

            if (!hideVerts)
            {
                // Draw the arrangement vertices.
                Viewshed::Vs_Arrangement_2::Vertex_const_iterator vit;
                for (vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit)
                {
                    add_point(vit->point());
                }
            }

            // Draw the arrangement edges.
            Viewshed::Vs_Arrangement_2::Edge_const_iterator eit;
            if (!onlyVisible)
            {
                //Draw all edges
                for (eit = arr.edges_begin(); eit != arr.edges_end(); ++eit)
                {
                    add_segment(eit->curve().source(), eit->curve().target());
                    //std::cout << "[" << eit->curve() << "]" << std::endl;
                }
            }
            else
            {
                //Only draw visible edges
                for (eit = arr.edges_begin(); eit != arr.edges_end(); ++eit)
                {
                    if (eit->data().visible)
                    {
                        add_segment(eit->curve().source(), eit->curve().target());
                        //std::cout << "[" << eit->curve() << "]" << std::endl;
                    }
                }
            }
            // Draw the arrangement faces.
            //typename Arr::Face_const_iterator fit;
            //for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit)
            //{
            //    compute_face(fit);
            //}
        }

        virtual void keyPressEvent(QKeyEvent* e)
        {
            Base::keyPressEvent(e);
        }

    protected:
        const Viewshed::Vs_Arrangement_2& arr;
        bool m_nofaces;
    };


    inline void DrawViewshedProjection(const Viewshed::Vs_Arrangement_2 arr,
        bool onlyVisible = false,
        bool hideVerts = false,
        const char* title = "Basic Arrangement Viewer",
        bool nofill = false)
    {
#if defined(CGAL_TEST_SUITE)
        bool cgal_test_suite = true;
#else
        bool cgal_test_suite = qEnvironmentVariableIsSet("CGAL_TEST_SUITE");
#endif

        if (!cgal_test_suite)
        {
            int argc = 1;
            const char* argv[2] = { "arr_viewer","\0" };
            QApplication app(argc, const_cast<char**>(argv));
            SimpleProjectionViewerQt mainwindow(app.activeWindow(), arr, onlyVisible, hideVerts, title);
            mainwindow.show();
            app.exec();
        }
    }
}
#endif
#endif