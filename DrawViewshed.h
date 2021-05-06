#ifndef CGAL_DRAW_VIEWSHED_H
#define CGAL_DRAW_VIEWSHED_H

#include <CGAL/Qt/Basic_viewer_qt.h>

#ifdef CGAL_USE_BASIC_VIEWER

#include <CGAL/Arrangement_2.h>
#include <CGAL/Random.h>

namespace CGAL
{
    class SimpleViewshedArrangementViewerQt : public Basic_viewer_qt
    {
        typedef Basic_viewer_qt Base;

    public:
        /// Construct the viewer.
        /// @param a_a the arrangement to view
        /// @param title the title of the window
        /// @param anofaces if true, do not draw faces (faces are not computed; this can be
        ///        usefull for very big object where this time could be long)
        SimpleViewshedArrangementViewerQt(QWidget* parent,
            const Viewshed::ViewshedArrangement& a_arr,
            bool hideVerts = false,
            const char* title = "Basic Arrangement Viewer",
            bool nofaces = false) :
            // First draw: vertices; edges, faces; multi-color; inverse normal
            Base(parent, title, true, true, true, false, false),
            arr(a_arr),
            m_nofaces(nofaces)
        {
            compute_elements(hideVerts);
        }

    protected:

        void compute_face(Viewshed::ViewshedArrangement::Face_const_handle fh)
        {
            if (fh->is_unbounded())
            {
                return;
            }

            //CGAL::Random random((unsigned long)(&*fh));
            CGAL::Color c = CGAL::Color(30, 30, 220);

            face_begin(c);

            auto edgeCirculator = fh->outer_ccb();
            auto startEdgeCirc = fh->outer_ccb();

            //Circle along the edges and add the vertices to the draw face
            do
            {
                add_point_in_face(edgeCirculator->source()->point());
            } while (++edgeCirculator != startEdgeCirc);


            face_end();
        }
        void compute_elements(bool hideVerts)
        {
            clear();

            // Draw the arrangement faces.
            if (!m_nofaces)
            {
                Viewshed::ViewshedArrangement::Face_const_iterator fit;
                for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit)
                {
                    //If visible
                    if (fit->data())
                    {
                        compute_face(fit);
                    }
                }
            }

            // Draw the arrangement edges.
            Viewshed::ViewshedArrangement::Edge_const_iterator eit;
            for (eit = arr.edges_begin(); eit != arr.edges_end(); ++eit)
            {
                add_segment(eit->curve().source(), eit->curve().target(), CGAL::Color(0, 0, 0));
                //std::cout << "[" << eit->curve() << "]" << std::endl;
            }

            if (!hideVerts)
            {
                // Draw the arrangement vertices.
                typename Viewshed::ViewshedArrangement::Vertex_const_iterator vit;
                for (vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit)
                {
                    add_point(vit->point());
                }
            }
        }

        virtual void keyPressEvent(QKeyEvent* e)
        {
            Base::keyPressEvent(e);
        }

    protected:
        const Viewshed::ViewshedArrangement& arr;
        bool m_nofaces;
    };

    inline void DrawViewshed(const Viewshed::ViewshedArrangement arr,
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
            SimpleViewshedArrangementViewerQt mainwindow(app.activeWindow(), arr, hideVerts, title, nofill);
            mainwindow.show();
            app.exec();
        }
    }

} // End namespace CGAL


#endif // CGAL_DRAW_LCC_H
#endif