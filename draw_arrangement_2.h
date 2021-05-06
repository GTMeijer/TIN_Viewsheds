#ifndef CGAL_DRAW_ARRANGEMENT_2_H
#define CGAL_DRAW_ARRANGEMENT_2_H

#include <CGAL/Qt/Basic_viewer_qt.h>

#ifdef CGAL_USE_BASIC_VIEWER

//#include <CGAL/Arrangement_on_surface_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Random.h>

namespace CGAL
{

    // Viewer class for Arr 
    template <class Arr>
    class SimpleArrangementViewerQt : public Basic_viewer_qt
    {
        typedef Basic_viewer_qt Base;
        typedef typename Arr::Halfedge_const_handle Halfedge_const_handle;
        typedef typename Arr::Face_const_handle Face_const_handle;
        typedef typename Arr::Geometry_traits_2 Kernel;
        typedef typename Kernel::Point_2 Point;
        typedef typename Kernel::Vector_2 Vector;

    public:
        /// Construct the viewer.
        /// @param a_a the arrangement to view
        /// @param title the title of the window
        /// @param anofaces if true, do not draw faces (faces are not computed; this can be
        ///        usefull for very big object where this time could be long)
        SimpleArrangementViewerQt(QWidget* parent,
            const Arr& a_arr,
            const char* title = "Basic Arrangement Viewer",
            bool anofaces = false) :
            // First draw: vertices; edges, faces; multi-color; inverse normal
            Base(parent, title, true, true, true, false, false),
            arr(a_arr),
            m_nofaces(anofaces)
        {
            compute_elements();
        }

    protected:

        void compute_face(Face_const_handle fh)
        {
            if (fh->is_unbounded())
            {
                return;
            }
            
            CGAL::Random random((unsigned long)(&*fh));
            CGAL::Color c = get_random_color(random);

            face_begin(c);

            auto edgeCirculator = fh->outer_ccb();
            auto startEdgeCirc = fh->outer_ccb();

            //Circle along the edges and add the vertices to the draw face
            do
            {
                add_point_in_face(edgeCirculator->source()->point());
                //std::cout << edgeCirculator->source()->point() << std::endl;
            } while (++edgeCirculator != startEdgeCirc);


            face_end();
        }
        void compute_elements()
        {
            clear();

            // Draw the arrangement vertices.
            typename Arr::Vertex_const_iterator vit;
            for (vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit)
            {
                add_point(vit->point());
            }

            // Draw the arrangement edges.
            typename Arr::Edge_const_iterator eit;
            for (eit = arr.edges_begin(); eit != arr.edges_end(); ++eit)
            {
                add_segment(eit->curve().source(), eit->curve().target());
                //std::cout << "[" << eit->curve() << "]" << std::endl;
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
        const Arr& arr;
        bool m_nofaces;
    };

    template<class GeomTraits_, class TopTraits_>
    void draw(const Arrangement_2<GeomTraits_, TopTraits_>& a_arr,
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
            SimpleArrangementViewerQt<Arrangement_2<GeomTraits_, TopTraits_> >
                mainwindow(app.activeWindow(), a_arr, title);
            mainwindow.show();
            app.exec();
        }
    }

} // End namespace CGAL

#endif // CGAL_DRAW_ARRANGEMENT_2_H

#endif // CGAL_DRAW_LCC_H