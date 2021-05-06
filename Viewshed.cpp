#include "precomp.h"
#include "Viewshed.h"

namespace Viewshed
{
    Viewshed::Viewshed()
    {
        viewshed.unbounded_face()->data() = false;
    }

    Viewshed::Viewshed(const Delaunay& DT, const Point_3& viewpoint)
    {
        std::cout << "\tViewshed Projections loading" << std::endl;
        std::vector<ViewshedProjection> viewshedParts;
        viewshedParts.push_back(ViewshedProjection(DT, viewpoint, ViewDirection::UP));
        viewshedParts.push_back(ViewshedProjection(DT, viewpoint, ViewDirection::DOWN));
        viewshedParts.push_back(ViewshedProjection(DT, viewpoint, ViewDirection::LEFT));
        viewshedParts.push_back(ViewshedProjection(DT, viewpoint, ViewDirection::RIGHT));
        viewshedParts.push_back(ViewshedProjection(DT, viewpoint, ViewDirection::FRONT));
        viewshedParts.push_back(ViewshedProjection(DT, viewpoint, ViewDirection::BEHIND));
        std::cout << "\r\tViewshed Projections loaded" << std::endl;

        ViewshedArrangement prevViewshedCombination;
        prevViewshedCombination.unbounded_face()->data() = false;
        ViewshedOverlayTraits viewshedOverlayTraits;

        int i = 0;
        for (ViewshedProjection& viewshedPart : viewshedParts)
        {
            ++i;
            if (viewshedPart.IsEmpty())
            {
                continue;
            }

            std::cout << "\r\tDetermining visibility and combining " << i << "/" << viewshedParts.size();
            ViewshedArrangement viewshedArr = viewshedPart.DetermineVisibility();
            CGAL::overlay(prevViewshedCombination, viewshedArr, viewshed, viewshedOverlayTraits);
            prevViewshedCombination = viewshed;
        }
        std::cout << "\r\tDetermining visibility and combining " << viewshedParts.size() << "/" << viewshedParts.size() << std::endl;


        //CGAL::DrawViewshed(viewshed, false, "Viewshed for one viewpoint", true);


        //Remove redundant vertices
        std::vector<ViewshedArrangement::Vertex_handle> vertexHandles;
        for (ViewshedArrangement::Vertex_handle vertexHandle : viewshed.vertex_handles())
        {
            vertexHandles.push_back(vertexHandle);
        }
        for (ViewshedArrangement::Vertex_handle vertexHandle : vertexHandles)
        {
            CGAL::remove_vertex(viewshed, vertexHandle);
        }
        vertexHandles.clear();

        std::cout << "\tViewshed done" << std::endl;

        //CGAL::DrawViewshed(viewshed, false, "Viewshed - Redundant vertices removed", true);
        //CGAL::DrawViewshed(viewshed, false, "Viewshed - Redundant vertices removed", false);
    }

    void Viewshed::OverlayViewsheds(const ViewshedArrangement& otherViewshed)
    {
        ViewshedArrangement overlayedViewsheds;
        ViewshedOverlayTraits viewshedOverlayTraits;
        CGAL::overlay(viewshed, otherViewshed, overlayedViewsheds, viewshedOverlayTraits);
        viewshed = overlayedViewsheds;

        //Remove redundant vertices
        std::vector<ViewshedArrangement::Vertex_handle> vertexHandles;
        for (ViewshedArrangement::Vertex_handle vertexHandle : viewshed.vertex_handles())
        {
            vertexHandles.push_back(vertexHandle);
        }
        for (ViewshedArrangement::Vertex_handle vertexHandle : vertexHandles)
        {
            CGAL::remove_vertex(viewshed, vertexHandle);
        }
        vertexHandles.clear();
    }

    /*!
        Counts the amount of vertices on the boundaries of the visible regions
    */
    int Viewshed::Complexity() const
    {
        int complexity = 0;
        ViewshedArrangement::Edge_const_iterator edgeit;
        for (edgeit = viewshed.edges_begin(); edgeit != viewshed.edges_end(); ++edgeit)
        {
            //If one of the faces is empty
            if (!(edgeit->face()->data() && edgeit->twin()->face()->data()))
            {
                //Count start vertex
                ++complexity;
            }
        }
        return complexity;
    }

    void Viewshed::Draw(std::string title, bool hideVerts, bool hideFaces) const
    {
        CGAL::DrawViewshed(viewshed, hideVerts, title.c_str(), hideFaces);
    }

    void Viewshed::WriteToFile(const std::filesystem::path& filePath) const
    {
        if (!std::filesystem::exists(filePath.parent_path()))
        {
            std::filesystem::create_directories(filePath.parent_path());
        }

        std::ofstream outFile(filePath);
        ViewshedIOFormatter formatter;

        CGAL::write(viewshed, outFile, formatter);
        outFile.close();
    }

    void Viewshed::ReadFromFile(const std::filesystem::path& filePath)
    {
        std::ifstream inFile(filePath);
        ViewshedIOFormatter formatter;

        CGAL::read(viewshed, inFile, formatter);
        inFile.close();
    }

    void Viewshed::WriteToIpe(const std::filesystem::path& filePath) const
    {
        const std::string ipeFileStart = "<?xml version=\"1.0\"?>\n"
            "<!DOCTYPE ipe SYSTEM \"ipe.dtd\">\n"
            "<ipe version = \"70206\" creator=\"Ipe 7.2.7\">\n"
            "<page>\n"
            "<layer name = \"alpha\"/>\n"
            "<view layers = \"alpha\" active=\"alpha\"/>\n";

        const std::string ipeFileEnd = "</page>\n</ipe>";

        std::ofstream ipeFileStream(filePath);

        if (!ipeFileStream.is_open())
        {
            std::cout << "\t Ipe file path invalid or not openable.." << std::endl;
            return;
        }

        ipeFileStream << ipeFileStart;

        ViewshedArrangement::Face_const_iterator fit;

        for (fit = viewshed.faces_begin(); fit != viewshed.faces_end(); ++fit)
        {
            //Only export finite faces
            if (!fit->is_unbounded() && !fit->is_fictitious() && fit->has_outer_ccb())
            {
                //Start stroke and set color
                if (fit->data())
                {
                    //Visible blue
                    ipeFileStream << "<path stroke = \"0 0 0\" fill=\"0 0 0.9\">\n";
                }
                else
                {
                    //Invis black
                    ipeFileStream << "<path stroke = \"0 0 0\" fill=\"1 1 1\">\n";
                }

                auto edgeCirculator = fit->outer_ccb();
                auto startEdgeCirc = fit->outer_ccb();

                //Circle along the edges and add the vertices to the ipe polygon
                bool first = true;
                do
                {
                    ipeFileStream << (edgeCirculator->source()->point().x()) << " " << (edgeCirculator->source()->point().y());

                    if (first)
                    {
                        ipeFileStream << " m\n";
                        first = false;
                    }
                    else
                    {
                        ipeFileStream << " l\n";
                    }
                } while (++edgeCirculator != startEdgeCirc);

                ipeFileStream << "h\n";

                //Polygon closing tag
                ipeFileStream << "</path>\n";
            }
        }

        ipeFileStream << ipeFileEnd;
        ipeFileStream.close();
    }
}