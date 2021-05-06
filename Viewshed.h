#pragma once

namespace Viewshed
{
    class Viewshed
    {
    public:
        Viewshed();
        Viewshed(const Delaunay& DT, const Point_3& viewpoint);

        ViewshedArrangement viewshed;

        void OverlayViewsheds(const ViewshedArrangement& otherViewshed);
        int Complexity() const;

        void Draw(std::string title, bool hideVerts, bool hideFaces) const;

        void WriteToFile(const std::filesystem::path& filePath) const;
        void ReadFromFile(const std::filesystem::path& filePath);
        void WriteToIpe(const std::filesystem::path& filePath) const;
    };
}