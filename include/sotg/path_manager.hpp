#pragma once

#include <iterator>
#include <list>
#include <memory>
#include <vector>

#include "sotg/kinematic_solver.hpp"
#include "sotg/path.hpp"
#include "sotg/section.hpp"

namespace SOTG {
namespace detail {

    // Stores and maintains the input path aswell as all generated sections and
    // segments
    class PathManager {
    private:
        Path path_;
        std::list<Section> sections_;

        std::shared_ptr<KinematicSolver> kinematic_solver_;

        std::vector<std::map<std::string, double>>& debug_info_vec_;

        void resetSections();

    public:
        PathManager(std::shared_ptr<KinematicSolver> solver,
                     std::vector<std::map<std::string, double>>& debug_info_vec);

        void resetPath(Path path);

        int getNumSections() { return sections_.size(); }

        const std::list<Section>& getSections() const { return sections_; }

        Section& getSectionAtTime(double time);

        const Path& getPath() { return path_; }

        std::ostream& operator<<(std::ostream& out);
    };

}  // namespace detail

}  // namespace SOTG