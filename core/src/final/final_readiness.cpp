#include "final/final_readiness.h"

#include <cmath>
#include <sstream>

namespace bs {

std::string ReadinessReportJson(const ReadinessReport& report) {
  std::ostringstream out;
  out.precision(2);
  out << std::fixed;
  if (!report.present) {
    // Say so explicitly rather than omitting the member. A consumer that
    // finds no "readiness" key cannot tell "this build does not produce it"
    // from "this scan produced nothing scoreable", and those call for
    // different responses.
    out << "  \"readiness\": {\"present\": false},\n";
    return out.str();
  }

  out << "  \"readiness\": {\n"
      << "    \"present\": true,\n"
      << "    \"overall\": " << report.overall << ",\n"
      << "    \"overall_sub\": [";
  for (int i = 0; i < 5; ++i) {
    out << report.overall_sub[i] << (i < 4 ? ", " : "");
  }
  out << "],\n    \"regions\": [\n";
  for (size_t i = 0; i < report.regions.size(); ++i) {
    const RegionReport& r = report.regions[i];
    out << "      {\"id\": " << r.id << ", \"name\": \"" << r.name
        << "\", \"score\": " << r.score << ", \"sub\": [";
    for (int a = 0; a < 5; ++a) out << r.sub[a] << (a < 4 ? ", " : "");
    out << "], \"area_m2\": " << r.area_m2
        << ", \"patches\": " << r.patch_count
        << ", \"weak_areas\": " << r.weak_area_count
        << ", \"worst_deficiency\": " << r.worst_deficiency
        << ", \"min\": [" << r.min[0] << ", " << r.min[1] << ", " << r.min[2]
        << "], \"max\": [" << r.max[0] << ", " << r.max[1] << ", " << r.max[2]
        << "]}" << (i + 1 < report.regions.size() ? "," : "") << "\n";
  }
  out << "    ]\n  },\n";
  return out.str();
}

}  // namespace bs
