
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/common/find_resource.h"


namespace drake {
namespace geometry {
namespace optimization {

int do_main(){
    const std::string filename = "drake/geometry/optimization/test/faulty_hpolyhedron.yaml";
    const auto file = FindResourceOrThrow(filename);
    const auto polytope = yaml::LoadYamlFile<HPolyhedron>(file);
    log()->info("number of hyperplanes: {}", polytope.A().rows());
    const auto E = polytope.MaximumVolumeInscribedEllipsoid();
    log()->info("E Volume is {}", E.Volume());
    log()->info("E center is {}", E.center());
    return 0;
}

} // namespace optimization
} // namespace geometry
} // namespace drake

int main(){
    return drake::geometry::optimization::do_main();
}