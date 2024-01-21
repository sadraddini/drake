#include <cstdio>
#include <fstream>
#include <iostream>

#include "drake/common/find_resource.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

using math::RigidTransformd;
using math::RotationMatrixd;

int do_main() {
  auto meshcat = std::make_shared<Meshcat>();
  
  meshcat->SetObject("sphere", Sphere(0.25), Rgba(1.0, 0, 0, 1));
  meshcat->SetTransform("sphere", RigidTransformd());

  meshcat->StartRecording();

  // change the color of the sphere
  meshcat->SetProperty("sphere", "color", {1, 0, 0, 1}, 0.0);
  meshcat->SetProperty("sphere", "color", {0, 1, 0, 1}, 1.0);
  meshcat->SetProperty("sphere", "color", {0, 0, 1, 1}, 2.0);
  meshcat->SetProperty("sphere", "color", {0, 1, 1, 1}, 3.0);
  meshcat->SetProperty("sphere", "color", {1, 1, 0, 1}, 4.0);
  // change if the sphere is visible
  meshcat->SetProperty("sphere", "visible", true, 0.0);
  meshcat->SetProperty("sphere", "visible", false, 1.0);
  meshcat->SetProperty("sphere", "visible", true, 2.0);
  meshcat->SetProperty("sphere", "visible", false, 3.0);
  meshcat->SetProperty("sphere", "visible", true, 4.0);

  meshcat->PublishRecording();

  const std::string filepath = FindResourceOrThrow("drake/geometry/test/minimal_example.html");
  const auto html_string = meshcat->StaticHtml();
  std::ofstream out;
  out.open(filepath, std::ofstream::out);
  out << html_string;
  out.close();
  return 0;
}

}  // namespace geometry
}  // namespace drake

int main() {
  return drake::geometry::do_main();
}
