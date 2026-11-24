#include "shelf_scene/world.hpp"

#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string_view>

#include "shelf_scene/layout.hpp"

namespace shelf_scene
{
std::string world(std::int64_t seed, double width, double depth)
{
  std::ostringstream sdf;
  sdf.imbue(std::locale::classic());
  sdf << std::setprecision(std::numeric_limits<double>::max_digits10);
  sdf << "<sdf version=\"1.6\">\n"
         "  <world name=\"shelf\">\n"
         "    <plugin name=\"gazebo_ros_state\" filename=\"libgazebo_ros_state.so\">\n"
         "      <ros><namespace>/gazebo</namespace></ros>\n"
         "    </plugin>\n";

  for (const auto & box : layout(seed, width, depth)) {
    const bool dynamic = box.name == "target";
    const auto & [x, y, z] = box.center;
    const auto & [length, breadth, height] = box.size;
    sdf << "    <model name=\"" << box.name << "\">\n"
        << "      <static>" << (dynamic ? "false" : "true") << "</static>\n"
        << "      <pose>" << x << ' ' << y << ' ' << z << " 0 0 " << box.yaw << "</pose>\n"
        << "      <link name=\"body\">\n";

    for (const std::string_view kind : {"collision", "visual"}) {
      sdf << "        <" << kind << " name=\"" << kind << "\">\n"
          << "          <geometry><box><size>" << length << ' ' << breadth << ' ' << height
          << "</size></box></geometry>\n";
      if (dynamic && kind == "collision") {
        // Damp ODE contact correction while the target rests on the shelf or tray.
        sdf << "          <surface><contact><ode>\n"
               "            <max_vel>0.01</max_vel>\n"
               "            <min_depth>0.0001</min_depth>\n"
               "            <kp>100000</kp>\n"
               "            <kd>10</kd>\n"
               "          </ode></contact></surface>\n";
      }
      sdf << "        </" << kind << ">\n";
    }

    if (dynamic) {
      constexpr double mass = .1;
      const auto inertia = [](double a, double b) {return mass * (a * a + b * b) / 12;};
      sdf << "        <inertial>\n"
          << "          <mass>" << mass << "</mass>\n"
          << "          <inertia>\n"
          << "            <ixx>" << inertia(breadth, height) << "</ixx>\n"
          << "            <iyy>" << inertia(length, height) << "</iyy>\n"
          << "            <izz>" << inertia(length, breadth) << "</izz>\n"
          << "          </inertia>\n"
          << "        </inertial>\n";
    }
    sdf << "      </link>\n    </model>\n";
  }
  sdf << "  </world>\n</sdf>\n";
  return sdf.str();
}
}  // namespace shelf_scene
