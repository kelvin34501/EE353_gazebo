// this plugin initialize the world

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "ignition/math/Pose3.hh"

#include <iostream>
#include <sstream>
#include <vector>

using std::cout;
using std::vector;

namespace gazebo {
class WorldInit : public WorldPlugin {
    public:
    void Load(physics::WorldPtr _parent, sdf::ElementPtr) {
        // task: insert things from string

        // ========================================================================================================
        vector<vector<double>> ob_pos{{5, 2.8},   {5.5, 2.5}, {6.3, 2.4}, {6.8, 2.2}, {7.1, 2},  {7.4, 1.8},
                                      {7.6, 1.6}, {7.9, 1.2}, {8.2, 1},   {8.4, 0.8}, {8.5, 0.6}};

        // insert a box:
        int ob_id = 0;
        for (const auto& pos : ob_pos) {
            std::cout << "======\n";
            std::cout << "Placing Box: " << pos[0] << ' ' << pos[1] << '\n';

            sdf::SDF ob_sdf;
            std::stringstream sdf_ss;

            sdf_ss << "<sdf version='1.4'>\n";
            sdf_ss << "    <model name='ob_" << ob_id++ << "'>\n";
            sdf_ss << "        <pose>" << pos[0] << ' ' << pos[1] << " 0 0 0 0</pose>\n";
            sdf_ss << "        <link name ='link'>\n";
            sdf_ss << "            <pose>0 0 0.25 0 0 0</pose>\n";
            sdf_ss << "            <collision name ='collision'>\n";
            sdf_ss << "                <geometry>\n";
            sdf_ss << "                    <box><size>0.2 0.2 0.5</size></box>\n";
            sdf_ss << "                </geometry>\n";
            sdf_ss << "            </collision>\n";
            sdf_ss << "            <visual name ='visual'>\n";
            sdf_ss << "                <geometry>\n";
            sdf_ss << "                    <box><size>0.2 0.2 0.5</size></box>\n";
            sdf_ss << "                </geometry>\n";
            sdf_ss << "            </visual>\n";
            sdf_ss << "        </link>\n";
            sdf_ss << "    </model>\n";
            sdf_ss << "</sdf>";

            std::cout << "\tThe SDF:\n" << sdf_ss.str() << '\n';

            ob_sdf.SetFromString(sdf_ss.str());
            // sdf::ElementPtr ob_model = ob_sdf.Root()->GetElement("model");

            _parent->InsertModelSDF(ob_sdf);

            std::cout << "<<<<<<\n";
        }
        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        // ========================================================================================================
        vector<vector<double>> robot_pos{{-1, 6, 0}, {-2, 6, 0}, {-3, 6, 0}, {-4, 6, 0}, {0, 6, 0}};
        int fol_num = 4;
        int N       = robot_pos.size();

        // insert robots
        int robot_id = 0;
        for (const auto& pos : robot_pos) {
            std::cout << "======\n";
            bool is_follower = (robot_id < fol_num);

            if (is_follower) {
                std::cout << "Placing Robot (follower): " << pos[0] << ' ' << pos[1] << '\n';
            } else {
                std::cout << "Placing Robot (leader): " << pos[0] << ' ' << pos[1] << '\n';
            }

            sdf::SDF robot_sdf;
            std::stringstream sdf_ss;

            sdf_ss << "<sdf version='1.7'>\n";
            sdf_ss << "    <model name='robot_" << robot_id++ << "'>\n";
            sdf_ss << "        <pose>" << pos[0] << ' ' << pos[1] << " 0 0 0 0</pose>\n";
            sdf_ss << "        <include><uri>model://robot_model_4</uri></include>\n";
            sdf_ss << "        <plugin name='robot_drive' filename='libproject_robot_drive.so'>\n";
            sdf_ss << "            <velocity>5.0</velocity>\n";
            sdf_ss << "        </plugin>\n";
            sdf_ss << "    </model>\n";
            sdf_ss << "</sdf>";

            std::cout << "\tThe SDF:\n" << sdf_ss.str() << '\n';

            robot_sdf.SetFromString(sdf_ss.str());

            _parent->InsertModelSDF(robot_sdf);

            std::cout << "<<<<<<\n";
            // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        }
    }
};

GZ_REGISTER_WORLD_PLUGIN(WorldInit)
} // namespace gazebo
