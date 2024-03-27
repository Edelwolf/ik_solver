ALTE VERSION WIRD NICHT MEHR BENUTZT

// #include "rclcpp/rclcpp.hpp"

// #include "geometry_msgs/msg/pose.hpp"

// #include "ik_solver/opw_kinematics_impl.h"
// #include "ik_solver/opw_kinematics.h"
// #include "ik_solver/opw_parameters.h"
// #include "ik_solver/opw_utilities.h"
// #include <Eigen/Geometry>
// #include <vector>

// #include "ik_interfaces/srv/calc_ik.hpp"
// // #include "ik_interfaces/msg/client_msg.hpp"


// class KinematicsService : public rclcpp::Node {
// public:
//     KinematicsService() : Node("kinematics_service") {
//         // Lade die Kinematik-Parameter aus der YAML-Datei
//         if (!loadParameters()) {
//             RCLCPP_ERROR(get_logger(), "Failed to load parameters from YAML file.");
//             return;
//         }

//         // Initialisiere den ROS 2 Service
//         kinematics_service_ = create_service<ik_interfaces::srv::CalcIK>(
//             "kinematics",
//             std::bind(&KinematicsService::handleKinematicsService, this, std::placeholders::_1, std::placeholders::_2)
//         );
//     }

// private:
//     bool loadParameters() {
//         try {
//             // Lade die Parameter aus der YAML-Datei
//             this->declare_parameter("a1", 0.0);
//             this->declare_parameter("a2", 0.0);
//             this->declare_parameter("b", 0.0);
//             this->declare_parameter("c1", 0.0);
//             this->declare_parameter("c2", 0.0);
//             this->declare_parameter("c3", 0.0);
//             this->declare_parameter("c4", 0.0);

//             parameters_.a1 = this->get_parameter("a1").as_double();
//             parameters_.a2 = this->get_parameter("a2").as_double();
//             parameters_.b = this->get_parameter("b").as_double();
//             parameters_.c1 = this->get_parameter("c1").as_double();
//             parameters_.c2 = this->get_parameter("c2").as_double();
//             parameters_.c3 = this->get_parameter("c3").as_double();
//             parameters_.c4 = this->get_parameter("c4").as_double();

//             return true;
//         } catch (const std::exception &e) {
//             RCLCPP_ERROR(get_logger(), "Exception while loading parameters: %s", e.what());
//             return false;
//         }
//     }

//     void handleKinematicsService(
//         const std::shared_ptr<ik_interfaces::srv::CalcIK::Request> request,
//         std::shared_ptr<ik_interfaces::srv::CalcIK::Response> response) {

//         // Erstelle eine Transformation basierend auf der Pose
//         // opw_kinematics::Transform<double> pose = Eigen::Isometry3d::Identity();
//         opw_kinematics::Transform<double> pose = opw_kinematics::Transform<double>::Identity();
//         pose.translation() = Eigen::Vector3d(request->pose.position.x, request->pose.position.y, request->pose.position.z);
//         // Konvertiere den Quaternion in eine Rotationsmatrix und setze die Orientierung
//         Eigen::Quaterniond quaternion(request->pose.orientation.w, request->pose.orientation.x, request->pose.orientation.y, request->pose.orientation.z);
//         // Eigen::Vector3d euler_angles = pose.linear().eulerAngles(0, 1, 2); // Roll, Pitch, Yaw (XYZ Konvention)
//         pose.linear() = quaternion.toRotationMatrix();




//         // Inverse Kinematik berechnen
//         opw_kinematics::Solutions<double> sols = opw_kinematics::inverse(parameters_, pose);


//         // Harmonisieren und Gültigkeit der Lösungen prüfen
//          bool hasValidSolution = false;
//         for (auto& s : sols) {
//             if (opw_kinematics::isValid(s)) {
//                 opw_kinematics::harmonizeTowardZero(s);
//                 hasValidSolution = true; // Mindestens eine gültige Lösung gefunden
//             }
//         }

//     // Überprüfe den Erfolg oder das Versagen der Berechnungen
//     if (!hasValidSolution) {
//         RCLCPP_ERROR(get_logger(), "Inverse Kinematics calculation failed: No valid solutions found.");
//         return;
//     }


//         // Fülle die Antwort-Nachricht mit den berechneten Gelenkwinkeln
//         response->ik_joint_states.resize(sols.size()); 

//     size_t validSolutionCount = 0;
//     for (size_t i = 0; i < sols.size(); ++i) {
//         if (opw_kinematics::isValid(sols[i])) {
//             auto& joint_state = response->ik_joint_states[validSolutionCount++];
//             joint_state.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"}; // NAMEN???
//             joint_state.state.insert(joint_state.state.end(), sols[i].begin(), sols[i].end());
//             joint_state.is_valid = opw_kinematics::isValid(sols[i]); // Diese Lösung ist gültig und wurde bereits harmonisiert
//             // Überprüfen, ob die Lösung innerhalb der Gelenkgrenzen liegt
//             joint_state.is_within_limits = true; // TODO:  Beispiel, benötigt noch echte Überprüfung
//         }
//     }
//     response->ik_joint_states.resize(validSolutionCount); // Anpassen der Größe basierend auf der Anzahl gültiger Lösungen
    


//         // for (auto &s : sols) {
//         //     if (opw_kinematics::isValid(s))
//         //         opw_kinematics::harmonizeTowardZero(s);
//         // }

//         // // Überprüfe den Erfolg oder das Versagen der Berechnungen
//         // if (sols.empty()) {
//         //     RCLCPP_ERROR(get_logger(), "Inverse Kinematics calculation failed: No solutions found.");
//         //     return;
//         // }

//         // // Antwort-Nachricht basierend auf den Ergebnissen
//         // response->ik_result.resize(sols.size());
//         // response->valid.resize(sols.size());
//         // for (size_t i = 0; i < sols.size(); ++i) {
//         //     response->valid[i] = opw_kinematics::isValid(sols[i]);
//         //     response->ik_result[i].x = sols[i][0];
//         //     response->ik_result[i].y = sols[i][1];
//         //     response->ik_result[i].z = sols[i][2];
//         //     // Hier müssen die Roll-, Pitch- und Yaw-Werte entsprechend eingestellt werden
//         //     response->ik_result[i].roll = 0.0; 
//         //     response->ik_result[i].pitch = 0.0;
//         //     response->ik_result[i].yaw = 0.0;
//         // }
//     }

//     rclcpp::Service<ik_interfaces::srv::CalcIK>::SharedPtr kinematics_service_;
//     opw_kinematics::Parameters<double> parameters_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<KinematicsService>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
