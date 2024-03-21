#include "rclcpp/rclcpp.hpp"
#include "interfaces_for_ik_service/srv/client_srv.hpp"
#include "interfaces_for_ik_service/msg/client_msg.hpp"
#include "ik_solver/opw_kinematics_impl.h"
#include "ik_solver/opw_kinematics.h"
#include "ik_solver/opw_parameters.h"
#include "ik_solver/opw_utilities.h"

class KinematicsService : public rclcpp::Node {
public:
    KinematicsService() : Node("kinematics_service") {
        // Lade die Kinematik-Parameter aus der YAML-Datei
        if (!loadParameters()) {
            RCLCPP_ERROR(get_logger(), "Failed to load parameters from YAML file.");
            return;
        }

        // Initialisiere den ROS 2 Service
        kinematics_service_ = create_service<interfaces_for_ik_service::srv::ClientSrv>(
            "kinematics",
            std::bind(&KinematicsService::handleKinematicsService, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    bool loadParameters() {
        try {
            // Lade die Parameter aus der YAML-Datei
            this->declare_parameter("a1", 0.0);
            this->declare_parameter("a2", 0.0);
            this->declare_parameter("b", 0.0);
            this->declare_parameter("c1", 0.0);
            this->declare_parameter("c2", 0.0);
            this->declare_parameter("c3", 0.0);
            this->declare_parameter("c4", 0.0);

            parameters_.a1 = this->get_parameter("a1").as_double();
            parameters_.a2 = this->get_parameter("a2").as_double();
            parameters_.b = this->get_parameter("b").as_double();
            parameters_.c1 = this->get_parameter("c1").as_double();
            parameters_.c2 = this->get_parameter("c2").as_double();
            parameters_.c3 = this->get_parameter("c3").as_double();
            parameters_.c4 = this->get_parameter("c4").as_double();

            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Exception while loading parameters: %s", e.what());
            return false;
        }
    }

    void handleKinematicsService(
        const std::shared_ptr<interfaces_for_ik_service::srv::ClientSrv::Request> request,
        std::shared_ptr<interfaces_for_ik_service::srv::ClientSrv::Response> response) {

        opw_kinematics::Transform<double> pose = opw_kinematics::Transform<double>::Identity();
        pose.translation() = Eigen::Vector3d(request->x, request->y, request->z);

        // Inverse Kinematik berechnen
        opw_kinematics::Solutions<double> sols = opw_kinematics::inverse(parameters_, pose);

        for (auto &s : sols) {
            if (opw_kinematics::isValid(s))
                opw_kinematics::harmonizeTowardZero(s);
        }

        // Überprüfe den Erfolg oder das Versagen der Berechnungen
        if (sols.empty()) {
            RCLCPP_ERROR(get_logger(), "Inverse Kinematics calculation failed: No solutions found.");
            return;
        }

        // Antwort-Nachricht basierend auf den Ergebnissen
        response->ik_result.resize(sols.size());
        response->valid.resize(sols.size());
        for (size_t i = 0; i < sols.size(); ++i) {
            response->valid[i] = opw_kinematics::isValid(sols[i]);
            response->ik_result[i].x = sols[i][0];
            response->ik_result[i].y = sols[i][1];
            response->ik_result[i].z = sols[i][2];
            // Hier müssen die Roll-, Pitch- und Yaw-Werte entsprechend eingestellt werden
            response->ik_result[i].roll = 0.0; 
            response->ik_result[i].pitch = 0.0;
            response->ik_result[i].yaw = 0.0;
        }
    }

    rclcpp::Service<interfaces_for_ik_service::srv::ClientSrv>::SharedPtr kinematics_service_;
    opw_kinematics::Parameters<double> parameters_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsService>());
    rclcpp::shutdown();
    return 0;
}
