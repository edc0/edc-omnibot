#ifndef GAZEBO_INTERFACE_H
#define GAZEBO_INTERFACE_H


// Did not put declare variables as extern here because this interface is to be
// used exclusively with the <kinematics> library.

namespace gazebo {

    class OmniPlatformPlugin : public ModelPlugin {

        private: physics::JointPtr backJoint;
        private: physics::JointPtr leftJoint;
        private: physics::ModelPtr model;
        private: physics::JointPtr rightJoint;
        private: event::ConnectionPtr updateConnection;
        private: physics::WorldPtr world;

        public: void fireMovementAbsoluteM(double x, double y, double theta);
        public: void fireMovementAbsoluteMRaw(double x, double y, double theta);
        public: void fireMovementAbsoluteW(double x, double y, double theta);
        public: void fireMovementBezierM(std::vector<Point> * points,
                std::vector<double> * angles, double step,
                bool offsetT, bool offsetR);
        public: void fireMovementBezierW(std::vector<Point> * points,
                std::vector<double> * angles, double step,
                bool offsetT, bool offsetR);
        public: void fireMovementDirectHybrid(double Vxm, double Vym, double omegap);
        public: void fireMovementDirectMobile(double Vxm, double Vym, double omegap);
        public: void fireMovementDirectWheel(double Vleft, double Vback, double Vright);
        public: void fireMovementDirectWorld(double Vxw, double Vyw, double omegap);
        public: void fireMovementRelativeM(double x, double y, double theta);
        public: void fireMovementRelativeMRaw(double x, double y, double theta);
        public: void fireMovementRelativeW(double x, double y, double theta);
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
        private: void odometry();
        public: void OnUpdate(const common::UpdateInfo & /*_info*/);
        private: void updateIndicator();
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(OmniPlatformPlugin)
}
