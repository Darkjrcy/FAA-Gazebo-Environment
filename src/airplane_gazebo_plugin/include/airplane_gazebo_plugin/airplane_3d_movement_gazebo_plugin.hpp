#ifndef AIRPLANE_3D_MOVEMENT_GAZEBO_PLUGIN_HPP_
#define AIRPLANE_3D_MOVEMENT_GAZEBO_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace airplane_3d_movement_gazebo_plugin
{
    class Airplane3DMovementGazeboPluginPrivate;
    class Airplane3DMovementGazeboPlugin :public gazebo::ModelPlugin
    {
        public:
        //Constructor:
        Airplane3DMovementGazeboPlugin();
        //Destructor:
        ~Airplane3DMovementGazeboPlugin();
        protected:
        //Load information
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;        
        void Reset() override;
        
        private:
        std::unique_ptr<Airplane3DMovementGazeboPluginPrivate> impl_;
    };
}

#endif

