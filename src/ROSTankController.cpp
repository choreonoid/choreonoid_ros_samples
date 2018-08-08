/**
   ROS Tank Controller
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/SpotLight>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <mutex>

using namespace std;
using namespace cnoid;

namespace {

const int axisID[] = { 0, 1, 3, 4 };
const int buttonID[] = { 0, 2, 3 };

}

class ROSTankController : public SimpleController
{
    ros::NodeHandle node;
    ros::Subscriber joystickSubscriber;
    sensor_msgs::Joy latestJoystickState;
    std::mutex joystickMutex;
    
    bool usePseudoContinousTrackMode;
    Link::ActuationMode turretActuationMode;
    Link* trackL;
    Link* trackR;
    Link* turretJoint[2];
    double qref[2];
    double qprev[2];
    double dt;
    LightPtr light;
    SpotLightPtr spotLight;
    bool prevLightButtonState;

public:

    virtual bool configure(SimpleControllerConfig* config) override
    {
        //config->sigChanged().connect();
        return true;
    }
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        ostream& os = io->os();
        Body* body = io->body();
        dt = io->timeStep();

        turretActuationMode = Link::ActuationMode::JOINT_TORQUE;
        for(auto opt : io->options()){
            if(opt == "velocity"){
                turretActuationMode = Link::ActuationMode::JOINT_VELOCITY;
            }
        }

        turretJoint[0] = body->link("TURRET_Y");
        turretJoint[1] = body->link("TURRET_P");
        for(int i=0; i < 2; ++i){
            Link* joint = turretJoint[i];
            qref[i] = qprev[i] = joint->q();
            joint->setActuationMode(turretActuationMode);
            io->enableIO(joint);
        }

        trackL = body->link("WHEEL_L0");
        trackR = body->link("WHEEL_R0");
        if(trackL && trackR){
            usePseudoContinousTrackMode = false;
            trackL->setActuationMode(Link::JOINT_VELOCITY);
            trackR->setActuationMode(Link::JOINT_VELOCITY);
        } else {
            usePseudoContinousTrackMode = true;
            trackL = body->link("TRACK_L");
            trackR = body->link("TRACK_R");
            trackL->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
            trackR->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
        }
        io->enableOutput(trackL);
        io->enableOutput(trackR);

        DeviceList<Light> lights(body->devices());
        if(!lights.empty()){
            light = lights.front();
            spotLight = dynamic_pointer_cast<SpotLight>(light);
        }
        prevLightButtonState = false;

        joystickSubscriber = node.subscribe("joy", 1, &ROSTankController::joystickCallback, this);

        return true;
    }

    void joystickCallback(const sensor_msgs::Joy& msg)
    {
        std::lock_guard<std::mutex> lock(joystickMutex);
        latestJoystickState = msg;
    }

    virtual bool control() override
    {
        sensor_msgs::Joy joystick;
        {
            std::lock_guard<std::mutex> lock(mutex);
            joystick = latestJoystickState;
            joystick.axes.resize(10, 0.0f);
            joystick.buttons.resize(10, 0);
        }
            
        double pos[2];
        for(int i=0; i < 2; ++i){
            pos[i] = joystick.axes[axisID[i]];
            if(fabs(pos[i]) < 0.2){
                pos[i] = 0.0;
            }
        }
        // set the velocity of each tracks
        if(usePseudoContinousTrackMode){
            double k = 1.0;
            trackL->dq() = k * (2.0 * pos[1] - pos[0]);
            trackR->dq() = k * (2.0 * pos[1] + pos[0]);
        } else {
            double k = 4.0;
            trackL->dq() = k * (pos[1] - pos[0]);
            trackR->dq() = k * (pos[1] + pos[0]);
        }

        static const double P = 200.0;
        static const double D = 50.0;

        for(int i=0; i < 2; ++i){
            Link* joint = turretJoint[i];
            double pos = -joystick.axes[axisID[i + 2]];
            if(fabs(pos) < 0.15){
                pos = 0.0;
            }
            if(turretActuationMode == Link::JOINT_VELOCITY){
                joint->dq() = pos;
            } else if(turretActuationMode == Link::JOINT_TORQUE){
                double q = joint->q();
                double dq = (q - qprev[i]) / dt;
                double dqref = 0.0;
                double deltaq = 0.002 * pos;
                qref[i] += deltaq;
                dqref = deltaq / dt;
                joint->u() = P * (qref[i] - q) + D * (dqref - dq);
                qprev[i] = q;
            }
        }

        if(light){
            bool changed = false;
            bool lightButtonState = joystick.buttons[buttonID[0]];
            if(lightButtonState){
                if(!prevLightButtonState){
                    light->on(!light->on());
                    changed = true;
                }
            }
            prevLightButtonState = lightButtonState;

            if(spotLight){
                if(joystick.buttons[buttonID[1]]){
                    spotLight->setBeamWidth(std::max(0.1f, spotLight->beamWidth() - 0.001f));
                    changed = true;
                } else if(joystick.buttons[buttonID[2]]){
                    spotLight->setBeamWidth(std::min(0.7854f, spotLight->beamWidth() + 0.001f));
                    changed = true;
                }
            }
            if(changed){
                light->notifyStateChange();
            }
        }

        return true;
    }

    virtual void stop() override
    {
        joystickSubscriber.shutdown();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ROSTankController)
