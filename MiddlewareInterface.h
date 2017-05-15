#ifndef MIDDLEWAREINTERFACE_H
#define MIDDLEWAREINTERFACE_H

//yarp implementation for MiddlewareInterface

#include<string>
#include<iostream>
#include <thread>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

namespace MWI
{



class MiddlewareInterface
{
public:
    MiddlewareInterface();
    long GetError() const;

protected:
    long error;

private:
    yarp::os::Network yarpNet;

};


class Port : MiddlewareInterface
{

public:
    Port();
    Port(const std::string portname);
    bool Read(std::istream &indices, std::ostream &data);
    bool Read(int indices[], std::vector<double> & data);

    bool ReadAllData(std::ostream &data);
    bool ShowAllData();
    bool Setup(std::string portname);

private:
    yarp::os::BufferedPort<yarp::os::Bottle> PortBuffer;
    yarp::os::Bottle* onePortData;
    std::string yarpPortString;


};

class Joint : MiddlewareInterface
{
public:
    bool GetPos();
    bool SetPos(double);
private:

};

class Robot : public MiddlewareInterface
{
public:
    Robot();
    Robot(std::istream &config);
    Robot(std::string robotName, std::string limbName);
    bool GetJoints(std::ostream & positions);
    bool GetJoints(std::vector<double> &positions);
    double GetJoint(int encoderAxis);
    double GetJointVel(int encoderAxis);
    bool SetJointVel(int axis, double value);
    bool SetJointPos(int axis, double value);
    bool SetControlMode(int newMode);
    double GetJointVelocity(int encoderAxis);
    bool DefaultPosition();
    bool Stop();


    bool SetJointVels(std::vector<double> &value);
    bool SetJointPositions(std::vector<double> value);
private:
    yarp::os::Property robotOptions;
    yarp::dev::PolyDriver deviceDriver;
    std::vector<Joint> joints;
    yarp::dev::IVelocityControl2 *iVel;                 //Velocity controller
    yarp::dev::IPositionControl2 *iPos;                 //position controller
    yarp::dev::IEncoders *iEnc;         //encoders
    int encoderAxes;
    int velAxes, posAxes;
    double vLimit;
    int controlMode; //1:pos, 2:vel

    double encoderValue, encoderSpeed;


    //std::thread posThread;
    //std::thread velThread;


    long Initialize();
};

} //end namespace MWI
#endif // MIDDLEWAREINTERFACE_H
