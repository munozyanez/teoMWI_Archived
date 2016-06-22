#ifndef MIDDLEWAREINTERFACE_H
#define MIDDLEWAREINTERFACE_H

//yarp implementation for MiddlewareInterface

#include<string>
#include<iostream>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

namespace MWI
{



class MiddlewareInterface
{
public:
    MiddlewareInterface();
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

class Robot : MiddlewareInterface
{
public:
    Robot(std::istream &config);
    bool GetJoints(std::ostream & positions);
    bool GetJoint(int encoderAxis, double& encoderValue);
    bool SetJointVel(int axis, double& value);
    bool SetJointPos(int axis, double& value);
    bool SetControlMode(int newMode);
private:
    yarp::os::Property robotOptions;
    yarp::dev::PolyDriver deviceDriver;
    std::vector<Joint> joints;
    yarp::dev::IVelocityControl *iVel;                 //Velocity controller
    yarp::dev::IPositionControl2 *iPos;                 //position controller
    yarp::dev::IEncoders *iEnc;         //encoders
    int encoderAxes;
    int velAxes, posAxes;
    double vLimit;
    int controlMode; //1:pos, 2:vel

};

}
#endif // MIDDLEWAREINTERFACE_H
