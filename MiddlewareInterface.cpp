#include "MiddlewareInterface.h"

using namespace MWI;

MiddlewareInterface::MiddlewareInterface()
{

}

Port::Port(const std::string portname)
{
    Port::Setup(portname);

}

bool Port::Setup(std::string portname)
{
    yarpPortString = portname;
    //OPEN imu port
    PortBuffer.open(yarpPortString+":i");
    if (PortBuffer.isClosed())
    {
        std::cerr << "Can not open "<< yarpPortString+":i"  << std::endl;
    }
    else
    {
        yarp::os::Network::connect(yarpPortString, yarpPortString+":i");
        std::cout << "Connected to "<< PortBuffer.getName() << std::endl;
        //std::cout << PortBuffer.getName();

    }
    //Time::delay(10);  //Wait for port to open [s]

    return true;

}


bool Port::Read(std::istream &indices, std::ostream& data)
{

    int index;

    onePortData = PortBuffer.read(false); //waiting data. TODO: manage wait.
    if (onePortData==NULL)
    {
        std::cerr << "No data in " << yarpPortString << std::endl;
    }
    else
    {

        while(indices >> index)
        {
            data << onePortData->get(index).asString();
        }

    }
    return true;
}

bool Port::ReadAllData(std::ostream& data)
{

    //int index,indices;

    onePortData = PortBuffer.read(false); //waiting data. TODO: manage wait.
    if (onePortData==NULL)
    {
        std::cerr << "No data in " << yarpPortString << std::endl;
    }
    else
    {
        for(int index=0; index<onePortData->size(); index++)
        {
            data << onePortData->get(index).asString();
        }

    }
    return true;
}


bool Port::ShowAllData()
{

    //int index,indices;
    onePortData = PortBuffer.read(false); //waiting data. TODO: manage wait.


    if (onePortData==NULL)
    {
        std::cerr << "No data in " << yarpPortString << std::endl;
    }
    else
    {
        std::cout << onePortData->size();
        for(int index=0; index<onePortData->size(); index++)
        {
            std::cout << onePortData->get(index).asString();
        }

    }
    return true;
}


// Joint

bool Joint::GetPos()
{

}

bool Joint::SetPos(double)
{
    std::cout << "TODO";
}

Robot::Robot(std::istream& config)
{
    std::string name,value;
    while(config >> name)
    {
        config >> value;
        robotOptions.put(name,value);
        std::cout << name << value;
    }
    deviceDriver.open(robotOptions);               //YARP multi-use driver with the given options
    if(!deviceDriver.isValid())
    {
      std::cerr << "Not avilable: " << robotOptions.toString() << std::endl;
      deviceDriver.close();
      //return;
    }

    if ( ! deviceDriver.view(iVel) )
    {
        std::cerr << "vControl Not avilable." << std::endl;
        velAxes = 0;
    }
    else
    {
        iVel->setVelocityMode();
        iVel->getAxes(&velAxes);
    }

    if ( ! deviceDriver.view(iEnc) )
    {
        std::cerr << "encoders Not avilable." << std::endl;
        encoderAxes=0;
    }
    else
    {
        iEnc->getAxes(&encoderAxes);

    }
    vLimit = 1;

}

bool Robot::GetJoints(std::ostream &positions)
{
    double* encValuePtr;

    if (encoderAxes == 0)
    {
        std::cerr << "encoderAxes = 0" << std::endl;
        return false;
    }

    iEnc->getEncoders(encValuePtr);

    for (int i=0; i<encoderAxes; i++)
    {
        positions << *encValuePtr << " ";
        encValuePtr++;

    }


    return true;
}

bool Robot::GetJoint(int encoderAxe, double& encoderValue)
{

    if (encoderAxe > encoderAxes)
    {
        std::cerr << "No such axe number" << std::endl;
        return false;
    }

    iEnc->getEncoder(encoderAxe, &encoderValue);

    return true;
}

bool Robot::SetJointVel(int axe, double &value)
{

    if (axe > velAxes)
    {
        std::cerr << "No such axe number" << std::endl;
        return false;
    }
    iVel->velocityMove(axe, std::min(value,vLimit) );
/*
    if (value <= vLimit)
    {
        iVel->velocityMove(axe, value);
        std::cout << "value";

    }
    else
    {
        iVel->velocityMove(axe, vLimit);
        std::cout << "vLimit";

    }
*/
    return true;

}





