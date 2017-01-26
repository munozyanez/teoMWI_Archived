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
        std::cout << "Can not open "<< yarpPortString+":i"  << std::endl;
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
        std::cout << "No data in " << yarpPortString << std::endl;
        return false;
    }
    else
    {

        while(indices)
        {
            indices >> index;
            std::cout << index << ", ";
            data << onePortData->get(index).asString();
        }

    }
    return true;
}

bool Port::Read(int indices[], std::vector<double> & data)
{


    onePortData = PortBuffer.read(false); //waiting data. TODO: manage wait.
    if (onePortData==NULL)
    {
        std::cout << "No data in " << yarpPortString << std::endl;
        return false;

    }
    else
    {

        for (int i=0; i<data.size(); i++)
        {

            std::cout << indices[i] << ", ";
            data[i] = onePortData->get(indices[i]).asDouble();
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
        std::cout << "No data in " << yarpPortString << std::endl;
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
        std::cout << "No data in " << yarpPortString << std::endl;
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
      std::cout << "Not avilable: " << robotOptions.toString() << std::endl;
      deviceDriver.close();
      //return;
    }

    if ( ! deviceDriver.view(iVel) )
    {
        std::cout << "Velocity Control Not avilable." << std::endl;
        velAxes = 0;
    }
    else
    {
        //iVel->setVelocityMode(); deprecated
        iVel->getAxes(&velAxes);
    }

    if ( ! deviceDriver.view(iEnc) )
    {
        std::cout << "encoders Not avilable." << std::endl;
        encoderAxes=0;
    }
    else
    {
        iEnc->getAxes(&encoderAxes);

    }
    if ( ! deviceDriver.view(iPos) )
    {
        std::cout << "Position Control Not avilable." << std::endl;
        posAxes=0;
    }
    else
    {
        iPos->getAxes(&posAxes);

    }
    vLimit = 2;

}
//
//Modes are deprecated so this function is not needed anymore
bool Robot::SetControlMode(int newMode)
{
  /*  std::cout << "Modes are deprecated so this function is not needed anymore" << std::endl;
    */
    if (controlMode == newMode)
    {
        return true;
    }
    else
    {
       switch (newMode)
       {
       case 1:
           if(iPos->setPositionMode())
           {
               controlMode=newMode;
           }
           else
           {
               std::cout << "Control mode not available. Keeping actual mode: " << controlMode << std::endl;
           }
           break;
       case 2:
           if (iVel->setVelocityMode())
           {
               controlMode=newMode;
           }
           else
           {
               std::cout << "Control mode not available. Keeping actual mode: " << controlMode << std::endl;
           }
           break;
       default:
           std::cout << "Control mode not available. Keeping actual mode: " << controlMode << std::endl;
           break;

       }
    }
}

bool Robot::GetJoints(std::ostream &positions)
{
    double* encValuePtr;

    if (encoderAxes == 0)
    {
        std::cout << "encoderAxes = 0" << std::endl;
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

double Robot::GetJoint(int encoderAxis)
{

    double encoderValue;
    if (encoderAxis > encoderAxes)
    {
        std::cout << "No such axis number" << std::endl;
        return false;
    }

    iEnc->getEncoder(encoderAxis, &encoderValue);

    return encoderValue;
}

double Robot::GetJointVelocity(int encoderAxis)
{

    double encoderVelocityValue;
    if (encoderAxis > encoderAxes)
    {
        std::cout << "No such axis number" << std::endl;
        return false;
    }

    iEnc->getEncoderSpeed(encoderAxis, &encoderVelocityValue);

    return encoderVelocityValue;
}

bool Robot::SetJointVel(int axis, double value)
{


    SetControlMode(2);
    if (axis > velAxes)
    {
        std::cout << "No such axis number" << std::endl;
        return false;
    }

    if(value>0)
    {
        iVel->velocityMove(axis, std::min(value,vLimit) );
    }
    else
    {
        iVel->velocityMove(axis, std::max(value,-vLimit) );
    }

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

bool Robot::SetJointPos(int axis, double value)
{

    SetControlMode(1);
    if (axis > posAxes)
    {
        std::cout << "No such axis number" << std::endl;
        return false;
    }


    iPos->positionMove(axis, value );

    return true;

}

bool Robot::DefaultPosition()
{

    SetControlMode(1);
    for (int axis=0;axis < posAxes;axis++)
    {
        iPos->positionMove(axis, 0 );
        std::cout << "Zeroing "  << axis << std::endl;

    }
    return true;

}

bool Robot::Stop()
{

    SetControlMode(2);
    for (int axis=0;axis < posAxes;axis++)
    {
        iVel->velocityMove(axis, 0 );
        std::cout << "Stopping "  << axis << std::endl;

    }
    return true;

}


