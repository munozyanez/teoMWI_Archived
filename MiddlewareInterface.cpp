#include "MiddlewareInterface.h"

void posFunction(int posParam);


using namespace MWI;

MiddlewareInterface::MiddlewareInterface()
{
    //INITIALISE AND CHECK YARP

    if ( !yarpNet.checkNetwork(2) )
    {
        std::cout << "[error] %s found no YARP network (try running \"yarp detect --write\")." << std::endl;
        error = -1;
    }
    else
    {
        std::cout << "[success] YARP network found." << std::endl;
        error = 0;

    }


}

long MiddlewareInterface::GetError() const
{
    return error;
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

long Robot::Initialize()
{
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
    vLimit = 100;

    if ( (velAxes&&encoderAxes&&posAxes) == 0 )
    {
        error = -10;
        std::cout << "Robot Not avilable. Error: " << error << std::endl;
        return -1;

    }
    SetControlMode(1);

    SetControlMode(2);


    return 0;

}

Robot::Robot()
{

}

Robot::Robot(std::istream& config)
{
    //example

    /*  //Robot teo right arm
      //YARP device
      std::stringstream robConfig;
      robConfig << "device remote_controlboard" << " ";
      //To what will be connected
      robConfig << "remote /teo/rightArm" << " ";
      //How will be called on YARP network
      robConfig << "local /local/rightArm" << " ";
      Robot(robConfig);*/
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

    Initialize();


    //posThread = std::thread(posFunction, 1);

}

Robot::Robot(std::string robotName, std::string limbName)
{

    robotOptions.put("device","remote_controlboard");
    robotOptions.put("remote", "/"+robotName+"/"+limbName );
    robotOptions.put("local","/local/"+limbName);

    deviceDriver.open(robotOptions);               //YARP multi-use driver with the given options
    if(!deviceDriver.isValid())
    {
      std::cout << "Not avilable: " << robotOptions.toString() << std::endl;
      deviceDriver.close();
      //return;
    }

    Initialize();


}
//
//Modes are deprecated so this function is not needed anymore
bool Robot::SetControlMode(int newMode)
{
  /*  std::cout << "Modes are deprecated so this function is not needed anymore" << std::endl;
    */
  /*  if (controlMode == newMode)
    {
        return true;
    }
    else
    {*/
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
               std::cout << "Velocity mode"<< std::endl;;
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
   // }
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

bool Robot::GetJoints(std::vector<double> &positions)
{
    //double* encValuePtr;

    if (encoderAxes == 0)
    {
        std::cout << "encoderAxes = 0" << std::endl;
        return false;
    }

    //std::vector<double> encoders(encoderAxes);

    positions.clear();
    positions.resize(encoderAxes);

    iEnc->getEncoders(&positions[0]);
/*
    for (int i=0; i<encoderAxes; i++)
    {
        positions.push_back(*encValuePtr);
        encValuePtr++;

    }

*/
    return true;
}

double Robot::GetJoint(int encoderAxis)
{

    if (encoderAxis > encoderAxes)
    {
        std::cout << "No such axis number" << std::endl;
        return false;
    }

    iEnc->getEncoder(encoderAxis, &encoderValue);

    return encoderValue;
}

double Robot::GetJointVel(int encoderAxis)
{
    if (encoderAxis > encoderAxes)
    {
        std::cout << "No such axis number" << std::endl;
        return false;
    }

    iEnc->getEncoderSpeed(encoderAxis, &encoderSpeed);

    return encoderSpeed;

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


    if (axis > velAxes)
    {
        //std::cout << "No such axis number" << std::endl;
        return false;
    }
    //SetControlMode(2);
    iVel->velocityMove(axis, value );

//    if(value>5)
//    {
//        //std::cout << "value>3:" << value << std::endl;

//        iVel->velocityMove(axis, std::min(value,vLimit) );
//    }
//    else if((value<=5) and (value >0))
//    {
//        //std::cout << "(value<=4) and (value >0)" << value << std::endl;

//        iVel->velocityMove(axis, 5.0 );
//    }
//    else if((value<0) and (value >=-5))
//    {
//        //std::cout << "(value<0) and (value >=-4)" << value << std::endl;

//        iVel->velocityMove(axis, -5.0 );
//    }
//    else if(value<-5)
//    {
//        //std::cout << "(value<-4)" << value << std::endl;

//        iVel->velocityMove(axis, std::max(value,-vLimit) );
//    }
//    else if(value==0)
//    {
//        //std::cout << "(value==0)" << value << std::endl;

//        iVel->velocityMove(axis, 0 );
//    }
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

    if (axis > posAxes)
    {
        std::cout << "No such axis number" << std::endl;
        return false;
    }

    SetControlMode(1);

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


void posFunction(int posParam)
{

}


