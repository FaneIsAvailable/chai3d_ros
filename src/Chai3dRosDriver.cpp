//
// Created by stefan spiss on 06.07.17.
//

//#define DEBUGG

#include <chai3d_ros/Chai3dRosDriver.h>

Chai3dRosDriver::Chai3dRosDriver(ros::NodeHandle node, float loopRate, std::string positionTopic,
                                 std::string velocityTopic, std::string buttonsTopic, std::string forceSubTopic,
                                 std::string rateTopic, bool forceOutput) :
        node(node), loopRate(loopRate), positionTopic(positionTopic), velocityTopic(velocityTopic),
        buttonsTopic(buttonsTopic), forceSubTopic(forceSubTopic), forceOutput(forceOutput), rateTopic(rateTopic),
        position(0.0f, 0.0f, 0.0f), velocity(0.0f, 0.0f, 0.0f), force(0.0f, 0.0f, 0.0f), buttons(4, 0),goodForce(0.0f, 0.0f, 0.0f),LinearMotionCondition(0.0f,0.0f,0.0f),
        hapticLoop(true), forceConsumed(true) {
    int initState = this->initFalcon();
    if(initState == 0) {
        std::cout << "Error, no device found!" << std::endl;
        std::exit(-1);
    } else if(initState == -1) {
        std::cout << "Error, could not open connection to device 0!" << std::endl;
        std::exit(-2);
    }

    this->position_pub = this->node.advertise<geometry_msgs::Twist>(this->positionTopic.c_str(), 1);
    this->velocity_pub = this->node.advertise<geometry_msgs::Vector3Stamped>(this->velocityTopic.c_str(), 1);
    this->buttons_pub = this->node.advertise<std_msgs::Int8MultiArray>(this->buttonsTopic.c_str(), 1);

    this->force_sub = this->node.subscribe<geometry_msgs::Vector3>(this->forceSubTopic, 1,
                                                                   &Chai3dRosDriver::forceCallback, this);
    
    this->linear_sub = this->node.subscribe<geometry_msgs::Vector3>("/interface/lock_axis", 1, &Chai3dRosDriver::LinearMotionCallback, this);
    this->rate_pub = this->node.advertise<std_msgs::Float32>(this->rateTopic.c_str(),1);
    this->gripper_pub = this->node.advertise<geometry_msgs::Twist>("gripper_topic",1);
}

Chai3dRosDriver::~Chai3dRosDriver() {
    //cleanUpFalcon();
}

int Chai3dRosDriver::initFalcon() {
    // create device handler
    handler = new chai3d::cHapticDeviceHandler();

    // get a handle to the first haptic device
    if (!handler->getDevice(hapticDevice, 0)) {
        return 0;
    }

    // open a connection to haptic device
    if (!hapticDevice->open()) {
        return -1;
    }

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // retrieve information about the current haptic device
    chai3d::cHapticDeviceInfo info = hapticDevice->getSpecifications();
    std::cout << "Connected to: " << info.m_manufacturerName << " " << info.m_modelName << std::endl;
    return 1;
}

void Chai3dRosDriver::saturatAndSetForces(double x, double y, double z) {

    if (x < -MAX_FORCE) x = -MAX_FORCE;
    if (y < -MAX_FORCE) y = -MAX_FORCE;
    if (z < -MAX_FORCE) z = -MAX_FORCE;
    if (x > MAX_FORCE) x = MAX_FORCE;
    if (y > MAX_FORCE) y = MAX_FORCE;
    if (z > MAX_FORCE) z = MAX_FORCE;
    this->force.set(z, x, y);
}

void Chai3dRosDriver::forceCallback(const geometry_msgs::Vector3::ConstPtr &data) {

       /*<0.008 primul prag doar aici misca cucu
        *>0.12 al doilea prag aici se simte fortele deja, dar fara msicare
	    *>0.45 deadman stop aici nu e nimic, e un vid inmasurabil, nonconceptibil, inodor, insipid, si incolor, e doar o farama din ce a fost odata, MARCEL CIOLACU
        */

    if(this->gripperP < 0.4)
        this->goodForce.set(data->x, data->y, data->z);  
    else
        this->goodForce.set(0.0,0.0,0.0); 
}
void Chai3dRosDriver::LinearMotionCallback(const geometry_msgs::Vector3::ConstPtr &data){
    this->LinearMotionCondition.set(data->x, data->y, data->z);
}

void Chai3dRosDriver::publishFalconData() {

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok()) {

        geometry_msgs::Vector3Stamped pos;
        geometry_msgs::Vector3Stamped velocity;
        std_msgs::Int8MultiArray but;
        std_msgs::Float32 r;
        geometry_msgs::Vector3Stamped angles;
        geometry_msgs::Twist gripper;
        
        pos.header.frame_id = velocity.header.frame_id = angles.header.frame_id = ros::this_node::getName();
        pos.header.stamp = velocity.header.stamp = angles.header.stamp = ros::Time::now();

        pos.vector.x = this->position.y();
        pos.vector.y = this->position.z();
        pos.vector.z = this->position.x();

        velocity.vector.x = this->velocity.y();
        velocity.vector.y = this->velocity.z();
        velocity.vector.z = this->velocity.x();

        but.data.push_back(buttons[0]);
        but.data.push_back(buttons[1]);
        but.data.push_back(buttons[2]);
        but.data.push_back(buttons[3]);
        
        r.data = freqCounterHaptics.getFrequency();

        geometry_msgs::Twist posit;

        posit.linear.x = this->position.x()*1000;
        posit.linear.y = this->position.y()*1000;
        posit.linear.z = this->position.z()*1000;

        posit.angular.x = this->orientation[0];
        posit.angular.y = this->orientation[1];
        posit.angular.z = this->orientation[2];

        gripper.linear.x = this->gripperP;
        gripper.angular.x = this->gripperV;

        this->position_pub.publish(posit);
        this->velocity_pub.publish(velocity);
        this->buttons_pub.publish(but);
        this->rate_pub.publish(r);
        this->gripper_pub.publish(gripper);

        if(forceConsumed) {
            force.set(0.0f, 0.0f, 0.0f);
        }

        this->loopRate.sleep();
    }
    spinner.stop();
    hapticLoop = false;
}

void Chai3dRosDriver::startChai3dRosNode() {

    boost::thread hapticsThread(boost::bind(&Chai3dRosDriver::falconCallback, this));

    publishFalconData();

    hapticsThread.join();

    cleanUpFalcon();
}

void Chai3dRosDriver::cleanUpFalcon() {

    hapticDevice->close();
    delete(handler);
}

void Chai3dRosDriver::falconCallback() {

    double gripForc = 0;
    // main haptic simulation loop
    while (hapticLoop) {

        chai3d::cVector3d currentPos;
        chai3d::cVector3d currentForce;
        chai3d::cVector3d currentVel;
        chai3d::cMatrix3d currentOr;
        chai3d::cVector3d linearVelocity;
        double gripperAngle;
        double gripperSpeed;

        hapticDevice->getLinearVelocity(linearVelocity);
        hapticDevice->setEnableGripperUserSwitch(true);
        
        hapticDevice->getPosition(currentPos);
        hapticDevice->getRotation(currentOr);

        hapticDevice->getGripperAngleRad(gripperAngle);
        hapticDevice->getGripperAngularVelocity(gripperSpeed);

        double r11 = currentOr.getRow(0).x();
        double r12 = currentOr.getRow(0).y();
        double r13 = currentOr.getRow(0).z();

        double r21  = currentOr.getRow(1).x();
        double r22  = currentOr.getRow(1).y();
        double r23  = currentOr.getRow(1).z();

        double r31  = currentOr.getRow(2).x();
        double r32  = currentOr.getRow(2).y();
        double r33  = currentOr.getRow(2).z();

        _Float64 euler1 = atan2((( - r23 / sqrt((1 - pow(r13, 2))))), (r33 / sqrt(1 - pow(r13, 2))));
        _Float64 euler2 = atan2(r13, sqrt((1 - pow(r13,2))));
        _Float64 euler3 = atan2(( - r12 / sqrt((1 - pow(r13, 2)))), (r11 / sqrt((1 - pow(r13, 2)))));
        
        euler1 = euler1*180/3.14;
        euler2 = euler2*180/3.14;
        euler3 = euler3*180/3.14;

        #ifdef DEBUGG
        std::cout<<gripperAngle<<" "<<gripperSpeed<<'\n';
        #endif

        // read user-switch status (button 0)
        bool button0, button1, button2, button3;
        button0 = false;
        button1 = false;
        button2 = false;
        button3 = false;

        hapticDevice->getUserSwitch(0, button0);
        hapticDevice->getUserSwitch(1, button1);
        hapticDevice->getUserSwitch(2, button2);
        hapticDevice->getUserSwitch(3, button3);

        position = currentPos;
        velocity = currentVel;
	    orientation[0] = euler1;
        orientation[1] = euler2;
        orientation[2] = euler3;

        this->buttons[0] = button0;
        this->buttons[1] = button1;
        this->buttons[2] = button2;
        this->buttons[3] = button3;

        // signal frequency counter
        freqCounterHaptics.signal(1);

        /**
         ************ Apply forces *************
         */

        chai3d::cVector3d desiredPosition;

        desiredPosition.set(0.0, 0.0, 0.0);

        #ifdef DEBUGG
        std::cout<<this->LinearMotionCondition.x()<<" "<<this->LinearMotionCondition.y()<<" "<<this->LinearMotionCondition.z()<<" \n";
        std::cout<<euler1<<" "<<euler2<<" "<<euler3<<'\n';
        #endif

        chai3d::cVector3d torque;
        torque.set(0.0, 0.0, 0.0);
      
        //currentForce.set(received_forces[0], received_forces[1], received_forces[2]);
        //currentForce.set(this->received_forces[0],this->received_forces[1],this->received_forces[2]);
        currentForce = goodForce;
        double Kpx, Kpy, Kpz;
        if      (LinearMotionCondition.x() == 1) Kpx = 500; 
        else if (gripperAngle > 0.45 )           Kpx = 25;
             else                                Kpx = 0;
        if      (LinearMotionCondition.y() == 1) Kpy = 500;
        else if (gripperAngle > 0.45 )           Kpy = 25;
             else                                Kpy = 0;
        if      (LinearMotionCondition.z() == 1) Kpz = 500;
        else if (gripperAngle > 0.45 )           Kpz = 25;
             else                                Kpz = 0;


        chai3d::cVector3d diff = (desiredPosition - currentPos);
        
        chai3d::cVector3d forceField;// = Kp * diff
        forceField.set(Kpx*diff.x(), Kpy * diff.y(), Kpz * diff.z() );
        currentForce.add(forceField);


        gripperP = gripperAngle;
        gripperV = gripperSpeed;
        
	    //<0.008 primul prag doar aici misca cucu
        //>0.12 al doilea prag aici se simte fortele deja, dar fara msicare
	    //>0.45 deadman stop aici nu e nimic, e un vid inmasurabil, nonconceptibil, inodor, insipid, si incolor, e doar o farama din ce a fost odata, MARCEL CIOLACU

        if(gripperAngle >0.45 ) gripForc = 2.0;
	    if(gripperAngle <0.35 ) gripForc = 0.5;

        forceConsumed = false;

        if (forceOutput) {
           // std::cout << "Force: " << rosDriver->getForce() << std::endl;
            hapticDevice->setForceAndTorqueAndGripperForce(currentForce,torque,gripForc);
            //forceConsumed = true;
        } else {
            hapticDevice->setForce(chai3d::cVector3d(0.0f, 0.0f, 0.0f));
        }
    }
}
