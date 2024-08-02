//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://www.chai3d.org>
    \author    Your name, institution, or company name.
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/CFrankaDevices.h"
#include <franka/robot.h>
#include <iostream> // TODO: REMOVE IF NOT NEEDED
#include <thread>   // TODO: REMOVE IF NOT NEEDED
#include <chrono>   // TODO: REMOVE IF NOT NEEDED
//------------------------------------------------------------------------------
#if defined(C_ENABLE_FRANKA_DEVICE_SUPPORT)
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
/*
    INSTRUCTION TO IMPLEMENT YOUR OWN CUSTOM DEVICE:

    Please review header file CFrankaDevice.h for some initial
    guidelines about how to implement your own haptic device using this
    template.

    When ready, simply completed the next 11 documented steps described here
    below.
*/
////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------
namespace chai3d
{
    //------------------------------------------------------------------------------

    //==============================================================================
    /*!
        Constructor of CFrankaDevice.
    */
    //==============================================================================
    cFrankaDevice::cFrankaDevice(unsigned int a_deviceNumber)
        : fr3(new franka::Robot("172.16.0.2")), // Initialize fr3. TODO FIX
          model(fr3->loadModel())               // Initialize model with fr3

    {
        // the connection to your device has not yet been established.
        m_deviceReady = false;
        ////////////////////////////////////////////////////////////////////////////
        /*
            STEP 1:

            Here you should define the specifications of your device.
            These values only need to be estimates. Since haptic devices often perform
            differently depending of their configuration withing their workspace,
            simply use average values.
        */
        ////////////////////////////////////////////////////////////////////////////

        //--------------------------------------------------------------------------
        // NAME: Franka FR3
        //--------------------------------------------------------------------------

        // haptic device model (see file "CGenericHapticDevice.h")
        m_specifications.m_model = C_HAPTIC_DEVICE_FRANKA;

        // name of the device manufacturer, research lab, university.
        m_specifications.m_manufacturerName = "Franka Emika";

        // name of your device
        m_specifications.m_modelName = "Franka FR3";

        //--------------------------------------------------------------------------
        // CHARACTERISTICS: (The following values must be positive or equal to zero)
        //--------------------------------------------------------------------------

        // the maximum force [N] the device can produce along the x,y,z axis.
        m_specifications.m_maxLinearForce = 30.0; // [N] FR3's payload is 3kg

        // the maximum amount of torque your device can provide arround its
        // rotation degrees of freedom.
        m_specifications.m_maxAngularTorque = 1.0; // [N*m]

        // the maximum amount of torque which can be provided by your gripper
        m_specifications.m_maxGripperForce = 3.0; // [N]

        // the maximum closed loop linear stiffness in [N/m] along the x,y,z axis
        m_specifications.m_maxLinearStiffness = 3000; // default 1000.0;  Franka's maximum Cartesian stiffness 3000 N/m

        // the maximum amount of angular stiffness 
        m_specifications.m_maxAngularStiffness = 1.0; // [N*m/Rad] default 1.0; // Franka's maximum RPY stiffness 300 N*m/Rad

        // the maximum amount of stiffness supported by the gripper
        m_specifications.m_maxGripperLinearStiffness = 1000; // [N*m] // Franka's hand does not have load sensing.

        // the radius of the physical workspace of the device (x,y,z axis)
        m_specifications.m_workspaceRadius = 0.3; // [m]

        // the maximum opening angle of the gripper
        m_specifications.m_gripperMaxAngleRad = cDegToRad(30.0);

        ////////////////////////////////////////////////////////////////////////////
        /*
            DAMPING PROPERTIES:

            Start with small values as damping terms can be highly sensitive to
            the quality of your velocity signal and the spatial resolution of your
            device. Try gradually increasing the values by using example "01-devices"
            and by enabling viscosity with key command "2".
        */
        ////////////////////////////////////////////////////////////////////////////

        // Maximum recommended linear damping factor Kv
        m_specifications.m_maxLinearDamping = 1.0; // default 20.0;   // [N/(m/s)]

        //! Maximum recommended angular damping factor Kv (if actuated torques are available)
        m_specifications.m_maxAngularDamping = 0.0; // [N*m/(Rad/s)]

        //! Maximum recommended angular damping factor Kv for the force gripper. (if actuated gripper is available)
        m_specifications.m_maxGripperAngularDamping = 0.0; // [N*m/(Rad/s)]

        //--------------------------------------------------------------------------
        // CHARACTERISTICS: (The following are of boolean type: (true or false)
        //--------------------------------------------------------------------------

        // does your device provide sensed position (x,y,z axis)?
        m_specifications.m_sensedPosition = true;

        // does your device provide sensed rotations (i.e stylus)?
        m_specifications.m_sensedRotation = true;

        // does your device provide a gripper which can be sensed?
        m_specifications.m_sensedGripper = true;

        // is you device actuated on the translation degrees of freedom?
        m_specifications.m_actuatedPosition = true;

        // is your device actuated on the rotation degrees of freedom?
        m_specifications.m_actuatedRotation = true;

        // is the gripper of your device actuated?
        m_specifications.m_actuatedGripper = true;

        // can the device be used with the left hand?
        m_specifications.m_leftHand = true;

        // can the device be used with the right hand?
        m_specifications.m_rightHand = true;

        ////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////
        initial_tau_ext = Eigen::VectorXd::Zero(7);
        cartesian_force.setZero();
        cartesian_torque.setZero();

        fr3 = nullptr;
        try
        {
            fr3 = new franka::Robot("172.16.0.2"); // TODO: make it an input argument
            cFrankaDevice::setDefaultBehavior(*fr3);
            model = fr3->loadModel();
            m_deviceAvailable = true;
        }
        catch (const std::exception &ex)
        {
            // print exception
            std::cout << ex.what() << std::endl;
            m_deviceAvailable = false;
        }
        // std::cout << "in cFrankaDevice: Franka Robot found and conncection made successfully ." << std::endl;


        // define callback for the FR3's torque control loop
        hapticLoopRunning = false;
        firstRunOfHapticLoop = true;
        haptic_control_callback = [this](const franka::RobotState &robot_state, franka::Duration period) -> franka::Torques
        {
            if (firstRunOfHapticLoop)
            {
                std::lock_guard<std::mutex> guard(mtx);

                hapticLoopRunning = true;
                firstRunOfHapticLoop = false;
            }
            if (hapticLoopRunning)
            {
                internal_time += period.toSec();

                franka::RobotState robot_state_copy = robot_state;
                if (internal_time > 0 && (get_position(robot_state_copy) - initial_EE_position).norm() > 0.30)
                {
                    throw std::runtime_error("Aborting! Too far away from starting pose!");
                }

                {
                    std::lock_guard<std::mutex> guard(mtx);

                    // Update the robot's end effector position from the measurements
                    current_EE_position = get_position(robot_state_copy);

                    // Update the robot's end effector rotation from the measurements
                    current_EE_rotationMatrix = get_rotationMatrix(robot_state_copy);
                }
                // Calculate the joint torques from the interactions in the virtual environment
                Eigen::VectorXd tau_cmd(6);
                tau_cmd << calc_jointTorques(robot_state_copy);

                // Send the joint torques to the robot
                std::array<double, 7> tau_cmd_array{};
                Eigen::VectorXd::Map(&tau_cmd_array[0], 7) = tau_cmd;
                return tau_cmd_array;
            }
            else
            {
                // throw std::runtime_error("Torque control loop stopped running.");
                std::cout << "torque control loop is off" << std::endl;
                exit(1); // This is the only way I could terminate the torque control loop!
                std::array<double, 7> tau_cmd_array{};

                return tau_cmd_array;
            }
        };
    }

    //==============================================================================
    /*!
        Destructor of cFrankaDevice.
    */
    //==============================================================================
    cFrankaDevice::~cFrankaDevice()
    {
        // close connection to device
        if (m_deviceReady)
        {
            close();
        }
    }

    //==============================================================================
    /*!
        This method opens a connection to your device.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cFrankaDevice::open()
    {
        // check if the system is available
        if (!m_deviceAvailable)
        {
            // std::cout << "No device is available." << std::endl;
            return (C_ERROR);
        }

        // if system is already opened then return
        if (m_deviceReady)
        {
            // std::cout << "Device already open and ready." << std::endl;
            return (C_SUCCESS);
        }

        // Read initial position, orientation and bias torques like initial contacts from the robot
        getRobotInitialStates();

        // This is where the FR3's torque control loop is initiated
        internal_time = 0.0;
        std::function<void()> lambdaFun = [this]() { fr3->control(cFrankaDevice::haptic_control_callback); };
        FR3_hapticThread = std::unique_ptr<std::thread>(new std::thread(lambdaFun));
        FR3_hapticThread->detach();
        int waitTime = 100; // in ms
        cSleepMs(waitTime);

        if (hapticLoopRunning)
        {
            std::cout << "Franka torque control thread is running." << std::endl;
            // std::cout << "Returning true in open() function." << std::endl;
            m_deviceReady = true;
            return (C_SUCCESS);
        }
        else
        {
            std::cout << "Franka torque control thread failed to start after " << waitTime << " ms wait." << std::endl;
            // std::cout << "Returning false in open() function." << std::endl;
            m_deviceReady = false;
            return (C_ERROR);
        }
    }

    //==============================================================================
    /*!
        This method closes the connection to your device.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cFrankaDevice::close()
    {
        // check if the system has been opened previously
        if (!m_deviceReady)
            return (C_ERROR);

        {
            std::lock_guard<std::mutex> guard(mtx);
            hapticLoopRunning = false;
            std::cout << "Stopping the haptic loop and exiting with code (1)." << std::endl;
        }
        if (FR3_hapticThread && FR3_hapticThread->joinable())
        {
            FR3_hapticThread->join();
        }
        cSleepMs(100);

        // update status
        m_deviceReady = false;
        bool result = C_SUCCESS; // if the operation fails, set value to C_ERROR.
        return (result);
    }

    //==============================================================================
    /*!
        This method calibrates your device.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cFrankaDevice::calibrate(bool a_forceCalibration)
    {
        // check if the device is read. See step 3.
        if (!m_deviceReady)
            return (C_ERROR);

        // There is no need for calibration. The Franka robots automatically calibrate themselves.
        bool result = C_SUCCESS;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        return (result);
    }

    //==============================================================================
    /*!
        This method returns the number of devices available from this class of device.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    unsigned int cFrankaDevice::getNumDevices()
    {
        // TODO: How to get the number of devices connected to the computer?
        int numberOfDevices = 1; // At least set to 1 if a device is available.

        // numberOfDevices = getNumberOfDevicesConnectedToTheComputer();

        return (numberOfDevices);
    }

    //==============================================================================
    /*!
        This method returns the position of your device. Units are meters [m].

        \param   a_position  Return value.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cFrankaDevice::getPosition(cVector3d &a_position)
    {
        // check if the device is read. See step 3.
        if (!m_deviceReady)
            return (C_ERROR);

        bool result = C_SUCCESS;
        double robotX, robotY, robotZ;
        // store new position values
        {
            std::lock_guard<std::mutex> guard(mtx);

            robotX = current_EE_position(0) - initial_EE_position(0);
            robotY = current_EE_position(1) - initial_EE_position(1);
            robotZ = current_EE_position(2) - initial_EE_position(2);
        }
        // // std::cout << "In FrankaDevice.cpp: Robot position: " << robotX << " " << robotY << " " << robotZ << std::endl;
        a_position.set(robotX, robotY, robotZ);

        // estimate linear velocity
        estimateLinearVelocity(a_position);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        // exit
        return (result);
    }

    //==============================================================================
    /*!
        This method returns the orientation frame of your device end-effector

        \param   a_rotation  Return value.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cFrankaDevice::getRotation(cMatrix3d &a_rotation)
    {
        // check if the device is read. See step 3.
        if (!m_deviceReady)
            return (C_ERROR);
        {
            std::lock_guard<std::mutex> guard(mtx);

            // TODO: TAKE INTO ACCOUNT THE INITIAL ROTATION MATRIX AND POSSIBLE ROTATION OF THE END EFFECTOR
            cMatrix3d frame(current_EE_rotationMatrix);
            a_rotation = frame;
        }

        // estimate angular velocity
        estimateAngularVelocity(a_rotation);

        bool result = C_SUCCESS;

        // exit
        return (result);
    }

    //==============================================================================
    /*!
        This method returns the gripper angle in radian.

        \param   a_angle  Return value.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cFrankaDevice::getGripperAngleRad(double &a_angle)
    {
        // check if the device is read. See step 3.
        if (!m_deviceReady)
            return (C_ERROR);

        bool result = C_SUCCESS;

        // FR3 does not have a gripper. The Franka Hand can be used as a gripper.
        a_angle = 0.0;

        // estimate gripper velocity
        estimateGripperVelocity(a_angle);

        // exit
        return (result);
    }

    //==============================================================================
    /*!
        This method sends a force [N] and a torque [N*m] and gripper torque [N*m]
        to your haptic device.

        \param   a_force  Force command.
        \param   a_torque  Torque command.
        \param   a_gripperForce  Gripper force command.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cFrankaDevice::setForceAndTorqueAndGripperForce(const cVector3d &a_force,
                                                         const cVector3d &a_torque,
                                                         const double a_gripperForce)
    {
        // check if the device is read. See step 3.
        if (!m_deviceReady)
            return (C_ERROR);

        ////////////////////////////////////////////////////////////////////////////
        /*
            Note:
            For consistency, units must be in Newtons and Newton-meters
            If your device is placed in front of you, the x-axis is pointing
            towards you (the operator). The y-axis points towards your right
            hand side and the z-axis points up towards the sky.

            For instance: if the force = (1,0,0), the device should move towards
            the operator, if the force = (0,0,1), the device should move upwards.
            A torque (1,0,0) would rotate the handle counter clock-wise around the
            x-axis.
        */
        ////////////////////////////////////////////////////////////////////////////

        // store new force value.
        m_prevForce = a_force;
        m_prevTorque = a_torque;
        m_prevGripperForce = a_gripperForce;

        {
            std::lock_guard<std::mutex> guard(mtx);

            // prepare force/torque commands for the FR3 from a_force and a_torque that come from the virtual environment
            cartesian_force << a_force(0), a_force(1), a_force(2);
            cartesian_torque << a_torque(0), a_torque(1), a_torque(2);
        }
        bool result = C_SUCCESS;
        return (result);
    }

    //==============================================================================
    /*!
        This method returns status of all user switches
        [__true__ = __ON__ / __false__ = __OFF__].

        \param  a_userSwitches  Return the 32-bit binary mask of the device buttons.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cFrankaDevice::getUserSwitches(unsigned int &a_userSwitches)
    {
        // check if the device is read. See step 3.
        if (!m_deviceReady)
            return (C_ERROR);

        // FR3 does not have user switches
        a_userSwitches = 0;

        return (C_SUCCESS);
    }

    ////////////////////////////////////////////////////////////////////////////
    // ADDITIONAL METHODS:
    ////////////////////////////////////////////////////////////////////////////
    void cFrankaDevice::setDefaultBehavior(franka::Robot &robot)
    {
        // These are moninal upper and lower accelerations. Outside these ranges, the robot infers a "collision" with something
        robot.setCollisionBehavior(
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

        // Thses are the maximum allowed torques in the joints. Troques outside these range will evoke a "reflex response"
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    }

    Eigen::Vector3d cFrankaDevice::get_position(const franka::RobotState &robot_state)
    {
        return Eigen::Vector3d(robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                               robot_state.O_T_EE[14]);
    };

    Eigen::Matrix3d cFrankaDevice::get_rotationMatrix(const franka::RobotState &robot_state)
    {
        Eigen::Matrix4d O_T_EE = Eigen::Map<const Eigen::Matrix4d>(robot_state.O_T_EE.data());
        Eigen::Matrix3d rotationMatrix = O_T_EE.block<3, 3>(0, 0);
        return rotationMatrix;
    };

    Eigen::VectorXd cFrankaDevice::calc_jointTorques(const franka::RobotState &robot_state)
    {
        std::array<double, 7> gravity_array_current = model.gravity(robot_state);
        std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array_current.data());

        Eigen::VectorXd desired_force_torque(6), tau_d(7), tau_cmd(7), tau_ext(7);

        {
            std::lock_guard<std::mutex> guard(mtx);

            desired_force_torque << cartesian_force, cartesian_torque;
            tau_ext << tau_measured - gravity - initial_tau_ext;
        }
        tau_d << jacobian.transpose() * desired_force_torque;

        tau_cmd << tau_d; // tau_cmd may be modified to include PI control or something like that for better tracking
        return tau_cmd;
    };

    void cFrankaDevice::getRobotInitialStates()
    {
        // Read initial state of the robot and bias torques
        franka::RobotState initial_state = fr3->readOnce();

        std::array<double, 7> initial_gravity_array = model.gravity(initial_state);
        Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(initial_gravity_array.data());
        std::array<double, 7> tau_J_copy = initial_state.tau_J;
        Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(tau_J_copy.data());

        initial_tau_ext = initial_tau_measured - initial_gravity;

        initial_EE_position = get_position(initial_state);
        initial_EE_rotationMatrix = get_rotationMatrix(initial_state);
    }


    //------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
#endif // C_ENABLE_FRANKA_DEVICE_SUPPORT
//------------------------------------------------------------------------------
