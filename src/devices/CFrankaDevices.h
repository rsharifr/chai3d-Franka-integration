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
    \version   3.2.0 $Rev: 1875 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CFrankaDeviceH
#define CFrankaDeviceH
//------------------------------------------------------------------------------
#if defined(C_ENABLE_FRANKA_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
#include <franka/duration.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include "system/CThread.h"
#include <thread> // Todo: REMOVE
#include <mutex>    // for conflict resolution in multi-threading


//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d
{
    //------------------------------------------------------------------------------

    //==============================================================================
    /*!
        \file       CFrankaDevice.h

        \brief
        Implements support for Franka devices.
    */
    //==============================================================================

    //------------------------------------------------------------------------------
    class cFrankaDevice;
    typedef std::shared_ptr<cFrankaDevice> cFrankaDevicePtr;
    //------------------------------------------------------------------------------

    //==============================================================================
    /*!
        \class      cFrankaDevice
        \ingroup    devices

        \brief
        This class is a interface to support Franka devices.

        \details
        This class provides the basics to easily interface CHAI3D to Franka devices. \n\n

        Simply follow the 11 commented step in file CFrankaDevices.cpp
        and complete the code accordingly.
        Depending of the numbers of degrees of freedom of your device, not
        all methods need to be implemented. For instance, if your device
        does not provide any rotation degrees of freedom, simply ignore
        the getRotation() method. Default values will be returned correctly
        if these are not implemented on your device. In the case of rotations
        for instance, the identity matrix is returned.\n\n

        You may also rename this class in which case you will also want to
        customize the haptic device handler to automatically detect your device.
        Please consult method update() of the cHapticDeviceHandler class
        that is located in file CHapticDeviceHandler.cpp .
        Simply see how the haptic device handler already looks for
        device of type cFrankaDevice.\n\n

        If you are encountering any problems with your implementation, check
        for instance file cDeltaDevices.cpp which implement supports for the
        Force Dimension series of haptic devices. In order to verify the implementation
        use the 01-device example to get started. Example 11-effects is a great
        demo to verify how basic haptic effects may behave with you haptic
        devices. If you do encounter vibrations or instabilities, try reducing
        the maximum stiffness and/or damping values supported by your device.
        (see STEP-1 in file CFrankaDevice.cpp).\n

        Make  sure that your device is also communicating fast enough with
        your computer. Ideally the communication period should take less
        than 1 millisecond in order to reach a desired update rate of at least 1000Hz.
        Problems can typically occur when using a slow serial port (RS232) for
        instance.\n
    */
    //==============================================================================
    class cFrankaDevice : public cGenericHapticDevice
    {
        //--------------------------------------------------------------------------
        // CONSTRUCTOR & DESTRUCTOR:
        //--------------------------------------------------------------------------

    public:
        //! Constructor of cFrankaDevice.
        cFrankaDevice(unsigned int a_deviceNumber = 0);

        //! Destructor of cFrankaDevice.
        virtual ~cFrankaDevice();

        //! Shared cFrankaDevice allocator.
        static cFrankaDevicePtr create(unsigned int a_deviceNumber = 0) { return (std::make_shared<cFrankaDevice>(a_deviceNumber)); }

        //--------------------------------------------------------------------------
        // PUBLIC METHODS:
        //--------------------------------------------------------------------------

    public:
        //! This method opens a connection to the haptic device.
        virtual bool open();

        //! This method closes the connection to the haptic device.
        virtual bool close();

        //! This method calibrates the haptic device.
        virtual bool calibrate(bool a_forceCalibration = false);

        //! This method returns the position of the device.
        virtual bool getPosition(cVector3d &a_position);

        //! This method returns the orientation frame of the device end-effector.
        virtual bool getRotation(cMatrix3d &a_rotation);

        //! This method returns the gripper angle in radian [rad].
        virtual bool getGripperAngleRad(double &a_angle);

        //! This method returns the status of all user switches [__true__ = __ON__ / __false__ = __OFF__].
        virtual bool getUserSwitches(unsigned int &a_userSwitches);

        //! This method sends a force [N] and a torque [N*m] and gripper force [N] to the haptic device.
        virtual bool setForceAndTorqueAndGripperForce(const cVector3d &a_force, const cVector3d &a_torque, double a_gripperForce);

        //--------------------------------------------------------------------------
        // PUBLIC STATIC METHODS:
        //--------------------------------------------------------------------------

    public:
        //! This method returns the number of devices available from this class of device.
        static unsigned int getNumDevices();

        //--------------------------------------------------------------------------
        // PROTECTED METHODS:
        //--------------------------------------------------------------------------

    protected:
        void setDefaultBehavior(franka::Robot &);
        std::function<franka::Torques(const franka::RobotState &, franka::Duration)> haptic_control_callback;
        Eigen::Vector3d get_position(const franka::RobotState &);
        Eigen::Matrix3d get_rotationMatrix(const franka::RobotState &);
        Eigen::VectorXd calc_jointTorques(const franka::RobotState &);
        void getRobotInitialStates();

        //--------------------------------------------------------------------------
        // PROTECTED MEMBERS:
        //--------------------------------------------------------------------------

        ////////////////////////////////////////////////////////////////////////////
        /*
            INTERNAL VARIABLES:

            If you need to declare any local variables or methods for your device,
            you may do it here below.
        */
        ////////////////////////////////////////////////////////////////////////////

    protected:
        //! A short description of my variable
        franka::Robot *fr3;
        franka::Model model;
        double internal_time;
        Eigen::Vector3d initial_EE_position;
        Eigen::Vector3d current_EE_position;
        Eigen::Matrix3d initial_EE_rotationMatrix;
        Eigen::Matrix3d current_EE_rotationMatrix;
        Eigen::VectorXd initial_tau_ext;
        Eigen::Vector3d cartesian_force;
        Eigen::Vector3d cartesian_torque;
        bool hapticLoopRunning;
        bool firstRunOfHapticLoop;

        std::unique_ptr<std::thread> FR3_hapticThread; // Thread for haptic loop

        std::mutex mtx;  // Global mutex for data synchronization

    };

    //------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
#endif // C_ENABLE_FRANKA_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif // #ifndef CFrankaDevice
//------------------------------------------------------------------------------