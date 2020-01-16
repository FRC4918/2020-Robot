/*
 * Robot.cpp
 * This is FRC Team 4918's 2020 Robot code.
 * Original creation on 15jan2020:
 */


// #include <frc/WPILib.h>  // uncomment to include everything
#include "ctre/Phoenix.h"
#include "frc/AnalogInput.h"
#include "frc/Compressor.h"
#include "frc/DigitalInput.h"
#include "frc/DigitalOutput.h"
#include "frc/DigitalSource.h"
#include "frc/DoubleSolenoid.h"
#include "frc/Joystick.h"
#include "frc/Servo.h"
#include "frc/Solenoid.h"
#include "frc/TimedRobot.h"
#include "frc/Timer.h"
#include "frc/RobotDrive.h"
#include "frc/drive/DifferentialDrive.h"
#include "cameraserver/CameraServer.h"
#include "vision/VisionRunner.h"
#include <iostream>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <unistd.h>
#include <sstream>

#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

using std::cout;
using std::endl;


class Robot : public frc::TimedRobot {
 public:
   Robot() {
      m_robotDrive.SetExpiration( 0.1 );
      m_timer.Start();
   }

   /*
    * The RobotInit() function will be called once when the robot is
    * first turned on or reset.
    */
   void RobotInit() override {
      m_chooser.SetDefaultOption( kAutoNameDefault, kAutoNameDefault );
      m_chooser.AddOption(        kAutoNameCustom,  kAutoNameCustom );
      frc::SmartDashboard::PutData( "Auto Modes", &m_chooser );
   }

   /*
    * The RobotPeriodic() function is called after every robot packet,
    * no matter the operating mode.  Use this for items like diagnostics that
    * you want run during disabled, autonomous, teleoperated and test modes.
    *
    * This runs after the mode specific periodic functions, but before
    * LiveWindow and SmartDashboard integrated updating.
    */
   void RobotPeriodic() override {

   }

   /*
    * The AutonomousInit() function will be called once at the beginning of
    * Autonomous mode.
    */
   void AutonomousInit() override {
      m_timer.Reset();
      m_timer.Start();
         /*
          * The following code (along with the chooser code above) shows how
          * to select between different autonomous modes using the dashboard.
          *
          * You can add additional auto modes by adding additional comparisons
          * to the if-else structure below with additional strings.
          * Make sure to add them to the chooser code above as well.
          */
      m_autoSelected = m_chooser.GetSelected();
      cout << "Auto selected: " << m_autoSelected << endl;

      if ( m_autoSelected == kAutoNameCustom ) {
         // Custom Auto goes here
      } else {
         // Default Auto goes here
      }
   }

   /*
    * The AutonomousPeriodic() function will be called every 20 milliseconds
    * throughout Autonomous mode.
    */
   void AutonomousPeriodic() override {

      if ( m_autoSelected == kAutoNameCustom ) {
         // Custom Auto goes here
      } else {
         // Default Auto goes here
      }

      // Drive for 2 seconds
      if (m_timer.Get() < 2.0) {
         // Drive forwards half speed
         m_robotDrive.ArcadeDrive(-0.5, 0.0);
      } else {
         // Stop robot
         m_robotDrive.ArcadeDrive(0.0, 0.0);
      }
   }

   /*
    * The TeleopInit() function will be called once at the beginning of
    * Teleop mode.
    */
   void TeleopInit() override {
      // Initialization code for Teleop mode goes here
   }

   /*
    * The TeleopPeriodic() function will be called every 20 millisec
    * throughout Teleop mode.
    */
   void TeleopPeriodic() override {
      // continual operation code for Teleop mode goes here

      // Drive with arcade style (use right stick)
      m_robotDrive.ArcadeDrive( m_stick.GetY(), m_stick.GetX() );
   }

   void TestPeriodic() override {

   }

 private:
   // Robot drive system
   WPI_TalonSRX m_left{  0 };   // Left  side drive motor
   WPI_TalonSRX m_right{ 1 };   // Right side drive motor   
   frc::DifferentialDrive m_robotDrive{m_left, m_right};

   frc::Joystick m_stick{0};
   frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
   frc::Timer m_timer;

   // The chooser for the autonomous routines
   frc::SendableChooser<std::string> m_chooser;
   const std::string kAutoNameDefault = "Default";
   const std::string kAutoNameCustom = "My Auto";
   std::string m_autoSelected;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
