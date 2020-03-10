/*
 * FRC Team 4918 2020 Competition Robot C/C++ code.
 * (C) Team 4918, the Roboctopi, original creation on 20feb2020.
 * This code was derived from the MentorBot code.
 */

// #include <frc/WPILib.h>  // uncomment to include everything
#include "ctre/Phoenix.h"
#include "frc/AnalogInput.h"
#include "frc/BuiltInAccelerometer.h"
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
#include <iomanip>
#include <vector>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <unistd.h>
#include <sstream>

using std::cout;
using std::endl;
using std::setw;
using std::setfill; 
using std::abs;             // so we can use "setfill('0') in cout streams
using namespace cv;

class Robot : public frc::TimedRobot {
 private:

         // Note for future: Need to add a gyro here.
   WPI_TalonSRX m_motorLSMaster{      0 };   // Left  side drive motor
   WPI_TalonSRX m_motorRSMaster{     15 };   // Right side drive motor   
   WPI_VictorSPX m_motorLSSlave1{     1 };   // Left  side slave motor
   WPI_VictorSPX m_motorLSSlave2{     2 };   // Left side slave motor
   WPI_VictorSPX m_motorRSSlave1{    14 };   // Right side slave motor
   WPI_VictorSPX m_motorRSSlave2{    13 };   // Right side slave motor
   WPI_TalonSRX m_motorTopShooter{    3 };   // top motor on shooter
   WPI_TalonSRX m_motorBotShooter{   12 };  // bottom motor on shooter
   WPI_TalonSRX m_motorClimberPole{   4 };  // telescoping pole motor
   WPI_VictorSPX m_motorConveyMaster{ 5 };  // conveyor motor 1
   WPI_VictorSPX m_motorConveySlave{ 10 };  // conveyor motor 2
   WPI_TalonSRX m_motorClimberWinch{ 11 };  // winch motor
   WPI_VictorSPX m_motorIntake{       6 };  // intake motor
   WPI_VictorSPX m_motorFlippyFlippy{ 9 }; //  motor to spin the color wheel 
   frc::Compressor m_compressor{      0 }; // compressor

   frc::Solenoid m_shiftingSolenoid{ 0, 7};
   frc::DoubleSolenoid m_flipperSolenoid{ 0, 1};

   

   // PigeonIMU    pigeonIMU{ 1 };   // uncomment all pigeon code when it is connected

   frc::DigitalInput conveyorDIO0{0};
   frc::DigitalInput conveyorDIO1{1};
   
   int iAutoCount;
   float drivePowerFactor = 0.8;
   frc::Joystick m_stick{0};
   frc::Joystick m_console{1};
   // frc::DifferentialDrive m_drive{ m_motorLSMaster, m_motorRSMaster };
   //frc::PowerDistributionPanel pdp{0};
   frc::AnalogInput distSensor0{0};
   frc::BuiltInAccelerometer RoborioAccel{};

   std::shared_ptr<NetworkTable> limenttable =
               nt::NetworkTableInstance::GetDefault().GetTable( "limelight" );

   struct sState {
      double joyX;
      double joyY;
      double joyZ;
      double conX;
      double conY;
      bool   joyButton[12];
      bool   conButton[13];
      int    iLSMasterPosition;
      int    iRSMasterPosition;
      int    iLSMasterVelocity;
      int    iRSMasterVelocity;
      double yawPitchRoll[3];  // data from Pigeon IMU
      double initialYaw;
      bool   powercellInIntake;
      bool   powercellInPosition5;
      bool   highGear;
   } sCurrState, sPrevState;

   struct sMotorState {
      double targetVelocity_UnitsPer100ms;
      double sensorVmax, sensorVmin;
   };

   struct sMotorState LSMotorState, RSMotorState, TSMotorState, BSMotorState;

   bool aBooleanVariable = false;    // just an example of a boolean variable
   int    iCallCount = 0;
   double dTimeOfLastCall = 0.0;

 public:

   static struct sPowercellOnVideo {
      bool SeenByCamera;
      int  X;
      int  Y;
      int  Radius;
      bool SwitchToColorWheelCam;
      bool TestMode;
   } powercellOnVideo;

 private:
                 /* limelight variables: x: offset from centerline,         */
                 /*                      y: offset from centerline,         */
                 /*                      a: area of target, % (0-100),      */
                 /*                      v: whether the data is valid,      */
                 /*                      s: skew or rotation, deg (-90-0).  */
   double limex, limey, limea, limev, limes;

      /*---------------------------------------------------------------------*/
      /* VisionThread()                                                      */
      /* This function executes as a separate thread, to take 640x480-pixel  */
      /* video frames from the USB video camera, change to grayscale,        */
      /* and send to the DriverStation. It is documented here:               */
      /* https://docs.wpilib.org/en/latest/docs/software/vision-processing/  */
      /*         introduction/using-the-cameraserver-on-the-roborio.html     */
      /* http://opencv-cpp.blogspot.com/2016/10/                             */
      /*        object-detection-and-tracking-color-separation.html          */
      /*---------------------------------------------------------------------*/
   static void VisionThread() {
      static double dTimeOfLastCall;
      static bool   prevSwitchToColorWheelCam = false;

      int iBiggestCircleIndex = -1;
      int iBiggestCircleRadius = -1;

      bool bDiagnosticMode = false;

      cs::UsbCamera camera =
                 frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
         //camera.SetResolution( 640, 480 );   // too detailed and slow
         //camera.SetResolution( 160, 120 );   // too coarse
      camera.SetResolution( 320, 240 );        // just right
      cs::UsbCamera camera2 =
                  frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
      camera2.SetResolution( 320, 240 );

      powercellOnVideo.SeenByCamera = false;  // Make sure no other code thinks
      powercellOnVideo.X = 0;                 // we see a powercell until we
      powercellOnVideo.Y = 0;                 // actually see one!
      powercellOnVideo.Radius = -1;
      powercellOnVideo.SwitchToColorWheelCam = false;
      powercellOnVideo.TestMode = false;

      cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
      cs::CvSource outputStreamStd =
              frc::CameraServer::GetInstance()->PutVideo( "Gray", 320, 240 );
              //frc::CameraServer::GetInstance()->GetVideo();
      cv::Mat source;
      cv::Mat output;
      cv::Mat hsvImg;
      cv::Mat threshImg;
      cv::Mat *pOutput;

      std::vector<Vec3f> v3fCircles;      // 3-element vector of floats;
                                          // this will be the pass-by reference
                                          // output of HoughCircles()
         /* HUE for YELLOW is 21-30.                                         */
         /* Adjust Hue depending on the lighting condition                   */
         /* of the environment as well as the surface of the object.         */
      int     lowH = 19;       // Set Hue
      int     highH = 37;      // (orig: 30)  

      int     lowS = 0;        // Set Saturation (orig: 200)
      int     highS = 255;

      int     lowV = 0;         // Set Value (orig: 102)
      int     highV = 255;      // (orig: 225)

      cvSink.SetSource( camera );
      while( true ) {
         static int iFrameCount = 0;
         static int iLoopCount  = 0;

         if ( powercellOnVideo.SwitchToColorWheelCam &&
              !prevSwitchToColorWheelCam ) {
            cvSink.SetSource( camera2 );       // switch to color wheel camera
            prevSwitchToColorWheelCam = powercellOnVideo.SwitchToColorWheelCam;
         } else if ( !powercellOnVideo.SwitchToColorWheelCam &&
                     prevSwitchToColorWheelCam ) {
            cvSink.SetSource( camera );     // switch back to powercell camera
            prevSwitchToColorWheelCam = powercellOnVideo.SwitchToColorWheelCam;
            bDiagnosticMode = !bDiagnosticMode;       // toggle diagnostic mode
            cout << "mel says diagnostic mode was changed" <<endl;
            cout << bDiagnosticMode <<endl; 
         }

                   // set the pointer to the frame to be sent to DriverStation
         if ( bDiagnosticMode ) {
            pOutput = &threshImg;   // diagnostic image (shows where yellow is)
         } else if ( powercellOnVideo.SwitchToColorWheelCam ) {
            pOutput = &source;          // full-color, direct from videocamera
         } else {
            pOutput = &output;            // gray-scale image, for low latency
         }

         usleep( 1000 );                               // wait one millisecond
         if ( cvSink.GrabFrame(source) )
         {
            dTimeOfLastCall = frc::GetTime();
            iFrameCount++;
            cvtColor( source, output, cv::COLOR_BGR2GRAY );
            cvtColor( source, hsvImg, cv::COLOR_BGR2HSV );
            cv::inRange( hsvImg, cv::Scalar(lowH, lowS, lowV),
                         cv::Scalar(highH, highS, highV), threshImg );
                                                                  //Blur Effect
            cv::GaussianBlur( threshImg, threshImg, cv::Size(9, 9), 0);
            cv::dilate( threshImg, threshImg, 0 );      // Dilate Filter Effect
            cv::erode( threshImg, threshImg, 0 );       // Erode  Filter Effect
                     // fill circles vector with all circles in processed image
                     // HoughCircles() is an algorithm for detecting circles  
            cv::HoughCircles( threshImg, v3fCircles, CV_HOUGH_GRADIENT,
                              2,          // dp: inverse accumulator resolution
                              threshImg.rows / 2,   // min dist between centers
                                                    // of the detected circles
                              100,          // param1 (edge detector threshold)
                              84,  // p2: increase this to reduce false circles
                              36,                      // minimum circle radius
                              64 );                    // maximum circle radius
                              // was: threshImg.rows / 4, 100, 50, 10, 800 );

            iBiggestCircleIndex = -1;     // init to an impossible index
            iBiggestCircleRadius = -1;    // init to an impossibly-small radius

                                                             // for each circle
            for ( unsigned int i = 0; i < v3fCircles.size(); i++ ) {

               if ( 0 == iFrameCount%6000 ) {          // every 10 seconds or so
                                      // Log the x and y position of the center
                                      // point of circle, and the radius.
                  std::cout << "Ball position X = " << v3fCircles[i][0] <<
                               ",\tY = " << v3fCircles[i][1] <<
                               ",\tRadius = " << v3fCircles[i][2] <<
                               " (" << v3fCircles[i][0] - threshImg.cols / 2 <<
                               ", " << threshImg.rows / 2 - v3fCircles[i][1] <<
                               ")" << endl;

//                  std::cout << "Video Frame Cols/Rows: ";
//                  std::cout << hsvImg.cols << "/" << hsvImg.rows << endl;
                  std::cout << "Center pixel H/S/V: ";
                                     // make a copy of the center pixel vector
                  cv::Vec3b pixel = hsvImg.at<cv::Vec3b>( hsvImg.rows/2,
                                                          hsvImg.cols/2);
                  std::cout << "pixel: " << (int)(pixel[0]) << "/" <<
                                            (int)(pixel[1]) << "/" <<
                                            (int)(pixel[2]) << endl;

                  std::cout << "Vision Processing duration: ";
                  std::cout << frc::GetTime() - dTimeOfLastCall << endl;
                                  // use frpc:Timer::GetFPGATimestamp() instead?
               }      // if on a 10-second boundary

                     // if a bigger circle has been found than any found before
               if ( iBiggestCircleRadius < (int)v3fCircles[i][2] ) {
                  iBiggestCircleIndex = i;
                  iBiggestCircleRadius = (int)v3fCircles[i][2];
               }

                        // draw small green circle at center of object detected
               cv::circle( *pOutput,             // draw on DriverStation image
                           cv::Point( (int)v3fCircles[i][0],       // center of
                                      (int)v3fCircles[i][1] ),     // circle
                           3,                     // radius of circle in pixels
                           cv::Scalar(0, 255, 0),                 // draw green
                           CV_FILLED );                            // thickness

                                      // draw red circle around object detected 
               cv::circle( *pOutput,          // draw on DriverStation image
                           cv::Point( (int)v3fCircles[i][0],       // center of
                                      (int)v3fCircles[i][1] ),     // circle
                           (int)v3fCircles[i][2], // radius of circle in pixels
                           cv::Scalar(0, 0, 255),                   // draw red
                           3 );                                    // thickness
            }      // for every circle found

            if ( -1 < iBiggestCircleIndex ) {  // if at least one ball was seen
                   // Then save all the info so other code can drive toward it.
                            // Convert X and Y positions so 0,0 is at center of
                            // camera image, with X increasing to the right and
                            // Y increasing up.
               powercellOnVideo.X = (int)v3fCircles[iBiggestCircleIndex][0] -
                                                         (int)threshImg.cols/2;
               powercellOnVideo.Y = (int)threshImg.rows/2 -
                                       (int)v3fCircles[iBiggestCircleIndex][1];
               powercellOnVideo.Radius = iBiggestCircleRadius;
               powercellOnVideo.SeenByCamera = true;
               if ( 0 == iFrameCount%60 ) { 
                  cout << "Powercell Seen flag " << powercellOnVideo.SeenByCamera <<
                  ": " << powercellOnVideo.X << "/" << powercellOnVideo.Y;
                  cout << ", " << powercellOnVideo.Radius  << "." << endl;
               }
            } else {
               powercellOnVideo.SeenByCamera = false;
            }

            if ( powercellOnVideo.TestMode ) {              // if in Test Mode
                           // display the H/S/V values every couple of seconds
               if ( 0 == iFrameCount%30 ) {
                  std::cout << "Center pixel H/S/V: ";
                                     // make a copy of the center pixel vector
                  cv::Vec3b pixel = hsvImg.at<cv::Vec3b>( hsvImg.rows/2,
                                                          hsvImg.cols/2 );
                  std::cout << "pixel: " << (int)(pixel[0]) << "/" <<
                                            (int)(pixel[1]) << "/" <<
                                            (int)(pixel[2]) << endl;
               }

                            // draw small blue circle at center of video frame
               cv::circle( *pOutput,                 // draw on original image
                           cv::Point( hsvImg.cols/2,        // center of frame
                                      hsvImg.rows/2 ),
                           6,                    // radius of circle in pixels
                           cv::Scalar( 255, 255, 255 ),          // draw white
                           2 );                                   // thickness
                                                       // draw small blue circle at center of video frame
               cv::circle( *pOutput,                 // draw on original image
                           cv::Point( hsvImg.cols/2,        // center of frame
                                      hsvImg.rows/2 ),
                           8,                    // radius of circle in pixels
                           cv::Scalar( 0, 0, 0 ),                // draw black
                           2 );                                   // thickness
            }      // if in Test Mode

            outputStreamStd.PutFrame( *pOutput );

            v3fCircles.clear();

         } // if we got an image
         iLoopCount++;
      }   // do while 
   }      // VisionThread() 

 public:

      /*---------------------------------------------------------------------*/
      /* SwitchCameraIfNecessary()                                           */
      /* This function switches USB cameras, in case they were initialized   */
      /* in a different order than usual.                                    */
      /*---------------------------------------------------------------------*/
   void SwitchCameraIfNecessary( void ) {
              // Switch cameras if console button 5 pressed.
      if ( !sPrevState.conButton[5] &&
            sCurrState.conButton[5]    ) {

         powercellOnVideo.SwitchToColorWheelCam =
                   !powercellOnVideo.SwitchToColorWheelCam;

         cout << "Switching camera to ";
         if ( powercellOnVideo.SwitchToColorWheelCam ) {
            cout << "ColorWheelCam" << endl;
         } else {
            cout << "PowercellCam" << endl;
         }
      }
   }


      /*---------------------------------------------------------------------*/
      /* GetAllVariables()                                                   */
      /* Retrieves all variable values from sensors, encoders,               */
      /* the limelight, etc.  It should be called at the beginning of        */
      /* every 20-millisecond tick.  Doing it this way, rather than          */
      /* having each function retrieve the values it needs when it needs     */
      /* them, should minimize CANbus traffic and keep the robot CPU fast.   */
      /*---------------------------------------------------------------------*/
   void GetAllVariables()  {

                                // use frc::Timer::GetFPGATimestamp() instead?
      dTimeOfLastCall = frc::GetTime();
//      cout << std::setw( 20 ) << std::setprecision( 16 ) << dTimeOfLastCall << " ";


      sPrevState = sCurrState;                  // save all previous variables

      sCurrState.joyX = m_stick.GetX();
      sCurrState.joyY = m_stick.GetY();
      sCurrState.joyZ = m_stick.GetZ();
      sCurrState.conX = m_console.GetX();
      sCurrState.conY = m_console.GetY();
      for ( int iLoopCount=1; iLoopCount<=11; iLoopCount++ ) {
         sCurrState.joyButton[iLoopCount] = m_stick.GetRawButton(iLoopCount);
         sCurrState.conButton[iLoopCount] = m_console.GetRawButton(iLoopCount);
      }
      sCurrState.conButton[12] = m_console.GetRawButton(12);

      sCurrState.iLSMasterPosition = m_motorLSMaster.GetSelectedSensorPosition();
      sCurrState.iRSMasterPosition = m_motorRSMaster.GetSelectedSensorPosition();
      sCurrState.iLSMasterVelocity = m_motorLSMaster.GetSelectedSensorVelocity();
      sCurrState.iRSMasterVelocity = m_motorRSMaster.GetSelectedSensorVelocity();

      // pigeonIMU.GetYawPitchRoll( sCurrState.yawPitchRoll );

                  // Record the positions of powercells in the conveyor system.
                  // The Digital I/O (DIO) connections are made to IR sensors,
                  // which return false if the IR beam is blocked (which means
                  // there is a powercell there) -- so we invert them here. 
      sCurrState.powercellInIntake    = !conveyorDIO0.Get();
      sCurrState.powercellInPosition5 = !conveyorDIO1.Get();

      limev = limenttable->GetNumber("tv",0.0);  // valid
      limex = limenttable->GetNumber("tx",0.0);  // x position
      limea = limenttable->GetNumber("ta",0.0);  // area
      limey = limenttable->GetNumber("ty",0.0);  // y position
      limes = limenttable->GetNumber("ts",0.0);  // skew
   }      // GetAllVariables()


      /*---------------------------------------------------------------------*/
      /* JoystickDisplay()                                                   */
      /* Display all the joystick values on the console log.                 */
      /*---------------------------------------------------------------------*/
   void JoystickDisplay( void ) {
         cout << "joy (y/x): " << setw(8) << sCurrState.joyY << "/" <<
                 setw(8) << sCurrState.joyX << endl;
   }


      /*---------------------------------------------------------------------*/
      /* IMUOrientationDisplay()                                             */
      /* Display all the yaw pitch & roll values on the console log.         */
      /*---------------------------------------------------------------------*/
   void IMUOrientationDisplay( void ) {
         cout << "pigeonIMU (yaw/pitch/roll): " <<
               sCurrState.yawPitchRoll[0] << "/" <<
               sCurrState.yawPitchRoll[1] << "/" <<
               sCurrState.yawPitchRoll[2] << endl;
//         cout << "pigeontemp: " << pigeonIMU.GetTemp() << endl; 
   }      // IMUOrientationDisplay()


      /*---------------------------------------------------------------------*/
      /* MotorDisplay()                                                      */
      /* Display all the current motor values for a specified motor on the   */
      /* console log.                                                        */
      /*---------------------------------------------------------------------*/
   void MotorDisplay( const char * cTitle,
                      WPI_TalonSRX & m_motor,
                      struct sMotorState & sMState ) {
      double motorVelocity = m_motor.GetSelectedSensorVelocity();
      cout << cTitle << " vel(min:max)tgt/% A (pos): ";
      cout << setw(5) << motorVelocity*600/4096;
      cout << "(" << setw(5) << sMState.sensorVmin*600/4096 << ":";
      cout <<        setw(5) << sMState.sensorVmax*600/4096 << ")";
      cout << setw(5) << sMState.targetVelocity_UnitsPer100ms*600/4096 << "/ ";
      cout << setw(3) << m_motor.GetMotorOutputPercent() << "% ";
      cout << setw(5) << m_motor.GetStatorCurrent() << "A ";
      cout << "(" << setw(10) << m_motor.GetSelectedSensorPosition() << ")";
      cout << endl;
      sMState.sensorVmin = sMState.sensorVmax = motorVelocity;
   }      // MotorDisplay()


      /*---------------------------------------------------------------------*/
      /* motorFindMinMaxVelocity()                                           */
      /* Keep a motor's Vmin and Vmax updated, for display on the            */
      /* console log.                                                        */
      /*---------------------------------------------------------------------*/
   void motorFindMinMaxVelocity( WPI_TalonSRX & m_motor,
                                 struct sMotorState & sMState ) {
      double motorVelocity = m_motor.GetSelectedSensorVelocity();
      if ( motorVelocity < sMState.sensorVmin ) {
         sMState.sensorVmin = motorVelocity;
      } else if ( sMState.sensorVmax < motorVelocity ) {
         sMState.sensorVmax = motorVelocity;
      }
   }      // motorFindMinMaxVelocity()


      /*---------------------------------------------------------------------*/
      /* Team4918Drive()                                                     */
      /* This function implements something similar to                       */
      /* frc::DifferentialDrive::ArcadeDrive() or CurvatureDrive, but        */
      /* customized for Team 4918's Robot.  In particular, it uses           */
      /* ControlMode::Velocity rather than ControlMode::PercentOutput        */
      /* to drive the motors.  So this should be much more precise for the   */
      /* drivers.                                                            */
      /* It uses desiredForward (-1.0 to +1.0) as the desired forward speed, */
      /* and desiredTurn (-1.0 to +1.0, positive to the right) for the       */
      /* desired turn rate.                                                  */
      /*---------------------------------------------------------------------*/
   void Team4918Drive( double desiredForward, double desiredTurn ) {
                  /* To drive the robot using the drive motor encoders,     */
                  /* specifying each motor separately, we must              */
                  /* convert the desired speed to units / 100ms,            */
                  /* because the velocity setpoint is in units/100ms.       */
                  /* For example, to convert 500 RPM to units / 100ms:      */
                  /* 4096 Units/Rev * 500 RPM / 600 100ms/min               */
                  /* So code to drive the robot at up to 500 rpm in either  */
                  /* direction could look like this:                        */
      double  leftMotorOutput = 0.0;
      double rightMotorOutput = 0.0;
      // m_drive.StopMotor();
      if ( ( -0.045 < desiredForward ) && ( desiredForward < 0.025 ) ) {
         desiredForward = 0.0;
      }
      if ( ( -0.105 < desiredTurn ) && ( desiredTurn < 0.025 ) ) {
         desiredTurn = 0.0;
      }
      leftMotorOutput  = -desiredForward - desiredTurn;
      rightMotorOutput = +desiredForward - desiredTurn;
      leftMotorOutput  = std::min(  1.0, leftMotorOutput );
      rightMotorOutput = std::min(  1.0, rightMotorOutput );
      leftMotorOutput  = std::max( -1.0, leftMotorOutput );
      rightMotorOutput = std::max( -1.0, rightMotorOutput );
      m_motorLSMaster.Set( ControlMode::Velocity, 
                          leftMotorOutput  * 5200.0 * 4096 / 600 );
      m_motorRSMaster.Set( ControlMode::Velocity, 
                           rightMotorOutput * 5200.0 * 4096 / 600 );
      LSMotorState.targetVelocity_UnitsPer100ms = leftMotorOutput  *
	                                          5200.0 * 4096 / 600 ;
      RSMotorState.targetVelocity_UnitsPer100ms = rightMotorOutput *
	                                          5200.0 * 4096 / 600 ;
      //m_motorLSMaster.Set( ControlMode::PercentOutput, 
                           //leftMotorOutput  * 500.0 * 4096 / 600 / 1024);
      //m_motorRSMaster.Set( ControlMode::PercentOutput, 
                           //rightMotorOutput * 500.0 * 4096 / 600 / 1024);
   }      // Team4918Drive()


         /*---------------------------------------------------------------------*/
         /* DriveByJoystick()                                                   */
         /* Drive robot according to the commanded Y and X joystick position.   */
         /*---------------------------------------------------------------------*/
   void DriveByJoystick( void ) {
                                        /* Drive the robot according to the     */
                                        /* commanded Y and X joystick position. */
         // m_drive.ArcadeDrive( m_stick.GetY(), -m_stick.GetX() );
              // our joystick increases Y when pulled BACKWARDS, and increases
              // X when pushed to the right.
      if ( sCurrState.joyButton[2] ) {
         Team4918Drive( sCurrState.joyY*abs(sCurrState.joyY),
                        sCurrState.joyX*abs(sCurrState.joyX) );
         limenttable->PutNumber( "ledMode", 3 );   // turn Limelight LEDs on
      } else {
         Team4918Drive( -sCurrState.joyY*abs(sCurrState.joyY),
                         sCurrState.joyX*abs(sCurrState.joyX) );
         limenttable->PutNumber( "ledMode", 1 );  // turn Limelight LEDs off
      }
   }      // DriveByJoystick()


      /*---------------------------------------------------------------------*/
      /* DriveToLimelightTarget()                                            */
      /* DriveToLimelightTarget() drives autonomously towards a limelight    */
      /* vision target.                                                      */
      /* It returns true if the limelight data is valid, false otherwise.    */
      /*---------------------------------------------------------------------*/
   bool DriveToLimelightTarget()  {

      bool returnVal = true;
      static int  iCallCount = 0;

      iCallCount++;

      limenttable->PutNumber( "ledMode", 3 );                   // turn LEDs on

      if ( 1 == limev )  {                       // if limelight data is valid
         double autoDriveSpeed;
             // limea is the area of the target seen by the limelight camera
             // and is in percent (between 0 and 100) of the whole screen area.
             // limey is the height above the center of the field of view
             // Could change the if/else statements below to calculate
             // autoDriveSpeed by using a math expression based on limey.
         if        ( 15 < limey ) {
            autoDriveSpeed = -0.1;
         } else if ( 12 < limey )  { // if we're really close...
            autoDriveSpeed = 0.0;    //   stop (or 0.08 to go slow)
         } else if (  8 < limey ) {  // if we're a little farther...
            autoDriveSpeed = 0.1;    //   go a little faster
         } else if (  2 < limey ) {  // if we're farther still...
            autoDriveSpeed = 0.15;   //   go a little faster still
         } else {                    // else we must be really far...
            autoDriveSpeed = 0.20;   //   go as fast as we dare
         }

                          // LATER: May want to modify autoDriveSpeed depending
                          // on the distance from the target determined
                          // by sonar transducers.

         // May have to add/subtract a constant from limex here, to account
         // for the offset of the camera away from the centerline of the robot.
         if ( aBooleanVariable ) {
            // m_drive.CurvatureDrive( -autoDriveSpeed, 0, 0 );
            Team4918Drive( -autoDriveSpeed, 0.0 );
         } else if ( 0 <= limex )  {
                             // if target to the right, turn towards the right
            // m_drive.CurvatureDrive( -autoDriveSpeed, -(limex/30.0), 1 );
            Team4918Drive( -autoDriveSpeed, (limex/30.0) );
         } else if ( limex < 0 ) {
                               // if target to the left, turn towards the left
            // m_drive.CurvatureDrive( -autoDriveSpeed, -(limex/30.0), 1 );
            Team4918Drive( -autoDriveSpeed, (limex/30.0) );
         } else {
            // m_drive.CurvatureDrive( -autoDriveSpeed, 0, 0 );
            Team4918Drive( -autoDriveSpeed, 0.0 );
         }

      } else {                    // else limelight data is not valid any more
         // should we continue forward here?
         // m_drive.CurvatureDrive( 0.0, 0, 0 );                // stop the robot
         // Team4918Drive( 0.0, 0.0 );
         DriveByJoystick();     // no powercell seen, drive according to joystick
         returnVal = false;
      }

      if ( 0 == iCallCount%100 )  {
         cout << "lime: " << limev << ":" << limex << "/" << limey;
         cout << ", " << limea << ":" << limes << "." << endl;
      }

      return returnVal;

   }  /* DriveToLimelightTarget() */


      /*---------------------------------------------------------------------*/
      /* DriveToPowercell()                                                  */
      /* DriveToPowercell() drives autonomously towards a vision target.     */
      /* It returns true if the usb vision data has detected a power cell,   */
      /* false otherwise.                                                    */
      /*---------------------------------------------------------------------*/
   bool DriveToPowercell()  {

      bool returnVal = true;
      static int  iCallCount = 0;

      iCallCount++;

      if ( powercellOnVideo.SeenByCamera ) {      // if USB video data is valid
         double autoDriveSpeed;
             // If powercellOnVideo.SeenByCamera is true, that means that the
             // vision-processing code in VisionThread() has found a yellow
             // circle in the latest video frame from the USB videocamera (and
             // we hope that yellow circle is a powercell).
             // powercellOnVideo.Y is the Y-position in the video frame of the
             //    powercell; the range is from -120 to +120 (pixels).
             // powercellOnVideo.X is the X-position in the video frame of the
             //    powercell; the range is from -160 to +160 (pixels).
             // powercellOnVideo.Radius is the radius of the powercell;
             //    the range is from 20 to 70 (pixels).
             // In the code below, we use those powercellOnVideo values to
             // determine how fast and in which direction to drive, to go
             // towards the powercell.
             // We could change the if/else statements below to calculate
             // autoDriveSpeed by using a math expression based on
             // powercellOnVideo.Y values.
         if        ( powercellOnVideo.Y < -50 ) {  // if we're super close
            autoDriveSpeed = -0.35;   //   go backward slowly
         } else if ( powercellOnVideo.Y < -30 ) {  // if we're super close
            autoDriveSpeed = -0.25;   //   go backward slowly
            autoDriveSpeed = -0.35 * float( - 30 - powercellOnVideo.Y ) / 20.0;
         } else if ( powercellOnVideo.Y < 0 )   { // if we're really close...
            autoDriveSpeed = 0.0;     //   stop (or 0.08 to go slow)
         } else if ( powercellOnVideo.Y <  20 ) {  // if we're a little farther
            autoDriveSpeed = 0.15;    //   go a little faster
            autoDriveSpeed = 0.20 * float( powercellOnVideo.Y ) / 20.0;
         } else if (  powercellOnVideo.Y < 40 ) {  // if we're farther still...
            autoDriveSpeed = 0.20;    //   go a little faster still
            autoDriveSpeed = 0.20 + 0.20 * float( powercellOnVideo.Y - 20 ) / 40.0;
         } else {                     // else we must be really far...
            autoDriveSpeed = 0.30;    //   go as fast as we dare
         }

                          // LATER: May want to modify autoDriveSpeed depending
                          // on the distance from the target determined
                          // by sonar transducers.

         // May have to add/subtract a constant from x-values here, to account
         // for the offset of the camera away from the centerline of the robot.
         if        ( 0 <= powercellOnVideo.X ) {
                             // if target to the right, turn towards the right
            //m_drive.CurvatureDrive( -autoDriveSpeed,
            //                        -sqrt((powercellOnVideo.X/300.0)), 1 );
            Team4918Drive( autoDriveSpeed, sqrt(powercellOnVideo.X/300.0) );
         } else if ( powercellOnVideo.X < 0 ) {
                               // if target to the left, turn towards the left
            //m_drive.CurvatureDrive( -autoDriveSpeed,
            //                        sqrt((-powercellOnVideo.X/300.0)), 1 );
            Team4918Drive( autoDriveSpeed, -sqrt(-powercellOnVideo.X/300.0) );           
         } else {
            //m_drive.CurvatureDrive( -autoDriveSpeed, 0, 0 );

            Team4918Drive( autoDriveSpeed, 0.0 );     // drive straight forward
         }

      } else {               // else USB videocamera data is not valid any more
         // should we continue forward here?
         // m_drive.CurvatureDrive( 0.0, 0, 0 );                 // stop the robot
         //Team4918Drive( 0.0, 0.0 );
         DriveByJoystick();     // no powercell seen, drive according to joystick
         returnVal = false;
      }

      if ( 0 == iCallCount%100 )  {
         cout << "Powercell Seen flag " << powercellOnVideo.SeenByCamera <<
                 ": " << powercellOnVideo.X << "/" << powercellOnVideo.Y;
         cout << ", " << powercellOnVideo.Radius  << "." << endl;
      }

      return returnVal;

   }  /* DriveToPowercell() */


      /*---------------------------------------------------------------------*/
      /* RunDriveMotors()                                                    */
      /* RunDriveMotors() drives the robot.  It uses joystick and console    */
      /* inputs to determine what the robot should do, and then runs the     */
      /* m_motorLSMaster and m_motorRSMaster motors to make the robot do it. */
      /*---------------------------------------------------------------------*/
   bool RunDriveMotors( void ) {
      static int iCallCount = 0;
      iCallCount++;

      //m_shiftingSolenoid.Set(true);     // high gear?
      //m_shiftingSolenoid.Set(false);    // low gear?

                  /* If joystick button 5 pressed, use the joystick position */
                  /* to adjust some variables to specific speeds, so we can  */
                  /* set the drive motors to those speeds in later code.     */
      if ( ( 0 == iCallCount%100 )  &&
           sCurrState.joyButton[5]     ) {

         if ( 0.5 < m_stick.GetY() &&
              LSMotorState.targetVelocity_UnitsPer100ms < 790.0 * 4096 / 600 ) {
            LSMotorState.targetVelocity_UnitsPer100ms += 200.0 * 4096 / 600;
         } else if ( m_stick.GetY() < -0.5 &&
                     -790.0 * 4096 / 600 <
                                  LSMotorState.targetVelocity_UnitsPer100ms ) {
            LSMotorState.targetVelocity_UnitsPer100ms -= 200.0 * 4096 / 600;
         }

         if ( 0.5 < m_stick.GetX() &&
              RSMotorState.targetVelocity_UnitsPer100ms < 790.0 * 4096 / 600 ) {
            RSMotorState.targetVelocity_UnitsPer100ms += 200.0 * 4096 / 600;
         } else if ( m_stick.GetX() < -0.5 && 
                     -790.0 * 4096 / 600 <
                                  RSMotorState.targetVelocity_UnitsPer100ms ) {
            RSMotorState.targetVelocity_UnitsPer100ms -= 200.0 * 4096 / 600;
         }
      } 

      motorFindMinMaxVelocity( m_motorLSMaster, LSMotorState );
      motorFindMinMaxVelocity( m_motorRSMaster, RSMotorState );

      if ( ( 15000 < abs(LSMotorState.sensorVmin) ) &&
           ( 15000 < abs(RSMotorState.sensorVmin) )    ) {
         if (!sCurrState.highGear){
            cout << "shifting to high" << endl; 
         }
         sCurrState.highGear = true; // could move inside if statement
         m_shiftingSolenoid.Set( true );  // high gear got 5700/5200
      } else if ( ( abs(LSMotorState.sensorVmin) < 14000 ) &&
                  ( abs(RSMotorState.sensorVmin) < 14000 )  ) {
         if (sCurrState.highGear){
            cout << "shifting to low" << endl;
         }
         sCurrState.highGear = false; // could move inside if statement
         m_shiftingSolenoid.Set( false );    // low gear got 2800/2700
      } 

      if ( 0 == iCallCount%100 )  {   // every 2 seconds
         JoystickDisplay();

         MotorDisplay( "LS:", m_motorLSMaster, LSMotorState );
         MotorDisplay( "RS:", m_motorRSMaster, RSMotorState );
         //IMUOrientationDisplay();

         // max free speed for MinCims is about 6200
         //cout << "Accel: x/y/z: " << RoborioAccel.GetX() << "/";
         //cout << RoborioAccel.GetY() << "/";
         //cout << RoborioAccel.GetZ() << endl;
      }

                                       /* Button 1 is the trigger button on */
                                       /* the front of the joystick.        */
                                       /* Button 2 is the bottom button on  */
                                       /* the rear of the joystick.         */
      if ( ( sCurrState.joyButton[2] ) &&      // If driver is pressing the
           ( sCurrState.joyButton[1] ) &&      // "ReverseDrive" and the
           ( 1  == limev )                ) {  // "DriveToLimelightTarget"
                                 // buttons, and the limelight has a target,
                                 // then autonomously drive towards the target
         DriveToLimelightTarget();

      } else if ( ( !sCurrState.joyButton[2] ) &&  // If driver is NOT pressing
                  ( sCurrState.joyButton[1] ) &&   // the "ReverseDrive" button 
                  ( powercellOnVideo.SeenByCamera ) ) { // and is pressing the
                              // trigger ("DriveToPowercell"), and
                              // the USB videocamera has seen a powercell,
         DriveToPowercell();  // then autonomously drive towards the powercell

                                        /* Button 3 is the topmost center   */
                                        /* button on the back of joystick.  */
      } else if ( sCurrState.joyButton[3] ) {

                     /* If button 3 pressed, drive the motors separately    */
                     /* (rather than with ArcadeDrive) with Percent Output, */
                     /* using the variables that have been set with the     */
                     /* joystick while button 3 is pressed.                 */
         if ( !sPrevState.joyButton[3] ) {  // if button has just been pressed
            // m_drive.StopMotor();
                                // Set current sensor positions to zero
            m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
            m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
         }
         // m_motorLSMaster.Set( ControlMode::PercentOutput,
         //                      -m_stick.GetY() );
         // m_motorRSMaster.Set( ControlMode::PercentOutput,
         //                      -m_stick.GetX() );
                        /* Use MotionMagic to set the position of the drive */
                        /* wheels, rather than setting their velocity.      */
         // m_motorLSMaster.Set( ControlMode::MotionMagic, sCurrState.joyY*4096 );
         // m_motorRSMaster.Set( ControlMode::MotionMagic, sCurrState.joyX*4096 );

                                      /* Button 5 is the Right-most button  */
                                      /* on the back of the joystick.       */
      } else if ( sCurrState.joyButton[5] ) {
         static int iButtonPressCallCount = 0;
         if ( !sPrevState.joyButton[5] ) {  // if button has just been pressed
            // m_drive.StopMotor();
            iButtonPressCallCount = 0;
         }
         // m_drive.StopMotor();    // this is needed to eliminate motor warnings

         if ( iButtonPressCallCount < 50 ) {       // turn motors on gently...
            // m_motorLSMaster.Set( ControlMode::Velocity,
            //       m_motorLSMaster.GetSelectedSensorVelocity() +
            //          0.2 * ( LSMotorState.targetVelocity_UnitsPer100ms -
            //                  m_motorLSMaster.GetSelectedSensorVelocity() ) );
            // m_motorRSMaster.Set( ControlMode::Velocity,
            //       m_motorRSMaster.GetSelectedSensorVelocity() +
            //          0.2 * ( RSMotorState.targetVelocity_UnitsPer100ms -
            //                  m_motorRSMaster.GetSelectedSensorVelocity() ) );
         } else {
            // m_motorLSMaster.Set( ControlMode::Velocity, 
            //                        LSMotorState.targetVelocity_UnitsPer100ms);
            // m_motorRSMaster.Set( ControlMode::Velocity, 
            //                        RSMotorState.targetVelocity_UnitsPer100ms);
         }
         iButtonPressCallCount++;
      } else {
                                    /* Drive the robot according to the     */
                                    /* commanded Y and X joystick position. */
         // m_drive.ArcadeDrive( m_stick.GetY(), -m_stick.GetX() );
              // our joystick increases Y when pulled BACKWARDS, and increases
              // X when pushed to the right.
         if ( sCurrState.joyButton[2] ) {
            Team4918Drive( sCurrState.joyY*abs(sCurrState.joyY),
                           sCurrState.joyX*abs(sCurrState.joyX) );
            limenttable->PutNumber( "ledMode", 3 );   // turn Limelight LEDs on
         } else {
            Team4918Drive( -sCurrState.joyY*abs(sCurrState.joyY),
                            sCurrState.joyX*abs(sCurrState.joyX) );
            limenttable->PutNumber( "ledMode", 1 );  // turn Limelight LEDs off
         }

      }
      return true;
   }      // RunDriveMotors()


      /*---------------------------------------------------------------------*/
      /* TurnToHeading()                                                     */
      /* This function turns the robot to a specified heading.               */
      /* heading is a parameter, in the same degree units as the Pigeon IMU  */
      /* produces; it is positive for left turns (the same as trigonometry,  */
      /* but the opposite of ordinary 0-360 degree compass directions).      */
      /* bInit is a boolean that tells the function to initialize (to record */
      /* the current yaw as the starting yaw).                               */
      /* This function returns false if the yaw value has not been reached   */
      /* yet, and returns true when the yaw has been reached.                */
      /*---------------------------------------------------------------------*/
   bool OldTurnToHeading ( double heading ) {
      static bool bReturnValue = true;
      static double startYaw = 0;

      if ( bReturnValue ) {
         startYaw = sCurrState.yawPitchRoll[0];
	 cout << "TurnToHeading(): startYaw = " << startYaw << endl;
      }

      if ( sCurrState.yawPitchRoll[0] < heading ) { // do we need to turn left?
                                                // If we have a long way to go,  
         if ( sCurrState.yawPitchRoll[0] < heading-50.0 ) {   // turn left fast
            if ( sCurrState.yawPitchRoll[0]-50 <startYaw){
               LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
               //LSMotorState.targetVelocity_UnitsPer100ms = 0.0;
               RSMotorState.targetVelocity_UnitsPer100ms = 300.0 * 4096 / 600;
            } else {
               LSMotorState.targetVelocity_UnitsPer100ms = 300.0 * 4096 / 600;
               RSMotorState.targetVelocity_UnitsPer100ms = 300.0 * 4096 / 600;
            }
            bReturnValue = false;           // else if a medium way to go, turn
         } else if ( sCurrState.yawPitchRoll[0] < heading-5.0 ) {  // left slow
            LSMotorState.targetVelocity_UnitsPer100ms = 300.0 * 4096 / 600 *
		                   (heading - sCurrState.yawPitchRoll[0])/50.0;
            //LSMotorState.targetVelocity_UnitsPer100ms = 0.0;
            RSMotorState.targetVelocity_UnitsPer100ms = 300.0 * 4096 / 600 *
		                   (heading - sCurrState.yawPitchRoll[0])/50.0;
            bReturnValue = false;
         } else {                              // else we're done; stop turning
            LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
            RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;  
            bReturnValue = true; 
         }
      } else {                                    // else we need to turn right
                                                // If we have a long way to go,
         if ( heading+50.0 < sCurrState.yawPitchRoll[0] ) {  // turn right fast
            LSMotorState.targetVelocity_UnitsPer100ms = -300.0 * 4096 / 600;
            //RSMotorState.targetVelocity_UnitsPer100ms = -500.0 * 4096 / 600;
            RSMotorState.targetVelocity_UnitsPer100ms = 0.0;
            bReturnValue = false;
                                            // else if a medium way to go, turn
         }else if ( heading+5.0 < sCurrState.yawPitchRoll[0] ) {  // right slow
            LSMotorState.targetVelocity_UnitsPer100ms = -300.0 * 4096 / 600 *
		                     (sCurrState.yawPitchRoll[0]-heading)/50.0;
            RSMotorState.targetVelocity_UnitsPer100ms = -100.0 * 4096 / 600 *
		                     (sCurrState.yawPitchRoll[0]-heading)/50.0;
            //RSMotorState.targetVelocity_UnitsPer100ms = 0.0;
            bReturnValue = false;
         } else {                              // else we're done; stop turning
            LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
            RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
            bReturnValue = true;
         }
      }
      if ( bReturnValue ) {
         cout << "TurnToHeading() returning TRUE!!!!!!!!!" << endl;
      }
      return bReturnValue;
   }      // OldTurnToHeading()


   bool TurnToHeading ( double desiredYaw, bool bInit ) {
      static bool   bReturnValue = true;
      static double startYaw = 0;

      double      currentYaw = sCurrState.yawPitchRoll[0];
      double      prevYaw    = sPrevState.yawPitchRoll[0];

      static double dDesiredSpeed = 0.01; // -1.0 to +1.0, positive is forward
      static double dDesiredTurn = 0.0;  // -1.0 to +1.0, positive to the right

                 // If we've been told to initialize, or the last call returned
                 // true (indicating that we finished the previous turn).
      if ( bInit || bReturnValue ) {
         startYaw = currentYaw;
         dDesiredSpeed = 0.01;
         cout << "TurnToHeading(): startYaw = " << startYaw;
         cout << " desiredYaw = " << desiredYaw << endl;
      }
        // calculate desired turn speed, with 100% if off by 50 degrees or more
      dDesiredTurn = ( currentYaw-desiredYaw ) * 1.0/50.0;

                       // If we're turning at less than 50 degrees per second,
      if ( std::abs( currentYaw - prevYaw ) < 1.0 ) {
           // Increase the forward speed, to reduce friction and ease the turn.
         dDesiredSpeed = std::min( 1.0,  dDesiredSpeed * 1.1 );
      } else {                // Else we're turning at a good speed, so
                              // reduce the forward speed, to tighten the turn.
         dDesiredSpeed = std::max( 0.01, dDesiredSpeed * 0.9 );
      }

      if ( startYaw < desiredYaw ) {                // Do we need to turn left?
	                                  // dDesiredTurn is left negative, but
                            	          // *Yaw variables are left positive.
         if ( currentYaw < desiredYaw-5.0 ) {        // Should we keep turning?
            bReturnValue = false;
         } else {                              // else we're done; stop turning
            dDesiredSpeed = 0.0;
            bReturnValue = true; 
         }
      } else {                                   // Else we need to turn right.
                                                // If we have a long way to go,
         if ( desiredYaw+5.0 < currentYaw ) {        // Should we keep turning?
            bReturnValue = false;
         } else {                              // else we're done; stop turning
            dDesiredSpeed = 0.0;
            bReturnValue = true;
         }
      }
      dDesiredSpeed = std::min(  1.0, dDesiredSpeed );
      dDesiredTurn  = std::min(  1.0, dDesiredTurn  );
      dDesiredSpeed = std::max( -1.0, dDesiredSpeed );
      dDesiredTurn  = std::max( -1.0, dDesiredTurn  );

//    cout << "Turn2Hdg(): " << dDesiredSpeed << ", " << dDesiredTurn << endl;
      Team4918Drive( dDesiredSpeed, dDesiredTurn );

      if ( bReturnValue ) {
         cout << "TurnToHeading() returning TRUE!!!!!!!!!" << endl;
         cout << "Final yaw: " <<  currentYaw << endl;
      }

      return bReturnValue;
   }      // TurnToHeading()


      /*---------------------------------------------------------------------*/
      /* DriveToDistance()                                                   */
      /* This function drives the robot on a specified heading,              */
      /* to a specified distance.                                            */
      /* desiredYaw is a parameter, in the same degree units as the Pigeon   */
      /* IMU produces; it is positive for left turns (the same as            */
      /* trigonometry, but the opposite of ordinary 0-360 degree compass     */
      /* directions).                                                        */
      /* desiredDistance is a parameter, in feet.                            */
      /* bInit is a boolean that tells the function to initialize (to record */
      /* the current position as the starting position).                     */
      /* This function returns false if the distance has not been reached    */
      /* yet, and returns true when the distance has been reached.           */
      /*---------------------------------------------------------------------*/
   bool DriveToDistance( double desiredYaw,
               	       double desiredDistance,
                         bool   bInit            ) {
      static bool bReturnValue = true;
      static int  iLSStartPosition = 0;
      static int  iRSStartPosition = 0;
      int         iDistanceDriven;    // distance driven in encoder ticks
      double      dDistanceDriven;    // distance driven in feet (floating pt.)
      double      dDesiredSpeed;  // -1.0 to +1.0, positive is forward
      double      dDesiredTurn;   // -1.0 to +1.0, positive is to the right
      static int  iCallCount = 0;
      iCallCount++;

          // If we've been told to initialize, or the last call returned
          // true (indicating that we finished the previous drive to distance).
      if ( bInit || bReturnValue ) {
         iLSStartPosition = sCurrState.iLSMasterPosition;
         iRSStartPosition = sCurrState.iRSMasterPosition;
      }
      iDistanceDriven =
                   ( ( sCurrState.iLSMasterPosition - iLSStartPosition ) +
                     ( sCurrState.iRSMasterPosition - iRSStartPosition ) ) / 2;
            // Convert encoder ticks to feet, using the diameter of the wheels,
            // the number of ticks/revolution, and the number of inches/foot.
      dDistanceDriven = iDistanceDriven * 3.1415 * 8.0 / 4096.0 / 12.0;
                                         // if we haven't driven far enough yet
      if ( std::abs( dDistanceDriven ) < std::abs( desiredDistance ) ) {
         if ( 0.0 < desiredDistance ) {             // If we're driving forward
                                      // and still have more than 10 feet to go
            if ( dDistanceDriven < desiredDistance - 10.0 ) {
               dDesiredSpeed = 1.0;                            // go full speed
            } else {
                   // Otherwise speed is proportional to distance still needed.
               dDesiredSpeed = 0.1 +
		                  ( desiredDistance - dDistanceDriven ) / 10.0;
            }
         } else {                               // else we're driving backwards
                                      // and still have more than 10 feet to go
            if ( desiredDistance + 10.0 < dDistanceDriven ) {
               dDesiredSpeed = -1.0;               // go full speed (backwards)
            } else {
                   // Otherwise speed is proportional to distance still needed.
               dDesiredSpeed = -0.1 +
		                  ( desiredDistance - dDistanceDriven ) / 10.0;
            }
         }

         if ( sCurrState.yawPitchRoll[0] < desiredYaw-50.0 ) {
            dDesiredTurn = -0.2;
         } else if ( desiredYaw+50.0 < sCurrState.yawPitchRoll[0] ) {
            dDesiredTurn = 0.2;
         } else {
            dDesiredTurn = (sCurrState.yawPitchRoll[0]-desiredYaw) * 0.2/50.0;
            if ( ( 0.02 < dDesiredTurn ) && ( dDesiredTurn ) < 0.1 ) {
               dDesiredTurn = 0.1;
            } else if ( ( -0.1 < dDesiredTurn ) && ( dDesiredTurn < -0.02 ) ) {
               dDesiredTurn = -0.1;
            }
         }
         
         Team4918Drive( dDesiredSpeed, dDesiredTurn );
         bReturnValue = false;   // tell caller we are still driving
         if ( 0 == iCallCount%25 ) {                        // Every 2 seconds
            cout << "D2D(): curYaw/desYaw desTurn: " <<
                    sCurrState.yawPitchRoll[0] << 
                    "/" << desiredYaw << " " <<
                    " " << dDesiredTurn << endl;
         }
      } else {
         Team4918Drive( 0.0, 0.0 );   // stop the robot
         m_motorLSMaster.SetIntegralAccumulator( 0.0 );
         m_motorRSMaster.SetIntegralAccumulator( 0.0 );
         bReturnValue = true;    // tell caller we've reached desired distance
      }

      if ( bReturnValue ) {
         cout << "DriveToDistance() returning TRUE!!!!!!!!!" << endl;
         cout << "Final Distance: " <<  dDistanceDriven << endl;
         cout << " Final Yaw: " <<  sCurrState.yawPitchRoll[0] << endl;
      }

      return bReturnValue;
   }  // DriveToDistance()


         /*------------------------------------------------------------------*/
         /* RunShooter()                                                     */
         /* RunShooter() drives the 2 shooter motors.                        */
         /*------------------------------------------------------------------*/
   bool RunShooter(void) {
      if (   ( 0.5 < sCurrState.conY ) &&           // if console "joystick" is
            !( 0.5 < sPrevState.conY ) ) {          // newly-pressed downward
         TSMotorState.targetVelocity_UnitsPer100ms =  2100 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = -2900 * 4096 / 600;
         m_motorTopShooter.Set( ControlMode::Velocity, 
                                TSMotorState.targetVelocity_UnitsPer100ms );
         m_motorBotShooter.Set( ControlMode::Velocity, 
                                BSMotorState.targetVelocity_UnitsPer100ms );
      } else if ( !( 0.5 < sCurrState.conY ) &&
                   ( 0.5 < sPrevState.conY ) ) {     // newly-released downward
         TSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         m_motorTopShooter.Set( ControlMode::Velocity,
               0.95 * (double)m_motorTopShooter.GetSelectedSensorVelocity() );
         m_motorBotShooter.Set(ControlMode::Velocity,
               0.95 * (double)m_motorBotShooter.GetSelectedSensorVelocity() );
      } else if (  ( sCurrState.conY < -0.5 ) &&  // else if console "joystick"
                  !( sPrevState.conY < -0.5 ) ) { // is newly-pressed upward
         TSMotorState.targetVelocity_UnitsPer100ms =  2200 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = -3000 * 4096 / 600;
         m_motorTopShooter.Set( ControlMode::Velocity, 
                                TSMotorState.targetVelocity_UnitsPer100ms );
         m_motorBotShooter.Set( ControlMode::Velocity, 
                                BSMotorState.targetVelocity_UnitsPer100ms );
      } else if ( !( sCurrState.conY < -0.5 ) &&
                   ( sPrevState.conY < -0.5 ) ) {     // newly-released upward
         TSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         m_motorTopShooter.Set( ControlMode::Velocity,
               0.95 * (double)m_motorTopShooter.GetSelectedSensorVelocity() );
         m_motorBotShooter.Set(ControlMode::Velocity,
               0.95 * (double)m_motorBotShooter.GetSelectedSensorVelocity() );
      } else if ( ( -0.5 < sCurrState.conY       ) && 
                  (        sCurrState.conY < 0.5 ) ) {
         m_motorTopShooter.Set( ControlMode::Velocity,
               0.95 * (double)m_motorTopShooter.GetSelectedSensorVelocity() );
         m_motorBotShooter.Set(ControlMode::Velocity,
               0.95 * (double)m_motorBotShooter.GetSelectedSensorVelocity() );
      }
         
      return true;
   }     // RunShooter()


         /*------------------------------------------------------------------*/
         /* Shoot()                                                          */
         /* Shoot() waits until the shooter rollers are moving at a desired  */
         /* speed, then moves the conveyor to shoot powercells.              */
         /*------------------------------------------------------------------*/
   void Shoot( void ) {
      if ( 0.5 < sCurrState.conY ) {
   	 if ( ( 1800 * 4096 / 600 <
                   abs( m_motorTopShooter.GetSelectedSensorVelocity() ) ) &&
              ( 2600 * 4096 / 600 <
                   abs( m_motorBotShooter.GetSelectedSensorVelocity() ) )   ) {
	                                 // run the conveyor to shoot the balls
            m_motorConveyMaster.Set( ControlMode::PercentOutput, -0.8 );
         }
      } else if (sCurrState.conY < -0.5) {
         if ( ( 1900 * 4096 / 600 <
                   abs( m_motorTopShooter.GetSelectedSensorVelocity() ) ) &&
              ( 2700 * 4096 / 600 <
                   abs( m_motorBotShooter.GetSelectedSensorVelocity() ) )   ) {
	                                 // run the conveyor to shoot the balls
            m_motorConveyMaster.Set( ControlMode::PercentOutput, -0.8 );
         }
      }
   }

   
      /*---------------------------------------------------------------------*/
      /* RunConveyor()                                                       */
      /* Run the conveyor belt motors, to move balls into and through the    */
      /* conveyor system.                                                    */
      /*---------------------------------------------------------------------*/
   void RunConveyor( void ) {

      if ( sPrevState.powercellInIntake != sCurrState.powercellInIntake ) {
         if ( sCurrState.powercellInIntake ) {
            cout << "powercell in the intake." << endl;
         } else {
            cout << "powercell NOT in the intake." << endl; 
         } 
      }

      if ( sPrevState.powercellInPosition5 !=
                                          sCurrState.powercellInPosition5 ) {
         if ( sCurrState.powercellInPosition5 ) {              // if this sensor is blocked
            cout << "powercell in position 5" << endl;
         } else {
            cout << "powercell NOT in position 5" << endl; 
         } 
      }
      if ( sCurrState.conButton[11] ) {             // Is manual mode selected?
         if ( sCurrState.conButton[2] )   {            // Run conveyor forward.
            m_motorConveyMaster.Set( ControlMode::PercentOutput, -0.8 );
         } else if ( sCurrState.conButton[4] ) {     // Run conveyor backwards.
            m_motorConveyMaster.Set( ControlMode::PercentOutput,  0.8 );
         } else {                                         // Stop the conveyor.
                 // comment out for now, until we get a dedicated motor
                 // for this which doesn't compete with RunClimberPole().
            if (sPrevState.conButton[2]||sPrevState.conButton[4]){
               m_motorConveyMaster.Set( ControlMode::PercentOutput, 0.0);
            }
         } 
      } else {  
         if (  sCurrState.powercellInIntake &&
              !sCurrState.powercellInPosition5 ) {
            // for testing only, until we connect the real conveyor motors
            m_motorConveyMaster.Set( ControlMode::PercentOutput, -0.5 );
         } else {
            //if (sPrevState.powercellInIntake && 
                //!sPrevState.powercellInPosition5 ){
               m_motorConveyMaster.Set( ControlMode::PercentOutput, 0.0 );
            //}
         } 
      }
   }   // RunConveyor()

      /*---------------------------------------------------------------------*/
      /* RunColorWheel()                                                     */
      /* Extend or retract the color wheel flipper.                          */
      /*---------------------------------------------------------------------*/
   void RunColorWheel( void ) {
      static int iCallCount = 0;
      static bool bFlipperState = false;
      iCallCount++;

                                 // Console button 6 is the lowest-left button
      if ( !sPrevState.conButton[6] && sCurrState.conButton[6] ) {
         if ( bFlipperState ){
            m_flipperSolenoid.Set( frc::DoubleSolenoid::Value::kForward);
         } else {
            m_flipperSolenoid.Set( frc::DoubleSolenoid::Value::kReverse);
         }
         bFlipperState = !bFlipperState; 
      }
      if ( sCurrState.conButton[7]){
         m_motorFlippyFlippy.Set(ControlMode::PercentOutput, 0.2);
      } else {
         m_motorFlippyFlippy.Set(ControlMode::PercentOutput, 0.0);
      }
   }

      /*---------------------------------------------------------------------*/
      /* RunClimberPole()                                                    */
      /* Extend or retract the telescoping climber pole.                     */
      /*---------------------------------------------------------------------*/
   void RunClimberPole( void ) {
      static int iCallCount = 0;
      static bool limitSwitchHasBeenHit = false;
      iCallCount++;

      if (sCurrState.conButton[1] && sCurrState.conButton[3] ) {
         m_motorClimberPole.Set( ControlMode::PercentOutput,
                                 0.5*sCurrState.joyZ);
      
      } else if ( sCurrState.conButton[1] ){
         if ( !sPrevState.conButton[1] ) {         // if button 1 has just been
            limitSwitchHasBeenHit = false;         // pressed, reset to start
            m_compressor.Stop();
         }
         if ( limitSwitchHasBeenHit ) {
                 // we are at the top; just supply a little power to stay there
            m_motorClimberPole.Set( ControlMode::PercentOutput, 0.10 );
         } else {
                                                   // apply full climbing power
            m_motorClimberPole.Set( ControlMode::PercentOutput, 0.45 );
            if ( m_motorClimberPole.IsFwdLimitSwitchClosed() ) {
               limitSwitchHasBeenHit = true;
            }
         }
      } else if (sCurrState.conButton[3] ) {
         m_motorClimberPole.Set( ControlMode::PercentOutput, -0.05);
           
      } else { 
         m_motorClimberPole.Set( ControlMode::PercentOutput, 0.0);
      }
      

      if ( 0 == iCallCount%50 ) { 
         if ( sCurrState.conButton[1] || sCurrState.conButton[3] ) {                            // every 2 seconds
            if ( sCurrState.conButton[1] ) {
               cout << "ClimberUp: ";
            } else {
               cout << "ClimberDown: ";
            }
            cout << setw(5) <<
               m_motorClimberPole.GetStatorCurrent() << "A" << endl;
            if ( m_motorClimberPole.IsFwdLimitSwitchClosed() ) {
               cout << "Climber pole at top." << endl;
            } else if ( m_motorClimberPole.IsRevLimitSwitchClosed() ) {
               cout << "Climber pole at bottom." << endl;
            }  
         }
      }
   }      // RunClimberPole() 


      /*---------------------------------------------------------------------*/
      /* RunClimberWinch()                                                   */
      /* Run the winch motor, to make the robot climb.                       */
      /*---------------------------------------------------------------------*/
   void RunClimberWinch( void ) {
      static int iCallCount = 0;
      iCallCount++;

      if ( sCurrState.conButton[9] ){
         m_motorClimberWinch.Set( ControlMode::PercentOutput, 0.5 );
         if ( 0 == iCallCount%50 ) {
            cout << "ClimberWinch Current: " << setw(5) <<
                    m_motorClimberWinch.GetStatorCurrent() << "A" << endl;
         }
      } else { 
         m_motorClimberWinch.Set( ControlMode::PercentOutput, 0.0 );
      }
   }      // RunClimberWinch()


      /*---------------------------------------------------------------------*/
      /* MotorInit()                                                         */
      /* Setup the initial configuration of a motor.  These settings can be  */
      /* superseded after this function is called, for the needs of each     */
      /* specific motor.                                                     */
      /*---------------------------------------------------------------------*/
   void MotorInit( WPI_TalonSRX & m_motor ) {

      m_motor.ConfigFactoryDefault( 10 );

                /* Configure Sensor Source for Primary PID */
          /* Config to stop motor immediately when limit switch is closed. */
                                                   // if encoder is connected
      if ( OK == m_motor.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motor.ConfigForwardLimitSwitchSource(
                     LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                     LimitSwitchNormal::LimitSwitchNormal_NormallyOpen );
         m_motor.ConfigReverseLimitSwitchSource(
                     LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                     LimitSwitchNormal::LimitSwitchNormal_NormallyOpen );
         m_motor.OverrideLimitSwitchesEnable(true);
      }

         /*
          * Configure Talon SRX Output and Sensor direction.
          * Invert Motor to have green LEDs when driving Talon Forward
          * ( Requesting Positive Output ),
          * Phase sensor to have positive increment when driving Talon Forward
          * (Green LED)
          */
      m_motor.SetSensorPhase(true);   // invert encoder value positive/negative
      // m_motor.SetInverted(false);  // invert direction of motor itself.

                        /* Set relevant frame periods to be at least as fast */
                        /* as the periodic rate.                             */
      m_motor.SetStatusFramePeriod(
                         StatusFrameEnhanced::Status_13_Base_PIDF0,  10, 10 );
      m_motor.SetStatusFramePeriod(
                         StatusFrameEnhanced::Status_10_MotionMagic, 10, 10 );

                                         /* Set the peak and nominal outputs */
      m_motor.ConfigNominalOutputForward( 0, 10 );
      m_motor.ConfigNominalOutputReverse( 0, 10 );
      m_motor.ConfigPeakOutputForward(    1, 10 );
      m_motor.ConfigPeakOutputReverse(   -1, 10 );

            /* Set limits to how much current will be sent through the motor */
      m_motor.ConfigPeakCurrentLimit(60);    // 60 works here for miniCIMs
      m_motor.ConfigPeakCurrentDuration(1);  // 1000 milliseconds (for 60 Amps)
                                             // works fine here, with 40 for
                                             // ConfigContinuousCurrentLimit(),
                                             // but we can reduce to 10, 1, 10
                                             // for safety while debugging
      m_motor.ConfigContinuousCurrentLimit(40);
      m_motor.EnableCurrentLimit(true);

                                          // Config 100% motor output to 12.0V
      m_motor.ConfigVoltageCompSaturation( 12.0 );
      m_motor.EnableVoltageCompensation( true );

                 /* Set Closed Loop PIDF gains in slot0 - see documentation */
                                                    // if encoder is connected
      if ( OK == m_motor.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motor.SelectProfileSlot( 0, 0 );
         m_motor.Config_kF( 0, 0.15,   10 );
         m_motor.Config_kP( 0, 0.2,    10 );
         m_motor.Config_kI( 0, 0.0002, 10 );
         m_motor.Config_kD( 0, 10.0,   10 );
      } else {
         m_motor.SelectProfileSlot( 0, 0 );
         m_motor.Config_kF( 0, 0.15, 10 );
         m_motor.Config_kP( 0, 0.0, 10 );
         m_motor.Config_kI( 0, 0.0, 10 );
         m_motor.Config_kD( 0, 0.0, 10 );
      }

                /* Set acceleration and cruise velocity - see documentation */
      m_motor.ConfigMotionCruiseVelocity( 1500, 10 );
      m_motor.ConfigMotionAcceleration(   1500, 10 );

               /* Set ramp rate (how fast motor accelerates or decelerates) */
      m_motor.ConfigClosedloopRamp(0.0);
      m_motor.ConfigOpenloopRamp(  0.0);

      m_motor.SetNeutralMode( NeutralMode::Coast );

   }      // MotorInit()


      /*---------------------------------------------------------------------*/
      /* RobotInit()                                                         */
      /* This function is called once when the robot is powered up.          */
      /* It performs preliminary initialization of all hardware that will    */
      /* be used in Autonomous or Teleop modes.                              */
      /*---------------------------------------------------------------------*/
   void RobotInit() {
      static int iRobotInitCallCount = 0;

      iRobotInitCallCount++;

      if ( 1 == iRobotInitCallCount ) {
                                // start a thread processing USB camera images
         std::thread visionThread(VisionThread);
         visionThread.detach();
      }

      powercellOnVideo.TestMode = false;

      m_motorLSSlave1.Follow(m_motorLSMaster);
      m_motorLSSlave2.Follow(m_motorLSMaster);
      m_motorRSSlave1.Follow(m_motorRSMaster);
      m_motorRSSlave2.Follow(m_motorRSMaster);
      m_motorConveySlave.Follow(m_motorConveyMaster);

      MotorInit( m_motorLSMaster );
      MotorInit( m_motorRSMaster );
      MotorInit( m_motorTopShooter );
      MotorInit( m_motorBotShooter );
      MotorInit( m_motorClimberPole );
                                    // invert encoder value positive/negative
                                    // and motor direction, for some motors.
      //    m_motorLSMaster.SetSensorPhase(true);
      m_motorLSMaster.SetSensorPhase(false);
      // m_motorLSMaster.SetInverted(false);
      //    m_motorRSMaster.SetSensorPhase(true);
      m_motorRSMaster.SetSensorPhase(false);
      // m_motorRSMaster.SetInverted(false);
      //    m_motorTopShooter.SetSensorPhase(false);
      m_motorTopShooter.SetSensorPhase(true);
      // m_motorTopShooter.SetInverted(false);
      //    m_motorBotShooter.SetSensorPhase(false);
      m_motorBotShooter.SetSensorPhase(true);
      // m_motorBotShooter.SetInverted(false);
      m_motorClimberPole.SetSensorPhase(false);
      m_motorClimberPole.SetInverted(false);

            /* Set Closed Loop PIDF gains in slot0 - see documentation */
                                                    // if encoder is connected
      if ( OK == m_motorLSMaster.ConfigSelectedFeedbackSensor(
                            FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorLSMaster.SelectProfileSlot( 0, 0 );
         m_motorLSMaster.Config_kF( 0, 0.025,   10 );   // these work well for .15
         m_motorLSMaster.Config_kP( 0, 0.02,    10 );   // RPMs above ~200 0.2
         m_motorLSMaster.Config_kI( 0, 0.0,     10 );// 0.0002
         m_motorLSMaster.Config_kD( 0, 0.0,     10 );
         cout << "LSMaster encoder is okay" << endl;
      } else {
         m_motorLSMaster.SelectProfileSlot( 0, 0 );
         m_motorLSMaster.Config_kF( 0, 0.15, 10 );   // may have to be higher
         m_motorLSMaster.Config_kP( 0, 0.0,  10 );
         m_motorLSMaster.Config_kI( 0, 0.0,  10 );
         m_motorLSMaster.Config_kD( 0, 0.0,  10 );
         cout << "LSMaster encoder is DISCONNECTED" << endl;
      }

                                                    // if encoder is connected
      if ( OK == m_motorRSMaster.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorRSMaster.SelectProfileSlot( 0, 0 );
         m_motorRSMaster.Config_kF( 0, 0.025,   10 );   // these work well for .15
         m_motorRSMaster.Config_kP( 0, 0.02,    10 );   // RPMs above ~200 0.2
         m_motorRSMaster.Config_kI( 0, 0.0, 10 ); // 0.0002
         m_motorRSMaster.Config_kD( 0, 0.0,   10 ); // 10.0
         cout << "RSMaster encoder is okay" << endl;

      } else {
         m_motorRSMaster.SelectProfileSlot( 0, 0 );
         m_motorRSMaster.Config_kF( 0, 0.15, 10 );   // may have to be higher
         m_motorRSMaster.Config_kP( 0, 0.0,  10 );
         m_motorRSMaster.Config_kI( 0, 0.0,  10 );
         m_motorRSMaster.Config_kD( 0, 0.0,  10 );
         cout << "RSMaster encoder is DISCONNECTED" << endl;
      }

      m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorLSMaster.SetIntegralAccumulator( 0.0 );
      m_motorRSMaster.SetIntegralAccumulator( 0.0 );
      LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      Team4918Drive( 0.0, 0.0 );          // make sure drive motors are stopped

                                                     // if encoder is connected
      if ( OK == m_motorTopShooter.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorTopShooter.SelectProfileSlot( 0, 0 );
         m_motorTopShooter.Config_kF( 0, 0.02,    10 ); // 0.02 0.01
         m_motorTopShooter.Config_kP( 0, 0.3,     10 ); // 0.2 0.08
         m_motorTopShooter.Config_kI( 0, 0.0,     10 ); // was 0.0 0.00008
         m_motorTopShooter.Config_kD( 0, 0.8,     10 );
      } else {
         m_motorTopShooter.SelectProfileSlot( 0, 0 );
         m_motorTopShooter.Config_kF( 0, 0.01, 10 );   // may have to be higher
         m_motorTopShooter.Config_kP( 0, 0.0,  10 );
         m_motorTopShooter.Config_kI( 0, 0.0,  10 );
         m_motorTopShooter.Config_kD( 0, 0.0,  10 );
      }

                                                     // if encoder is connected
      if ( OK == m_motorBotShooter.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorBotShooter.SelectProfileSlot( 0, 0 );
         m_motorBotShooter.Config_kF( 0, 0.02,    10 ); // 0.02 0.01
         m_motorBotShooter.Config_kP( 0, 0.3,     10 ); // 0.2 0.08
         m_motorBotShooter.Config_kI( 0, 0.0,     10 ); // 0.0 0.00008
         m_motorBotShooter.Config_kD( 0, 0.8,     10 );
      } else {
         m_motorBotShooter.SelectProfileSlot( 0, 0 );
         m_motorBotShooter.Config_kF( 0, 0.01, 10 );   // may have to be higher
         m_motorBotShooter.Config_kP( 0, 0.0,  10 );
         m_motorBotShooter.Config_kI( 0, 0.0,  10 );
         m_motorBotShooter.Config_kD( 0, 0.0,  10 );
      }

      m_motorConveyMaster.SetNeutralMode( NeutralMode::Brake );

      sCurrState.highGear = false;
      m_shiftingSolenoid.Set(false);
      iCallCount++;
   }      // RobotInit()


      /*---------------------------------------------------------------------*/
      /* RobotPeriodic()                                                     */
      /* This function is called every 20 milliseconds, regardless of what   */
      /* mode the robot is in (Autonomous, Teleop, or Test mode).            */
      /* If the robot is in one of those modes, this function is called      */
      /* immediately *after* the call to AutonomousPeriodic(),               */
      /* TeleopPeriodic(), or TestPeriodic().                                */
      /* If not in one of those 3 modes, the Roborio cannot drive any        */
      /* motors, but it can still check the joystick, joystick/console       */
      /* buttons, and sensors.                                               */
      /*---------------------------------------------------------------------*/
//    void RobotPeriodic() override {
//       static int iCallCount = 0;      iCallCount++;
// 
//      GetAllVariables();
// 
//      SwitchCameraIfNecessary();
//      cout << "RobotPeriodic()" << endl;
//    }


      /*---------------------------------------------------------------------*/
      /* DisabledInit()                                                      */
      /* This function is called once when the robot is disabled.            */
      /*---------------------------------------------------------------------*/
   void DisabledInit() override {

      m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorLSMaster.SetIntegralAccumulator( 0.0 );
      m_motorRSMaster.SetIntegralAccumulator( 0.0 );
      LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      Team4918Drive( 0.0, 0.0 );          // make sure drive motors are stopped

      powercellOnVideo.TestMode = false;
      // Should power off all motors here.
   }


      /*---------------------------------------------------------------------*/
      /* TestInit()                                                          */
      /* This function is called once when the robot enters Test mode.       */
      /*---------------------------------------------------------------------*/
   void TestInit() override {
      limenttable->PutNumber( "ledMode", 1 );                  // turn LEDs off
      powercellOnVideo.TestMode = true;  // display the center pixel HSV values
   }


      /*---------------------------------------------------------------------*/
      /* TestPeriodic()                                                      */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Test mode.                                                    */
      /* In this mode the Roborio cannot drive any motors, but it can read   */
      /* the joystick, joystick/console buttons, and sensors.                */
      /*---------------------------------------------------------------------*/
   void TestPeriodic() override {
      GetAllVariables();
      iCallCount++;

      SwitchCameraIfNecessary();

      if ( 0 == iCallCount%10000 ) {                           // every 200 seconds
         // cout << "Sonar0 sensor 0: " << distSensor0.GetAverageValue() << endl;
         // cout << "Sonar0 average voltage:  " << distSensor0.GetAverageVoltage()
         //      << endl;
         // cout << "Sonar0 voltage:  " << distSensor0.GetVoltage() << endl;
             // convert to inches, with 100 centimeters/volt and 2.54 cm/inch
         cout << "Sonar0 distance: " << distSensor0.GetVoltage() * 100 / 2.54
              << " inches (" << distSensor0.GetVoltage() << ")." << endl; 
      }

      if ( 0 == iCallCount%100 )  {   // every 2 seconds
         JoystickDisplay();
      }
//      cout << "TestPeriodic()" << endl;
   }


      /*---------------------------------------------------------------------*/
      /* AutonomousInit()                                                    */
      /* This function is called once when the robot enters Autonomous mode. */
      /*---------------------------------------------------------------------*/
   void AutonomousInit() override {
      RobotInit();
      m_compressor.Start();
      limenttable->PutNumber( "ledMode", 3 );                   // turn LEDs on
      cout << "shoot 3 balls" << endl;
      // m_drive.StopMotor();
      iCallCount=0;
      GetAllVariables();
      sCurrState.initialYaw = sCurrState.yawPitchRoll[0]; 
      // TurnToHeading( sCurrState.initialYaw, true );
      // DriveToDistance( sCurrState.initialYaw, 0.0, true );
      m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorLSMaster.SetIntegralAccumulator( 0.0 );
      m_motorRSMaster.SetIntegralAccumulator( 0.0 );
      LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      Team4918Drive( 0.0, 0.0 );          // make sure drive motors are stopped

   }      // AutonomousInit()


      /*---------------------------------------------------------------------*/
      /* AutonomousPeriodic()                                                */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Autonomous mode.                                              */
      /*---------------------------------------------------------------------*/
   void AutonomousPeriodic() override {

      // static double dDesiredYaw = 0.0;

      GetAllVariables();

      iCallCount++;

      //m_motorIntake.Set(ControlMode::PercentOutput, -0.4);

      if ( sCurrState.conButton[8] )   {            // Run intake forward.
         m_motorIntake.Set( ControlMode::PercentOutput, -0.4 );
      } else {                                         // Stop the intake.
         m_motorIntake.Set( ControlMode::PercentOutput, 0.0 );
      }

      // m_drive.StopMotor();
      LSMotorState.targetVelocity_UnitsPer100ms = 0;        // Left Side drive
      RSMotorState.targetVelocity_UnitsPer100ms = 0;        // Right Side drive

      // dDesiredYaw = sCurrState.initialYaw;

      if (iCallCount<200) {
         if ( 0 && sCurrState.conButton[10]){
            if ( TurnToHeading( sCurrState.initialYaw+270.0, false ) ) {
               // dDesiredYaw = sCurrState.initialYaw + 270.0;
               iCallCount = 200;
               m_motorLSMaster.SetIntegralAccumulator( 0.0 );
               m_motorRSMaster.SetIntegralAccumulator( 0.0 );
            }
         } else if ( 0 && sCurrState.conButton[11]) {
            if ( TurnToHeading( sCurrState.initialYaw-90.0, false ) ) {
               // dDesiredYaw = sCurrState.initialYaw - 90.0;
               iCallCount = 200;
               m_motorLSMaster.SetIntegralAccumulator( 0.0 );
               m_motorRSMaster.SetIntegralAccumulator( 0.0 );
            }
         } else {
            sCurrState.conY = 1.0;   // Tell RunShooter() to go to low speed
            sPrevState.conY = 0.0;   // as if console joystick was
	                             // newly-pressed downward.
            RunShooter();
	                             // if the motor are both spinning fast
	    if ( ( 1800 * 4096 / 600 <
                   abs( m_motorTopShooter.GetSelectedSensorVelocity() ) ) &&
                 ( 2600 * 4096 / 600 <
                   abs( m_motorBotShooter.GetSelectedSensorVelocity() ) )   ) {
	                  // run the conveyor to shoot the balls
               m_motorConveyMaster.Set( ControlMode::PercentOutput, -0.8 );
	    }
            // dDesiredYaw = sCurrState.initialYaw;
            // iCallCount = 200;
         }
      } else if ( iCallCount<250 ) {
         m_motorConveyMaster.Set( ControlMode::PercentOutput, 0.0 );

         sCurrState.conY = 0.0;   // Tell RunShooter() to stop
         sPrevState.conY = 1.0;   // as if console joystick was
         RunShooter();            // newly-released.
                                                       // go backward 2 feet
         Team4918Drive( 0.2, 0.0 );           // change to DriveToDistance()
		                              // when the Pigeon is connected.
  //       if ( DriveToDistance( dDesiredYaw, -2.0, false ) ) {
  //          iCallCount = 250;
  //       }
//         LSMotorState.targetVelocity_UnitsPer100ms = 100.0 * 4096 / 600;
//         RSMotorState.targetVelocity_UnitsPer100ms =-100.0 * 4096 / 600;
//         m_motorLSMaster.Set( ControlMode::Velocity, 
//                              LSMotorState.targetVelocity_UnitsPer100ms );
//         m_motorRSMaster.Set( ControlMode::Velocity, 
//                              RSMotorState.targetVelocity_UnitsPer100ms );
      } else if ( iCallCount<300 ) {
         m_motorConveyMaster.Set( ControlMode::PercentOutput, 0.0 );

         sCurrState.conY = 0.0;   // Tell RunShooter() to stop
         sPrevState.conY = 1.0;   // as if console joystick was
         RunShooter();            // newly-released.
                                                       // go backward 2 feet
         Team4918Drive( 0.3, 0.0 );           // change to DriveToDistance()
		                              // when the Pigeon is connected.
      } else if ( iCallCount<325 ) {
         m_motorConveyMaster.Set( ControlMode::PercentOutput, 0.0 );

         sCurrState.conY = 0.0;   // Tell RunShooter() to stop
         sPrevState.conY = 1.0;   // as if console joystick was
         RunShooter();            // newly-released.
                                                       // go backward 2 feet
         Team4918Drive( 0.1, 0.0 );           // change to DriveToDistance()
		                              // when the Pigeon is connected.
      } else {
         Team4918Drive( 0.0, 0.0 );
         sCurrState.conY = 0.0;   // Tell RunShooter() to stop
         sPrevState.conY = 1.0;   // as if console joystick was
         RunShooter();            // newly-released.
//         m_motorLSMaster.Config_kF( 0, 0.15, 10 );     // 0.15 normally
//         m_motorRSMaster.Config_kF( 0, 0.15, 10 );
//         LSMotorState.targetVelocity_UnitsPer100ms =   0.0 * 4096 / 600;
//         RSMotorState.targetVelocity_UnitsPer100ms =   0.0 * 4096 / 600;
//         m_motorLSMaster.Set( ControlMode::Velocity, 
//                              LSMotorState.targetVelocity_UnitsPer100ms );
//         m_motorRSMaster.Set( ControlMode::Velocity, 
//                              RSMotorState.targetVelocity_UnitsPer100ms );
      }

      motorFindMinMaxVelocity( m_motorLSMaster, LSMotorState );
      motorFindMinMaxVelocity( m_motorRSMaster, RSMotorState );

      if ( 0 == iCallCount%50 ) {
         MotorDisplay( "LS:", m_motorLSMaster, LSMotorState );
         MotorDisplay( "RS:", m_motorRSMaster, RSMotorState );
         IMUOrientationDisplay();
      }
   }      // AutonomousPeriodic()


      /*---------------------------------------------------------------------*/
      /* TeleopInit()                                                        */
      /* This function is called once when the robot enters Teleop mode.     */
      /*---------------------------------------------------------------------*/
   void TeleopInit() override {
      RobotInit();
      m_compressor.Start();
                                                    // zero the drive encoders
      m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorLSMaster.SetIntegralAccumulator( 0.0 );
      m_motorRSMaster.SetIntegralAccumulator( 0.0 );
      LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      Team4918Drive( 0.0, 0.0 );          // make sure drive motors are stopped
   }      // TeleopInit()


      /*---------------------------------------------------------------------*/
      /* TeleopPeriodic()                                                    */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Teleop mode.                                                  */
      /*---------------------------------------------------------------------*/
   void TeleopPeriodic() override {

      GetAllVariables();  // this is necessary if we use any
                          // of the Canbus variables.

      //m_motorIntake.Set(ControlMode::PercentOutput, -0.4);

      if ( sCurrState.conButton[8] )   {            // Run intake forward.
         m_motorIntake.Set( ControlMode::PercentOutput, -0.4 );
      } else {                                         // Stop the intake.
         m_motorIntake.Set( ControlMode::PercentOutput, 0.0 );
      } 


      RunDriveMotors();

      RunShooter();

      Shoot();

      RunConveyor();

      RunColorWheel();

      RunClimberPole();
      RunClimberWinch();
      SwitchCameraIfNecessary();


      sPrevState = sCurrState;

      if ( 0 == iCallCount%100000 )  {   // every 20 seconds
         cout << "TelPeriodic loop duration: ";
         cout << frc::GetTime() - dTimeOfLastCall << endl;
               // use frc:Timer::GetFPGATimestamp() instead?
      }

      iCallCount++;
   }      // TeleopPeriodic()
 
};      // class Robot definition (derives from frc::TimedRobot )


Robot::sPowercellOnVideo Robot::powercellOnVideo = { true, 0, 0, -1, false, false };

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

