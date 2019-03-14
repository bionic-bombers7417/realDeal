/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DMC60;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  AHRS ahrs;

  private final PWMVictorSPX left1 = new PWMVictorSPX(0);
  private final PWMVictorSPX right1 = new PWMVictorSPX(2);
  private final PWMVictorSPX left2 = new PWMVictorSPX(1);
  private final PWMVictorSPX right2 = new PWMVictorSPX(3);
  private final PWMVictorSPX elevator = new PWMVictorSPX(5);
  private final PWMVictorSPX grabber = new PWMVictorSPX(9);
  private final Joystick m_stick = new Joystick(0);
  private final Joystick m_joy = new Joystick(1);
  private final Timer m_timer = new Timer();
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    CameraServer.getInstance().startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    m_timer.reset();
    m_timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    ///if (m_timer.get() < 2.0) {
      //m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    //} else {
      //m_robotDrive.stopMotor(); // stop robot
    //}
  }
  @Override
  public void teleopInit(){}

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //drive code for two stick drive on Logitech controller
    left1.set(-m_stick.getRawAxis(1)*.7 + m_stick.getRawAxis(4)*.5);
    left2.set(-m_stick.getRawAxis(1)*.7 + m_stick.getRawAxis(4)*.5);
    right1.set(m_stick.getRawAxis(1)*.7 + m_stick.getRawAxis(4)*.5);
    right2.set(m_stick.getRawAxis(1)*.7 + m_stick.getRawAxis(4)*.5);

    if(m_joy.getRawButton(1)){
      elevator.set(1);
    }else if(m_joy.getRawButton(2)){
      elevator.set(-1);
    }else{
      //stops the elevator from gradually decreasing in height
      elevator.set(-.08);
    }

  //grabber motors
  if(m_joy.getRawButton(9)){
    m_timer.reset();
    m_timer.start();
    while(true){
      
      if (m_timer.get() < 0.25) {
        grabber.set(.3);
        elevator.set(-.08);
      } else {
        break;
      }
    }
  }else if(m_joy.getRawButton(10)){
    m_timer.reset();
    m_timer.start();
    while(true){
      
      if (m_timer.get() < 0.25) {
        grabber.set(-.3);
        elevator.set(-.08);
      } else {
        break;
      }
    }
  }else{
    grabber.set(0);
  }

  if(m_joy.getRawButton(11)){
    grabber.set(.6);
  }else if(m_joy.getRawButton(12)){
    grabber.set(-.6);
  }else{
    grabber.set(0);
  }

    //Lift to Ball Stages
    if(m_joy.getRawButton(3)){
      m_timer.reset();
      m_timer.start();
      while(true){
        
        if (m_timer.get() < 1.05) {
          //m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
          elevator.set(-1);
        } else {
          //m_robotDrive.stopMotor(); // stop robot
          elevator.set(-.08);
          break;
        }
      }
    }

    //Lift to Hatch Panel First Stage
    if(m_joy.getRawButton(4)){
      m_timer.reset();
      m_timer.start();
      while(true){
        
        if (m_timer.get() < 0.7) {
          //m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
          elevator.set(-1);
        } else {
          //m_robotDrive.stopMotor(); // stop robot
          elevator.set(-.08);
          break;
        }
      }
    }
    

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }






  public Robot() {
    //stick = new Joystick(0);
    try {
  /***********************************************************************
   * navX-MXP:
   * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
   * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
   * 
   * navX-Micro:
   * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
   * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
   * 
   * Multiple navX-model devices on a single robot are supported.
   ************************************************************************/
        ahrs = new AHRS(SerialPort.Port.kUSB1);
        //ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
        ahrs.enableLogging(true);
    } catch (RuntimeException ex ) {
        DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    Timer.delay(1.0);
  //UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
  //cam.setResolution(640, 480);        
}
/**
     * Runs during autonomous mode
     */
    public void autonomous() {
      Timer.delay(2.0);		//    for 2 seconds
  }

  /**
   * Display navX MXP Sensor Data on Smart Dashboard
   */
  public void operatorControl() {
      while (isOperatorControl() && isEnabled()) {
          
          Timer.delay(0.020);		/* wait for one motor update time period (50Hz)     */
          
          boolean zero_yaw_pressed = false; //stick.getTrigger();
          if ( zero_yaw_pressed ) {
              ahrs.zeroYaw();
          }

          /* Display 6-axis Processed Angle Data                                      */
          SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
          SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
          SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
          SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
          SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
          
          /* Display tilt-corrected, Magnetometer-based heading (requires             */
          /* magnetometer calibration to be useful)                                   */
          
          SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
          
          /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
          SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

          /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
          /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
          
          SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
          SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

          /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
          
          SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
          SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
          SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
          SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

          /* Display estimates of velocity/displacement.  Note that these values are  */
          /* not expected to be accurate enough for estimating robot position on a    */
          /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
          /* of these errors due to single (velocity) integration and especially      */
          /* double (displacement) integration.                                       */
          
          SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
          SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
          SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
          SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
          
          /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
          /* NOTE:  These values are not normally necessary, but are made available   */
          /* for advanced users.  Before using this data, please consider whether     */
          /* the processed data (see above) will suit your needs.                     */
          
          SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
          SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
          SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
          SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
          SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
          SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
          SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
          SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
          SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
          SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
          SmartDashboard.putNumber(   "IMU_Timestamp",        ahrs.getLastSensorTimestamp());
          
          /* Omnimount Yaw Axis Information                                           */
          /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
          AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
          SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
          SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
          
          /* Sensor Board Information                                                 */
          SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
          
          /* Quaternion Data                                                          */
          /* Quaternions are fascinating, and are the most compact representation of  */
          /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
          /* from the Quaternions.  If interested in motion processing, knowledge of  */
          /* Quaternions is highly recommended.                                       */
          SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
          SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
          SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
          SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
          
          /* Connectivity Debugging Support                                           */
          SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
          SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
      }
  }
  









}