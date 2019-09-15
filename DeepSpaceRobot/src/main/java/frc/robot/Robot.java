/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import static frc.robot.Config.Camera.LED_OFF;
import static frc.robot.Config.RamseteControllerConstants.DRIVE_RADIUS;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_ACTUAL_POSITION;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_LEFT_ENCODER;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_RIGHT_ENCODER;
import static frc.robot.Config.SmartDashboardKeys.IS_RIGHT_AUTON;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_TARGET;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CLAW_ForwardSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CLAW_ReverseSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_ForwardSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_HEIGHT;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_ReverseSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_TARGET;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_ForwardSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_ReverseSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_TARGET;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_INTAKE_OPEN;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_LOWER_LIMIT;
import static frc.robot.Config.SmartDashboardKeys.USE_AUTON;
import static frc.robot.RobotMap.clawRotationMotor;
import static frc.robot.RobotMap.elevatorMotor;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;
import static frc.robot.RobotMap.hatchRotationMotor;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config.Camera;
import frc.robot.command.teleop.FollowTrajectory;
import frc.robot.command.teleop.util.Transform;
import frc.robot.config.LimitedRobot;
import frc.robot.robot.CompDeepSpace;
import frc.robot.robot.CompPowerUp;
import frc.robot.robot.CompSteamWorks;
import frc.robot.robot.PracticeDeepSpace;
import frc.robot.subsystem.Drivetrain;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;
import frc.robot.util.RobotBuilder;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  public static final boolean isCompBot;
  public static final LimitedRobot currentRobot;
  public static final Drivetrain drivetrain;
  public static final ElevatorCargoHatchSubsystem godSubsystem;
  public static final SendableChooser<Transform> transformSendableChooser = new SendableChooser<>();
  private static final RobotBuilder<LimitedRobot> robotBuilder;

  static {
    isCompBot = new DigitalInput(9).get();
    SmartDashboard.putBoolean("identifier", isCompBot);
    robotBuilder = new RobotBuilder<>(new CompPowerUp(), new CompSteamWorks(), new PracticeDeepSpace(),
        new CompDeepSpace());
    currentRobot = robotBuilder.getCurrentRobotConfig();
    System.out.println(currentRobot);
    drivetrain = new Drivetrain();
    godSubsystem = new ElevatorCargoHatchSubsystem();
  }

  private boolean hasSetPipeline = false;

  public Robot() {
    super(0.04);
  }

  private static void initHardware() {
    currentRobot.setupController(clawRotationMotor, currentRobot.getCargoSubsystemLimits(), CargoPosition.SAFE);
    currentRobot.setupController(elevatorMotor, currentRobot.getElevatorSubsystemLimits(), ElevatorLevel.HATCH_BASE);
    currentRobot.setupController(hatchRotationMotor, currentRobot.getHatchSubsystemLimits(), HatchPosition.SAFE);
  }

  private static void initShuffleBoard() {

    SmartDashboard.putNumber(MOTORS_ELEVATOR_HEIGHT, 0);
    SmartDashboard.putNumber(MOTORS_HATCH_ANGLE, 0);
    SmartDashboard.putBoolean(MOTORS_INTAKE_OPEN, false);
    SmartDashboard.putNumber(MOTORS_CARGO_ANGLE, 0);
    SmartDashboard.putBoolean(MOTORS_LOWER_LIMIT, false);
    SmartDashboard.putBoolean(MOTORS_ELEVATOR_ForwardSoftLimit, false);
    SmartDashboard.putBoolean(MOTORS_ELEVATOR_ReverseSoftLimit, false);
    SmartDashboard.putBoolean(MOTORS_CLAW_ForwardSoftLimit, false);
    SmartDashboard.putBoolean(MOTORS_CLAW_ReverseSoftLimit, false);
    SmartDashboard.putBoolean(MOTORS_HATCH_ForwardSoftLimit, false);
    SmartDashboard.putBoolean(MOTORS_HATCH_ReverseSoftLimit, false);

    SmartDashboard.putNumber(MOTORS_ELEVATOR_POWER, 0);
    SmartDashboard.putNumber(MOTORS_ELEVATOR_TARGET, 0);
    SmartDashboard.putNumber(MOTORS_HATCH_POWER, 0);
    SmartDashboard.putNumber(MOTORS_HATCH_TARGET, 0);
    SmartDashboard.putNumber(MOTORS_CARGO_POWER, 0);
    SmartDashboard.putNumber(MOTORS_CARGO_TARGET, 0);

    SmartDashboard.putBoolean(IS_RIGHT_AUTON, true);
    SmartDashboard.putBoolean(USE_AUTON, false);
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.reset();

    drivetrain.getController().getCameraReader().startCollecting();

    initShuffleBoard();

    initCamera();

    initHardware();
  }

  private void initCamera() {
//    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
//    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);

//    UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
//    usbCamera.setVideoMode(PixelFormat.kYUYV, 640, 480, 30);
//    usbCamera.setFPS(FPS);

//    new Thread(() -> {
//      CameraServer cameraServer = CameraServer.getInstance();
//
//      UsbCamera fishEyeCamera = cameraServer.startAutomaticCapture();
//
////      if (!fishEyeCamera.isConnected() || !fishEyeCamera.isValid() || !fishEyeCamera.isEnabled()) {
////        fishEyeCamera.close();
////        return;
////      }
//
//      fishEyeCamera.setResolution(WIDTH, HEIGHT);
//
//      CvSink cvSink = cameraServer.getVideo();
//      CvSource outputStream = cameraServer.putVideo(DRIVETEAM_FISHEYE_CAMERA, WIDTH, HEIGHT);
//      outputStream.setFPS(FPS);
//
//      MjpegServer fisheyeServer = cameraServer.addServer("Fisheye Camera Server");
//      fisheyeServer.setSource(outputStream);
//      fisheyeServer.setCompression(DEFAULT_CAMERA_COMPRESSION_QUALITY);
//      fisheyeServer.setDefaultCompression(DEFAULT_CAMERA_COMPRESSION_QUALITY);
//      fisheyeServer.setResolution(WIDTH, HEIGHT);
//
////      fisheyeServer.getProperty("compression").set(DEFAULT_CAMERA_COMPRESSION_QUALITY);
////      fisheyeServer.getProperty("default_compression").set(DEFAULT_CAMERA_COMPRESSION_QUALITY);
//
//      Mat source = new Mat();
//
//      System.out.println("Fisheye Camera Connected");
//      while (!Thread.interrupted()) {
//        cvSink.grabFrame(source);
//        Core.flip(source, source, -1);
//        ParkingLines.setFocusPoint(
//            SmartDashboard.getNumber(PARKING_LINE_FOCUS_X, WIDTH / 2.0),
//            SmartDashboard.getNumber(PARKING_LINE_FOCUS_Y, HEIGHT / 2.0)
//        );
//        ParkingLines.setPercentage(SmartDashboard.getNumber(PARKING_LINE_PERCENTAGE, 1.0));
//        ParkingLines.setXOffset(SmartDashboard.getNumber(PARKING_LINE_OFFSET, 0));
//
//        ParkingLines.drawParkingLines(source);
//        outputStream.putFrame(source);
//      }
//    }).start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like diagnostics that you want
   * ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if (!hasSetPipeline) {
      NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

      if (limelight.containsKey("pipeline")) {
        limelight.getEntry("pipeline").setDouble(Camera.DRIVER_PIPELINE);
        hasSetPipeline = true;

        limelight.getEntry("ledMode").setNumber(LED_OFF);
      }
    }

    SmartDashboard.putNumber(DRIVETRAIN_LEFT_ENCODER, encoderLeft.getDistance());
    SmartDashboard.putNumber(DRIVETRAIN_RIGHT_ENCODER, encoderRight.getDistance());
    SmartDashboard.putString(DRIVETRAIN_ACTUAL_POSITION, String.valueOf(drivetrain.getActualPosition()));
    // System.out.println("robot Periodic");
//    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
//    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You can use it to reset any subsystem
   * information you want to clear when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    godSubsystem.setAutonomousEnabled(false);
    godSubsystem.setEnabled(false);
    drivetrain.cancelControllerMotion();
    drivetrain.getMotionLogger().writeMotionDataCSV(true);
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }


  @Override
  public void autonomousInit() {
//    godSubsystem.setEnabled(false);
    godSubsystem.setEnabled(true);
    godSubsystem.setAutonomousEnabled(SmartDashboard.getBoolean(USE_AUTON, false));
//    godSubsystem.setAutonomousEnabled(false);
    drivetrain.cancelControllerMotion();
    drivetrain.clearControllerMotions();
    drivetrain.shiftUp();

    new FollowTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), "e", DRIVE_RADIUS).start();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    godSubsystem.setAutonomousEnabled(false);
    godSubsystem.setEnabled(true);
    drivetrain.cancelControllerMotion();
    drivetrain.shiftUp();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public String toString() {
    return "Robot{" +
        "hasSetPipeline=" + hasSetPipeline +
        "} " + super.toString();
  }
}
