/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Config.Camera.DEFAULT_CAMERA_COMPRESSION_QUALITY;
import static frc.robot.Config.Camera.FPS;
import static frc.robot.Config.Camera.HEIGHT;
import static frc.robot.Config.Camera.WIDTH;
import static frc.robot.Config.SmartDashboardKeys.PARKING_LINE_FOCUS_X;
import static frc.robot.Config.SmartDashboardKeys.PARKING_LINE_FOCUS_Y;
import static frc.robot.Config.SmartDashboardKeys.PARKING_LINE_OFFSET;
import static frc.robot.Config.SmartDashboardKeys.PARKING_LINE_PERCENTAGE;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.command.teleop.util.NormalSpeed;
import frc.robot.command.teleop.util.Sigmoid;
import frc.robot.command.teleop.util.Sqrt;
import frc.robot.command.teleop.util.Transform;
import frc.robot.robot.CompPowerUp;
import frc.robot.robot.CompSteamWorks;
import frc.robot.subsystem.Drivetrain;
import frc.robot.util.ParkingLines;
import frc.robot.util.RobotBuilder;
import org.opencv.core.Mat;
import org.waltonrobotics.command.SimpleCameraPositioning;
import org.waltonrobotics.controller.CameraData;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.util.RobotConfig;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  public static final RobotConfig currentRobot;
  public static final Drivetrain drivetrain;
  private static final RobotBuilder robotBuilder;

  static {
    robotBuilder = new RobotBuilder(new CompPowerUp(), new CompSteamWorks());
    currentRobot = robotBuilder.getCurrentRobotConfig();
    drivetrain = new Drivetrain();
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.reset();

    initShuffleBoard();

    initCamera();
  }

  private void initShuffleBoard() {
    SendableChooser<Transform> sendableChooser = new SendableChooser<>();
    sendableChooser.setDefaultOption("Normal", new NormalSpeed());
    sendableChooser.addOption("Sigmoid", new Sigmoid());
    sendableChooser.addOption("Sqrt", new Sqrt());

    SmartDashboard.putData("Transform Select", sendableChooser);

    SmartDashboard.putNumber("dx", -2);
    SmartDashboard.putNumber("dy", -.5);
    SmartDashboard.putNumber("angle", 30);
  }

  private void initCamera() {
    new Thread(() -> {
      SmartDashboard.putNumber(PARKING_LINE_OFFSET, 0);
      SmartDashboard.putNumber(PARKING_LINE_FOCUS_X, WIDTH / 2.0);
      SmartDashboard.putNumber(PARKING_LINE_FOCUS_Y, HEIGHT / 2.0);
      SmartDashboard.putNumber(PARKING_LINE_PERCENTAGE, 1.0);

      CameraServer cameraServer = CameraServer.getInstance();

      UsbCamera fishEyeCamera = cameraServer.startAutomaticCapture();
      fishEyeCamera.setResolution(WIDTH, HEIGHT);

      CvSink cvSink = cameraServer.getVideo();
      CvSource outputStream = cameraServer.putVideo("Fisheye Camera", WIDTH, HEIGHT);
      outputStream.setFPS(FPS);

      MjpegServer fisheyeServer = cameraServer.addServer("Fisheye Camera Server");
      fisheyeServer.setSource(outputStream);

      fisheyeServer.getProperty("compression").set(DEFAULT_CAMERA_COMPRESSION_QUALITY);
      fisheyeServer.getProperty("default_compression").set(DEFAULT_CAMERA_COMPRESSION_QUALITY);

      if (!fishEyeCamera.isConnected()) {
        fishEyeCamera.close();
        fisheyeServer.close();
        return;
      }

      Mat source = new Mat();

      while (!Thread.interrupted()) {
        cvSink.grabFrame(source);
        ParkingLines.setFocusPoint(
            SmartDashboard.getNumber(PARKING_LINE_FOCUS_X, WIDTH / 2.0),
            SmartDashboard.getNumber(PARKING_LINE_FOCUS_Y, HEIGHT / 2.0)
        );
        ParkingLines.setPercentage(SmartDashboard.getNumber(PARKING_LINE_PERCENTAGE, 1));
        ParkingLines.setXOffset(SmartDashboard.getNumber(PARKING_LINE_OFFSET, 0));

        ParkingLines.drawParkingLines(source);
        outputStream.putFrame(source);
      }
    }).start();
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
    SmartDashboard.putNumber("Encoder Left", encoderLeft.getDistance());
    SmartDashboard.putNumber("Encoder Right", encoderRight.getDistance());
    SmartDashboard.putString("Position", String.valueOf(drivetrain.getActualPosition()));
    // System.out.println("robot Periodic");
    drivetrain.getCurrentCameraData();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You can use it to reset any subsystem
   * information you want to clear when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.getMotionLogger().writeMotionDataCSV(true);
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different autonomous modes using
   * the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the auto name from the text box below the
   * Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to the switch structure below with
   * additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.startControllerMotion();
    drivetrain.reset();
    drivetrain.shiftUp();

    CameraData currentCameraData = drivetrain.getController().getCurrentCameraData();
    System.out.println(currentCameraData);

    if (currentCameraData.getNumberOfTargets() > 0) {
//    if (currentCameraData.getNumberOfTargets() >= 0) {
      double x = SmartDashboard.getNumber("dx", -2);
      double y = SmartDashboard.getNumber("dy", -.5);
      double angle = SmartDashboard.getNumber("angle", 0);

      SimpleCameraPositioning simpleCameraPositioning = SimpleCameraPositioning
//          .toCameraTarget(new Pose(x, y, StrictMath.toRadians(angle)));
          .toCameraTarget(currentCameraData);

      simpleCameraPositioning.start();
    }
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
    drivetrain.cancelControllerMotion();
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
}
