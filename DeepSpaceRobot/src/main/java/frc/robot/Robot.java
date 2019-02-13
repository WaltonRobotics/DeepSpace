/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.test.CargoIntakerTestCommand;
import frc.robot.test.ElevatorTestCommand;
import frc.robot.test.EncoderTestCommand;
import frc.robot.test.HatchIntakerTestCommand;
import frc.robot.test.MotorTestCommand;
import frc.robot.command.teleop.util.NormalSpeed;
import frc.robot.command.teleop.util.Sigmoid;
import frc.robot.command.teleop.util.Sqrt;
import frc.robot.command.teleop.util.Transform;
import frc.robot.robot.CompPowerUp;
import frc.robot.robot.CompSteamWorks;
import frc.robot.subsystem.Drivetrain;
import frc.robot.util.RobotBuilder;
import frc.robot.util.TestRunner;
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
  private static final int DEFAULT_CAMERA_COMPRESSION_QUALITY = 40; // between 0 and 100, 100 being the max, -1 being left to Shuffleboard

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

    SmartDashboard.putNumber("dx", 2);
    SmartDashboard.putNumber("dy", .5);
    SmartDashboard.putNumber("angle", 30);
  }

  private void initCamera() {
    CameraServer cameraServer = CameraServer.getInstance();

    UsbCamera fishEyeCamera = new UsbCamera("Fisheye Camera", 0);
    fishEyeCamera.setResolution(320, 240);
    fishEyeCamera.setFPS(30);

    cameraServer.addCamera(fishEyeCamera);

    MjpegServer fisheyeServer = CameraServer.getInstance().addServer("Fisheye Camera Server");
    fisheyeServer.setSource(fishEyeCamera);

    fisheyeServer.getProperty("compression").set(DEFAULT_CAMERA_COMPRESSION_QUALITY);
    fisheyeServer.getProperty("default_compression").set(DEFAULT_CAMERA_COMPRESSION_QUALITY);

    if (!fishEyeCamera.isConnected()) {
      fishEyeCamera.close();
      fisheyeServer.close();
    }
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
    TestRunner motorTestCommand = new TestRunner();
    motorTestCommand.addSequential(new MotorTestCommand());
    motorTestCommand.addSequential(new EncoderTestCommand());
    motorTestCommand.addSequential(new CargoIntakerTestCommand());
    motorTestCommand.addSequential(new HatchIntakerTestCommand());
    motorTestCommand.addSequential(new ElevatorTestCommand());
    motorTestCommand.start();
    SmartDashboard.putData("Test commands", motorTestCommand);
    System.out.println(motorTestCommand.isCompleted());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }


  @Override
  public void testInit() {
    System.out.println("testInit");
}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
      Scheduler.getInstance().run();
  }
}
