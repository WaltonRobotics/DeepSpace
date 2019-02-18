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
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KACC;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KANGLE;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KK;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KL;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KS;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KV;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_MAX_ACCELERATION;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_MAX_VELOCITY;
import static frc.robot.Config.SmartDashboardKeys.DRIVETEAM_FISHEYE_CAMERA;
import static frc.robot.Config.SmartDashboardKeys.DRIVETEAM_TRANSFORM_SELECT;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_ACTUAL_POSITION;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_LEFT_ENCODER;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_LEFT_JOYSTICK_Y;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_LEFT_MOTOR_PERCENT_OUTPUT;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_RIGHT_ENCODER;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_RIGHT_JOYSTICK_Y;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_RIGHT_MOTOR_PERCENT_OUTPUT;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_HEIGHT;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.PARKING_LINE_FOCUS_X;
import static frc.robot.Config.SmartDashboardKeys.PARKING_LINE_FOCUS_Y;
import static frc.robot.Config.SmartDashboardKeys.PARKING_LINE_OFFSET;
import static frc.robot.Config.SmartDashboardKeys.PARKING_LINE_PERCENTAGE;
import static frc.robot.OI.gamepad;
import static frc.robot.RobotMap.clawRotationMotor;
import static frc.robot.RobotMap.elevatorMotor;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;
import static frc.robot.RobotMap.hatchIntake;
import static frc.robot.RobotMap.hatchRotationMotor;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
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
import frc.robot.config.LimitedRobot;
import frc.robot.robot.CompDeepSpace;
import frc.robot.robot.CompPowerUp;
import frc.robot.robot.CompSteamWorks;
import frc.robot.robot.PracticeDeepSpace;
import frc.robot.subsystem.Drivetrain;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClawControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchControlMode;
import frc.robot.util.ParkingLines;
import frc.robot.util.RobotBuilder;
import org.waltonrobotics.command.SimpleCameraPositioning;
import org.waltonrobotics.command.SimpleMotion;
import org.waltonrobotics.util.RobotConfig;
import frc.robot.util.TestRunner;
import org.opencv.core.Mat;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  public static final LimitedRobot currentRobot;
  public static final Drivetrain drivetrain;
  public static final ElevatorCargoHatchSubsystem godSubsystem;
  public static final SendableChooser<Transform> transformSendableChooser = new SendableChooser<>();
  private static final RobotBuilder<LimitedRobot> robotBuilder;


  static {
    robotBuilder = new RobotBuilder<>(new CompPowerUp(), new CompSteamWorks(), new PracticeDeepSpace(),
        new CompDeepSpace());
    currentRobot = robotBuilder.getCurrentRobotConfig();
    System.out.println(currentRobot);
    drivetrain = new Drivetrain();
    godSubsystem = new ElevatorCargoHatchSubsystem();
  }

  public static boolean catchHatch;
  public static boolean outtake;
  public static boolean intake;

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.reset();

    initShuffleBoard();

    initCamera();

    initHardware();
  }

  private void initHardware() {
    currentRobot.setupController(clawRotationMotor, currentRobot.getCargoSubsystemLimits(), null);
    currentRobot.setupController(elevatorMotor, currentRobot.getElevatorSubsystemLimits(), null);
    currentRobot.setupController(hatchRotationMotor, currentRobot.getHatchSubsystemLimits(), null);
  }

  private void initShuffleBoard() {

    transformSendableChooser.setDefaultOption("Normal", new NormalSpeed());
    transformSendableChooser.addOption("Sigmoid", new Sigmoid());
    transformSendableChooser.addOption("Sqrt", new Sqrt());

    SmartDashboard.putData(DRIVETEAM_TRANSFORM_SELECT, transformSendableChooser);

    SmartDashboard.putNumber(CONSTANTS_KV, currentRobot.getKV());
    SmartDashboard.putNumber(CONSTANTS_KACC, currentRobot.getKAcc());
    SmartDashboard.putNumber(CONSTANTS_KK, currentRobot.getKK());
    SmartDashboard.putNumber(CONSTANTS_KS, currentRobot.getKS());
    SmartDashboard.putNumber(CONSTANTS_KANGLE, currentRobot.getKAng());
    SmartDashboard.putNumber(CONSTANTS_MAX_VELOCITY, currentRobot.getMaxVelocity());
    SmartDashboard.putNumber(CONSTANTS_MAX_ACCELERATION, currentRobot.getMaxAcceleration());
    SmartDashboard.putNumber(CONSTANTS_KL, currentRobot.getKL());

    SmartDashboard.putNumber(MOTORS_ELEVATOR_HEIGHT, 0);
    SmartDashboard.putNumber(MOTORS_HATCH_ANGLE, 0);
    SmartDashboard.putNumber(MOTORS_CARGO_ANGLE, 0);

    SmartDashboard.putNumber(PARKING_LINE_OFFSET, 0);
    SmartDashboard.putNumber(PARKING_LINE_FOCUS_X, WIDTH / 2.0);
    SmartDashboard.putNumber(PARKING_LINE_FOCUS_Y, HEIGHT / 2.0);
    SmartDashboard.putNumber(PARKING_LINE_PERCENTAGE, 1.0);

    SmartDashboard.putString(DRIVETRAIN_ACTUAL_POSITION, "No position has been reported");
    SmartDashboard.putNumber(DRIVETRAIN_RIGHT_ENCODER, 0);
    SmartDashboard.putNumber(DRIVETRAIN_LEFT_ENCODER, 0);
    SmartDashboard.putNumber(DRIVETRAIN_LEFT_JOYSTICK_Y, 0);
    SmartDashboard.putNumber(DRIVETRAIN_RIGHT_JOYSTICK_Y, 0);
    SmartDashboard.putNumber(DRIVETRAIN_LEFT_MOTOR_PERCENT_OUTPUT, 0);
    SmartDashboard.putNumber(DRIVETRAIN_RIGHT_MOTOR_PERCENT_OUTPUT, 0);
  }

  private void initCamera() {
    new Thread(() -> {
      CameraServer cameraServer = CameraServer.getInstance();

      UsbCamera fishEyeCamera = cameraServer.startAutomaticCapture();
      fishEyeCamera.setResolution(WIDTH, HEIGHT);

      CvSink cvSink = cameraServer.getVideo();
      CvSource outputStream = cameraServer.putVideo(DRIVETEAM_FISHEYE_CAMERA, WIDTH, HEIGHT);
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
        ParkingLines.setPercentage(SmartDashboard.getNumber(PARKING_LINE_PERCENTAGE, 1.0));
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
    SmartDashboard.putNumber(DRIVETRAIN_LEFT_ENCODER, encoderLeft.getDistancePerPulse());
    SmartDashboard.putNumber(DRIVETRAIN_RIGHT_ENCODER, encoderRight.getDistancePerPulse());
    SmartDashboard.putString(DRIVETRAIN_ACTUAL_POSITION, String.valueOf(drivetrain.getActualPosition()));

    SmartDashboard.putNumber(MOTORS_ELEVATOR_HEIGHT, godSubsystem.getElevator().getElevatorHeight());
    SmartDashboard.putNumber(MOTORS_HATCH_ANGLE, godSubsystem.getHatch().getAngle());
    SmartDashboard.putNumber(MOTORS_CARGO_ANGLE, godSubsystem.getCargo().getAngle());
    // System.out.println("robot Periodic");
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You can use it to reset any subsystem
   * information you want to clear when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    godSubsystem.setEnabled(false);
    drivetrain.cancelControllerMotion();
    drivetrain.getMotionLogger().writeMotionDataCSV(true);
    System.out.println("am disabled ");
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
    godSubsystem.setEnabled(true);
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
    godSubsystem.setEnabled(true);
    drivetrain.cancelControllerMotion();

    godSubsystem.setEnabled(false);
    godSubsystem.getElevator().setElevatorControlMode(ElevatorControlMode.MANUAL);
    godSubsystem.getCargo().setClawControlMode(ClawControlMode.MANUAL);
    godSubsystem.getHatch().setHatchControlMode(HatchControlMode.MANUAL);

    elevatorMotor.setSelectedSensorPosition(0, 0, 100);
    clawRotationMotor.setSelectedSensorPosition(0, 0, 100);
    hatchRotationMotor.setSelectedSensorPosition(0, 0, 100);


  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    godSubsystem.getElevator().setElevatorPower(godSubsystem.getElevator().getElevatorJoystick());
    godSubsystem.getHatch().setHatchRotationPower(godSubsystem.getCargo().getCargoJoystick());
    godSubsystem.getCargo().setClawRotationPower(gamepad.getLeftX());

    if (catchHatch) {
      if (!hatchIntake.get()) {
        godSubsystem.getHatch().setIntake(true);
      }
    } else {
      if (hatchIntake.get()) {
        godSubsystem.getHatch().setIntake(false);
      }
    }

    if (intake && !outtake) {
      intake = false;
      godSubsystem.getCargo().intakeCargo(700);
    }
    if (outtake && !intake) {
      outtake = false;
      godSubsystem.getCargo().outtakeCargoFast(1000);
    }
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
