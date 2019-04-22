/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Config.Camera.LED_OFF;
import static frc.robot.Config.Camera.WIDTH;
import static frc.robot.Config.Point.backup;
import static frc.robot.Config.Point.frontRocketR;
import static frc.robot.Config.Point.hatchIntakeR;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_ACTUAL;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_HEIGHT;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_NUMBER_OF_TARGETS;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_PROPORTIONAL_POWER;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_TARGET;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_TARGET_OFFSET;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_TIME;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_USES_AUTOASSIST;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_X;
import static frc.robot.Config.SmartDashboardKeys.CAMERA_DATA_Y;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KACC;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KANGLE;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KK;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KL;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KS;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_KV;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_MAX_ACCELERATION;
import static frc.robot.Config.SmartDashboardKeys.CONSTANTS_MAX_VELOCITY;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_ACTUAL_TARGET;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_CAMERA_OFFSET;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_CAMERA_VISION;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_CHOSEN_TARGET;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_HAS_VALID_CAMERA_DATA;
import static frc.robot.Config.SmartDashboardKeys.DEBUG_JUST_BEFORE;
import static frc.robot.Config.SmartDashboardKeys.DRIVETEAM_TRANSFORM_SELECT;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_ACTUAL_POSITION;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_LEFT_ENCODER;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_LEFT_JOYSTICK_Y;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_LEFT_MOTOR_PERCENT_OUTPUT;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_RIGHT_ENCODER;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_RIGHT_JOYSTICK_Y;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_RIGHT_MOTOR_PERCENT_OUTPUT;
import static frc.robot.Config.SmartDashboardKeys.IS_RIGHT_AUTON;
import static frc.robot.Config.SmartDashboardKeys.MOTION_BACKUP_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTION_BACKUP_X;
import static frc.robot.Config.SmartDashboardKeys.MOTION_BACKUP_Y;
import static frc.robot.Config.SmartDashboardKeys.MOTION_FRONT_ROCKET_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTION_FRONT_ROCKET_X;
import static frc.robot.Config.SmartDashboardKeys.MOTION_FRONT_ROCKET_Y;
import static frc.robot.Config.SmartDashboardKeys.MOTION_HATCH_PICKUP_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTION_HATCH_PICKUP_X;
import static frc.robot.Config.SmartDashboardKeys.MOTION_HATCH_PICKUP_Y;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_MODE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CARGO_TARGET;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CLAW_ForwardSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CLAW_ReverseSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CLIMBER_MODE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_CLIMBER_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_ForwardSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_HEIGHT;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_MODE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_ReverseSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_ELEVATOR_TARGET;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_ForwardSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_MODE;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_POWER;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_ReverseSoftLimit;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_HATCH_TARGET;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_INTAKE_OPEN;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_LOWER_LIMIT;
import static frc.robot.Config.SmartDashboardKeys.MOTORS_STATE;
import static frc.robot.Config.SmartDashboardKeys.PARKING_LINE_FOCUS_X;
import static frc.robot.Config.SmartDashboardKeys.PARKING_LINE_FOCUS_Y;
import static frc.robot.Config.SmartDashboardKeys.PARKING_LINE_OFFSET;
import static frc.robot.Config.SmartDashboardKeys.PARKING_LINE_PERCENTAGE;
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
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;
import frc.robot.util.RobotBuilder;

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
//    currentRobot.setupController(clawRotationMotor, currentRobot.getCargoSubsystemLimits(), null);
    currentRobot.setupController(elevatorMotor, currentRobot.getElevatorSubsystemLimits(), ElevatorLevel.HATCH_BASE);
    currentRobot.setupController(hatchRotationMotor, currentRobot.getHatchSubsystemLimits(), HatchPosition.SAFE);
//    currentRobot.setupController(hatchRotationMotor, currentRobot.getHatchSubsystemLimits(), null);
  }

  private static void initShuffleBoard() {

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
    SmartDashboard.putBoolean(MOTORS_INTAKE_OPEN, false);
    SmartDashboard.putNumber(MOTORS_CARGO_ANGLE, 0);
    SmartDashboard.putBoolean(MOTORS_LOWER_LIMIT, false);
    SmartDashboard.putBoolean(MOTORS_ELEVATOR_ForwardSoftLimit, false);
    SmartDashboard.putBoolean(MOTORS_ELEVATOR_ReverseSoftLimit, false);
    SmartDashboard.putBoolean(MOTORS_CLAW_ForwardSoftLimit, false);
    SmartDashboard.putBoolean(MOTORS_CLAW_ReverseSoftLimit, false);
    SmartDashboard.putBoolean(MOTORS_HATCH_ForwardSoftLimit, false);
    SmartDashboard.putBoolean(MOTORS_HATCH_ReverseSoftLimit, false);

    SmartDashboard.putString(MOTORS_STATE, "No state");
    SmartDashboard.putString(MOTORS_ELEVATOR_MODE, "No mode");
    SmartDashboard.putString(MOTORS_CARGO_MODE, "No mode");
    SmartDashboard.putString(MOTORS_HATCH_MODE, "No mode");

    SmartDashboard.putNumber(MOTORS_ELEVATOR_POWER, 0);
    SmartDashboard.putNumber(MOTORS_ELEVATOR_TARGET, 0);
    SmartDashboard.putNumber(MOTORS_HATCH_POWER, 0);
    SmartDashboard.putNumber(MOTORS_HATCH_TARGET, 0);
    SmartDashboard.putNumber(MOTORS_CARGO_POWER, 0);
    SmartDashboard.putNumber(MOTORS_CARGO_TARGET, 0);

    SmartDashboard.putNumber(MOTORS_CLIMBER_POWER, 0);
    SmartDashboard.putString(MOTORS_CLIMBER_MODE, "No mode");

    SmartDashboard.putNumber(PARKING_LINE_OFFSET, 60.0);
    SmartDashboard.putNumber(PARKING_LINE_FOCUS_X, WIDTH / 2.0);
    SmartDashboard.putNumber(PARKING_LINE_FOCUS_Y, 240.0);
    SmartDashboard.putNumber(PARKING_LINE_PERCENTAGE, 0.5);

    SmartDashboard.putString(DRIVETRAIN_ACTUAL_POSITION, "No position has been reported");
    SmartDashboard.putNumber(DRIVETRAIN_RIGHT_ENCODER, 0);
    SmartDashboard.putNumber(DRIVETRAIN_LEFT_ENCODER, 0);
    SmartDashboard.putNumber(DRIVETRAIN_LEFT_JOYSTICK_Y, 0);
    SmartDashboard.putNumber(DRIVETRAIN_RIGHT_JOYSTICK_Y, 0);
    SmartDashboard.putNumber(DRIVETRAIN_LEFT_MOTOR_PERCENT_OUTPUT, 0);
    SmartDashboard.putNumber(DRIVETRAIN_RIGHT_MOTOR_PERCENT_OUTPUT, 0);

    SmartDashboard.putNumber(CAMERA_DATA_X, -1.0);
    SmartDashboard.putNumber(CAMERA_DATA_Y, 0.1);
    SmartDashboard.putNumber(CAMERA_DATA_HEIGHT, 0);
    SmartDashboard.putNumber(CAMERA_DATA_ANGLE, 0);
    SmartDashboard.putNumber(CAMERA_DATA_NUMBER_OF_TARGETS, 1.0);
    SmartDashboard.putNumber(CAMERA_DATA_TIME, 0);
    SmartDashboard.putBoolean(CAMERA_DATA_USES_AUTOASSIST, false);
    SmartDashboard.putString(CAMERA_DATA_ACTUAL, "No actual data");
    SmartDashboard.putString(CAMERA_DATA_TARGET, "No target data");
    SmartDashboard.putNumber(CAMERA_DATA_PROPORTIONAL_POWER, 0.2);
    SmartDashboard.putNumber(CAMERA_DATA_TARGET_OFFSET, 0.0);

    SmartDashboard.putString(DEBUG_CAMERA_OFFSET, "No camera offset");
    SmartDashboard.putString(DEBUG_CHOSEN_TARGET, "No camera data");
    SmartDashboard.putString(DEBUG_JUST_BEFORE, "No camera data");
    SmartDashboard.putString(DEBUG_ACTUAL_TARGET, "No camera data");
    SmartDashboard.putString(DEBUG_CAMERA_VISION, "No Camera Data");
    SmartDashboard.putBoolean(DEBUG_HAS_VALID_CAMERA_DATA, false);

    SmartDashboard.putNumber(MOTION_FRONT_ROCKET_X, frontRocketR.getX());
    SmartDashboard.putNumber(MOTION_FRONT_ROCKET_Y, frontRocketR.getY());
    SmartDashboard.putNumber(MOTION_FRONT_ROCKET_ANGLE, frontRocketR.getDegrees());

    SmartDashboard.putNumber(MOTION_BACKUP_X, backup.getX());
    SmartDashboard.putNumber(MOTION_BACKUP_Y, backup.getY());
    SmartDashboard.putNumber(MOTION_BACKUP_ANGLE, backup.getDegrees());

    SmartDashboard.putNumber(MOTION_HATCH_PICKUP_X, hatchIntakeR.getX());
    SmartDashboard.putNumber(MOTION_HATCH_PICKUP_Y, hatchIntakeR.getY());
    SmartDashboard.putNumber(MOTION_HATCH_PICKUP_ANGLE, hatchIntakeR.getDegrees());

    SmartDashboard.putBoolean(IS_RIGHT_AUTON, true);
    SmartDashboard.putBoolean(USE_AUTON, false);

    SmartDashboard.putNumber("Steer K", 0.065);
    SmartDashboard.putNumber("Drive K", 0.26);
    SmartDashboard.putNumber("Camera Distance", -1.0);

    SmartDashboard.putNumber("Distance", 2.0);
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
    SmartDashboard.putString(DEBUG_CAMERA_VISION, String.valueOf(drivetrain.getCameraData()));
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
//    godSubsystem.setEnabled(false);
    godSubsystem.setEnabled(true);
    godSubsystem.setAutonomousEnabled(SmartDashboard.getBoolean(USE_AUTON, false));
//    godSubsystem.setAutonomousEnabled(false);
    drivetrain.cancelControllerMotion();
    drivetrain.clearControllerMotions();
    drivetrain.shiftUp();

//    Pose pose = new Pose(0, 0, StrictMath.toRadians(90));
//    drivetrain.startControllerMotion(pose);

//    SimpleSpline.pathFromPosesWithAngle(false, pose, pose.offset(2,2, 0)).start();

//    Pose backup = new Pose(SmartDashboard.getNumber("x", 2.1), SmartDashboard.getNumber("y", 2.75),
//        Math.toRadians(SmartDashboard.getNumber("angle", 0)));

//    SimpleSpline.pathFromPosesWithAngleAndScale(true, .2 ,.2,frontRocketR,  backup).start();

//    boolean isBackwards = SmartDashboard.getBoolean("isBackwards", true);

//    Pose pose1 = new Pose(SmartDashboard.getNumber("x", -1), SmartDashboard.getNumber("y", -1),
//        Math.toRadians(SmartDashboard.getNumber("angle", 90)));
//
//    SimpleSpline.pathFromPosesWithAngle(isBackwards, pose, pose1).start();

//    double distance = SmartDashboard.getNumber("Distance", 2);
//    SimpleSpline.pathFromPosesWithAngle(false, pose, pose.offset(distance), frontRocketR).start();
//    drivetrain.startControllerMotion(pose);
//    SimpleLine.lineWithDistance(SmartDashboard.getNumber("Distance", 2)).start();
//    SimpleSpline.pathFromPosesWithAngle(false, Pose.ZERO, new Pose(1.5, 2)).start();
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

//    godSubsystem.setEnabled(false);
//    godSubsystem.getElevator().setControlMode(ElevatorControlMode.MANUAL);
//    godSubsystem.getCargo().setControlMode(ClawControlMode.MANUAL);
//    godSubsystem.getHatch().setControlMode(HatchControlMode.MANUAL);
//
//    elevatorMotor.setSelectedSensorPosition(0, 0, 100);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

//    godSubsystem.getElevator().setElevatorPower(godSubsystem.getElevator().getElevatorJoystick());
////    godSubsystem.getHatch().setRotationPower(godSubsystem.getCargo().getCargoJoystick());
//    godSubsystem.getCargo().setRotationPower(godSubsystem.getCargo().getCargoJoystick());
//
//    if (catchHatch) {
//      if (!hatchIntake.get()) {
//        godSubsystem.getHatch().setIntake(true);
//      }
//    } else {
//      if (hatchIntake.get()) {
//        godSubsystem.getHatch().setIntake(false);
//      }
//    }
//
//    if (intake && !outtake) {
//      intake = false;
//      godSubsystem.getCargo().intakeCargoFast(700);
//    }
//    if (outtake && !intake) {
//      outtake = false;
//      godSubsystem.getCargo().outtakeCargoFast(1000);
//    }
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
