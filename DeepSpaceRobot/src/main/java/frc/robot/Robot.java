/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

import javax.management.*;
import java.lang.management.ManagementFactory;

import static frc.robot.Config.Camera.LED_OFF;
import static frc.robot.Config.Hardware.*;
import static frc.robot.Config.Point.*;
import static frc.robot.Config.WaltonDashboardKeys.*;
import static frc.robot.RobotMap.*;

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
    public static final NetworkTable waltonDashboard;
    private static final RobotBuilder<LimitedRobot> robotBuilder;

    static {
        robotBuilder = new RobotBuilder<>(new CompPowerUp(), new CompSteamWorks(), new PracticeDeepSpace(),
                new CompDeepSpace());
        currentRobot = robotBuilder.getCurrentRobotConfig();
        System.out.println(currentRobot);
        drivetrain = new Drivetrain();
        godSubsystem = new ElevatorCargoHatchSubsystem();
        waltonDashboard = NetworkTableInstance.getDefault().getTable("WaltonDashboard");
        isCompBot = new DigitalInput(9).get();
        waltonDashboard.getEntry(IDENTIFIER).setBoolean(isCompBot);
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

    private static void initWaltonDashboard() {

        transformSendableChooser.setDefaultOption("Normal", new NormalSpeed());
        transformSendableChooser.addOption("Sigmoid", new Sigmoid());
        transformSendableChooser.addOption("Sqrt", new Sqrt());

        waltonDashboard.getEntry(DRIVETEAM_TRANSFORM_SELECT).setValue(transformSendableChooser);

        waltonDashboard.getEntry(CONSTANTS_KV).setNumber(currentRobot.getKV());
        waltonDashboard.getEntry(CONSTANTS_KACC).setNumber(currentRobot.getKAcc());
        waltonDashboard.getEntry(CONSTANTS_KK).setNumber(currentRobot.getKK());
        waltonDashboard.getEntry(CONSTANTS_KS).setNumber(currentRobot.getKS());
        waltonDashboard.getEntry(CONSTANTS_KANGLE).setNumber(currentRobot.getKAng());
        waltonDashboard.getEntry(CONSTANTS_MAX_VELOCITY).setNumber(currentRobot.getMaxVelocity());
        waltonDashboard.getEntry(CONSTANTS_MAX_ACCELERATION).setNumber(currentRobot.getMaxAcceleration());
        waltonDashboard.getEntry(CONSTANTS_KL).setNumber(currentRobot.getKL());

        waltonDashboard.getEntry(MOTORS_ELEVATOR_HEIGHT).setNumber(0);
        waltonDashboard.getEntry(MOTORS_HATCH_ANGLE).setNumber(0);
        waltonDashboard.getEntry(MOTORS_INTAKE_OPEN).setBoolean(false);
        waltonDashboard.getEntry(MOTORS_CARGO_ANGLE).setNumber(0);
        waltonDashboard.getEntry(MOTORS_LOWER_LIMIT).setBoolean(false);
        waltonDashboard.getEntry(MOTORS_ELEVATOR_ForwardSoftLimit).setBoolean(false);
        waltonDashboard.getEntry(MOTORS_ELEVATOR_ReverseSoftLimit).setBoolean(false);
        waltonDashboard.getEntry(MOTORS_CLAW_ForwardSoftLimit).setBoolean(false);
        waltonDashboard.getEntry(MOTORS_CLAW_ReverseSoftLimit).setBoolean(false);
        waltonDashboard.getEntry(MOTORS_HATCH_ForwardSoftLimit).setBoolean(false);
        waltonDashboard.getEntry(MOTORS_HATCH_ReverseSoftLimit).setBoolean(false);

        waltonDashboard.getEntry(MOTORS_STATE).setString("No state");
        waltonDashboard.getEntry(MOTORS_ELEVATOR_MODE).setString("No mode");
        waltonDashboard.getEntry(MOTORS_CARGO_MODE).setString("No mode");
        waltonDashboard.getEntry(MOTORS_HATCH_MODE).setString("No mode");

        waltonDashboard.getEntry(MOTORS_ELEVATOR_POWER).setNumber(0);
        waltonDashboard.getEntry(MOTORS_ELEVATOR_TARGET).setNumber(0);
        waltonDashboard.getEntry(MOTORS_HATCH_POWER).setNumber(0);
        waltonDashboard.getEntry(MOTORS_HATCH_TARGET).setNumber(0);
        waltonDashboard.getEntry(MOTORS_CARGO_POWER).setNumber(0);
        waltonDashboard.getEntry(MOTORS_CARGO_TARGET).setNumber(0);

        waltonDashboard.getEntry(MOTORS_CLIMBER_POWER).setNumber(0);
        waltonDashboard.getEntry(MOTORS_CLIMBER_MODE).setString("No mode");

        // Previously used alignment entries
    /*
    SmartDashboard.putNumber(PARKING_LINE_OFFSET, 60.0);
    SmartDashboard.putNumber(PARKING_LINE_FOCUS_X, WIDTH / 2.0);
    SmartDashboard.putNumber(PARKING_LINE_FOCUS_Y, 240.0);
    SmartDashboard.putNumber(PARKING_LINE_PERCENTAGE, 0.5);
    */

        waltonDashboard.getEntry(DRIVETRAIN_ACTUAL_POSITION).setString("No position has been reported");
        waltonDashboard.getEntry(DRIVETRAIN_RIGHT_ENCODER).setNumber(0);
        waltonDashboard.getEntry(DRIVETRAIN_LEFT_ENCODER).setNumber(0);
        waltonDashboard.getEntry(DRIVETRAIN_LEFT_JOYSTICK_Y).setNumber(0);
        waltonDashboard.getEntry(DRIVETRAIN_RIGHT_JOYSTICK_Y).setNumber(0);
        waltonDashboard.getEntry(DRIVETRAIN_LEFT_MOTOR_PERCENT_OUTPUT).setNumber(0);
        waltonDashboard.getEntry(DRIVETRAIN_RIGHT_MOTOR_PERCENT_OUTPUT).setNumber(0);

        // Previously used camera debugging entries
    /*
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
    */

        waltonDashboard.getEntry(DEBUG_CAMERA_VISION).setString("No Camera Data");
        waltonDashboard.getEntry(DEBUG_HAS_VALID_CAMERA_DATA).setBoolean(false);

        waltonDashboard.getEntry(MOTION_FRONT_ROCKET_X).setNumber(frontRocketR.getX());
        waltonDashboard.getEntry(MOTION_FRONT_ROCKET_Y).setNumber(frontRocketR.getY());
        waltonDashboard.getEntry(MOTION_FRONT_ROCKET_ANGLE).setNumber(frontRocketR.getDegrees());

        waltonDashboard.getEntry(MOTION_BACKUP_X).setNumber(backup.getX());
        waltonDashboard.getEntry(MOTION_BACKUP_Y).setNumber(backup.getY());
        waltonDashboard.getEntry(MOTION_BACKUP_ANGLE).setNumber(backup.getDegrees());

        waltonDashboard.getEntry(MOTION_HATCH_PICKUP_X).setNumber(hatchIntakeR.getX());
        waltonDashboard.getEntry(MOTION_HATCH_PICKUP_Y).setNumber(hatchIntakeR.getY());
        waltonDashboard.getEntry(MOTION_HATCH_PICKUP_ANGLE).setNumber(hatchIntakeR.getDegrees());

        waltonDashboard.getEntry(IS_RIGHT_AUTON).setBoolean(true);
        waltonDashboard.getEntry(USE_AUTON).setBoolean(false);

        waltonDashboard.getEntry(ALIGNMENT_STEER_K).setNumber(0.065);
        waltonDashboard.getEntry(ALIGNMENT_DRIVE_K).setNumber(0.26);
        waltonDashboard.getEntry(ALIGNMENT_CAMERA_DISTANCE).setNumber(-1.0);
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        drivetrain.cancelControllerMotion();
        drivetrain.reset();

        drivetrain.getController().getCameraReader().startCollecting();

        initWaltonDashboard();

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

    private double getRIOCPUUse() throws MalformedObjectNameException, ReflectionException, InstanceNotFoundException {
        MBeanServer mbs = ManagementFactory.getPlatformMBeanServer();
        ObjectName name = ObjectName.getInstance("java.lang:type=OperatingSystem");
        AttributeList list = mbs.getAttributes(name, new String[]{"ProcessCpuLoad"});

        if (list.isEmpty()) return Double.NaN;

        Attribute att = (Attribute) list.get(0);
        Double value = (Double) att.getValue();

        // usually takes a couple of seconds before we get real values
        if (value == -1.0) return Double.NaN;
        // returns a percentage value with 1 decimal point precision
        return ((int) (value * 1000) / 10.0);
    }

    private double getRIORamUse() {
        long ramTotal = Runtime.getRuntime().totalMemory();
        long ramFree = Runtime.getRuntime().freeMemory();
        long ramUsed = ramTotal - ramFree;

        return ((ramUsed * 1.0) / ramTotal);
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

        waltonDashboard.getEntry(DRIVETRAIN_LEFT_ENCODER).setNumber(encoderLeft.getDistance());
        waltonDashboard.getEntry(DRIVETRAIN_RIGHT_ENCODER).setNumber(encoderRight.getDistance());
        waltonDashboard.getEntry(DRIVETRAIN_ACTUAL_POSITION).setString(String.valueOf(drivetrain.getActualPosition()));
        waltonDashboard.getEntry(DEBUG_CAMERA_VISION).setString(String.valueOf(drivetrain.getCameraData()));

        // System.out.println("robot Periodic");
//    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
//    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);

        // Update diagnostics for dashboard
        waltonDashboard.getEntry(DIAGNOSTICS_BATTERY_VOLTAGE).setNumber(pdp.getVoltage());
        waltonDashboard.getEntry(DIAGNOSTICS_POWER_DRAW).setNumber(pdp.getTotalCurrent());

        // Will likely need to intercept packets sent from RIO to driver station for these values
        // Estimated RAM and CPU use for now
        try {
            waltonDashboard.getEntry(DIAGNOSTICS_RIO_CPU_USE).setNumber(getRIOCPUUse());
        } catch (MalformedObjectNameException | ReflectionException | InstanceNotFoundException e) {
            e.printStackTrace();
        }
        waltonDashboard.getEntry(DIAGNOSTICS_RIO_RAM_USE).setNumber(getRIORamUse());

        waltonDashboard.getEntry(DIAGNOSTICS_FRONT_LEFT_MOTOR_POWER_USAGE).setNumber(pdp.getCurrent(FRONT_LEFT_MOTOR_PDP_CHANNEL));
        waltonDashboard.getEntry(DIAGNOSTICS_FRONT_RIGHT_MOTOR_POWER_USAGE).setNumber(pdp.getCurrent(FRONT_RIGHT_MOTOR_PDP_CHANNEL));
        waltonDashboard.getEntry(DIAGNOSTICS_BACK_LEFT_MOTOR_POWER_USAGE).setNumber(pdp.getCurrent(BACK_LEFT_MOTOR_PDP_CHANNEL));
        waltonDashboard.getEntry(DIAGNOSTICS_BACK_RIGHT_MOTOR_POWER_USAGE).setNumber(pdp.getCurrent(BACK_RIGHT_MOTOR_PDP_CHANNEL));
        waltonDashboard.getEntry(DIAGNOSTICS_CLIMBER_MOTOR_POWER_USAGE).setNumber(pdp.getCurrent(CLIMBER_MOTOR_PDP_CHANNEL));
        waltonDashboard.getEntry(DIAGNOSTICS_RIO_POWER_USAGE).setNumber(pdp.getCurrent(RIO_PDP_CHANNEL));

        // TODO: Need a hardware gyro attached!
        waltonDashboard.getEntry(DIAGNOSTICS_GYRO_ORIENTATION).setNumber(0);
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
        godSubsystem.setAutonomousEnabled(waltonDashboard.getEntry(USE_AUTON).getBoolean(false));
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