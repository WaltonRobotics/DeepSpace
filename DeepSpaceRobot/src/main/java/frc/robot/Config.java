package frc.robot;

import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import org.waltonrobotics.metadata.Pose;

public class Config {

  public static final class Camera {

    public static final int DRIVER_PIPELINE = 3;
    public static final int AUTO_ALIGN_PIPELINE = 2;
    public static final int LED_OFF = 1;
    public static final int LED_ON = 3;

    public static final int WIDTH = 320; //320
    public static final int HEIGHT = 240; //240
    public static final int FPS = 30;
    public static final int DEFAULT_CAMERA_COMPRESSION_QUALITY = 20; // between 0 and 100, 100 being the max, -1 being left to Shuffleboard

    private Camera() {
    }
  }

  public static final class Inputs {

    public static final int LEFT_JOYSTICK_PORT = 0;
    public static final int RIGHT_JOYSTICK_PORT = 1;
    public static final int GAMEPAD_PORT = 2;
    public static final int SHIFT_UP_PORT = 3;
    public static final int SHIFT_DOWN_PORT = 2;

    private Inputs() {
    }
  }

  public static final class Hardware {

    public static final int SHIFTER_CHANNEL = 0;
    public static final int HATCH_INTAKE_CHANNEL = 1;
    public static final int ELEVATOR_LOWER_LIMIT_CHANNEL = 4;

    public static final int LED_CHANNEL5 = 5;
    public static final int LED_CHANNEL6 = 6;

    private Hardware() {
    }
  }

  public static final class SmartDashboardKeys {

    public static final String CONSTANTS_KANGLE = "Constants/KAngle";
    public static final String CONSTANTS_MAX_VELOCITY = "Constants/Max Velocity";
    public static final String CONSTANTS_MAX_ACCELERATION = "Constants/Max Acceleration";
    public static final String CONSTANTS_KL = "Constants/KL";
    public static final String CONSTANTS_KV = "Constants/KV";
    public static final String CONSTANTS_KACC = "Constants/KAcc";
    public static final String CONSTANTS_KK = "Constants/KK";
    public static final String CONSTANTS_KS = "Constants/KS";

    public static final String PARKING_LINE_OFFSET = "Parking Line/Offset";
    public static final String PARKING_LINE_FOCUS_X = "Parking Line/Focus X";
    public static final String PARKING_LINE_FOCUS_Y = "Parking Line/Focus Y";
    public static final String PARKING_LINE_PERCENTAGE = "Parking Line/Percentage";

    public static final String DRIVETRAIN_ACTUAL_POSITION = "Drivetrain/Position";
    public static final String DRIVETRAIN_RIGHT_ENCODER = "Drivetrain/Encoder Right";
    public static final String DRIVETRAIN_LEFT_ENCODER = "Drivetrain/Encoder Left";
    public static final String DRIVETRAIN_LEFT_JOYSTICK_Y = "Drivetrain/leftSpeed";
    public static final String DRIVETRAIN_RIGHT_JOYSTICK_Y = "Drivetrain/rightSpeed";
    public static final String DRIVETRAIN_LEFT_MOTOR_PERCENT_OUTPUT = "Drivetrain/leftMotor";
    public static final String DRIVETRAIN_RIGHT_MOTOR_PERCENT_OUTPUT = "Drivetrain/rightMotor";

    public static final String MOTORS_ELEVATOR_HEIGHT = "Motors/Elevator Height";
    public static final String MOTORS_HATCH_ANGLE = "Motors/Hatch Angle";
    public static final String MOTORS_CARGO_ANGLE = "Motors/Cargo Angle";
    public static final String MOTORS_LOWER_LIMIT = "Motors/Lower Limit";
    public static final String MOTORS_ELEVATOR_ForwardSoftLimit = "Motors/Elevator Forwards Soft Limit";
    public static final String MOTORS_ELEVATOR_ReverseSoftLimit = "Motors/Elevator Reverse Soft Limit";
    public static final String MOTORS_CLAW_ForwardSoftLimit = "Motors/Claw Forwards Soft Limit";
    public static final String MOTORS_CLAW_ReverseSoftLimit = "Motors/Claw Reverse Soft Limit";
    public static final String MOTORS_HATCH_ForwardSoftLimit = "Motors/Hatch Forwards Soft Limit";
    public static final String MOTORS_HATCH_ReverseSoftLimit = "Motors/Hatch Reverse Soft Limit";

    public static final String MOTORS_ELEVATOR_POWER = "Motors/Elevator Power";
    public static final String MOTORS_ELEVATOR_TARGET = "Motors/Elevator Target";
    public static final String MOTORS_HATCH_POWER = "Motors/Hatch Power";
    public static final String MOTORS_HATCH_TARGET = "Motors/Hatch Target";
    public static final String MOTORS_CARGO_POWER = "Motors/Cargo Power";
    public static final String MOTORS_CARGO_TARGET = "Motors/Cargo Target";
    public static final String MOTORS_STATE = "Motors/State";
    public static final String MOTORS_ELEVATOR_MODE = "Motors/Elevator Mode";
    public static final String MOTORS_HATCH_MODE = "Motors/Hatch Mode";
    public static final String MOTORS_CARGO_MODE = "Motors/Cargo Mode";
    public static final String MOTORS_CLIMBER_MODE = "Motors/Climber Mode";
    public static final String MOTORS_CLIMBER_POWER = "Motors/Climber Power";
    public static final String MOTORS_INTAKE_OPEN = "Motors/Intake Open";

    public static final String DRIVETEAM_FISHEYE_CAMERA = "Fisheye Camera";
    public static final String DRIVETEAM_TRANSFORM_SELECT = "Driveteam/Transform Select";

    public static final String CAMERA_DATA_X = "CameraData/x";
    public static final String CAMERA_DATA_Y = "CameraData/y";
    public static final String CAMERA_DATA_HEIGHT = "CameraData/Height";
    public static final String CAMERA_DATA_ANGLE = "CameraData/Angle";
    public static final String CAMERA_DATA_NUMBER_OF_TARGETS = "CameraData/NumberOfTargets";
    public static final String CAMERA_DATA_TIME = "CameraData/Time";
    public static final String CAMERA_DATA_ACTUAL = "CameraData/Actual";
    public static final String CAMERA_DATA_TARGET = "CameraData/Target";
    public static final String CAMERA_DATA_USES_AUTOASSIST = "CameraData/Uses Auto Assist";
    public static final String CAMERA_DATA_PROPORTIONAL_POWER = "CameraData/Proportional Power";
    public static final String CAMERA_DATA_TARGET_OFFSET = "CameraData/Target Offset";

    public static final String DEBUG_CHOSEN_TARGET = "Debug/Chosen Target";
    public static final String DEBUG_JUST_BEFORE = "Debug/Just before";
    public static final String DEBUG_ACTUAL_TARGET = "Debug/Actual Target";
    public static final String DEBUG_CAMERA_VISION = "Debug/Camera Vision";
    public static final String DEBUG_CAMERA_OFFSET = "Debug/Camera Offset";
    public static final String DEBUG_HAS_VALID_CAMERA_DATA = "Debug/Has Valid Camera Data";

    public static final String MOTION_FRONT_ROCKET_X = "Motion/Front Rocket X";
    public static final String MOTION_FRONT_ROCKET_Y = "Motion/Front Rocket Y";
    public static final String MOTION_FRONT_ROCKET_ANGLE = "Motion/Front Rocket Angle";

    public static final String MOTION_BACKUP_X = "Motion/BackUp X";
    public static final String MOTION_BACKUP_Y = "Motion/BackUp Y";
    public static final String MOTION_BACKUP_ANGLE = "Motion/BackUp Angle";

    public static final String MOTION_HATCH_PICKUP_X = "Motion/Hatch PickUp X";
    public static final String MOTION_HATCH_PICKUP_Y = "Motion/Hatch PickUp Y";
    public static final String MOTION_HATCH_PICKUP_ANGLE = "Motion/Hatch PickUp Angle";

    public static final String IS_RIGHT_AUTON = "Is Right Auton?";
    public static final String USE_AUTON = "Use Auton";

    private SmartDashboardKeys() {
    }
  }

  public static final class Elevator {

    public static final double ZEROING = -0.2;

    private Elevator() {
    }
  }

  public static final class Cargo {

    public static final double CLIMB_MAX = 1.0;
    public static final double CARGO_LIMIT = 0.3;
    public static final double CARGO_TURBO_LIMIT = 1.0;

    private Cargo() {

    }
  }

  public static final class Point {

    //    public static final Pose frontRocketR = new Pose(2.82, 3.3, StrictMath.toRadians(60));
    public static final Pose frontRocketR = new Pose(2.55, 3.15, StrictMath.toRadians(60));
    public static final Pose backup = new Pose(2.0, 2.0, 0.0);

    public static final Pose hatchIntakeR = new Pose(2.1, -1.3462, StrictMath.toRadians(270.0));

    private Point() {
    }
  }

  public static final class RamseteControllerConstants{

    public static final double DRIVE_RADIUS = 0.7;
    public static final double K_BETA = 2.0;
    public static final double K_ZETA = 0.7;
    public static final Pose2d TOLERANCE_POSE = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(30));

  }

  public static final class SmartMotionConstants {

    public static final double LEFT_KP = 5e-5;
    public static final double LEFT_KI = 1e-6;
    public static final double LEFT_KD = 0;
    public static final double LEFT_KFF = 0;

    public static final double RIGHT_KP = 5e-5;
    public static final double RIGHT_KI = 1e-6;
    public static final double RIGHT_KD = 0;
    public static final double RIGHT_KFF = 0;

    public static final double R_KV = 3.19;
    public static final double R_KA = 0.462;
    public static final double R_KS = 0.178;
    public static final double R_KP = 8.85;

    public static final double L_KV = 3.19;
    public static final double L_KA = 0.462;
    public static final double L_KS = 0.178;
    public static final double L_KP = 8.85;

    public static final double KT = 0.04;
    public static final int KT_IN_MILLI = 40;

    public static final int K_SMART_CURRENT_LIMIT = 40;

    public static final int K_VOLTAGE_COMPENSATION = 12;

    public static final int K_OPENLOOP_RAMP = 0;

    public static final int DRIVE_CONTROL_MODE = 0;
    public static final int VELOCITY_CONTROL_MODE = 1;

    public static final double RPM_TO_METERS = 0.006649704450098395; // RPM TO M/S
  }
}
