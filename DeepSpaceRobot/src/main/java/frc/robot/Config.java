package frc.robot;

public class Config {

  public static final class Camera {

    public static final int WIDTH = 320;
    public static final int HEIGHT = 240;
    public static final int FPS = 30;
    public static final int DEFAULT_CAMERA_COMPRESSION_QUALITY = 40; // between 0 and 100, 100 being the max, -1 being left to Shuffleboard

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
    public static final String MOTORS_STATE = "Motors/Elevator State";

    public static final String DRIVETEAM_FISHEYE_CAMERA = "Driveteam/Fisheye Camera";
    public static final String DRIVETEAM_TRANSFORM_SELECT = "Driveteam/Transform Select";

    private SmartDashboardKeys() {
    }
  }

  public static final class Elevator {

    public static final double LOWERING_TO_BASE_POWER = 0.2;
    public static final double LOWERING_TO_BASE_TIMEOUT_SECONDS = 5;

    private Elevator() {
    }
  }

}
