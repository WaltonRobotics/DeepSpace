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
