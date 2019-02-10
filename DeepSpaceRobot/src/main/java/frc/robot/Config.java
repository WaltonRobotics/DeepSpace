package frc.robot;

public class Config {

  public static final class Inputs {

    public static final int LEFT_JOYSTICK_PORT = 0;
    public static final int RIGHT_JOYSTICK_PORT = 1;
    public static final int GAMEPAD_PORT = 2;
    public static final int SHIFT_UP_PORT = 3;
    public static final int SHIFT_DOWN_PORT = 2;

    //ports regarding speed
    public static final int HALF_SPEED_PORT = 5;
    public static final int SQRT_SPEED_PORT = 5;
    public static final int EXPONENTIAL_SPEED_PORT = 4;
    public static final int NORMAL_SPEED_PORT = 3;
    public static final int SIGMOID_SPEED_PORT = 3;
  }

  public static final class Hardware {

    public static final int SHIFTER_CHANNEL = 0;
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
  }

  public class Motor {

    public static final int LEFT_MOTOR_PORT = 0;
    public static final int RIGHT_MOTOR_PORT = 1;

  }
}