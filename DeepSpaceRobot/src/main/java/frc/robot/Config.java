package frc.robot;

public class Config {

  public static final class Inputs {

    public static final int LEFT_JOYSTICK_PORT = 0;
    public static final int RIGHT_JOYSTICK_PORT = 1;
    public static final int GAMEPAD_PORT = 2;
    public static final int SHIFT_UP_PORT = 3;
    public static final int SHIFT_DOWN_PORT = 2;
  }

  public static final class Hardware {

    public static final int LEFT_WHEEL_CHANNEL = 0;
    public static final int RIGHT_WHEEL_CHANNEL = 1;
    public static final int SHIFTER_CHANNEL = 0;

    public static final int LEFT_ENCODER_CHANNEL1 = 2;
    public static final int LEFT_ENCODER_CHANNEL2 = 3; // for second digital input

    public static final int RIGHT_ENCODER_CHANNEL1 = 0; // digital
    public static final int RIGHT_ENCODER_CHANNEL2 = 1; // digital
    public static final double DISTANCE_PER_PULSE = 0.00055805; // digital
  }

  public static final class Constants {

    //    Give the relationship between power and speed, calculated using the motion profiler log display
    public static final double kV = 0.5349180715909608;
    public static final double kK = -0.02306185102694297;
    public static final double kAcc = 0.28150490814209445;

    //    Constants to help tune the movement of the robot
    public static final double kS = 0.85;
    public static final double kAng = 0.3;

    public static final double kL = 2;
    public static final double iL = 0;
    public static final double iAng = 0;

    //    max velocity and acceleration the robot is should be able to go at
    public static final double maxVelocity = (1 - kK) / kV;
    public static final double maxAcceleration = 4;

    /*
    width of the robot. found by measuring the distance between the middle of the two wheel or,
    more easily by measuring from the outside of a wheel to the inside of the other
     */
    public static final double robotWidth = 0.7500;
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

    public static final String MAX_VELOCITY = "Constants/Max Velocity";
    public static final String MAX_ACCELERATION = "Constants/Max Acceleration";
  }
}
