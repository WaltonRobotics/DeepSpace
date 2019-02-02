package frc.robot;

public class Config {

  public static final class Inputs {

    public static final int LEFT_JOYSTICK_PORT = 0;
    public static final int RIGHT_JOYSTICK_PORT = 1;
    public static final int INTAKE_JOYSTICK_PORT = 1;
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

    public static final int LEFT_CARGO_INTAKE_MOTOR_CHANNEL = 1;
    public static final int RIGHT_CARGO_INTAKE_MOTOR_CHANNEL = 2;
  }
}

