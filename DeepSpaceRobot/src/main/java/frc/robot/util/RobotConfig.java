package frc.robot.util;

import java.util.function.Supplier;

public class RobotConfig {

  private final String robotName;

  /*
  width of the robot. found by measuring the distance between the middle of the two wheel or,
  more easily by measuring from the outside of a wheel to the inside of the other
   */
  private final double robotWidth;
  private final double robotLength;
  //    max velocity and acceleration the robot is should be able to go at
//    public static final double maxVelocity = (1 - kK) / kV;
  private final double maxVelocity;
  private final double maxAcceleration;
  private final int motorLeftChannel;
  private final boolean motorLeftInverted;
  private final int motorRightChannel;
  private final boolean motorRightInverted;
  private final int leftEncoderChannel1;
  private final int leftEncoderChannel2; // for second digital input
  private final boolean encoderLeftInverted;
  private final int rightEncoderChannel1; // digital
  private final int rightEncoderChannel2; // digital
  private final boolean encoderRightInverted;

  private final double distancePerPulse;

  //    Give the relationship between power and speed, calculated using the motion profiler log display
  private final double kV;
  private final double kK;
  private final double kAcc;

  //    Constants to help tune the movement of the robot
  private final double kS;
  private final double kAng;

  private final double kL;

  private final double iL;
  private final double iAng;
  private final Supplier<Boolean> isCurrentRobot;

  public RobotConfig(String robotName, double robotWidth, double robotLength, double maxVelocity,
      double maxAcceleration,
      int motorLeftChannel, boolean motorLeftInverted, int motorRightChannel, boolean motorRightInverted,
      int leftEncoderChannel1, int leftEncoderChannel2, boolean encoderLeftInverted, int rightEncoderChannel1,
      int rightEncoderChannel2, boolean encoderRightInverted, double distancePerPulse, double kV, double kK,
      double kAcc, double kS, double kAng,
      double kL, double iL, double iAng, Supplier<Boolean> isCurrentRobot) {
    this.robotName = robotName;
    this.robotWidth = robotWidth;
    this.robotLength = robotLength;
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.motorLeftChannel = motorLeftChannel;
    this.motorLeftInverted = motorLeftInverted;
    this.motorRightChannel = motorRightChannel;
    this.motorRightInverted = motorRightInverted;
    this.leftEncoderChannel1 = leftEncoderChannel1;
    this.leftEncoderChannel2 = leftEncoderChannel2;
    this.encoderLeftInverted = encoderLeftInverted;
    this.rightEncoderChannel1 = rightEncoderChannel1;
    this.rightEncoderChannel2 = rightEncoderChannel2;
    this.encoderRightInverted = encoderRightInverted;
    this.kV = kV;
    this.kK = kK;
    this.kAcc = kAcc;
    this.kS = kS;
    this.kAng = kAng;
    this.kL = kL;
    this.iL = iL;
    this.iAng = iAng;
    this.distancePerPulse = distancePerPulse;
    this.isCurrentRobot = isCurrentRobot;
  }

  public RobotConfig(String robotName, double robotWidth, double robotLength, double maxVelocity,
      double maxAcceleration,
      int motorLeftChannel, boolean motorLeftInverted, int motorRightChannel, boolean motorRightInverted,
      int leftEncoderChannel1, int leftEncoderChannel2, boolean encoderLeftInverted, int rightEncoderChannel1,
      int rightEncoderChannel2, boolean encoderRightInverted, double distancePerPulse, double kV, double kK,
      double kAcc, double kS, double kAng,
      double kL, Supplier<Boolean> isCurrentRobot) {
    this(robotName,
        robotWidth,
        robotLength,
        maxVelocity,
        maxAcceleration,
        motorLeftChannel,
        motorLeftInverted,
        motorRightChannel,
        motorRightInverted,
        leftEncoderChannel1,
        leftEncoderChannel2,
        encoderLeftInverted,
        rightEncoderChannel1,
        rightEncoderChannel2,
        encoderRightInverted,
        distancePerPulse,
        kV,
        kK,
        kAcc,
        kS,
        kAng,
        kL,
        0,
        0,
        isCurrentRobot);
  }

  public String getRobotName() {
    return robotName;
  }

  public double getDistancePerPulse() {
    return distancePerPulse;
  }

  public double getRobotWidth() {
    return robotWidth;
  }

  public double getRobotLength() {
    return robotLength;
  }

  public double getMaxVelocity() {
    return maxVelocity;
  }

  public double getMaxAcceleration() {
    return maxAcceleration;
  }

  public int getMotorLeftChannel() {
    return motorLeftChannel;
  }

  public boolean isMotorLeftInverted() {
    return motorLeftInverted;
  }

  public int getMotorRightChannel() {
    return motorRightChannel;
  }

  public boolean isMotorRightInverted() {
    return motorRightInverted;
  }

  public int getLeftEncoderChannel1() {
    return leftEncoderChannel1;
  }

  public int getLeftEncoderChannel2() {
    return leftEncoderChannel2;
  }

  public boolean isEncoderLeftInverted() {
    return encoderLeftInverted;
  }

  public int getRightEncoderChannel1() {
    return rightEncoderChannel1;
  }

  public int getRightEncoderChannel2() {
    return rightEncoderChannel2;
  }

  public boolean isEncoderRightInverted() {
    return encoderRightInverted;
  }

  public double getkV() {
    return kV;
  }

  public double getkK() {
    return kK;
  }

  public double getkAcc() {
    return kAcc;
  }

  public double getkS() {
    return kS;
  }

  public double getkAng() {
    return kAng;
  }

  public double getkL() {
    return kL;
  }

  public double getiL() {
    return iL;
  }

  public double getiAng() {
    return iAng;
  }

  boolean isCurrentRobot() {
    return isCurrentRobot.get();
  }
}
