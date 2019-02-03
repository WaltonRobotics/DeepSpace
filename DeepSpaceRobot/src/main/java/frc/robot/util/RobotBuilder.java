package frc.robot.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.Supplier;

public class RobotBuilder {

  private ArrayList<RobotConfig> robotConfigs = new ArrayList<>();

  public RobotBuilder(ArrayList<RobotConfig> robotConfigs) {
    this.robotConfigs = robotConfigs;
  }

  public RobotBuilder(RobotConfig... robotConfigs) {
    Collections.addAll(this.robotConfigs, robotConfigs);
  }

  public ArrayList<RobotConfig> getRobotConfigs() {
    return robotConfigs;
  }

  public RobotConfig getCurrentRobotConfig() {
    return robotConfigs.stream().filter(RobotConfig::isCurrentRobot).findFirst().orElse(null);
  }

  public RobotBuilder addRobot(String robotName, double robotWidth, double robotLength, double maxVelocity,
      double maxAcceleration,
      int motorLeftChannel, boolean motorLeftInverted, int motorRightChannel, boolean motorRightInverted,
      int leftEncoderChannel1, int leftEncoderChannel2, boolean encoderLeftInverted, int rightEncoderChannel1,
      int rightEncoderChannel2, boolean encoderRightInverted, double distancePerPulse, double kV, double kK,
      double kAcc, double kS, double kAng,
      double kL, double iL, double iAng, Supplier<Boolean> isCurrentRobot) {
    robotConfigs.add(new RobotConfig(robotName,
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
        iL,
        iAng,
        isCurrentRobot));

    return this;
  }

  public RobotBuilder addRobot(String robotName, double robotWidth, double robotLength, double maxVelocity,
      double maxAcceleration,
      int motorLeftChannel, boolean motorLeftInverted, int motorRightChannel, boolean motorRightInverted,
      int leftEncoderChannel1, int leftEncoderChannel2, boolean encoderLeftInverted, int rightEncoderChannel1,
      int rightEncoderChannel2, boolean encoderRightInverted, double distancePerPulse, double kV, double kK,
      double kAcc, double kS, double kAng,
      double kL, Supplier<Boolean> isCurrentRobot) {
    robotConfigs.add(new RobotConfig(robotName,
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
        isCurrentRobot));

    return this;
  }
}
