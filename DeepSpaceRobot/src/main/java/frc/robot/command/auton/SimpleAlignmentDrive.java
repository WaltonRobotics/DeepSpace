package frc.robot.command.auton;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.RobotMap;
import org.waltonrobotics.command.SimpleLine;


public class SimpleAlignmentDrive {

  public static double forwardAngle = 10;
  public static double safetyDistance = 0.5;

  public static void forwardPowerDrive(double distance) {
    SimpleLine.lineWithDistance(distance - safetyDistance).start();
  }

  public static void fixAlignment(Double currentAngle) {

    if (currentAngle != forwardAngle) {
      if (currentAngle > forwardAngle) {
        RobotMap.rightWheels.set(ControlMode.PercentOutput, .5);
        RobotMap.leftWheels.set(ControlMode.PercentOutput, -.5);
      }
      else if (currentAngle < forwardAngle) {
        RobotMap.rightWheels.set(ControlMode.PercentOutput, -.5);
        RobotMap.leftWheels.set(ControlMode.PercentOutput, .5);
      }
    }
  }
}