package frc.robot.command.auton;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import org.waltonrobotics.command.SimpleLine;


public class SimpleAlignmentDrive {

  public static double safetyDistance = 0.5;

  public static void forwardPowerDrive(double distance) {
    SimpleLine.lineWithDistance(distance - safetyDistance).start();
  }

  public static boolean inRange(double angle, double target, double tolerance) {

    return Math.abs(target - angle) < tolerance;
  }

  public static void fixAlignment(double currentAngle, double targetAngle) {

    if (inRange(currentAngle, targetAngle, 10)) {
      if (currentAngle > targetAngle) {
        RobotMap.rightWheels.set(ControlMode.PercentOutput, .5);
        RobotMap.leftWheels.set(ControlMode.PercentOutput, -.5);
        System.out.println("Current Angle: " + currentAngle);
        System.out.println("target Angle: " + targetAngle);
      }
      else if (currentAngle < targetAngle) {
        RobotMap.rightWheels.set(ControlMode.PercentOutput, -.5);
        RobotMap.leftWheels.set(ControlMode.PercentOutput, .5);
        System.out.println("Current Angle: " + currentAngle);
        System.out.println("target Angle: " + targetAngle);
      }
    }
  }
}