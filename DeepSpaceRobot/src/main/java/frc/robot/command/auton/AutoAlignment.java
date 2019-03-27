package frc.robot.command.auton;


import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.RobotMap;

@SuppressWarnings("FieldCanBeLocal")
public class AutoAlignment extends Command {
  private double error, leftPower, rightPower;

  public AutoAlignment() {
    requires(Robot.drivetrain);
  }

  @Override
  protected void execute() {

      error = Math.atan2(Robot.drivetrain.getCameraData().getCameraPose().getY(), Robot.drivetrain.getCameraData().getCameraPose().getX());
      leftPower = (error * Config.AutoAlineConstants.TURNING_kP) + Config.AutoAlineConstants.FORWARD;
      rightPower = (-error * Config.AutoAlineConstants.TURNING_kP) + Config.AutoAlineConstants.FORWARD;

      RobotMap.leftWheels.set(ControlMode.PercentOutput, leftPower);
      RobotMap.rightWheels.set(ControlMode.PercentOutput, rightPower);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}