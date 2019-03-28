package frc.robot.command.auton;


import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.RobotMap;

@SuppressWarnings("FieldCanBeLocal")
public class AutoAlignment extends Command {
  private double error, leftPower, rightPower;

  @Override
  public void initialize() {

      System.out.println("it ran");

  }

  public AutoAlignment() {
    requires(Robot.drivetrain);
  }

  @Override
  protected void execute() {

      error = -Math.atan2(Robot.drivetrain.getCameraData().getCameraPose().getY(), Robot.drivetrain.getCameraData().getCameraPose().getX());
      System.out.println("Error: " + error);
      leftPower = (error * Config.AutoAlineConstants.TURNING_kP) + Config.AutoAlineConstants.FORWARD;
      rightPower = (error * Config.AutoAlineConstants.TURNING_kP) + Config.AutoAlineConstants.FORWARD;

      //tried with rightpower negative


      System.out.println("error " + error);

      SmartDashboard.putString("Error ", String.valueOf(error));

      RobotMap.leftWheels.set(ControlMode.PercentOutput, leftPower);
      RobotMap.rightWheels.set(ControlMode.PercentOutput, rightPower);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}