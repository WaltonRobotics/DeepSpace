package frc.robot.command.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;


public class ZeroElevator extends Command {

  private ElevatorCargoHatchSubsystem.Elevator elevator;

  public ZeroElevator() {
    requires(Robot.godSubsystem);
    elevator = Robot.godSubsystem.getElevator();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    elevator.setElevatorControlMode(ElevatorControlMode.ZEROING);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return elevator.isLowerLimit();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    elevator.setZeroed(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
