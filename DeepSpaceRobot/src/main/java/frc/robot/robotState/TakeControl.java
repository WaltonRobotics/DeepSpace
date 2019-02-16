package frc.robot.robotState;

import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.command.teleop.ZeroElevator;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClawControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Elevator;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchControlMode;

public class TakeControl implements State {

  @Override
  public void initialize() {

    ElevatorCargoHatchSubsystem.Cargo cargo = Robot.godSubsystem.getCargo();
    ElevatorCargoHatchSubsystem.Hatch hatch = Robot.godSubsystem.getHatch();
    ElevatorCargoHatchSubsystem.Elevator elevator = Robot.godSubsystem.getElevator();

    int currentCargoAngle = cargo.getAngle();
    //FIXME:int elevatorPosition = elevator.getElevatorHeight();
    int currentHatchAngle = hatch.getAngle();

    cargo.setClawAngle(currentCargoAngle);
    hatch.setHatchAngle(currentHatchAngle);

    cargo.setClawControlMode(ClawControlMode.AUTO);
    hatch.setHatchControlMode(HatchControlMode.AUTO);

    elevator.setZeroed(false);
    Scheduler.getInstance().add(new ZeroElevator());

  }

  @Override
  public State periodic() {
    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if(Robot.godSubsystem.getElevator().isZeroed()) {
      //TODO:
    }
        return
    return this;
  }

  @Override
  public void finish() {

  }
}
