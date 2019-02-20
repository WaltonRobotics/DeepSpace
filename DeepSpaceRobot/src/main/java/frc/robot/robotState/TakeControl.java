package frc.robot.robotState;

import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.command.teleop.ZeroElevator;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Cargo;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClawControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Elevator;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Hatch;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class TakeControl implements State {

  @Override
  public void initialize() {
    Cargo cargo = Robot.godSubsystem.getCargo();
    Hatch hatch = Robot.godSubsystem.getHatch();
    Elevator elevator = Robot.godSubsystem.getElevator();

    int currentCargoAngle = cargo.getAngle();
    //FIXME:int elevatorPosition = elevator.getElevatorHeight();
    int currentHatchAngle = hatch.getAngle();

    cargo.setClawAngle(currentCargoAngle);
    hatch.setHatchAngle(currentHatchAngle);

    cargo.setClawControlMode(ClawControlMode.AUTO);
    hatch.setHatchControlMode(HatchControlMode.AUTO);

    elevator.setZeroed(false);
    cargo.setLimits(CargoPosition.SAFE);
    hatch.setLimits(HatchPosition.SAFE);
    elevator.releaseLowerLimit();
  }

  @Override
  public State periodic() {
    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (Robot.godSubsystem.getElevator().isZeroed()) {
      //TODO:move to comp start mode
      switch (Robot.godSubsystem.getCurrentActiveState()) {
        case DEFENSE:
          return new DefenseTransition();
        case CARGO_HANDLING:
          return new CargoHandlingTransition();
        case HATCH_HANDLING:
          return new HatchHandlingTransition();
        case ROBOT_SWITCHED_ON:
          return new HatchHandlingTransition();
      }
    }
    return this;
  }

  @Override
  public void finish() {
    Robot.godSubsystem.getElevator().setLimits(ElevatorLevel.BASE);
  }
}
