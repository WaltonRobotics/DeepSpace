package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Cargo;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClawControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Elevator;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Hatch;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class TakeControl implements State {

  private double timeout;
  private Elevator elevator = Robot.godSubsystem.getElevator();
  private Cargo cargo = Robot.godSubsystem.getCargo();
  private Hatch hatch = Robot.godSubsystem.getHatch();

  @Override
  public void initialize() {

    elevator.setElevatorControlMode(ElevatorControlMode.ZEROING);
    int currentCargoAngle = cargo.getAngle();
    //FIXME:int elevatorPosition = elevator.getElevatorHeight();
    int currentHatchAngle = hatch.getAngle();

    cargo.setClawAngle(currentCargoAngle);
    hatch.setHatchAngle(currentHatchAngle);

    cargo.setClawControlMode(ClawControlMode.AUTO);
    hatch.setHatchControlMode(HatchControlMode.AUTO);

//    elevator.setZeroed(true);
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

    if (elevator.isLowerLimit()
//        || (elevator.getElevatorHeight()  - elevator.getLastEncoderPosition()) >= 0
    ) {
      elevator.setZeroed(true);
    } else {
      timeout = Robot.godSubsystem.getCurrentTime() + 500;
    }

    if (Robot.godSubsystem.getElevator().isZeroed()) {
      if (Robot.godSubsystem.getCurrentTime() >= timeout) {
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
    }

    return this;
  }

  @Override
  public void finish() {
    Robot.godSubsystem.getElevator().enableLowerLimit();
    Robot.godSubsystem.getElevator().setLimits(ElevatorLevel.CARGO_BASE);
  }
}
