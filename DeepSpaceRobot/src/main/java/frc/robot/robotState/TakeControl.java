package frc.robot.robotState;

import static frc.robot.Robot.currentRobot;

import frc.robot.Robot;
import frc.robot.robotState.auto.AutoHabitatToRocketHatch;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
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

    elevator.setControlMode(ElevatorControlMode.ZEROING);
    int currentCargoAngle = cargo.getAngle();
    //FIXME:int elevatorPosition = elevator.getElevatorHeight();
    int currentHatchAngle = hatch.getAngle();

    cargo.setClawAngle(currentCargoAngle);
    hatch.setHatchAngle(currentHatchAngle);

    cargo.setControlMode(ClawControlMode.AUTO);
    hatch.setControlMode(HatchControlMode.AUTO);

//    elevator.setZeroed(true);
    elevator.setZeroed(false);
    cargo.setLimits(CargoPosition.SAFE);
    hatch.setLimits(HatchPosition.SAFE);
    elevator.releaseLowerLimit();

    if (cargo.getAngle() < currentRobot.getTarget(CargoPosition.DEPLOY).getTarget()) {
      cargo.setCurrentTarget(CargoPosition.DEPLOY);
    }
  }

  @Override
  public State periodic() {
    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if ((elevator.getElevatorHeight() <= currentRobot.getTarget(ElevatorLevel.CARGO2).getTarget())
        && (Robot.godSubsystem.getCurrentActiveState() != ActiveState.HATCH_HANDLING)) {
      Robot.godSubsystem.getHatch().setIntake(true);
      Robot.godSubsystem.setCurrentActiveState(ActiveState.HATCH_HANDLING);
    }

    if (elevator.isLowerLimit()
//        || (elevator.getElevatorHeight()  - elevator.getLastEncoderPosition()) >= 0
    ) {
      elevator.setZeroed(true);
    } else {
      timeout = Robot.godSubsystem.getCurrentTime() + 500L;
    }

    if (Robot.godSubsystem.getElevator().isZeroed()) {
      if (Robot.godSubsystem.getCurrentTime() >= timeout) {
//        if ()

        if (Robot.godSubsystem.isAutonomousEnabled()) {
          return new AutoHabitatToRocketHatch();
        }

        HatchPosition hatchPosition = Robot.godSubsystem.findHatchClosestPosition(hatch.getAngle());
        if (hatchPosition == HatchPosition.HATCH_START) {
          return new CompStartHatch();
        } else if (hatchPosition == HatchPosition.CARGO_START) {
          return new CompStartCargo();
        } else {
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
    }

    if (hatch.intakeButtonRising()) {
      if (Robot.godSubsystem.getHatch().getIntakeIsSet()) {
        Robot.godSubsystem.getHatch().setIntake(false);
      } else {
        Robot.godSubsystem.getHatch().setIntake(true);
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
