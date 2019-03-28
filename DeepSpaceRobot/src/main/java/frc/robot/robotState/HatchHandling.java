package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Elevator;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Hatch;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchControlMode;

public class HatchHandling implements State {


  private Elevator elevator = Robot.godSubsystem.getElevator();
  private Hatch hatch = Robot.godSubsystem.getHatch();

  @Override
  public void initialize() {
    Robot.godSubsystem.setCurrentActiveState(ActiveState.HATCH_HANDLING);
  }


  @Override
  public State periodic() {
    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (Robot.godSubsystem.defenceModeRising()) {
      return new DefenseTransition();
    }
    if (Robot.godSubsystem.cargoModeRising()) {
      return new CargoHandlingTransition();
    }

    boolean elevatorManual = Math.abs(elevator.getElevatorJoystick()) > 0.1;
    if (elevatorManual) {
      hatch.setControlMode(HatchControlMode.AUTO);
      elevator.setControlMode(ElevatorControlMode.MANUAL);
      elevator.setElevatorPower(elevator.getElevatorJoystick());

    } else {
      elevator.setControlMode(ElevatorControlMode.AUTO);

      if (elevator.isBasePressed()) {
        elevator.setElevatorLevel(ElevatorLevel.CARGO_BASE);
      } else if (elevator.isElevatorLevel1ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.HATCH_BASE);
      } else if (elevator.isElevatorLevel2ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.HATCH2);
      } else if (elevator.isElevatorLevel3ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.HATCH3);
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
//    Robot.godSubsystem.getHatch().setIntake(false);
  }
}
