package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Cargo;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClawControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Elevator;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;

public class CargoHandling implements State {

  private final Elevator elevator = Robot.godSubsystem.getElevator();
  private final Cargo cargo = Robot.godSubsystem.getCargo();

  @Override
  public void initialize() {
    Robot.godSubsystem.setCurrentActiveState(ActiveState.CARGO_HANDLING);
  }

  @Override
  public State periodic() {

    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (Robot.godSubsystem.defenceModeRising()) {
      return new DefenseTransition();
    }

    if (Robot.godSubsystem.hatchModeRising()) {
      return new HatchHandlingTransition();
    }

    boolean elevatorManual = Math.abs(elevator.getElevatorJoystick()) > 0.1;
    boolean cargoManual = Math.abs(cargo.getCargoJoystick()) > 0.1;

    if (elevatorManual || cargoManual) {
      if (elevatorManual) {
        elevator.setElevatorControlMode(ElevatorControlMode.MANUAL);
        elevator.setElevatorPower(elevator.getElevatorJoystick());
      } else {
        elevator.setElevatorControlMode(ElevatorControlMode.AUTO);
      }
      if (cargoManual) {
        cargo.setClawControlMode(ClawControlMode.MANUAL);

        double cargoJoystick = cargo.getCargoJoystick();

        cargoJoystick = Math.signum(cargoJoystick) * Math.min(Math.abs(cargoJoystick), 0.3);
        cargo.setClawRotationPower(cargoJoystick);
      } else {
        cargo.setClawControlMode(ClawControlMode.AUTO);
      }
    } else {
      elevator.setElevatorControlMode(ElevatorControlMode.AUTO);
      cargo.setClawControlMode(ClawControlMode.AUTO);

      if (elevator.isBasePressed()) {
        elevator.setElevatorLevel(ElevatorLevel.CARGO_HAB);
        cargo.setClawTarget(CargoPosition.DEPLOY);
      } else if (elevator.isElevatorLevel1ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.CARGO_BASE);
        cargo.setClawTarget(CargoPosition.DEPLOY);
      } else if (elevator.isElevatorLevel2ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.CARGO2);
        cargo.setClawTarget(CargoPosition.DEPLOY);
      } else if (elevator.isElevatorLevel3ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.CARGO3);
        cargo.setClawTarget(CargoPosition.ANGLE);
      }
    }

    if (cargo.outSlowButtonPressed()) {
      cargo.outtakeCargoSlow(0);
    } else if (cargo.outFastButtonPressed()) {
      cargo.outtakeCargoFast(0);
    } else if (cargo.inButtonPressed()) {
      cargo.intakeCargo(0);
    }

    return this;
  }

  @Override
  public void finish() {
    cargo.outtakeCargoSlow(1000);
  }
}
