package frc.robot.robotState;

import static frc.robot.Config.Cargo.CARGO_LIMIT;
import static frc.robot.Config.Cargo.CARGO_TURBO_LIMIT;

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

    if (Robot.godSubsystem.climbModeRising()) {
      return new ClimbHandlingTransition();
    }

    boolean elevatorManual = Math.abs(elevator.getElevatorJoystick()) > 0.1;
    boolean cargoManual = Math.abs(cargo.getCargoJoystick()) > 0.1;

    cargo.intakeCargoHold();

    if (elevatorManual || cargoManual) {
      if (elevatorManual) {
        elevator.setControlMode(ElevatorControlMode.MANUAL);
        elevator.setElevatorPower(elevator.getElevatorJoystick());
      } else {
        elevator.setControlMode(ElevatorControlMode.AUTO);
      }
      if (cargoManual) {
        cargo.setControlMode(ClawControlMode.MANUAL);

        double cargoJoystick = cargo.getCargoJoystick();

        double cargoLimit = cargo.cargoTurbo() ? CARGO_TURBO_LIMIT : CARGO_LIMIT;

        cargoJoystick = Math.signum(cargoJoystick) * Math.min(Math.abs(cargoJoystick), cargoLimit);
        cargo.setRotationPower(cargoJoystick);
      } else {
        cargo.setControlMode(ClawControlMode.AUTO);
      }
    } else {
      elevator.setControlMode(ElevatorControlMode.AUTO);
      cargo.setControlMode(ClawControlMode.AUTO);

      if (elevator.isBasePressed()) {
        elevator.setElevatorLevel(ElevatorLevel.CARGO_ROCKET);
        cargo.setCurrentTarget(CargoPosition.CARGO_1);
      } else if (elevator.isElevatorLevel1ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.CARGO_BASE);
        cargo.setCurrentTarget(CargoPosition.DEPLOY);
      } else if (elevator.isElevatorLevel2ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.CARGO2);
        cargo.setCurrentTarget(CargoPosition.CARGO_2);
      } else if (elevator.isElevatorLevel3ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.CARGO3);
        cargo.setCurrentTarget(CargoPosition.CARGO_3);
      } else if (elevator.isElevatorCargoShipButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.CARGO_HAB);
        cargo.setCurrentTarget(CargoPosition.HAB);
      }
    }

    if (cargo.outSlowButtonPressed()) {
      cargo.outtakeCargoSlow(0);
    } else if (cargo.outFastButtonPressed()) {
      cargo.outtakeCargoFast(0);
    } else if (cargo.inFastButtonPressed()) {
      cargo.intakeCargoFast(0);
    } else if (cargo.inSlowButtonPressed()) {
      cargo.intakeCargoSlow(0);
    }

    return this;
  }

  @Override
  public void finish() {
    cargo.outtakeCargoSlow(1000);
  }
}
