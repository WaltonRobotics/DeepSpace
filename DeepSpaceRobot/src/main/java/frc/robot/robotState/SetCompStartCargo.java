package frc.robot.robotState;

import static frc.robot.Robot.currentRobot;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Cargo;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Elevator;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Hatch;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

/**
 * @author Marius Juston
 **/
public class SetCompStartCargo implements State {

  private final Hatch hatch = Robot.godSubsystem.getHatch();
  private final Cargo cargo = Robot.godSubsystem.getCargo();
  private final Elevator elevator = Robot.godSubsystem.getElevator();

  @Override
  public void initialize() {
    hatch.setLimits(HatchPosition.CARGO_START);
    cargo.setLimits(CargoPosition.SAFE);
    elevator.setElevatorLevel(ElevatorLevel.HATCH_BASE);
  }

  @Override
  public State periodic() {
    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (currentRobot.getTarget(ElevatorLevel.HATCH_BASE).isClose(elevator.getElevatorHeight(), 250)) {
      hatch.setCurrentTarget(HatchPosition.CARGO_START);
      cargo.setCurrentTarget(CargoPosition.SAFE);
    }

    if (Robot.godSubsystem.cargoModeRising()) {
      return new CargoHandlingTransition();
    }

    if (Robot.godSubsystem.defenceModeRising()) {
      return new DefenseTransition();
    }

    if (Robot.godSubsystem.hatchModeRising()) {
      return new HatchHandlingTransition();
    }

    return this;
  }

  @Override
  public void finish() {

  }

  @Override
  public String toString() {
    return "SetCompStartCargo{" +
        "hatch=" + hatch +
        ", cargo=" + cargo +
        ", elevator=" + elevator +
        '}';
  }
}
