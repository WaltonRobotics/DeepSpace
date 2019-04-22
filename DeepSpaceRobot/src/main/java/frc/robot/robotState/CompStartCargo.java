package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Cargo;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;

public class CompStartCargo implements State {

  private final Cargo cargo = Robot.godSubsystem.getCargo();
  private long timeout;

  @Override
  public void initialize() {
    cargo.setLimits(CargoPosition.DEPLOY);
    cargo.setCurrentTarget(CargoPosition.DEPLOY);

    Robot.godSubsystem.getHatch().setIntake(false);
    timeout = Robot.godSubsystem.getCurrentTime() + 1000L;
  }

  @Override
  public State periodic() {

    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (Robot.godSubsystem.getCurrentTime() >= timeout) {
      return new CargoHandlingTransition();
    }

    return this;
  }

  @Override
  public void finish() {

  }

  @Override
  public String toString() {
    return "CompStartCargo{" +
        "cargo=" + cargo +
        ", timeout=" + timeout +
        '}';
  }
}
