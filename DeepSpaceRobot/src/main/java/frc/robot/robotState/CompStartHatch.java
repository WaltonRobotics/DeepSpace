package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Cargo;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Hatch;

public class CompStartHatch implements State {

  private double timeout;
  private Hatch hatch = Robot.godSubsystem.getHatch();
  private Cargo cargo = Robot.godSubsystem.getCargo();

  @Override
  public void initialize() {
    cargo.setLimits(CargoPosition.SAFE);

    hatch.setIntake(true);
    timeout = Robot.godSubsystem.getCurrentTime() + 500;
  }

  @Override
  public State periodic() {

    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (Robot.godSubsystem.getCurrentTime() >= timeout) {
      return new HatchHandlingTransition();
    }

    return this;
  }

  @Override
  public void finish() {

  }
}
