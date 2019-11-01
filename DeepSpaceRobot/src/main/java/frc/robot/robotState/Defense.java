package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Climber;

public class Defense implements State {

  private final Climber climber = Robot.godSubsystem.getClimber();

  @Override
  public void initialize() {
    Robot.godSubsystem.setCurrentActiveState(ActiveState.DEFENSE);
  }

  @Override
  public State periodic() {

    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }
    if (Robot.godSubsystem.cargoModeRising()) {
      return new CargoHandlingTransition();
    }
    if (Robot.godSubsystem.hatchModeRising()) {
      return new HatchHandlingTransition();
    }

    if (Robot.godSubsystem.setCompStartHatchModeRising()) {
      return new SetCompStartHatch();
    }

    if (Robot.godSubsystem.setCompStartCargoModeRising()) {
      return new SetCompStartCargo();
    }

    return this;
  }

  @Override
  public void finish() {

  }

  @Override
  public String toString() {
    return "Defense{" +
        "climber=" + climber +
        '}';
  }
}
