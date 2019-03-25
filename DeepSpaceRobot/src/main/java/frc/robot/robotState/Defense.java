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

    if (climber.isClimberUpPressed()) {
      climber.setClimberPower(-0.5);
    } else if (climber.isClimberDownPressed()) {
      climber.setClimberPower(1.0);
    } else {
      climber.setClimberPower(0.0);
    }

    return this;
  }

  @Override
  public void finish() {

  }
}
