package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class CargoHandlingTransition implements State {

  @Override
  public void initialize() {

    // set limits for play mode
    Robot.godSubsystem.setCurrentActiveState(ActiveState.CARGO_HANDLING);
    Robot.godSubsystem.getHatch().setHatchTarget(HatchPosition.SAFE);
    Robot.godSubsystem.getCargo().setClawTarget(CargoPosition.DEPLOY);
    Robot.godSubsystem.getCargo().setLimits(CargoPosition.DEPLOY);
  }

  @Override
  public State periodic() {

    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    int cargoAngle = Robot.godSubsystem.getCargo().getAngle();
    int hatchAngle = Robot.godSubsystem.getHatch().getAngle();

    return Robot.currentRobot.getTarget(CargoPosition.DEPLOY).isClose(cargoAngle) && Robot.currentRobot
        .getTarget(HatchPosition.SAFE).isClose(hatchAngle) ? new CargoHandling()
        : this;
  }

  @Override
  public void finish() {

  }
}
