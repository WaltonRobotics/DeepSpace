package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class CompStartCargo implements State {

  @Override
  public void initialize() {

  }

  @Override
  public State periodic() {

    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }
    int hatchAngle = Robot.godSubsystem.getHatch().getAngle();
    int cargoAngle = Robot.godSubsystem.getCargo().getAngle();

    if (HatchPosition.DEPLOY.isClose(cargoAngle) && CargoPosition.SAFE.isClose(hatchAngle)) {
      return new CargoHandlingTransition();
    } else if (HatchPosition.DEPLOY.inRange(cargoAngle) && CargoPosition.SAFE.inRange(hatchAngle)) {
      return new CargoHandlingTransition();
    }

    return this;
  }

  @Override
  public void finish() {

  }
}
