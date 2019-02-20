package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class CompStartHatch implements State {

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

    if (Robot.currentRobot.getTarget(HatchPosition.DEPLOY).isClose(hatchAngle) && Robot.currentRobot
        .getTarget(CargoPosition.SAFE).isClose(cargoAngle)) {
      return new HatchHandlingTransition();
    } else if (Robot.currentRobot.getTarget(HatchPosition.DEPLOY).inRange(hatchAngle) && Robot.currentRobot
        .getTarget(CargoPosition.SAFE).inRange(cargoAngle)) {
      return new HatchHandlingTransition();
    }

    return this;
  }

  @Override
  public void finish() {

  }
}
