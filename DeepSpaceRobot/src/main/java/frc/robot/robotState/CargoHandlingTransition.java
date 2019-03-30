package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class CargoHandlingTransition implements State {

  @Override
  public void initialize() {

    // set limits for play mode
    Robot.godSubsystem.setCurrentActiveState(ActiveState.CARGO_HANDLING);
    Robot.godSubsystem.getHatch().setCurrentTarget(HatchPosition.SAFE);
    Robot.godSubsystem.getCargo().setCurrentTarget(CargoPosition.DEPLOY);
    Robot.godSubsystem.getCargo().setLimits(CargoPosition.DEPLOY);

    Robot.godSubsystem.getElevator().setControlMode(ElevatorControlMode.AUTO);
    Robot.godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.CARGO_BASE);
    Robot.godSubsystem.getElevator().setLimits(ElevatorLevel.CARGO_BASE);

    Robot.godSubsystem.getHatch().setIntake(false);
  }

  @Override
  public State periodic() {

    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    int cargoAngle = Robot.godSubsystem.getCargo().getAngle();
    int hatchAngle = Robot.godSubsystem.getHatch().getAngle();

    return (Robot.currentRobot.getTarget(CargoPosition.DEPLOY).isClose(cargoAngle, 50) && Robot.currentRobot
        .getTarget(HatchPosition.SAFE).isClose(hatchAngle, 50)) ? new CargoHandling()
        : this;
  }

  @Override
  public void finish() {

  }
}
