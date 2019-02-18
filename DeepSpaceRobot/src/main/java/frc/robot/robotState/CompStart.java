package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;

public class CompStart implements State {

  @Override
  public void initialize() {
    Robot.godSubsystem.setCurrentActiveState(ActiveState.ROBOT_SWITCHED_ON);


  }

  @Override
  public State periodic() {
    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    ActiveState currentActiveState = Robot.godSubsystem.getCurrentActiveState();

    if (currentActiveState == ActiveState.CARGO_HANDLING) {
      return new CargoHandlingTransition();
    } else if (currentActiveState == ActiveState.DEFENSE) {
      return new DefenseTransition();
    } else {
      return new HatchHandlingTransition();
    }
  }

  @Override
  public void finish() {

  }
}
