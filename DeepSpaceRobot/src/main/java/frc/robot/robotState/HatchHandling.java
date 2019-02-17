package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

public class HatchHandling implements State {


  @Override
  public void initialize() {
    Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.HATCH_HANDLING);
  }


  @Override
  public State periodic() {
    if (!Robot.godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (Robot.godSubsystem.defenceModeRising()) {
      return new DefenseTransition();
    }

    return this;
  }


  @Override
  public void finish() {
    Robot.godSubsystem.getHatch().setIntake(false);
    Robot.godSubsystem.getCurrentCommand().cancel();
  }
}
