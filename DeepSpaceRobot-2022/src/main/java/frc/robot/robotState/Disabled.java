package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClawControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClimberControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchControlMode;

public class Disabled implements State {

  @Override
  public void initialize() {
    Robot.godSubsystem.getHatch().setControlMode(HatchControlMode.DISABLED);
    Robot.godSubsystem.getCargo().setControlMode(ClawControlMode.DISABLED);
    Robot.godSubsystem.getElevator().setControlMode(ElevatorControlMode.DISABLED);
    Robot.godSubsystem.getClimber().setClimberControlMode(ClimberControlMode.DISABLED);
  }

  @Override
  public State periodic() {
    if (Robot.godSubsystem.isEnabled()) {
      return new TakeControl();
    }

    return this;
  }

  @Override
  public void finish() {
  }
}
