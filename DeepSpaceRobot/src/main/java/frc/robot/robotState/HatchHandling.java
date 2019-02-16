package frc.robot.robotState;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

import static frc.robot.OI.*;
import static frc.robot.RobotMap.*;

public class HatchHandling implements State {


  @Override
  public void initialize() {
    Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.HATCH_HANDLING);
  }


  @Override
  public State periodic() {

    if (Robot.godSubsystem.defenceModeRising()) {
      return new DefenseTransition();
    }

    return this;
  }


  @Override
  public void finish() {
    Robot.godSubsystem.getHatch().setIntake(false);
  }
}
