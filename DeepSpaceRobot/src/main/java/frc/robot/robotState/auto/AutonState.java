package frc.robot.robotState.auto;

import static frc.robot.Robot.godSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.HatchHandlingTransition;
import frc.robot.state.State;

/**
 * @author Marius Juston
 **/
public abstract class AutonState implements State {

  private final CommandGroup commandGroup = new CommandGroup();

  @Override
  public final void initialize() {
    setSubsystemLimits();
    setMotionPath();
    commandGroup.start();
  }

  public abstract void setSubsystemLimits();

  public abstract void setMotionPath();

  @Override
  public final State periodic() {
    if (!godSubsystem.isEnabled()) {
      return new Disabled();
    }

    if (godSubsystem.isMasterOverride()) {
      return new HatchHandlingTransition();
    }

    return autonPeriodic();
  }

  protected abstract State autonPeriodic();

  public CommandGroup getCommandGroup() {
    return commandGroup;
  }

  @Override
  public String toString() {
    return "AutonState{" +
        "commandGroup=" + commandGroup +
        '}';
  }
}
