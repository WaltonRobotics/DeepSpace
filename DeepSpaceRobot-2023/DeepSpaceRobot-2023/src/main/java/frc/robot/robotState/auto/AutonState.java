package frc.robot.robotState.auto;

import static frc.robot.Robot.godSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.HatchHandlingTransition;
import frc.robot.state.State;

/**
 * @author Marius Juston
 **/
public abstract class AutonState implements State {

  private final CommandGroupBase commandGroupBase = new CommandGroupBase() {
    @Override
    public void addCommands(Command... commands) {

    }
  };

  @Override
  public final void initialize() {
    setSubsystemLimits();
    setMotionPath();
    commandGroupBase.execute();
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

  public CommandGroupBase getCommandGroup() {
    return commandGroupBase;
  }

  @Override
  public String toString() {
    return "AutonState{" +
        "commandGroup=" + commandGroupBase +
        '}';
  }
}
