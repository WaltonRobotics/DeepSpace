package frc.robot.robotState;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.command.teleop.ElevatorCargo;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

public class CargoHandling implements State {


  @Override
  public void initialize() {
    Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.CARGO_HANDLING);
    Scheduler.getInstance().add(new ElevatorCargo());
  }

  @Override
  public State periodic() {

    if(!Robot.godSubsystem.isEnabled()){
      return new Disabled();
    }

    if(Robot.godSubsystem.defenceModeRising())
      return new DefenseTransition();

    return this;
  }

  @Override
  public void finish() {
    Robot.godSubsystem.getCargo().outtakeCargo(1000);
    Robot.godSubsystem.getCurrentCommand().cancel();
  }
}
