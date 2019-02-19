package frc.robot.test;

import frc.robot.Robot;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.util.TestCommand;

public class ElevatorTestCommand extends TestCommand {

  @Override
  protected void initializeTest() {
    Robot.godSubsystem.getElevator().setElevatorControlMode(ElevatorControlMode.MANUAL);
  }

  @Override
  protected void executeTest() {
    if(Robot.godSubsystem.getCurrentActiveState() == ActiveState.CARGO_HANDLING) {
      if (Robot.godSubsystem.getElevator().isElevatorLevel1ButtonPressed()) {
        if (Robot.godSubsystem.getElevator().getElevatorLevel() != ElevatorLevel.CARGO1) {
          throw new AssertionError("Cargo1 level issue");
        }
      }
      if(Robot.godSubsystem.getElevator().isElevatorLevel2ButtonPressed()){
        if (Robot.godSubsystem.getElevator().getElevatorLevel() != ElevatorLevel.CARGO2) {
          throw new AssertionError("Cargo2 level issue");
        }
      }
      if(Robot.godSubsystem.getElevator().isElevatorLevel3ButtonPressed()){
        if (Robot.godSubsystem.getElevator().getElevatorLevel() != ElevatorLevel.CARGO3) {
          throw new AssertionError("Cargo3 level issue");
        }
      }
    }
    if(Robot.godSubsystem.getCurrentActiveState() == ActiveState.HATCH_HANDLING){
      if (Robot.godSubsystem.getElevator().isElevatorLevel1ButtonPressed()){
        if (Robot.godSubsystem.getElevator().getElevatorLevel() != ElevatorLevel.HATCH1) {
          throw new AssertionError("Hatch1 level issue");
        }
      }
      if (Robot.godSubsystem.getElevator().isElevatorLevel2ButtonPressed()){
        if (Robot.godSubsystem.getElevator().getElevatorLevel() != ElevatorLevel.HATCH2) {
          throw new AssertionError("Hatch2 level issue");
        }
      }
      if(Robot.godSubsystem.getElevator().isElevatorLevel3ButtonPressed()){
        if (Robot.godSubsystem.getElevator().getElevatorLevel() != ElevatorLevel.HATCH3) {
          throw new AssertionError("Hatch3 level issue");
        }
      }
    }
    if(Robot.godSubsystem.getElevator().isBasePressed()){
      if (Robot.godSubsystem.getElevator().getElevatorLevel() != ElevatorLevel.BASE) {
        throw new AssertionError("Base level issue");
      }
    }
  }

  @Override
  protected void endTest() {

  }

  @Override
  protected boolean isFinished() {
    if(Robot.godSubsystem.getElevator().isElevatorLevel1ButtonPressed() && Robot.godSubsystem.getElevator().isElevatorLevel2ButtonPressed() && Robot.godSubsystem.getElevator().isElevatorLevel3ButtonPressed() && Robot.godSubsystem.getElevator().isBasePressed()){
      return true;
    }
    return false;
  }
}
