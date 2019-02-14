package frc.robot.subsystem;


import edu.wpi.first.wpilibj.command.Subsystem;

public class ElevatorCargoHatchSubsystem extends Subsystem {

  public ActiveState currentActiveState = ActiveState.ROBOT_SWITCHED_ON;

  public ActiveState getCurrentActiveState() {
    return currentActiveState;
  }

  public void setCurrentActiveState(ActiveState currentActiveState) {
    this.currentActiveState = currentActiveState;
  }

  @Override
  public void periodic() {
    collectSensorData();
    processSensorData();
  }

  private void processSensorData() {
  }

  private void collectSensorData() {

  }

  @Override
  protected void initDefaultCommand() {

  }

  public enum ActiveState {
    ROBOT_SWITCHED_ON,
    CARGO_HANDLING,
    DEFENSE,
    HATCH_HANDLING
  }
}
