package frc.robot.subsystem;


import edu.wpi.first.wpilibj.command.Subsystem;

public class ElevatorCargoHatchSubsystem extends Subsystem {


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
}
