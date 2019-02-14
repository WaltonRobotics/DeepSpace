package frc.robot.subsystem;


import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.hatchRotationMotor;

public class ElevatorCargoHatchSubsystem extends Subsystem {
  private static final ElevatorCargoHatchSubsystem instance = new ElevatorCargoHatchSubsystem();

  public static ElevatorCargoHatchSubsystem getInstance() {
    return instance;
  }


  public ActiveState currentActiveState = ActiveState.ROBOT_SWITCHED_ON;

  public ActiveState getCurrentActiveState() {
    return currentActiveState;
  }

  public void intakeCargo() {

    RobotMap.leftIntakeMotor.set(1);
    RobotMap.rightIntakeMotor.set(1);

  }

  public void outTakeCargo() {

    RobotMap.leftIntakeMotor.set(-1);
    RobotMap.rightIntakeMotor.set(-1);

  }

  public void flipOutClawSystem() {
    RobotMap.clawRotationMotor.set(ControlMode.MotionMagic, 1);
  }

  public void flipInClawSystem() {
    RobotMap.clawRotationMotor.set(ControlMode.MotionMagic, -1);
  }

  public void resetElevator() {
    elevatorMotor.set(ControlMode.MotionMagic, 0);
  }

  public void openHatchIntake() {
    if(!hatchIntake.get()) {
      hatchIntake.set(true);
    }
  }

  public void closeHatchIntake() {
    if(hatchIntake.get()) {
      hatchIntake.set(false);
    }
  }

  public void flipOutHatchIntake() {
    hatchRotationMotor.set(ControlMode.MotionMagic, 1);
  }

  public void flipInHatchIntake() {
    hatchRotationMotor.set(ControlMode.MotionMagic, -1);
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
