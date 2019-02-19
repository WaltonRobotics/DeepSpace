/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Cargo;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClawControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Elevator;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;


public class ElevatorCargo extends Command {

  private final Elevator elevator;
  private final Cargo cargo;

  public ElevatorCargo() {
    requires(Robot.godSubsystem);
    elevator = Robot.godSubsystem.getElevator();
    cargo = Robot.godSubsystem.getCargo();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean elevatorManual = Math.abs(elevator.getElevatorJoystick()) > 0.1;
    boolean cargoManual = Math.abs(cargo.getCargoJoystick()) > 0.1;
    if (elevatorManual || cargoManual) {
      if (elevatorManual) {
        elevator.setElevatorControlMode(ElevatorControlMode.MANUAL);
        elevator.setElevatorPower(elevator.getElevatorJoystick());
      } else {
        elevator.setElevatorControlMode(ElevatorControlMode.AUTO);
      }
      if (cargoManual) {
        cargo.setClawControlMode(ClawControlMode.MANUAL);
        cargo.setClawRotationPower(cargo.getCargoJoystick());
      } else {
        cargo.setClawControlMode(ClawControlMode.AUTO);
      }
    } else {
      elevator.setElevatorControlMode(ElevatorControlMode.AUTO);

      if (elevator.isBasePressed()) {
        elevator.setElevatorLevel(ElevatorLevel.BASE);
      } else if (elevator.isElevatorLevel1ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.CARGO1);
      } else if (elevator.isElevatorLevel2ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.CARGO2);
      } else if (elevator.isElevatorLevel3ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.CARGO3);
      }
    }
    //TODO: 2 button for diff speeds
    if (cargo.outSlowButtonPressed()) {
      cargo.outtakeCargoSlow(0);
    } else if (cargo.outFastButtonPressed()) {
      cargo.outtakeCargoFast(0);
    } else if (cargo.inButtonPressed()) {
      cargo.intakeCargo(0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}