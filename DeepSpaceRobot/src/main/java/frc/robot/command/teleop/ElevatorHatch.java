/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command.teleop;

import static frc.robot.RobotMap.hatchIntake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Elevator;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Hatch;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchControlMode;


public class ElevatorHatch extends Command {

  private final Elevator elevator;
  private final Hatch hatch;

  public ElevatorHatch() {
    requires(Robot.godSubsystem);
    elevator = Robot.godSubsystem.getElevator();
    hatch = Robot.godSubsystem.getHatch();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    boolean elevatorManual = Math.abs(elevator.getElevatorJoystick()) > 0.1;
    if (elevatorManual) {
      hatch.setHatchControlMode(HatchControlMode.AUTO);

      if (hatch.isCurrentIntakeButtonPressed()) {
        if (hatchIntake.get()) {
          hatchIntake.set(false);
        } else {
          hatchIntake.set(true);
        }
      }

      elevator.setElevatorControlMode(ElevatorControlMode.MANUAL);
      elevator.setElevatorPower(elevator.getElevatorJoystick());

    } else {
      elevator.setElevatorControlMode(ElevatorControlMode.AUTO);

      if (elevator.isBasePressed()) {
        elevator.setElevatorLevel(ElevatorLevel.BASE);
      } else if (elevator.isElevatorLevel1ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.HATCH1);
      } else if (elevator.isElevatorLevel2ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.HATCH2);
      } else if (elevator.isElevatorLevel3ButtonPressed()) {
        elevator.setElevatorLevel(ElevatorLevel.HATCH3);
      }

      if (hatch.isCurrentIntakeButtonPressed()) {
        if (hatchIntake.get()) {
          hatchIntake.set(false);
        } else {
          hatchIntake.set(true);
        }
      }
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
