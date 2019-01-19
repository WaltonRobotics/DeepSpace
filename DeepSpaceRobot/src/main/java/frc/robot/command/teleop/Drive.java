/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command.teleop;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.command.teleop.util.Transform;

public class Drive extends Command {

  public Drive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    System.out.println("Driving");
    requires(Robot.drivetrain);
  }

  public Transform getTransform() {
    return ((SendableChooser<Transform>) SmartDashboard.getData("Transform Select")).getSelected();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  private double getLeftYJoystick() {
    return OI.leftJoystick.getY();
  }


  private double getRightYJoystick() {
    return OI.rightJoystick.getY();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("Hello");
    double leftYJoystick = -getLeftYJoystick();
    double rightYJoystick = getRightYJoystick();

    SmartDashboard.putNumber("leftJoystick", leftYJoystick);
    SmartDashboard.putNumber("rightJoystick", rightYJoystick);

    Transform transform = getTransform();
    leftYJoystick = transform.transform(leftYJoystick);
    rightYJoystick = transform.transform(rightYJoystick);

    Robot.drivetrain.setSpeed(leftYJoystick, rightYJoystick);

    if (OI.shiftUp.get()) {
      Robot.drivetrain.shiftUp();
    } else if (OI.shiftDown.get()) {
      Robot.drivetrain.shiftDown();
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