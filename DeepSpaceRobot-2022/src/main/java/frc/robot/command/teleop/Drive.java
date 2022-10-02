/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.command.teleop;

import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_LEFT_JOYSTICK_Y;
import static frc.robot.Config.SmartDashboardKeys.DRIVETRAIN_RIGHT_JOYSTICK_Y;
import static frc.robot.Robot.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config.Camera;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.command.teleop.util.Transform;
import frc.robot.util.EnhancedBoolean;
import frc.robot.util.LEDController;

public class Drive extends CommandBase {

  private static boolean enabled = true;
  private boolean isAlligning = false;
  private EnhancedBoolean rightTriggerPress = new EnhancedBoolean();
  private boolean limelightHasValidTarget;
  private double limelightDriveCommand;
  private double limelightSteerCommand;
  private double deadband = 0.05;

  public static void setIsEnabled(boolean b) {
    enabled = b;
  }

  public Drive() {
    addRequirements(drivetrain);

    //reset button not yet mapped
    //resetDrivetrainButton.whenPressed(() -> drivetrain.reset());
  }

  @Override
  public void execute() {
    driveModeChooser.getSelected().feed();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}