/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Config.Inputs.GAMEPAD_PORT;
import static frc.robot.Config.Inputs.LEFT_JOYSTICK_PORT;
import static frc.robot.Config.Inputs.RIGHT_JOYSTICK_PORT;
import static frc.robot.Config.Inputs.SHIFT_DOWN_PORT;
import static frc.robot.Config.Inputs.SHIFT_UP_PORT;
import static frc.robot.Gamepad.Button.*;
import static frc.robot.Robot.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import org.waltonrobotics.controller.Pose;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */
public class OI {

  public static final Joystick leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
  public static final Joystick rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
  public static final Gamepad gamepad = new Gamepad(GAMEPAD_PORT);

  public static final JoystickButton shiftUp = new JoystickButton(leftJoystick, SHIFT_UP_PORT);
  public static final JoystickButton shiftDown = new JoystickButton(leftJoystick, SHIFT_DOWN_PORT);
  public static final Gamepad.Button elevatorUpButton = RIGHT_TRIGGER;
  public static final Gamepad.Button elevatorDownButton = LEFT_TRIGGER;
  public static final Gamepad.Button elevatorZeroButton = _1;
  public static final Gamepad.Button hatchIntakeButton = _2;
  public static final Gamepad.Button hatchFlipOutButton = _4;
  public static final Gamepad.Button intakeCargoButton = RIGHT_BUMPER;
  public static final Gamepad.Button outtakeCargoButton = LEFT_BUMPER; 

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  static {
    JoystickButton resetEncoders = new JoystickButton(rightJoystick, 2);
    resetEncoders.whenPressed(new InstantCommand() {
      @Override
      protected void initialize() {
        drivetrain.reset();
        drivetrain.setStartingPosition(Pose.ZERO);
      }
    });
  }
}
