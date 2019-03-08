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
import static frc.robot.Gamepad.Button.LEFT_BUMPER;
import static frc.robot.Gamepad.Button.LEFT_TRIGGER;
import static frc.robot.Gamepad.Button.RIGHT_BUMPER;
import static frc.robot.Gamepad.Button.RIGHT_TRIGGER;
import static frc.robot.Gamepad.Button._1;
import static frc.robot.Gamepad.Button._10;
import static frc.robot.Gamepad.Button._2;
import static frc.robot.Gamepad.Button._3;
import static frc.robot.Gamepad.Button._4;
import static frc.robot.Gamepad.Button._9;
import static frc.robot.Robot.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import org.waltonrobotics.metadata.Pose;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */

/*

Alex is a demanding person

1. make 9 and 10 cargo and hatch
2. make dpad the defense
3. .
4. slow and fast outtake - 5 and 7
5. fast intake - 8
6. hatch - 6
7. elevator modes = bottom = 2,middle = 1 and 3, top position = 4
8. hold elevator is click down
9. right joystick is elevator
10. left joystick is cargo pivot

 */


public class OI {

  public static final Joystick leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
  public static final Joystick rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
  public static final Gamepad gamepad = new Gamepad(GAMEPAD_PORT);

  public static final JoystickButton shiftUp = new JoystickButton(leftJoystick, SHIFT_UP_PORT);
  public static final JoystickButton shiftDown = new JoystickButton(leftJoystick, SHIFT_DOWN_PORT);
  public static final JoystickButton elevatorLevel3Button = new JoystickButton(gamepad,
      _4.index()); // All elevator joystick button ports are makeshift for now.
  public static final JoystickButton elevatorLevel2Button = new JoystickButton(gamepad, _3.index());
  public static final JoystickButton elevatorLevel1Button = new JoystickButton(gamepad, _2.index());
  public static final JoystickButton elevatorZeroButton = new JoystickButton(gamepad, _1.index());
  public static final JoystickButton hatchIntakeButton = new JoystickButton(gamepad, RIGHT_BUMPER.index());
  public static final JoystickButton intakeCargoButton = new JoystickButton(gamepad, RIGHT_TRIGGER.index());
  public static final JoystickButton outtakeCargoButtonFast = new JoystickButton(gamepad, LEFT_TRIGGER.index());
  public static final JoystickButton outtakeCargoButtonSlow = new JoystickButton(gamepad, LEFT_BUMPER.index());

  public static final JoystickButton cargoModeButton = new JoystickButton(gamepad, _10.index());
  public static final JoystickButton hatchModeButton = new JoystickButton(gamepad, _9.index());

  public static final JoystickButton hatchStart = new JoystickButton(rightJoystick, 8);
  public static final JoystickButton cargoStart = new JoystickButton(rightJoystick, 9);

  public static final JoystickButton bringUp = new JoystickButton(rightJoystick, 8);
  public static final JoystickButton bringDown = new JoystickButton(rightJoystick, 9);
//  public static final JoystickButton defenseModeButton = new JoystickButton(gamepad, gamepad.getPO);

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

//    TODO uncomment this to use autoassist
//    JoystickButton rightTrigger = new JoystickButton(rightJoystick,0);
//    rightTrigger.whenPressed(new AutoAssist());
  }

  private OI() {
  }
}
