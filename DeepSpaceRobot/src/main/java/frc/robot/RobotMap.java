/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Config.Hardware.LEFT_ENCODER_CHANNEL1;
import static frc.robot.Config.Hardware.LEFT_ENCODER_CHANNEL2;
import static frc.robot.Config.Hardware.LEFT_WHEEL_CHANNEL;
import static frc.robot.Config.Hardware.RIGHT_ENCODER_CHANNEL1;
import static frc.robot.Config.Hardware.RIGHT_ENCODER_CHANNEL2;
import static frc.robot.Config.Hardware.RIGHT_WHEEL_CHANNEL;
import static frc.robot.Config.Hardware.SHIFTER_CHANNEL;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import frc.robot.Config.Motor;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into to a variable name. This provides
 * flexibility changing wiring, makes checking the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;


  public static final Talon rightWheel = new Talon(RIGHT_WHEEL_CHANNEL);
  public static final Talon leftWheel = new Talon(LEFT_WHEEL_CHANNEL);
  public static final Solenoid shifter = new Solenoid(SHIFTER_CHANNEL);


  public static final Encoder encoderRight = new Encoder(new DigitalInput(RIGHT_ENCODER_CHANNEL1),
      new DigitalInput(RIGHT_ENCODER_CHANNEL2));
  public static final Encoder encoderLeft = new Encoder(new DigitalInput(LEFT_ENCODER_CHANNEL1),
      new DigitalInput(LEFT_ENCODER_CHANNEL2));


}
