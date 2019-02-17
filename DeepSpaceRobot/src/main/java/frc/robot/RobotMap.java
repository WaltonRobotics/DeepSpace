/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Config.Hardware.SHIFTER_CHANNEL;
import static frc.robot.Robot.currentRobot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.util.VictorPair;

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


  public static final VictorPair rightWheels = new VictorPair(currentRobot.getRightTalonConfig().getChanell(), currentRobot.getRightTalonConfig().getChanell() + 1);
  public static final VictorPair leftWheels = new VictorPair(currentRobot.getLeftTalonConfig().getChanell(), currentRobot.getLeftTalonConfig().getChanell() + 1);

  public static final Talon leftIntakeMotor = new Talon(2);
  public static final Talon rightIntakeMotor = new Talon(3);
  public static final TalonSRX clawRotationMotor = new TalonSRX(currentRobot.getCargoSubsystemLimits().getDeviceID());
  public static final TalonSRX hatchRotationMotor = new TalonSRX(currentRobot.getHatchSubsystemLimits().getDeviceID());
  public static final TalonSRX elevatorMotor = new TalonSRX(currentRobot.getElevatorSubsystemLimits().getDeviceID());


  public static final Solenoid shifter = new Solenoid(SHIFTER_CHANNEL);
  public static final Solenoid hatchIntake = new Solenoid(1);

  public static final DigitalInput elevatorLowerLimit = new DigitalInput(4);

  public static final Encoder encoderRight = new Encoder(
      new DigitalInput(currentRobot.getRightEncoderConfig().getChannell1()),
      new DigitalInput(currentRobot.getRightEncoderConfig().getChannell2()));

  public static final Encoder encoderLeft = new Encoder(
      new DigitalInput(currentRobot.getLeftEncoderConfig().getChannell1()),
      new DigitalInput(currentRobot.getLeftEncoderConfig().getChannell2()));

//  public static final DigitalInput hatchSensor = new DigitalInput(1); //makeshift number

//  public static final Encoder cargoEncoder = new Encoder(
//      new DigitalInput(1),
//      new DigitalInput(1)
//  );

  static {
    leftIntakeMotor.setInverted(false);
    rightIntakeMotor.setInverted(true);
  }
}
