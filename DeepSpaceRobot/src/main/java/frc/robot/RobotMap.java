/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Config.Hardware.ELEVATOR_LOWER_LIMIT_CHANNEL;
import static frc.robot.Config.Hardware.HATCH_INTAKE_CHANNEL;
import static frc.robot.Config.Hardware.LED_CHANNEL5;
import static frc.robot.Config.Hardware.LED_CHANNEL6;
import static frc.robot.Config.Hardware.SHIFTER_CHANNEL;
import static frc.robot.Robot.currentRobot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.*;
import frc.robot.util.VictorPair;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into to a variable name. This provides
 * flexibility changing wiring, makes checking the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public final class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

//
//  public static final VictorPair rightWheels = new VictorPair(currentRobot.getRightTalonConfig().getChannel(),
//      currentRobot.getRightTalonConfig().getChannel() + 1);
//  public static final VictorPair leftWheels = new VictorPair(currentRobot.getLeftTalonConfig().getChannel(),
//      currentRobot.getLeftTalonConfig().getChannel() + 1);


  public static final CANSparkMax rightWheelsMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static final CANSparkMax rightWheelsSlave = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

  public static final CANSparkMax leftWheelsMaster = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static final CANSparkMax leftWheelsSlave = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

  public static final SpeedController leftIntakeMotor = new Talon(currentRobot.getLeftIntakeMotorConfig().getChannel());
  public static final SpeedController rightIntakeMotor = new Talon(
      currentRobot.getRightIntakeMotorConfig().getChannel());
  public static final TalonSRX clawRotationMotor = new TalonSRX(currentRobot.getCargoSubsystemLimits().getDeviceID());
  public static final TalonSRX hatchRotationMotor = new TalonSRX(currentRobot.getHatchSubsystemLimits().getDeviceID());
  public static final TalonSRX elevatorMotor = new TalonSRX(currentRobot.getElevatorSubsystemLimits().getDeviceID());


  public static final Solenoid shifter = new Solenoid(SHIFTER_CHANNEL);
  public static final DoubleSolenoid hatchIntake = new DoubleSolenoid(HATCH_INTAKE_CHANNEL, HATCH_INTAKE_CHANNEL + 1);
  public static final DigitalInput elevatorLowerLimit = new DigitalInput(ELEVATOR_LOWER_LIMIT_CHANNEL);

  public static final DigitalOutput LED1 = new DigitalOutput(LED_CHANNEL5);
  public static final DigitalOutput LED2 = new DigitalOutput(LED_CHANNEL6);

  public static final Encoder encoderRight = new Encoder(
      new DigitalInput(currentRobot.getRightEncoderConfig().getChannel1()),
      new DigitalInput(currentRobot.getRightEncoderConfig().getChannel2()));

  public static final Encoder encoderLeft = new Encoder(
      new DigitalInput(currentRobot.getLeftEncoderConfig().getChannel1()),
      new DigitalInput(currentRobot.getLeftEncoderConfig().getChannel2()));

  public static final AHRS ahrs = new AHRS(SPI.Port.kMXP);

//  public static final DigitalInput hatchSensor = new DigitalInput(1); //makeshift number

//  public static final Encoder cargoEncoder = new Encoder(
//      new DigitalInput(1),
//      new DigitalInput(1)
//  );

  public static final SpeedController climberMotor = "Comp DeepSpace".equals(currentRobot.getRobotName()) ? new Victor(
      currentRobot.getClimberMotorConfig().getChannel()) : new Talon(currentRobot.getClimberMotorConfig().getChannel());

  static {
    leftIntakeMotor.setInverted(currentRobot.getLeftIntakeMotorConfig().isInverted());
    rightIntakeMotor.setInverted(currentRobot.getRightIntakeMotorConfig().isInverted());
    climberMotor.setInverted(currentRobot.getClimberMotorConfig().isInverted());
    // rightWheelsMaster.setInverted(true);
    // rightWheelsSlave.setInverted(true);
  }

  private RobotMap() {
  }
}
