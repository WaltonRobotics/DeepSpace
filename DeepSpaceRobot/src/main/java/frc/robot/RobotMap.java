/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

<<<<<<< HEAD
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
=======
import static frc.robot.Config.Hardware.SHIFTER_CHANNEL;
import static frc.robot.Robot.currentRobot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
>>>>>>> Subsystems
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

<<<<<<< HEAD
import static frc.robot.Config.Hardware.*;
import static frc.robot.Config.Hardware.RIGHT_CARGO_INTAKE_MOTOR_CHANNEL;
=======
import java.util.Random;
>>>>>>> Subsystems

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


  public static final VictorSPX rightWheel = new VictorSPX(currentRobot.getRightTalonConfig().getChanell());// FIXME: 2019-02-13
  public static final VictorSPX leftWheel = new VictorSPX(currentRobot.getLeftTalonConfig().getChanell());// FIXME: 2019-02-13
  public static final Talon leftIntakeMotor = new Talon(1);
  public static final Talon rightIntakeMotor = new Talon(1);
  public static final TalonSRX clawRotationMotor = new TalonSRX(1);
  public static final TalonSRX hatchRotationMotor = new TalonSRX(1);
  public static final TalonSRX elevatorMotor = new TalonSRX(1);


  public static final Solenoid shifter = new Solenoid(SHIFTER_CHANNEL);
  public static final Solenoid hatchIntake = new Solenoid(1);


  public static final Encoder encoderRight = new Encoder(
      new DigitalInput(currentRobot.getRightEncoderConfig().getChannell1()),
      new DigitalInput(currentRobot.getRightEncoderConfig().getChannell2()));

  public static final Encoder encoderLeft = new Encoder(
      new DigitalInput(currentRobot.getLeftEncoderConfig().getChannell1()),
      new DigitalInput(currentRobot.getLeftEncoderConfig().getChannell2()));



<<<<<<< HEAD
  public static final Talon leftCargoIntake = new Talon(LEFT_CARGO_INTAKE_MOTOR_CHANNEL);
  public static final Talon rightCargoIntake = new Talon(RIGHT_CARGO_INTAKE_MOTOR_CHANNEL);

  public static final Encoder encoderRight = new Encoder(new DigitalInput(RIGHT_ENCODER_CHANNEL1),
      new DigitalInput(RIGHT_ENCODER_CHANNEL2));

  public static final Encoder encoderLeft = new Encoder(new DigitalInput(LEFT_ENCODER_CHANNEL1),
      new DigitalInput(LEFT_ENCODER_CHANNEL2));
=======
  public static final Encoder cargoEncoder = new Encoder(
      new DigitalInput(1),
      new DigitalInput(1)
  );

>>>>>>> Subsystems

  public static final Encoder hatchEncoder = new Encoder(new DigitalInput(HATCH_INTAKE_ENCODER_CHANNEL1), new DigitalInput(HATCH_INTAKE_ENCODER_CHANNEL2));

  public static final DigitalInput hatchSensor = new DigitalInput(HATCH_SENSOR_DIGITAL_INPUT_PORT);

  public static final Solenoid hatchProngs = new Solenoid(HATCH_PRONGS_ACTUATOR_PORT);

  public static final TalonSRX elevatorMotor = new TalonSRX(ELEVATOR_TALON_PORT);

  public static final Encoder elevatorEncoder = new Encoder(new DigitalInput(ELEVATOR_ENCODER_CHANNEL1),
          new DigitalInput(ELEVATOR_ENCODER_CHANNEL2));

  public static final Talon hatchMotor = new Talon(HATCH_MOTOR_PORT);
}
