/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;

import static com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;
import static frc.robot.RobotMap.leftCargoIntake;
import static frc.robot.RobotMap.rightCargoIntake;

/**
 * Add your docs here.
 */
public class CargoIntaker extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static final CargoIntaker instance = new CargoIntaker();

  private double getJoystick() {
    return OI.intakeJoystick.getY();
  }

  private CargoIntaker() {
    double intakeJoystick = getJoystick();
    OI.leftIntake.set(intakeJoystick);
    OI.rightIntake.set(intakeJoystick);
  }

  public static CargoIntaker getInstance() {
    return instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * Sets the motor powers.
   */
  private synchronized void setMotorPowers(double left, double right) {
    leftCargoIntake.set(left);
    rightCargoIntake.set(right);
  }

}
