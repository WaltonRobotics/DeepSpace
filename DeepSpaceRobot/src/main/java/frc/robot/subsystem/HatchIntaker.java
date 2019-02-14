/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import static frc.robot.RobotMap.hatchIntake;
import static frc.robot.RobotMap.hatchRotationMotor;

/**
 * Add your docs here.
 */
public class HatchIntaker extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  private static final HatchIntaker instance = new HatchIntaker();

  private HatchIntaker() {
  }


  public void openHatchIntake() {
    if(!hatchIntake.get()) {
      hatchIntake.set(true);
    }
  }

  public void closeHatchIntake() {
    if(hatchIntake.get()) {
      hatchIntake.set(false);
    }
  }

  public void flipOutHatchIntake() {
    hatchRotationMotor.set(ControlMode.MotionMagic, 1);
  }

  public void flipInHatchIntake() {
    hatchRotationMotor.set(ControlMode.MotionMagic, -1);
  }


  public static HatchIntaker getHinstance() {
    return instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
