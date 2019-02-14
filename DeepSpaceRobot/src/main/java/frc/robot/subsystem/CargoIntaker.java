/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
<<<<<<< HEAD
import frc.robot.command.teleop.CargoIntake;

import static frc.robot.Gamepad.Button.RIGHT_BUMPER;
import static frc.robot.Gamepad.Button.LEFT_BUMPER;
import static frc.robot.RobotMap.leftCargoIntake;
import static frc.robot.RobotMap.rightCargoIntake;
import static frc.robot.OI.gamepad;

=======
import frc.robot.RobotMap;
>>>>>>> Subsystems

public class CargoIntaker extends Subsystem {

  private static final CargoIntaker instance = new CargoIntaker();


  private CargoIntaker() {
  }


  public static CargoIntaker getInstance() {
    return instance;
  }

  public void activateIntake() {
      if (gamepad.getButton(RIGHT_BUMPER)) {

          rightCargoIntake.set(1);
          leftCargoIntake.set(1);

      } else if (gamepad.getButton(LEFT_BUMPER)) {
          rightCargoIntake.set(-1);
          leftCargoIntake.set(-1);
      }

  }

  public void getIntakeState() {

      

  }


  public void rotateIntake() {



  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CargoIntake());
  }
}