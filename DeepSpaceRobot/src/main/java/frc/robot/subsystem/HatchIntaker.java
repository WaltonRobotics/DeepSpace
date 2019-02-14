/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

import static frc.robot.OI.*;
import static frc.robot.RobotMap.*;

/**
 * Add your docs here.
 */
public class HatchIntaker extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private Timer timer = new Timer();

  private boolean lastHatchLoadButtonPressed = false;
  private boolean currentHatchLoadButtonPressed = false;
  private boolean isHatchLoose = false;

  private boolean isSlow = false;
  private double whenSlowWasStarted;

  private double speedCapDuringLoose = 0.1;
  private double looseStateDuration = 1.5;

  private double hatchUpMotor = 1;
  private double hatchDownMotor = -1;

  private boolean isHatchButtonPressed = false;

  private double hatchStartingEncoderPosition = 1000; //makeshift
  private double hatchIntakingEncoderPosition = 100;  //makeshift

  private double hatchCurrentEncoderPosition = hatchEncoder.get();

  private enum ProngsPosition {
    OPEN, CLOSED
  }

  private static final HatchIntaker instance = new HatchIntaker();

  private HatchIntaker() {
  }

  public static HatchIntaker getHinstance() {
    return instance;
  }

  public boolean wasHatchLoadButtonPressed() {
    return (currentHatchLoadButtonPressed != lastHatchLoadButtonPressed) && currentHatchLoadButtonPressed;
  }

  public boolean isHatchLoose() {
    if (currentHatchLoadButtonPressed) {
      return !isHatchLoose;
    }
    return isHatchLoose;
  }

  public void looseStateDrive() {
    if (isHatchLoose) {
      Robot.drivetrain.setSpeeds(speedCapDuringLoose, speedCapDuringLoose);
      isSlow = true;
      whenSlowWasStarted = timer.get();
    }
  }

  public void prongsActivation() {
    if (currentHatchLoadButtonPressed) {
      hatchProngs.close();
    }
  }

  public void hatchSwitchLocation() {
    if (isHatchButtonPressed && !lastHatchLoadButtonPressed) {
      while (hatchCurrentEncoderPosition <= hatchStartingEncoderPosition) {
        hatchMotor.set(hatchUpMotor);
      }
    } if (isHatchButtonPressed && lastHatchLoadButtonPressed) {
      while (hatchCurrentEncoderPosition >= hatchIntakingEncoderPosition) {
        hatchMotor.set(hatchDownMotor);
      }
    }
  }


  public void setProngsPosition(ProngsPosition p) {
    if (p == ProngsPosition.OPEN) {
      hatchProngs.set(true);
    } else if (p == ProngsPosition.CLOSED) {
      hatchProngs.set(false);
    } else {
      hatchProngs.set(false);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic() {
    /* Read state of inputs. */
    lastHatchLoadButtonPressed = currentHatchLoadButtonPressed;
    currentHatchLoadButtonPressed = hatchLoadButton.getPressed(gamepad);

    isHatchButtonPressed = hatchLoadButton.getPressed(gamepad);

    isHatchLoose = !hatchSensor.get();

    /* Process values relevant to subsystem. */

    if (timer.get() - whenSlowWasStarted > looseStateDuration && isSlow) {
      Robot.drivetrain.setSpeeds(leftJoystick.getY(), rightJoystick.getY());
      isSlow = false;

    }
  }

}
