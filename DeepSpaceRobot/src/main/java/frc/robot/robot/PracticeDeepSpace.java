package frc.robot.robot;


import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Cargo;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Elevator;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Hatch;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;
import frc.robot.subsystem.TalonSRXConfig;
import frc.robot.subsystem.SusystemTargets;
import org.waltonrobotics.util.Controls;
import org.waltonrobotics.util.EncoderConfig;
import org.waltonrobotics.util.TalonConfig;

public class PracticeDeepSpace extends LimitedRobot {

  public PracticeDeepSpace() {
    super("Practice DeepSpace");
  }

  @Override
  public EncoderConfig getRightEncoderConfig() {
    return new EncoderConfig() {
      @Override
      public double getDistancePerPulse() {
        return 0.0;
      }

      @Override
      public int getChannell1() {
        return 0;
      }

      @Override
      public int getChannell2() {
        return 0;
      }

      @Override
      public boolean isInverted() {
        return false;
      }
    };
  }

  @Override
  public EncoderConfig getLeftEncoderConfig() {
    return new EncoderConfig() {
      @Override
      public double getDistancePerPulse() {
        return 0;
      }

      @Override
      public int getChannell1() {
        return 0;
      }

      @Override
      public int getChannell2() {
        return 0;
      }

      @Override
      public boolean isInverted() {
        return false;
      }
    };
  }

  @Override
  public TalonConfig getLeftTalonConfig() {
    return new TalonConfig() {
      @Override
      public int getChanell() {
        return 0;
      }

      @Override
      public boolean isInverted() {
        return false;
      }
    };
  }

  @Override
  public TalonConfig getRightTalonConfig() {
    return new TalonConfig() {
      @Override
      public int getChanell() {
        return 0;
      }

      @Override
      public boolean isInverted() {
        return false;
      }
    };
  }

  @Override
  public Controls getRightJoystickConfig() {
    return () -> false;
  }

  @Override
  public Controls getLeftJoystickConfig() {
    return () -> false;
  }

  @Override
  public double getMaxAcceleration() {
    return 0;
  }

  @Override
  public double getMaxVelocity() {
    return 0;
  }

  @Override
  public double getKV() {
    return 0;
  }

  @Override
  public double getKAcc() {
    return 0;
  }

  @Override
  public double getKK() {
    return 0;
  }

  @Override
  public double getKS() {
    return 0;
  }

  @Override
  public double getKAng() {
    return 0;
  }

  @Override
  public double getKL() {
    return 0;
  }


  @Override
  public double getRobotWidth() {
    return 0.78;
  }

  @Override
  public double getRobotLength() {
    return 0.83;
  }

  @Override
  public boolean isCurrentRobot() {
    return false;
  }

  @Override
  public TalonSRXConfig getCargoSubsystemLimits() {
    return null;
  }

  @Override
  public TalonSRXConfig getHatchSubsystemLimits() {
    return null;
  }

  @Override
  public TalonSRXConfig getElevatorSubsystemLimits() {
    return null;
  }

  @Override
  public SusystemTargets getTargets() {
    return null;
  }

  @Override
  public void initLimits() {
    this.getLimits().put(HatchPosition.SAFE, new LimitPair(70, 90));
    this.getLimits().put(HatchPosition.DEPLOY, new LimitPair( 0, 90));
    this.getLimits().put(HatchPosition.CARGO_START, new LimitPair(90, 100));
    this.getLimits().put(HatchPosition.HATCH_START, new LimitPair(100, 110));
    this.getLimits().put(CargoPosition.SAFE, new LimitPair(70, 90));
    this.getLimits().put(CargoPosition.DEPLOY, new LimitPair(0, 90));
    this.getLimits().put(ElevatorLevel.BASE, new LimitPair(0, 100));
    this.getLimits().put(ElevatorLevel.LEVEL1, new LimitPair(100, 200));
    this.getLimits().put(ElevatorLevel.LEVEL2, new LimitPair(200, 300));
    this.getLimits().put(ElevatorLevel.LEVEL3, new LimitPair(300, 400));

  }
}
