package frc.robot.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.config.BaseMotorControllerConfig;
import frc.robot.config.LimitedRobot;
import frc.robot.lib.config.*;

public class CompSteamWorks extends LimitedRobot {

    private final ProfiledPIDController turnPIDController = new ProfiledPIDController(0.8, 0, 0,
            new TrapezoidProfile.Constraints(360, 80));

    private final SimpleMotorFeedforward drivetrainFeedforward = new SimpleMotorFeedforward(1.0, 1.0, 1.0);

    private final PIDController leftVoltagePIDController = new PIDController(1.0, 0.0, 0.0);
    private final PIDController rightVoltagePIDController = new PIDController(1.0, 0.0, 0.0);

    private final PIDController leftVelocityPIDController = new PIDController(1.0, 0.0, 0.0);
    private final PIDController rightVelocityPIDController = new PIDController(1.0, 0.0, 0.0);

    public CompSteamWorks() {
        super("SteamWorks Comp");
    }

    @Override
    public EncoderConfig getRightEncoderConfig() {
        return new EncoderConfig() {
            @Override
            public double getDistancePerPulse() {
                return 0.00055805;
            }

            @Override
            public int getChannel1() {
                return 2;
            }

            @Override
            public int getChannel2() {
                return 3;
            }

            @Override
            public boolean isInverted() {
                return true;
            }
        };
    }

    @Override
    public EncoderConfig getLeftEncoderConfig() {
        return new EncoderConfig() {
            @Override
            public double getDistancePerPulse() {
                return 0.00055805;
            }

            @Override
            public int getChannel1() {
                return 0;
            }

            @Override
            public int getChannel2() {
                return 1;
            }

            @Override
            public boolean isInverted() {
                return false;
            }
        };
    }

    public TalonConfig getLeftTalonConfig() {
        return new TalonConfig() {
            @Override
            public int getChannel() {
                return 0;
            }

            @Override
            public boolean isInverted() {
                return true;
            }
        };
    }

    public TalonConfig getRightTalonConfig() {
        return new TalonConfig() {
            @Override
            public int getChannel() {
                return 1;
            }

            @Override
            public boolean isInverted() {
                return false;
            }
        };
    }

    @Override
    public Controls getRightJoystickConfig() {
        return () -> true;
    }

    @Override
    public Controls getLeftJoystickConfig() {
        return () -> true;
    }

    @Override
    public double getMaxAcceleration() {
        return 4.0;
    }

    @Override
    public double getKV() {
        return 0.5349180715909608;
    }

    @Override
    public double getKAcc() {
        return 0.28150490814209445;
    }

    @Override
    public double getKK() {
        return -0.02306185102694297;
    }

    @Override
    public double getKS() {
        return 0.85;
    }

    @Override
    public double getKAng() {
        return 0.3;
    }

    @Override
    public double getKL() {
        return 2.0;
    }

    @Override
    public boolean reverseAngleCalculation() {
        return false;
    }


    @Override
    public double getRobotWidth() {
        return 0.75;
    }

    @Override
    public double getRobotLength() {
        return 0.85;
    }

    @Override
    public boolean isCurrentRobot() {
        return false;
    }

    @Override
    public double getKBeta() {
        return 0;
    }

    @Override
    public double getKZeta() {
        return 0;
    }

    @Override
    public double effectiveWheelbaseRadius() {
        return 0;
    }

    @Override
    public double wheelRadius() {
        return 0;
    }

    @Override
    public double robotMass() {
        return 0;
    }

    @Override
    public double robotMOI() {
        return 0;
    }

    @Override
    public double robotAngularDrag() {
        return 0;
    }

    @Override
    public MotorConfig leftMotorConfig() {
        return new MotorConfig() {
            @Override
            public int getChannel() {
                return 0;
            }

            @Override
            public boolean isInverted() {
                return true;
            }

            @Override
            public MotorParameters getMotorParameters() {
                return new MotorParameters(0, 0, 0);
            }
        };
    }

    @Override
    public MotorConfig rightMotorConfig() {
        return new MotorConfig() {
            @Override
            public int getChannel() {
                return 1;
            }

            @Override
            public boolean isInverted() {
                return false;
            }

            @Override
            public MotorParameters getMotorParameters() {
                return new MotorParameters(0, 0, 0);
            }
        };
    }


    @Override
    public BaseMotorControllerConfig getCargoSubsystemLimits() {
        return null;
    }

    @Override
    public BaseMotorControllerConfig getHatchSubsystemLimits() {
        return null;
    }

    @Override
    public BaseMotorControllerConfig getElevatorSubsystemLimits() {
        return null;
    }

    @Override
    public TalonConfig getLeftIntakeMotorConfig() {
        return null;
    }

    @Override
    public TalonConfig getRightIntakeMotorConfig() {
        return null;
    }

    @Override
    public ProfiledPIDController getTurnPIDController() {
        return turnPIDController;
    }

    @Override
    public SimpleMotorFeedforward getDrivetrainFeedforward() {
        return drivetrainFeedforward;
    }

    @Override
    public PIDController getLeftVoltagePIDController() {
        return leftVoltagePIDController;
    }

    @Override
    public PIDController getRightVoltagePIDController() {
        return rightVoltagePIDController;
    }

    @Override
    public PIDController getLeftVelocityPIDController() {
        return leftVelocityPIDController;
    }

    @Override
    public PIDController getRightVelocityPIDController() {
        return rightVelocityPIDController;
    }

    @Override
    public void initLimits() {

    }

    @Override
    public TalonConfig getClimberMotorConfig() {
        return null;
    }

    @Override
    public void defineTargets() {

    }
}
