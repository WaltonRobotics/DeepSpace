package frc.robot.command.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Paths;
import frc.robot.command.auto.SetElevatorLevel;
import frc.robot.command.auto.SetHatchIntake;
import frc.robot.command.auto.VisionAlign;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;
import lib.Utils.RamseteCommand;

import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

public class RightRocket2HatchFrontGroup extends CommandGroup {
    public RightRocket2HatchFrontGroup() {
        addSequential(new RamseteCommand(Paths.RightRocketFront2.generateToFrontLv1(),
                drivetrain::updateRobotPose,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new VisionAlign(1.5));
        addSequential(new SetHatchIntake(false));
        addSequential(new RamseteCommand(Paths.RightRocketFront2.generateBackUpFrontLv1(),
                drivetrain::updateRobotPose,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new RamseteCommand(Paths.RightRocketFront2.generateToLoadingStation(),
                drivetrain::updateRobotPose,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new VisionAlign(1.8));
        addSequential(new SetHatchIntake(true));
        addSequential(new RamseteCommand(Paths.RightRocketFront2.generateToFrontLv2FromLoading(),
                drivetrain::updateRobotPose,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new SetElevatorLevel(ElevatorCargoHatchSubsystem.ElevatorLevel.HATCH2));
        addSequential(new VisionAlign(3));
    }
}
