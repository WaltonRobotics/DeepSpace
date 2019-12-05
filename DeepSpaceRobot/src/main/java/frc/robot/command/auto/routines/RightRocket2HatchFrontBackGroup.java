package frc.robot.command.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Paths.RightRocketBackAndFrontLv1;
import frc.robot.command.auto.SetHatchIntake;
import frc.robot.command.auto.VisionAlign;
import lib.utils.RamseteCommand;
import lib.trajectory.Trajectory;

import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Robot.*;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

public class RightRocket2HatchFrontBackGroup extends CommandGroup {

    public RightRocket2HatchFrontBackGroup() {

        Trajectory toFrontLv1 = RightRocketBackAndFrontLv1.generateToFrontLv1();
        Trajectory backUpLv1 = RightRocketBackAndFrontLv1.generateBackUpFrontLv1();
        Trajectory toLoadingStationfromLv1 = RightRocketBackAndFrontLv1.generateToLoadingStation();
        Trajectory generateToBackLv1FromLoadingStation = RightRocketBackAndFrontLv1.generateToBackLv1FromLoadingStation();

        addSequential(new RamseteCommand(toFrontLv1,
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
        addSequential(new VisionAlign(1));
        addSequential(new SetHatchIntake(false));
        addSequential(new RamseteCommand(backUpLv1,
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
        addSequential(new RamseteCommand(toLoadingStationfromLv1,
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
        addSequential(new VisionAlign(1));
        addSequential(new SetHatchIntake(true));
        addSequential(new RamseteCommand(generateToBackLv1FromLoadingStation,
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
        addSequential(new VisionAlign(1.1));
        addSequential(new SetHatchIntake(false));
    }
}
