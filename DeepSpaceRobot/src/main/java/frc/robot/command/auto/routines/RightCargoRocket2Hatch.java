package frc.robot.command.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Paths;
import frc.robot.command.auto.SetHatchIntake;
import frc.robot.command.auto.VisionAlign;
import lib.utils.RamseteCommand;
import lib.trajectory.Trajectory;

import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Config.SmartMotionConstants.KA;
import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

public class RightCargoRocket2Hatch extends CommandGroup {
    public RightCargoRocket2Hatch() {

        Trajectory toCargo1 = Paths.Right2HatchCargoRocket.generateToCargo1();
        Trajectory backUpCargo1 = Paths.Right2HatchCargoRocket.generateCargo1BackUp();
        Trajectory toLoadingStationfromCargo1 = Paths.Right2HatchCargoRocket.generateCargo1ToLoadingStation();
        Trajectory toLv1FromLoading = Paths.Right2HatchCargoRocket.generateToFrontLv1FromLoading();

        addSequential(new RamseteCommand(toCargo1,
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
        addSequential(new RamseteCommand(backUpCargo1,
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
        addSequential(new RamseteCommand(toLoadingStationfromCargo1,
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
        addSequential(new RamseteCommand(toLv1FromLoading,
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
        addSequential(new VisionAlign(3));
        addSequential(new SetHatchIntake(false));
    }
}
