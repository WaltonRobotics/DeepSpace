package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

import static frc.robot.Config.FieldConfiguration.DISTANCE_TO_REFLECTION_LINE;
import static frc.robot.Config.FieldConfiguration.LIVE_DASHBOARD_FIELD_HEIGHT;

public class LiveDashboardHelper {

    public static void putRobotData(Pose2d currentPose) {
        if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
            LiveDashboard.getInstance().setRobotX(Units.metersToFeet(DISTANCE_TO_REFLECTION_LINE * 2 - currentPose.getTranslation().getX()));
            LiveDashboard.getInstance().setRobotY(Units.metersToFeet(LIVE_DASHBOARD_FIELD_HEIGHT - currentPose.getTranslation().getY()));
            LiveDashboard.getInstance().setRobotHeading(currentPose.getRotation().plus(Rotation2d.fromDegrees(180)).getRadians());
        } else {
            LiveDashboard.getInstance().setRobotX(Units.metersToFeet(currentPose.getTranslation().getX()));
            LiveDashboard.getInstance().setRobotY(Units.metersToFeet(currentPose.getTranslation().getY()));
            LiveDashboard.getInstance().setRobotHeading(currentPose.getRotation().getRadians());
        }
    }

    public static void putTrajectoryData(Pose2d trajectoryPose) {
        if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
            LiveDashboard.getInstance().setPathX(Units.metersToFeet(DISTANCE_TO_REFLECTION_LINE * 2 - trajectoryPose.getTranslation().getX()));
            LiveDashboard.getInstance().setPathY(Units.metersToFeet(LIVE_DASHBOARD_FIELD_HEIGHT - trajectoryPose.getTranslation().getY()));
            LiveDashboard.getInstance().setPathHeading(trajectoryPose.getRotation().plus(Rotation2d.fromDegrees(180)).getRadians());
        } else {
            LiveDashboard.getInstance().setPathX(Units.metersToFeet(trajectoryPose.getTranslation().getX()));
            LiveDashboard.getInstance().setPathY(Units.metersToFeet(trajectoryPose.getTranslation().getY()));
            LiveDashboard.getInstance().setPathHeading(trajectoryPose.getRotation().getRadians());
        }
    }

}
