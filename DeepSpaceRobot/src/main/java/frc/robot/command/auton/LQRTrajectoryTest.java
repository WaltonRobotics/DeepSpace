package frc.robot.command.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Config;
import frc.robot.command.Localization;
import lib.motionControl.Pose2d;
import lib.motionControl.RamseteController;
import lib.motionControl.RamseteTuple;
import org.ejml.simple.SimpleMatrix;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static frc.robot.Config.LQRControlConstants.*;
import static frc.robot.Config.LQRControlConstants.MAX_OUTPUT_VOLTAGE;
import static frc.robot.Robot.drivetrain;

public class LQRTrajectoryTest extends Command {

    private static final String CSV_FILE_PATH = "/home/lvuser/deploy/ramsete_traj.csv";

    private List<Double> t = new ArrayList<>();
    private List<Double> xprof = new ArrayList<>();
    private List<Double> yprof = new ArrayList<>();
    private List<Double> thetaprof = new ArrayList<>();
    private List<Double> vprof = new ArrayList<>();
    private List<Double> omegaprof = new ArrayList<>();

    private Pose2d pose;
    private Pose2d desiredPose = new Pose2d();

    private RamseteController controller = new RamseteController();

    private double vl = Double.POSITIVE_INFINITY;
    private double vr = Double.POSITIVE_INFINITY;

    private double[] xRec;
    private double[] yRec;
    private double[] thetaRec;
    private double[] vlRec;
    private double[] vlRefRec;
    private double[] vrRec;
    private double[] vrRefRec;
    private double[] ulRec;
    private double[] urRec;

    private int pathIndex = 0;

    public LQRTrajectoryTest() {
        requires(drivetrain);
    }

    private double[] getDiffVels(double v, double omega, double d) {
        return new double[]{v - omega * d / 2.0, v + omega * d / 2.0};
    }

    @Override
    protected void initialize() {
        System.out.println("Initialized LQR trajectory test");

        BufferedReader br = null;
        String line = "";
        String cvsSplitBy = ",";

        try {
            br = new BufferedReader(new FileReader(CSV_FILE_PATH));
            // skip first line
            br.readLine();
            while ((line = br.readLine()) != null) {

                // use comma as separator
                String[] trajectoryData = line.split(cvsSplitBy);

                t.add(Double.parseDouble(trajectoryData[0]));
                xprof.add(Double.parseDouble(trajectoryData[1]));
                yprof.add(Double.parseDouble(trajectoryData[2]));
                thetaprof.add(Double.parseDouble(trajectoryData[3]));
                vprof.add(Double.parseDouble(trajectoryData[4]));
                omegaprof.add(Double.parseDouble(trajectoryData[5]));
            }

        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            if (br != null) {
                try {
                    br.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }

        xRec = new double[t.size() - 1];
        yRec = new double[t.size() - 1];
        thetaRec = new double[t.size() - 1];
        vlRec = new double[t.size() - 1];
        vlRefRec = new double[t.size() - 1];
        vrRec = new double[t.size() - 1];
        vrRefRec = new double[t.size() - 1];
        ulRec = new double[t.size() - 1];
        urRec = new double[t.size() - 1];

        Localization.setStartingPose(new Pose2d(xprof.get(0) + 0.5, yprof.get(0) + 0.5, Math.PI));

        try {
            drivetrain.resetSystem(new SimpleMatrix(new double[][]{{MIN_OUTPUT_VOLTAGE}, {MIN_OUTPUT_VOLTAGE}}),
                    new SimpleMatrix(new double[][]{{MAX_OUTPUT_VOLTAGE}, {MAX_OUTPUT_VOLTAGE}}),
                    Config.LQRControlConstants.dt, new SimpleMatrix(2, 1),
                    new SimpleMatrix(2, 1), null);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void execute() {
        desiredPose.x = xprof.get(pathIndex);
        desiredPose.y = yprof.get(pathIndex);
        desiredPose.theta = thetaprof.get(pathIndex);

        RamseteTuple tuple = controller.ramsete(desiredPose, vprof.get(pathIndex), omegaprof.get(pathIndex), pose);
        double vref = tuple.v;
        double omegaref = tuple.omega;
        double[] diffVels = getDiffVels(vref, omegaref, ROBOT_RADIUS * 2.0);
        double vlref = diffVels[0];
        double vrref = diffVels[1];
        SimpleMatrix nextR = new SimpleMatrix(new double[][]{{vlref}, {vrref}});
        drivetrain.update(nextR);
        double vc = (drivetrain.x.get(0, 0) + drivetrain.x.get(1, 0)) / 2.0;
        double omega = (drivetrain.x.get(1, 0) - drivetrain.x.get(0, 0)) / (2.0 * ROBOT_RADIUS);
        double[] diffVels1 = getDiffVels(vc, omega, ROBOT_RADIUS * 2.0);
        vl = diffVels1[0];
        vr = diffVels1[1];

        drivetrain.setVoltages(drivetrain.u.get(0), drivetrain.u.get(1));

        pose = new Pose2d(Localization.getCurrentPose());

        vlRefRec[pathIndex] = vlref;
        vrRefRec[pathIndex] = vrref;
        xRec[pathIndex] = pose.x;
        yRec[pathIndex] = pose.y;
        thetaRec[pathIndex] = pose.theta;
        vlRec[pathIndex] = vl;
        vrRec[pathIndex] = vr;
        ulRec[pathIndex] = drivetrain.u.get(0, 0);
        urRec[pathIndex] = drivetrain.u.get(1, 0);

        if (pathIndex < t.size()) {
            pathIndex += 1;
        }
    }

    @Override
    protected boolean isFinished() {
        return pathIndex >= t.size();
    }

    @Override
    protected void end() {
        System.out.println("Ended LQR trajectory test");
    }

    @Override
    protected void interrupted() {
        System.out.println("LQR trajectory test interrupted");
        end();
    }

}
