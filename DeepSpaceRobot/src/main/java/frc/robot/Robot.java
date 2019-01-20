/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.command.teleop.util.NormalSpeed;
import frc.robot.command.teleop.util.Sigmoid;
import frc.robot.command.teleop.util.Sqrt;
import frc.robot.command.teleop.util.Transform;
import frc.robot.subsystem.Drivetrain;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  public static final Drivetrain drivetrain = new Drivetrain();

  public static void fillMat(Mat mat, double[][] data) {
    for (int i = 0; i < data.length; i++) {
      mat.put(i, 0, data[i]);
    }
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    System.out.println("Init");

    SendableChooser<Transform> sendableChooser = new SendableChooser<>();
    sendableChooser.setDefaultOption("Normal", new NormalSpeed());
    sendableChooser.addOption("Sigmoid", new Sigmoid());
    sendableChooser.addOption("Sqrt", new Sqrt());

    SmartDashboard.putData("Transform Select", sendableChooser);

    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      CvSink cvSink = CameraServer.getInstance().getVideo();

//
      int width = 640;
      int height = 480;
      CvSource outputStream = CameraServer.getInstance().putVideo("rect", width, height);
//
      Mat source = new Mat();
      Mat output = new Mat();

      double[][] k = {{540.6169741181128, 0.0, 979.9714037950379},
          {0.0, 540.4923723031342, 603.6179611901534}, {0.0, 0.0, 1.0}};
      Mat K = new Mat(3, 3, CvType.CV_64F);
      fillMat(K, k);
      System.out.println(K.dump());

      double[][] d = {{-0.04581054920752462},
          {0.00580921941468853}, {-0.0037112166109776372}, {-0.00015164559682399615}};
      Mat D = new Mat(3, 1, CvType.CV_64F);
      fillMat(D, d);
      System.out.println(D.dump());
      Size DIM = new Size(1920, 1080);

      while (!Thread.interrupted()) {
        cvSink.grabFrame(source);

        Size dim1 = source.size();
        assert (dim1.height / dim1.width == DIM.height / DIM.width);

        Size dim2 = dim1;
        Size dim3 = dim1;

        K.put(2,2 , 1);


        Mat scaledK = new Mat();

        double division = dim1.width / DIM.width;

        Core.multiply(K, new Scalar(division), scaledK);
        scaledK.put(2,2, 1.0);

//
//            new_K = cv2.fisheye
//            .estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance = balance)
//        map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
//        undistorted_img = cv2.remap(img, map1, map2, interpolation = cv2.INTER_LINEAR, borderMode = cv2.BORDER_CONSTANT)
//
        outputStream.putFrame(output);
      }
    }).start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like diagnostics that you want
   * ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // System.out.println("robot Periodic");
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You can use it to reset any subsystem
   * information you want to clear when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    System.out.println("disable init");
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different autonomous modes using
   * the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the auto name from the text box below the
   * Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to the switch structure below with
   * additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    System.out.println("auton init");
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    System.out.println("auton Periodic ");
  }

  @Override
  public void teleopInit() {
    System.out.println("Tele ini");
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    System.out.println("Tele periodci");
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
