package frc.robot;

public class Constants {

  public static final double kDriveWheelTrackWidthInches = 18.9;
  // public static final double kDriveWheelDiameterInches = 3.8;// * 0.99;
  // public static final double kDriveWheelDiameterInches = 4.08524;
  public static final double kDriveWheelDiameterInches = 3.94;
  public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
  public static final double kTrackScrubFactor = 1.0;  //Tune me!

  // Tuned dynamics
  public static final double kRobotLinearInertia = 44.9;  // kg
  public static final double kRobotAngularInertia = 10.0;  // kg m^2
  public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec)
  public static final double kDriveVIntercept = 1.055;  // V
  public static final double kDriveKv = 0.125;  // V per rad/s
  public static final double kDriveKa = 0.01;  // V per rad/s^2

  // Geometry
  // public static final double kCenterToFrontBumperDistance = 14.99;	//34.473 / 2.0;
  // public static final double kCenterToRearBumperDistance = 19.48;		//34.473 / 2.0;
  // public static final double kCenterToSideBumperDistance = 32.47 / 2.0;
  public static final double kCenterToFrontBumperDistance = 18.64;	//34.473 / 2.0;
  public static final double kCenterToRearBumperDistance = 18.64;		//34.473 / 2.0;
  public static final double kCenterToSideBumperDistance = 28.12 / 2.0;

  /* CONTROL LOOP GAINS */

  // Gearing and mechanical constants.
  public static final double kDriveDownShiftVelocity = 9.5 * 12.0;  // inches per second
  public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
  public static final double kDriveUpShiftVelocity = 11.0 * 12.0;  // inches per second

  public static final double kPathKX = 4.0;  // units/s per unit of error
  public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
  public static final double kPathMinLookaheadDistance = 24.0;  // inches

  // PID gains for drive velocity loop (LOW GEAR)
  // Units: setpoint, error, and output are in ticks per second.
  // public static final double kDriveLowGearVelocityKp = 0.0;
  public static final double kDriveLowGearVelocityKp = 0.7;	//0.9;
  public static final double kDriveLowGearVelocityKi = 0.0;
  // public static final double kDriveLowGearVelocityKd = 0.0;
  public static final double kDriveLowGearVelocityKd = 7.7;	//10.0;
  public static final double kDriveLowGearVelocityKf = 0.0;
  public static final int kDriveLowGearVelocityIZone = 0;
  public static final double kDriveVoltageRampRate = 0.0;

  //PID Drive Position Gains
  public static final double kDriveLowGearPositionKp = 0.4;//0.02;//0.025;//0.0002;// 1.0/5000;	//0.9;
  public static final double kDriveLowGearPositionKi = 0.002;//0.0001;// 0.002/5000;
  public static final double kDriveLowGearPositionKd = 60.0;//2.0;//0.2;//0.25;//0.02; //100.0/5000;	//10.0;
  public static final double kDriveLowGearPositionKf = 0.0;//0.32;
  public static final int kDriveLowGearPositionIZone = 700;
  public static final double kDriveLowGearPositionRampRate = 240.0; // V/s
  public static final double kDriveLowGearNominalOutput = 0.5; // V
  public static final double kDriveLowGearMaxVelocity = 6.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 6 fps
  // in RPM
  public static final double kDriveLowGearMaxAccel = 18.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 18 fps/s

  //Aiming gains
  public static final double kP_aim = 0.002;

  //CAN timeouts
  public static final int kCANTimeoutMs = 10; //use for on the fly updates
  public static final int kLongCANTimeoutMs = 100; //use for constructors

}
