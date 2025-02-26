// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.BooleanSupplier;

//import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
  import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
  import edu.wpi.first.math.kinematics.SwerveModulePosition;
  import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.util.PathPlannerLogging;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  SwerveModulePosition[] moduleStates;
  private final SwerveModule frontLeft = new SwerveModule(
    Constants.OperatorConstants.LEFT_FRONT_SPEED_MOTOR_ID,      //can ID for speed
    Constants.OperatorConstants.LEFT_FRONT_DIRECTION_MOTOR_ID,  //can ID for direction
    Constants.LEFT_FRONT_SPEED_IS_REVERSED,                     //boolean for if the speed motor is set to reverse 
    Constants.LEFT_FRONT_DIRECTION_IS_REVERSED,                 //boolean for if the direction motor is set to reverse
    Constants.OperatorConstants.LEFT_FRONT_CANCODER_ID,         //can ID for cancoder
    Constants.LEFT_FRONT_RADIAN_OFFSET,                         //to correct for encoder, so 0 is actually pointing ahead
    Constants.LEFT_FRONT_CANCODER_IS_REVERSED
  );

  private final SwerveModule frontRight = new SwerveModule(
    Constants.OperatorConstants.RIGHT_FRONT_SPEED_MOTOR_ID,      //can ID for speed
    Constants.OperatorConstants.RIGHT_FRONT_DIRECTION_MOTOR_ID,  //can ID for direction
    Constants.RIGHT_FRONT_SPEED_IS_REVERSED,                     //boolean for if the speed motor is set to reverse 
    Constants.RIGHT_FRONT_DIRECTION_IS_REVERSED,                 //boolean for if the direction motor is set to reverse
    Constants.OperatorConstants.RIGHT_FRONT_CANCODER_ID,         //can ID for cancoder
    Constants.RIGHT_FRONT_RADIAN_OFFSET,                         //to correct for encoder, so 0 is actually pointing ahead
    Constants.RIGHT_FRONT_CANCODER_IS_REVERSED
  );

  private final SwerveModule backLeft = new SwerveModule(
    Constants.OperatorConstants.LEFT_BACK_SPEED_MOTOR_ID,      //can ID for speed
    Constants.OperatorConstants.LEFT_BACK_DIRECTION_MOTOR_ID,  //can ID for direction
    Constants.LEFT_BACK_SPEED_IS_REVERSED,                     //boolean for if the speed motor is set to reverse 
    Constants.LEFT_BACK_DIRECTION_IS_REVERSED,                 //boolean for if the direction motor is set to reverse
    Constants.OperatorConstants.LEFT_BACK_CANCODER_ID,         //can ID for cancoder
    Constants.LEFT_BACK_RADIAN_OFFSET,                         //to correct for encoder, so 0 is actually pointing ahead
    Constants.LEFT_BACK_CANCODER_IS_REVERSED
  );

  private final SwerveModule backRight = new SwerveModule(
    Constants.OperatorConstants.RIGHT_BACK_SPEED_MOTOR_ID,      //can ID for speed
    Constants.OperatorConstants.RIGHT_BACK_DIRECTIION_MOTOR_ID,  //can ID for direction
    Constants.RIGHT_BACK_SPEED_IS_REVERSED,                     //boolean for if the speed motor is set to reverse 
    Constants.RIGHT_BACK_DIRECTION_IS_REVERSED,                 //boolean for if the direction motor is set to reverse
    Constants.OperatorConstants.RIGHT_BACK_CANCODER_ID,         //can ID for cancoder
    Constants.RIGHT_BACK_RADIAN_OFFSET,                         //to correct for encoder, so 0 is actually pointing ahead
    Constants.RIGHT_BACK_CANCODER_IS_REVERSED
  );

  private final Pigeon2 gyro = new Pigeon2(Constants.PIGEON_ID);

   //Create Odometer for swerve drive
  //QQQQQQQQQQQQQQQQQ   private SwerveDriveOdometry odometer;

  //////////// Trying path planner 12-16-23
  private SwerveDriveKinematics kinematics;  // Trying Pathplanner

  //////////XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  //private LimeLight limelight3;
  private LimeLight limelight3g;

  public RobotConfig config;

 // stateStdDevs Standard deviations of the pose estimate (x position in meters, y position in meters, and heading in radians). 
  // Increase these numbers to trust your state estimate less.  I think this is where the robot thinks it is right now
  private static final edu.wpi.first.math.Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05,
      Units.degreesToRadians(5));

   /**  How much should we trust what the camera is seeing vrs where our encoders think we are at
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final edu.wpi.first.math.Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(.5, 0.5,
      Units.degreesToRadians(5));

  public final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp3 = 0;
  private double previousPipelineTimestamp2 = 0;

  public boolean isRedAlliance = false;

  // Return position of the swerve module for odometry
  public SwerveModulePosition[] getModulePosition(){
    return (new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
      });
    }

    // Return State of the swerve module for odometry 
  public SwerveModuleState[] getModuleStates(){
    return (new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
      });
  }

  // Swerve subsystem constructor 
  public SwerveSubsystem(/*LimeLight limelight3,*/ LimeLight limelight3g) {

  //Restart robot encoders on startup
  resetAllEncoders();
  
  // if we find something similar, readd this back in
  //gyro.configFactoryDefault();
  
  zeroGyro();

   kinematics = new SwerveDriveKinematics(
    new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),     // Front Left wheel position from center of robot
    new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),    // Front Right wheel postion
    new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2),    // Back Left wheel postion
    new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2)    // Back Right wheel position
  );

  
//this.limelight3 = limelight3;
this.limelight3g = limelight3g;

NetworkTableInstance.getDefault().getTable("limelight-threeg").getEntry("pipeline").setInteger(0);



    ShuffleboardTab tab = Shuffleboard.getTab("Vision2");

    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        getRotation2d(),
        getModulePosition(),
        new Pose2d(),
        //new Pose2d(1.80, 5.54, swerveSubsystem.getRotation2d()),  // This puts robot infront of speaker
        stateStdDevs,
        visionMeasurementStdDevs);

    tab.addString("Pose Estimator", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  
    //RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder
  AutoBuilder.configure(
    //this::getPose, 
     this::getCurrentPose,
    //xxxx this::resetOdometry, 
    this::resetPose,
    this::getSpeeds, 
    this::driveRobotRelative, 
    
    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
          new PIDConstants(5, 0, 0), // Translation constants   5
         new PIDConstants(3, 0, 0) // Rotation constants
    ),
      config,
        () -> {
                    // For 2024 Game: Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        //isRedAlliance = (alliance.get() == DriverStation.Alliance.Red);
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
    this
  ); 


  

  var alliance = DriverStation.getAlliance();
  if (alliance.isPresent()) {
                        isRedAlliance = (alliance.get() == DriverStation.Alliance.Red);
                    }
   System.out.println(isRedAlliance);

  if (isRedAlliance == true) {  // update constants so buttons will go toward red alliance positions
       System.out.println("Falls into if statement");
       //Constants.CommandConstants.AmpX = Constants.CommandConstants.redAmpX;
       

  }
  }


  public double getHeading()
  {

    //SmartDashboard.putNumber("Compass Heading", gyro.getAbsoluteCompassHeading());
    return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
    //return gyro.getYaw();
  }
  
  public Rotation2d  getHeadingRot2d()
  {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  public Rotation2d getRotation2d()
  {
    return Rotation2d.fromDegrees(getHeading());
  }

public void resetAllEncoders()
{
  frontLeft.resetEncoders();
  frontRight.resetEncoders();
  backLeft.resetEncoders();
  backRight.resetEncoders();
  
 }

 public double  getZSpeed()
  {
  //double[] temp = {0,0,0};
  //gyro.getRawGyro(temp);
  return (gyro.getAccelerationZ(true).getValueAsDouble() / 180 * Math.PI);
   //return (temp[2] /180 * Math.PI);
  }

  public double  getRoll()
  {
    return (gyro.getRoll().getValueAsDouble());
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // convert StattusSignals from the Pigeon into Doubles to be used


   SmartDashboard.putNumber("just yaw", gyro.getYaw().getValueAsDouble());
    SmartDashboard.putNumber("Roll", gyro.getRoll().getValueAsDouble());
    SmartDashboard.putNumber("Robot Yaw", getHeading());

    SmartDashboard.putNumber("getZSpeed", getZSpeed());
    //SmartDashboard.putNumber("Compass Heading", gyro.getAbsoluteCompassHeading());
    SmartDashboard.putNumber("Left Front Encoder Rad", frontLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Right Front Encoder Rad", frontRight.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Left Back Encoder Rad", backLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Right Back Encoder Rad", backRight.getAbsoluteEncoderRad());
    
    SmartDashboard.putNumber("Left Front Angle", frontLeft.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Right Front Angle", frontRight.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Left Back Angle", backLeft.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Right Back Angle", backRight.getAbsoluteEncoderAngle());

    SmartDashboard.putNumber("Left front distance in Meters", frontLeft.getSpeedPosition());
    SmartDashboard.putNumber("Right front distance in Meters", frontRight.getSpeedPosition());
    SmartDashboard.putNumber("Left back distance in Meters", backLeft.getSpeedPosition());
    SmartDashboard.putNumber("Right back distance in Meters", backRight.getSpeedPosition());

    SmartDashboard.putNumber("Left front direction position", frontLeft.getDirectionPosition());

    boolean isAuto = DriverStation.isAutonomous();
    /////////////////////////////////xxxxxxxxxxxxxxxxxxxxxx
    var resultTimestamp3 = limelight3g.getLatency();
    if(!isAuto && (limelight3g.getTargets() && limelight3g.getArea() > 0.120 && resultTimestamp3 != previousPipelineTimestamp3)) {

      previousPipelineTimestamp3 = resultTimestamp3;  // Make current image from limight the one we are processing so we won't process it again

      double[] botPose = limelight3g.getBotPoseBlue();  // get position of robot on field based on BLue Alliance starting pose corner

      //double[] botPose = limelight3g.getCameraPose(); // Get position of Camera.  See if this works better than getBotPoseBlue

      //Breakout the values returned in botPose above into its parts and pass to get location of robot
      // 0 = x location, 1 = Y location, 2 = z location, 3 = Roll, 4 = Pitch, 5 = Yaw, 6 = total latency (cl+tl)
      Pose3d actualPose3 = new Pose3d(botPose[0], botPose[1], botPose[2], new Rotation3d(gyro.getRoll().getValueAsDouble(), gyro.getPitch().getValueAsDouble(), getHeading()));//Units.degreesToRadians(botPose[3]), Units.degreesToRadians(botPose[4]), Units.degreesToRadians(botPose[5])));
      Pose2d actualPoseBut2d3 = new Pose2d(botPose[0], botPose[1], getHeadingRot2d());
      //System.out.println(Timer.getFPGATimestamp() - (botPose[6]/1000.0));
      //Pose3d camPose = new Pose3d(new Translation3d(5, new Rotation3d(visionArray[3], visionArray[4], visionArray[5])), new Rotation3d());
      //Transform3d camToTarget = target.getBestCameraToTarget();
      //Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

// First, tell Limelight your robot's current orientation
double robotYaw = gyro.getYaw().getValueAsDouble();  
///////LimelightHelpers.SetIMUMode(getName(), 0);

// Get the pose estimate
LimelightHelpers.SetRobotOrientation(Constants.Limelight.limelightName3g, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight.limelightName3g);

// Add it to your pose estimator
poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
poseEstimator.addVisionMeasurement(
    limelightMeasurement.pose,
    limelightMeasurement.timestampSeconds
);

      // poseEstimator.addVisionMeasurement(actualPoseBut2d3, Timer.getFPGATimestamp()
      // - (botPose[6]/1000.0), visionMeasurementStdDevs);

      ////poseEstimator.addVisionMeasurement(actualPose3.toPose2d(), Timer.getFPGATimestamp()); //actualPoseBut2d3, Timer.getFPGATimestamp() - (botPose[6]/1000.0));
    }

   // var resultTimestamp2 = limelight2.getLatency();
   // if(!isAuto && (limelight2.getTargets() && limelight2.getArea() > 0.120 && resultTimestamp2 != previousPipelineTimestamp2)) {
//
   //   previousPipelineTimestamp2 = resultTimestamp2;  // Make current image from limight the one we are processing so we won't process it again
//
//      double[] botPose = limelight2.getBotPoseBlue();  // get position of robot on field based on BLue Alliance starting pose corner
//
      //Breakout the values returned in botPose above into its parts and pass to get location of robot
      // 0 = x location, 1 = Y location, 2 = z location, 3 = Roll, 4 = Pitch, 5 = Yaw, 6 = total latency (cl+tl)
//      Pose3d actualPose2 = new Pose3d(botPose[0], botPose[1], botPose[2], new Rotation3d(gyro.getRoll(), gyro.getPitch(), getHeading()));//Units.degreesToRadians(botPose[3]), Units.degreesToRadians(botPose[4]), Units.degreesToRadians(botPose[5])));
 //     Pose2d actualPoseBut2d2 = new Pose2d(botPose[0], botPose[1], getHeadingRot2d());
      //System.out.println(Timer.getFPGATimestamp() - (botPose[6]/1000.0));
      //Pose3d camPose = new Pose3d(new Translation3d(5, new Rotation3d(visionArray[3], visionArray[4], visionArray[5])), new Rotation3d());
      
      //Transform3d camToTarget = target.getBestCameraToTarget();
      //Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

 //     poseEstimator.addVisionMeasurement(/*actualPose3.toPose2d()*/ actualPoseBut2d2, Timer.getFPGATimestamp() - (botPose[6]/1000.0));
 //   }


    /*if(limelightSubsystem.getTargets() && Math.abs(NetworkTableInstance.getDefault().getTable("limelight-three").getEntry("tx").getDouble(0)) > 2 && (NetworkTableInstance.getDefault().getTable("limelight-three").getEntry("hb").getDouble(0)) % 50 == 0)
    {
      previousPipelineTimestamp = resultTimestamp;
      goToGoal(0, -(NetworkTableInstance.getDefault().getTable("limelight-three").getEntry("tx").getDouble(0)));
      JANK!
    } */

    poseEstimator.update(
     getRotation2d(),
     getModulePosition());

    field2d.setRobotPose(getCurrentPose());
    ////////////////////////////////zzzzzzzzzzzzzzzzzzzzzzzzz
  }

  public void stopModules()
  {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates)
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.physicalMaxSpeedMPS); //NormalizeWheelSpeeds doesn't exist, this is the closest thing idk
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

 public SwerveDriveKinematics getKinematics() {
    return kinematics;
   }
 
 //////////// Trying path planner 12-16-23
  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    //ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(robotRelativeSpeeds, getRotation2d());
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, .02);  //.02

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }
////////////////////////////////////////

//////////////XXXXXXXXXXXXXXXXXXXXX
private String getFomattedPose() {
  var pose = getCurrentPose();
  return String.format("(%.2f, %.2f) %.2f degrees", 
      pose.getX(), 
      pose.getY(),
      pose.getRotation().getDegrees());
}

public Pose2d getCurrentPose() {
  return poseEstimator.getEstimatedPosition();
}


public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
      getRotation2d(),
      getModulePosition(), 
      pose);
}

//// Button commands for swerve drive

 // This is a test example that should just move the robot 1m forward of its current position
/*public void ZeroHeading(){
      Pose2d currentPose = getCurrentPose(); ///////QQQQQQQQQ getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(1.0, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          1.0, 1.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, currentPose.getRotation())
      );

      AutoBuilder.followPath(path).schedule();
    }*/

public void slowModeToggle(){

  if (Constants.isSlow)
  {
    Constants.isSlow = false;
  }
  else
  {
    Constants.isSlow = true;
  }
  
}

public void robotCentricToggle(){

  if (Constants.isFieldCentric)
  {
    Constants.isFieldCentric = false;
  }
  else
  {
    Constants.isFieldCentric = true;
  }
  
}

/*public void goToGoal(double distance, double angle)
{
  Pose2d currentPose = getCurrentPose();

  Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
  Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(0.1, 0.0)), new Rotation2d().plus(Rotation2d.fromDegrees(angle / 2)));

  List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          1.0, 1.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, new Rotation2d()));

      AutoBuilder.followPath(path).schedule();
}*/
}