// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final int LEFT_FRONT_SPEED_MOTOR_ID = 20;
    public static final int LEFT_FRONT_DIRECTION_MOTOR_ID = 21;
    public static final int RIGHT_FRONT_SPEED_MOTOR_ID = 11; //11; ///// come back to this one
    public static final int RIGHT_FRONT_DIRECTION_MOTOR_ID = 10;
    public static final int LEFT_BACK_SPEED_MOTOR_ID = 30;
    public static final int LEFT_BACK_DIRECTION_MOTOR_ID = 31;
    public static final int RIGHT_BACK_SPEED_MOTOR_ID = 40;
    public static final int RIGHT_BACK_DIRECTIION_MOTOR_ID = 41;

    // Elevator Motors
    public static final int ELEVATOR_DIRECTION_MOTOR_ID = 50; //50;

    // Elevator Levels
    public static final double HOME_POSITION = 7.5; // rotations: 1 time stuck IF STUCK
    public static final double PICKUP_POSITION = 7.5; // rotations // coralARM: 4.1
    public static final double LEVEL_ONE_HEIGHT = 69.7; // rotations // coralARM: 4.1
    public static final double LEVEL_TWO_HEIGHT = 108.3; // rotations // coralARM: 4.1
    public static final double LEVEL_THREE_HEIGHT = 175; // rotations // coralARM: 13

   // public static final double tolerance = 2; // prevent jittering when reaching wanted position (elevator)

    //Absolute Encoders
    public static final int LEFT_FRONT_CANCODER_ID = 22;   
    public static final int RIGHT_FRONT_CANCODER_ID = 12;
    public static final int LEFT_BACK_CANCODER_ID = 32;
    public static final int RIGHT_BACK_CANCODER_ID = 42;

    // Hook 
    public static int FIRST_HOOK_MOTOR_ID = 58;
    public static int SECOND_HOOK_MOTOR_ID = 59;
  }

  public static final int PIGEON_ID = 1;

  public static final double LEFT_FRONT_RADIAN_OFFSET  = 0; // (0.3388) * 2 * Math.PI; //Math.toRadians(233.086);  //Math.toRadians(18.457); //19.424 18.457 // 18.193 //52.383
  public static final double RIGHT_FRONT_RADIAN_OFFSET = 0; // (0.4074) * 2 * Math.PI; //Math.toRadians(288.633); // Math.toRadians(321.943);  //Math.toRadians(144.932); //144.404 144.932 //290.742
  public static final double LEFT_BACK_RADIAN_OFFSET  = 0; // (0.3195) * 2 * Math.PI; //Math.toRadians(10.283); //9.229 10.283
  public static final double RIGHT_BACK_RADIAN_OFFSET = 0; // (0.0013) * 2 * Math.PI; // Math.toRadians(325.283);//Math.toRadians(147.92); //148.975 147.92

  public static final boolean LEFT_FRONT_SPEED_IS_REVERSED = true; // true? //false; 
  public static final boolean RIGHT_FRONT_SPEED_IS_REVERSED = false; //false; // true; 
  public static final boolean LEFT_BACK_SPEED_IS_REVERSED = true; 
  public static final boolean RIGHT_BACK_SPEED_IS_REVERSED = false; // true; 

  public static final boolean LEFT_FRONT_DIRECTION_IS_REVERSED = true; //true;
  public static final boolean RIGHT_FRONT_DIRECTION_IS_REVERSED = true; //true;
  public static final boolean LEFT_BACK_DIRECTION_IS_REVERSED = true;
  public static final boolean RIGHT_BACK_DIRECTION_IS_REVERSED = true;

  //Must stay false or offset angle will be the wrong direction and wheels will not be straignt on startup and driving
  public static final boolean LEFT_FRONT_CANCODER_IS_REVERSED = false; 
  public static final boolean RIGHT_FRONT_CANCODER_IS_REVERSED = false; 
  public static final boolean LEFT_BACK_CANCODER_IS_REVERSED = false; 
  public static final boolean RIGHT_BACK_CANCODER_IS_REVERSED = false; 

  public static final double wheelDiameter = Units.inchesToMeters(3.86);  //3.75  REAL:4 inches =  .1016 Meters  // Orig was 3.75
  public static final double speedMotorGearRatio  =  (1 / 6.75);    //.148148
  public static final double directionMotorGearRatio =  (1 / 21.4286);   //.04667
  
  // The speed encoder to rotation does not impact teleop just Auto
  public static final double speedEncoderRotationToMeter = speedMotorGearRatio * Math.PI * wheelDiameter;  // Orig - fast and goes way to far in auto.  Does not impact tele
  public static final double speedEncoderRPMToMPS = speedEncoderRotationToMeter / 60;
  
  //Changes to direction encoder do impact tele and Auto
  public static final double directionEncoderRotationToRadian = directionMotorGearRatio * 2 * Math.PI;
  public static final double directionEncoderRPMToRadsPS = directionEncoderRotationToRadian / 60;
  
  public static final double kPTurning = .5; //0.3 0.15; //how much p the turning motors get. anything > 1 makes the wheels shake

    ///**** NOTE: if physicalMaxSpeedMPS is too low the auto speeds are crazy fast and distances are off. .005 was NOT GOOD */
  public static final double physicalMaxSpeedMPS = 4.6; //5; Scott1 //5; // 0.005; //0.07 before fly-in //is this a double? what should the value be?
  public static final double physicalMaxSpeedMPSslow = 0.07;

  /** This impacts speed fo spin/turning of robot. */
  public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // 0.007 * 2 * Math.PI; //0.02 before flyin

  public static final double kTeleDriveMaxSpeedMetersPerSecond = physicalMaxSpeedMPS / 1.5;
  public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 1.5;
  public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 12; //1.5; scott2 was 1.5
  public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 4;

  public static boolean isSlow = false;
  public static boolean algaeArmExtended = false;

  public static boolean isFieldCentric = true;

   // Start - Created from running Sysid and running test on 12-16-23
  //public static final double ksvolts = 0.11307;
  //public static final double kvVoltSecondsPerMeter = 2.7105;
  //public static fin%al double kaVoltSecondsSquarePerMeter = 0.065426;
  //public static final double kPDriveVel = 0.010885;

  ////////////  path planner 12-16-23
  public static final class Swerve {
    //public static final Translation2d flModuleOffset = new Translation2d(0.3175, 0.3175);
    public static final Translation2d flModuleOffset = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
    //public static final Translation2d frModuleOffset = new Translation2d(0.3175, -0.3175);
    //public static final Translation2d blModuleOffset = new Translation2d(-0.3175, 0.3175);
    //public static final Translation2d brModuleOffset = new Translation2d(-0.3175, -0.3175);

    public static final double maxModuleSpeed = 8;// 4.5; // orig 4.5 M/S Max speed for auto
  
  public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
      new PIDConstants(5, 0, 0), // Translation constants   5
      new PIDConstants(3, 0, 0) // Rotation constants 
     // maxModuleSpeed, 
     // flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module) 
     // new ReplanningConfig()
    );
  }

  public static class BrakeConstants {
    public static double BRAKE_ON = 175;   
    public static double BRAKE_OFF = 120;
   }

   
 public static class HookConstants {
  public static float hookMaxClimb = 0.1f;
  public static float hookOpen = -1.6698889f;
}

  public static class Limelight {
    public static final int APRIL_TAG_PIPELINE_INDEX = 0;
    public static final int RETRO_REFLECTIVE_TAPE_PIPELINE_INDEX = 1;

    public static final String limelightName3 = "limelight-three";
    //public static final String limelightName2 = "limelight-two";
    public static final String limelightName3g = "limelight-threeg";

    //public static final double limelightHeightLowRider = 11.5; //inches
    //public static final double limelightHeightLowRiderInMeters = 0.2921;

    public static final double limelight3Height = 0.473075; //meters, 18 5/8
    public static final double limelight3HeightInches = 18.625;
    public static final double limelight3Degree = 16;
    public static final double limelight3CenterOffsetFrontToBack = 0.0254; //meters, 1 inch
    public static final double limelight3CenterOffsetLeftToRight = 0.1016; //meters, 4 inches

    //public static final double limelight2Height = 0.2921; //meters, 11.5 inches
    //public static final double limelight2Degree = -35;
    //public static final double limelight2CenterOffset = 0.42545; //meters, 16.75 inches
  }

  public static class Algae {
    
    public static final double intakeCurrentAlgaeControl  =   10;  
    public static final double outtakeCurrentAlgaeControl  =  -10; 
    
    
    //Algae are move Control motor
    public static final int ALGAE_ARM_CONTROL_MOTOR_ID = 60;

    //Algae Wheel motors
    public static final int RIGHT_ALGAE_CONTROL_MOTOR_ID = 3;
    public static final int LEFT_ALGAE_CONTROL_MOTOR_ID = 4;

    // Algae Arm positions in Rotations
    public static final int ARM_HOME_POSITION = 0; // rotations's
    public static final int ARM_LEVEL_ONE = 6; // rotations's
    public static final int ARM_LEVEL_TWO = 12; // rotations's
    public static final int ARM_LEVEL_THREE = 15; // rotations's 

  }

  public static class Coral {
    public static int CORAL_DIRECTION_LEFT_MOTOR_ID = 6;
    public static int CORAL_DIRECTION_RIGHT_MOTOR_ID = 7;
    public static int CORAL_ARM_MOTOR_ID = 8;

    public static double CORAL_ARM_POSITIONS = 1;

    public static double CORAL_ARM_POSITION_HOME = 3.8; //0.S03 // AKA "PICKUP"
    public static double CORAL_ARM_POSITION_ONE = 4.1; //0.05
    public static double CORAL_ARM_POSITION_TWO = 4.1; //0.1
    public static double CORAL_ARM_POSITION_THREE = 12.5; // usually **13**

  }
  
  public static final double kTrackWidth = Units.inchesToMeters(21.5); //From CAD: 21.5 // distance between left and right wheels
  public static final double kWheelBase = Units.inchesToMeters(27); //From Cad: 26.93 //distance between front and back wheels

  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(kWheelBase / 2, kTrackWidth / 2),     // Front Left wheel position from center of robot
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2),    // Front Right wheel postion
    new Translation2d(-kWheelBase / 2, kTrackWidth / 2),    // Back Left wheel postion
    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)    // Back Right wheel position 
  );
}
