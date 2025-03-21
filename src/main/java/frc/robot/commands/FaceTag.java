// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Limelight;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class FaceTag extends Command {
  /** Creates a new FaceTag. */
  LimeLight limelight;
  SwerveSubsystem swerve;
  double dx;
  double dy;
  double distanceFromTag;
  double xvelocity;
  double yvelocity;
  double trueDistance;
  double xcomponent;
  double ycomponent;
  int inverterAngle;
  int inverterDirection;
  ChassisSpeeds chassisSpeeds;
  
  public FaceTag(SwerveSubsystem swerve, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerve = swerve;

    addRequirements(limelight, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight-threeg").getEntry("pipeline").setInteger(2);

    
    if (limelight.getTargets())
    {

    dx = (NetworkTableInstance.getDefault().getTable("limelight-threeg").getEntry("tx").getDouble(0)) -6.08 ; //- 5.25;
    //dy = (NetworkTableInstance.getDefault().getTable("limelight-three").getEntry("ty").getDouble(0));

    //distanceFromTag = (0.7874) / (Math.tan(Math.toRadians(8+dy)));

    if (dx < 0)
    {
      inverterAngle = -1;
    }
    else
    {
      inverterAngle = 1;
    }

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -dx*.1, (swerve.getRotation2d().plus(new Rotation2d(dx * 0.8))));
    // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 1 * -inverterAngle, (swerve.getRotation2d().plus(new Rotation2d(dx * 0.9))));
   
    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().getTable("limelight-threeg").getEntry("pipeline").setInteger(0);
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(dx)<.7);
  }
}