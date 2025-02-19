// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTagsCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class FollowTag extends Command {
  /** Creates a new FollowTag. */
  LimeLight limelight;
  SwerveSubsystem swerve;
  double dx;
  double dy;
  double goal;
  double distanceFromTag;
  double xcomponent;
  double ycomponent;
  int inverterAngle;
  ChassisSpeeds chassisSpeeds;

  public FollowTag(LimeLight limelight, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    addRequirements(limelight);
    addRequirements(swerve);

    goal = 2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dx = (NetworkTableInstance.getDefault().getTable("limelight-threeg").getEntry("tx").getDouble(0));
    dy = (NetworkTableInstance.getDefault().getTable("limelight-threeg").getEntry("ty").getDouble(0));

    distanceFromTag = (0.7874) / (Math.tan(Math.toRadians(8+dy)));

    SmartDashboard.putNumber("limelight distance", distanceFromTag);

    if (!(Math.abs(distanceFromTag - goal) < 0.1))
    {

    if (distanceFromTag - goal < 0)
    {
      inverterAngle = -1;
    }
    else
    {
      inverterAngle = 1;
    }

    xcomponent = distanceFromTag;
    ycomponent = Math.tan(dx) * distanceFromTag;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xcomponent / 3, ycomponent / 3, 0, swerve.getRotation2d());
    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);
  }
  else
  {
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, swerve.getRotation2d());
    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}