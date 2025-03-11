// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.GenericHID;   //TCXbox
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {
  /** Creates a new SwerveJoystickCmd. */
  private final SwerveSubsystem swerveSubsystem;
  private final boolean fieldOrientedFunction;
  ////XboxController xbox;  //TCXbox
  CommandXboxController xbox;  //TCXbox

  double maxSpeedMPS;
  double maxAngularRPS;
  SlewRateLimiter xLimiter, yLimiter, turnLimiter;

  double red;
  double green;
  double blue;

  static boolean rev;

  //public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, XboxController xboxController,    //TCXbox
  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, CommandXboxController xboxController,   //TCXbox
  boolean fieldOrientedFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
      
    this.swerveSubsystem = swerveSubsystem;
    this.fieldOrientedFunction = Constants.isFieldCentric;
    this.xbox = xboxController;
    this.xLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond * .6);
    this.turnLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    maxSpeedMPS = Constants.kTeleDriveMaxSpeedMetersPerSecond;
    maxAngularRPS = Constants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    red = 0.0;
    green = 0.5;
    blue = 0.5;
    rev = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ySpeed = RobotContainer.Buttons.getWithDeadZone(-xbox.getLeftX(), 0.1);
    double xSpeed = RobotContainer.Buttons.getWithDeadZone(-xbox.getLeftY(), 0.1);
    double directionSpeed = RobotContainer.Buttons.getWithDeadZone(-xbox.getRightX(), 0.1); // todd 8-22-23 * maxAngularRPS;

        if (Math.abs(ySpeed) > .1 || Math.abs(xSpeed) > .1)  // If joystick is being used set this boolean to true so it will stop certain commands from running
        {
          rev = true;
        }
        else{
          rev = false;
        }

    //                                1         3.33
    xSpeed = xLimiter.calculate(xSpeed) * maxSpeedMPS;
    ySpeed = yLimiter.calculate(ySpeed)  * maxSpeedMPS;
    directionSpeed = turnLimiter.calculate(directionSpeed) * maxAngularRPS; // Todd 8-22-23
    
    //if (Math.abs(swerveSubsystem.getZSpeed()) > .1)
    if (Math.abs(xSpeed) + Math.abs(ySpeed) > .01 )
    directionSpeed = (directionSpeed * 2 - swerveSubsystem.getZSpeed());

    if (Constants.isSlow || Elevator.getCurrentElevatorPosition() >= Constants.OperatorConstants.LEVEL_TWO_HEIGHT)
    {
      xSpeed /= 2.5;
      ySpeed /= 2.5;
      directionSpeed /= 2.5;
      red = 0.0;
      green = 0.7;
      blue = 0.0;
    }
    else
    {
      xSpeed *= 1.5; //2;  // trying to speed up driving so I added this
      ySpeed *= 1.5; //*= 2;  // trying to speed up driving so I added this
      red = 0.7;
      green = 0.0;
      blue = 0.0;
    }

    ChassisSpeeds chassisSpeeds;

    SmartDashboard.putBoolean("Field Centric Enabled", Constants.isFieldCentric);
     SmartDashboard.putBoolean("Slow Mode Enabled", Constants.isSlow);

    if (Constants.isFieldCentric)
    {
     chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, directionSpeed, swerveSubsystem.getRotation2d()); 
 
      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      SmartDashboard.putNumber("directionSpeed", directionSpeed);
    }
    else
    {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, directionSpeed);
    }

   //// ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, -1);  // .02 Testing out discretize

   SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);  // Testing out discretize
   //SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);  // Testing out discretize 

    swerveSubsystem.setModuleStates(moduleStates);

    //LEDs.setLEDColor(Math.abs(xbox.getLeftY()), Math.abs(xbox.getRightX()), Math.abs(xbox.getLeftX()) / 1.5);
    //LEDs.setLEDColor(red, green, blue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}