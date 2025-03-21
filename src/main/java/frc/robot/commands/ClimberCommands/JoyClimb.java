// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SwerveSubsystem;

public class JoyClimb extends Command {
  /** Creates a new JoyClimb. */


  Climber climber;
  Joystick joystick;
  AlgaeArm algaeArm;
  SwerveSubsystem swerveSubsystem;
  public JoyClimb(SwerveSubsystem swerveSubsystem, Climber climber, AlgaeArm algaeArm, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.climber = climber;
    this.algaeArm = algaeArm;
    this.joystick = joystick;


    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
      Constants.isSlow = true;

      algaeArm.setAlgaeRotations(Constants.Algae.ARM_LEVEL_THREE);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ///////////////////// THIS MIGHT HAVE FRIED THE SERVO
   // if(climber.AllowClimb()) {
      
      climber.setClimberSpeed(joystick.getY() * 1);

   /*   }
    else {

      climber.setClimberSpeed(0);

    } */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
    climber.setClimberSpeed(0);

    Constants.isSlow = false;
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
