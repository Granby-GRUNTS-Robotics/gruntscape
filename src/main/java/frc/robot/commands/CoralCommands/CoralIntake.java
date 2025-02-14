// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralControl;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntake extends Command {

  CoralControl intake;
  double wantedPosition;

  public CoralIntake(CoralControl intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

 intake.setCoralControlRotations(wantedPosition);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //intake.setSpeeds(0);
    intake.setCoralControlRotations(wantedPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intake.getLimitSwitch(); //false; //return intake.getLimitSwitch(); //WHEN NOTE IS FULLY PAST THE INTAKE
  }
}
