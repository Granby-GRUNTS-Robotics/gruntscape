// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralControl;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PlaceCoral extends Command {

  CoralControl coralControl;
  
  /** Creates a new BackupCoralThree. */
  public PlaceCoral(CoralControl coralControl) {
    this.coralControl = coralControl;

    addRequirements(coralControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(coralControl.SecondBeamBroken() || coralControl.FirstBeamBroken()) {

      coralControl.setCoralVelocity(0.5);

    }
    else {

      coralControl.setCoralVelocity(0);
  
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(coralControl.SecondBeamBroken() || coralControl.FirstBeamBroken()) {

      coralControl.setCoralVelocity(0.6);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralControl.setCoralVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !coralControl.SecondBeamBroken() && !coralControl.FirstBeamBroken();
  }
}