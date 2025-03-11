// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralControl;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BackupCoralThree extends Command {

  CoralControl coralControl;
  
  /** Creates a new BackupCoralThree. */
  public BackupCoralThree(CoralControl coralControl) {
    this.coralControl = coralControl;

    addRequirements(coralControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!coralControl.FirstBeamBroken() && coralControl.SecondBeamBroken()) { // FIRST BEAM NOT BROKEN AND SECOND BROKEN
      coralControl.setCoralVelocity(-0.4);
    }
    else if (coralControl.FirstBeamBroken() && coralControl.SecondBeamBroken()) {  // FIRST BEAM BROKEN AND SECOND BROKEN
      coralControl.setCoralVelocity(-0.2);
    }
    else if (coralControl.FirstBeamBroken() && !coralControl.SecondBeamBroken()) { // FIRST BEAM BROKEN AND SECOND NOT BROKEN
      coralControl.setCoralVelocity(0);
    }
    else {
      coralControl.setCoralVelocity(0.1);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!coralControl.FirstBeamBroken() && coralControl.SecondBeamBroken()) {
      coralControl.setCoralVelocity(-0.4);
    }
    else if (coralControl.FirstBeamBroken() && coralControl.SecondBeamBroken()) {
      coralControl.setCoralVelocity(-0.2);
    }
    else if (coralControl.FirstBeamBroken() && !coralControl.SecondBeamBroken()) {
      coralControl.setCoralVelocity(0);
    }
    else {
      coralControl.setCoralVelocity(1);
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
    return coralControl.FirstBeamBroken() && !coralControl.SecondBeamBroken();
  }
}
