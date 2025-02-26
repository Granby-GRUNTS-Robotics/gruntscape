// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralMoveArm extends Command {
  /** Creates a new CoralPlace. */

  CoralArm coralArm;
  double wantedPosition;
  public CoralMoveArm(CoralArm coralArm, double wantedPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coralArm = coralArm;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralArm.setCoralArmPosition(wantedPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(CoralArm.getCurrentCoralArmRotation() - Constants.Coral.CORAL_ARM_POSITION_ONE) < 1);
  }
}
