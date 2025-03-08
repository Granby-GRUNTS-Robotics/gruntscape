// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralControl;
import edu.wpi.first.wpilibj.Joystick;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestCoralControl extends Command {
  Joystick joystick;
  CoralControl coralControl;

  /** Creates a new TestCoralControl. */
  public TestCoralControl(CoralControl coralControl, Joystick joystick) {
    this.joystick = joystick;
    this.coralControl = coralControl;
    
    addRequirements(coralControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    coralControl.testCoralPosition(joystick.getY() * -0.25); //Controls are inverted

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    coralControl.testCoralPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
