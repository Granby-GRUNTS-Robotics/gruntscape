// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;



public class SetColor extends Command {
  /** Creates a new SetColor. */

  private double red;
  private double green;
  private double blue;

  private boolean check;

  public SetColor(double red, double green, double blue) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.red = red;
    this.green = green;
    this.blue = blue;
    check = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LEDs.setLEDColor(red, green, blue);
    check = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return check;
  }
}
