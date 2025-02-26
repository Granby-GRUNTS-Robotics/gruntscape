// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorWaitCommand extends Command {
  /** Creates a new IntakeWaitCommand. */
  Elevator intake;
  double time;
  Timer timer;
  public ElevatorWaitCommand(Elevator intake, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start();
    intake.setElevatorPosition(Elevator.getCurrentElevatorPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setElevatorPosition(Elevator.getCurrentElevatorPosition());
    System.out.println("********************************* IntakeWaitCommand Complete *********************************");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
