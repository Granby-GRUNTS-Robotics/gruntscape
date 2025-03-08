// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralCommands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ElevatorWaitCommand;
import frc.robot.subsystems.CoralControl;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntake extends Command {

  CoralControl intake;


  public CoralIntake(CoralControl intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

   // intake.setCoralVelocity(0.5);
   
    intake.setCoralWheelPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Phase 2
   if(intake.FirstBeamBroken() && intake.SecondBeamBroken()) {
      
      intake.setCoralVelocity(0.05);

    }
    // Phase 3
     else if (!intake.FirstBeamBroken() && intake.SecondBeamBroken()) {
      intake.setCoralVelocity(0);
      
     }
     // Phase 1
     else {

      intake.setCoralVelocity(1);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


    intake.setCoralVelocity(0.0);
  }

  // Returns true when the command should end.

  @Override
  public boolean isFinished() {

    return !intake.FirstBeamBroken() && intake.SecondBeamBroken();
    // if beam break not hit yet, continue moving forward
    /*if(!passedLimitSwitch && !intake.touchingBeamBreak()) {
      return true;
    }

    // if beam break detected, change bool to false, and continue in end.
    else if (!passedLimitSwitch && intake.touchingBeamBreak()) {
      passedLimitSwitch = true;

      return true;
    }

     else if (intake.touchingBeamBreak() && force == false && intake.coralWheelPosition() <= 10) {
        force = true;
        intake.setCoralWheelPosition(0);

        return false;
    } 
    return true; */
  }
}
