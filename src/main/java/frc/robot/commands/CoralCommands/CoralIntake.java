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
  boolean passedLimitSwitch = false;
  boolean force;
  Elevator elevator;

  public CoralIntake(CoralControl intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.setCoralVelocity(10);
   
   // intake.setCoralWheelPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // if(!passedLimitSwitch) {
      
    //  intake.setCoralVelocity(10);

    //}
   /*  else if (intake.touchingBeamBreak() && force == true) {
      intake.setCoralVelocity(30);
      
     }*/
    /* else {

      intake.moveCoralForward(10);

    }*/

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


    intake.setCoralVelocity(0);
 //  intake.setCoralWheelPosition(0);
   // intake.moveCoralForward(2);
  }

  // Returns true when the command should end.

  @Override
  public boolean isFinished() {

    return intake.touchingBeamBreak();
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
