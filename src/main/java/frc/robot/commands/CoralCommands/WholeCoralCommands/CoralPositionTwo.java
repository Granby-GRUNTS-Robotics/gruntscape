// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralCommands.WholeCoralCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.Constants;
import frc.robot.commands.CoralCommands.CoralMoveArm;
import frc.robot.commands.ElevatorCommands.ElevatorLevels;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralControl;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralPositionTwo extends SequentialCommandGroup {
  /** Creates a new CoralPositionOne. */
  public CoralPositionTwo(Elevator elevator, CoralArm coralArm, CoralControl coralControl) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        
        new ElevatorLevels(elevator, Constants.OperatorConstants.LEVEL_TWO_HEIGHT),
         
        new CoralMoveArm(coralArm, Constants.Coral.CORAL_ARM_POSITION_TWO)
      
      )
      
     // new WaitUntilCommand(() -> coralControl.CoralPassed()),

       // new CoralPickup(elevator, coralArm, coralControl)
      

    );
  }
}
