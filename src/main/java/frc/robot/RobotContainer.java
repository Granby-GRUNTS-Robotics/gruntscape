// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeControl;
import frc.robot.subsystems.ClimberClamp;
import frc.robot.subsystems.ClimberWinch;
import frc.robot.subsystems.CoralControl;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final LimeLight limeLight3  = new LimeLight(Constants.Limelight.limelightName3);
  public static final LimeLight limeLight3g  = new LimeLight(Constants.Limelight.limelightName3g);
  
  // Xbox controller
  private final static CommandXboxController xboxController = new CommandXboxController(0);
  private static final GenericHID hid = xboxController.getHID();

  // Joystick buttons
  public static final Joystick BUTTON_JOYSTICK = new Joystick(1);
  public static JoystickButton IntakeAlgaeButton;
  public static JoystickButton OuttakeAlgaeButton;

  public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(limeLight3, limeLight3g);
  public static final AlgaeArm algaeArm = new AlgaeArm();
  public static final AlgaeControl algaeControl = new AlgaeControl();
  public static final ClimberClamp climberClamp = new ClimberClamp();
  public static final ClimberWinch climberWinch = new ClimberWinch();
  public static final CoralControl coralControl = new CoralControl();
  public static final Elevator elevator = new Elevator();
  
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, xboxController, Constants.isFieldCentric));
    
    // Define Joystick buttons
    IntakeAlgaeButton = new JoystickButton(BUTTON_JOYSTICK,2);
    OuttakeAlgaeButton = new JoystickButton(BUTTON_JOYSTICK,8);
    // Configure the trigger bindings
    configureBindings();

     // CameraServer.ADDLIMELIGHTSTREAMTOSHUFFLEBOARD(something, something);

      // This Should populate with all Autos (*.auto files) in deploy/pathplanner/autos directory. No need to populate here.
      autoChooser = AutoBuilder.buildAutoChooser();
      // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  
  public static final class Buttons
  {
    public static double getWithDeadZone(double value, double threshold)
    {
      if (Math.abs(value) < threshold) 
      {
          value = 0;
      }
      else
      {
          value = (Math.abs(value) - threshold) * (1.0 / (1 - threshold)) * Math.signum(value);
      }

      return value;
   }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    xboxController.start().whileTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro())); // 
    xboxController.start().whileTrue(new InstantCommand(() -> hid.setRumble(GenericHID.RumbleType.kBothRumble, 1)));
    xboxController.start().whileFalse(new InstantCommand(() -> hid.setRumble(GenericHID.RumbleType.kBothRumble, 0)));
    xboxController.leftStick().toggleOnTrue(new InstantCommand(() -> swerveSubsystem.slowModeToggle()));


    //Button Joystick Commands
    IntakeAlgaeButton.toggleOnTrue(new InstantCommand(() -> algaeControl.setSpeeds(0.1)));
    OuttakeAlgaeButton.whileTrue(new InstantCommand(() -> algaeControl.setSpeeds(0.1)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
