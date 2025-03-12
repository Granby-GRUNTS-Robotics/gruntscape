// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FaceTag;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AprilTagsCommands.FollowTag;
import frc.robot.commands.AutoCommands.AutoPlaceCoralOne;
import frc.robot.commands.AutoCommands.AutoPlaceCoralThree;
import frc.robot.commands.ClimberCommands.JoyClimb;
import frc.robot.commands.CoralCommands.CoralIntake;
import frc.robot.commands.CoralCommands.JoyCoralArm;
import frc.robot.commands.CoralCommands.WholeCoralCommands.CoralPickup;
import frc.robot.commands.CoralCommands.WholeCoralCommands.CoralPositionOne;
import frc.robot.commands.CoralCommands.WholeCoralCommands.CoralPositionThree;
import frc.robot.commands.CoralCommands.WholeCoralCommands.CoralPositionTwo;
import frc.robot.commands.ElevatorCommands.JoyElevatorControl;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeControl;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberClamp;
import frc.robot.subsystems.ClimberWinch;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralControl;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;
import pabeles.concurrency.ConcurrencyOps.Reset;
import frc.robot.subsystems.Brake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
 // public static final LimeLight limeLight3  = new LimeLight(Constants.Limelight.limelightName3);
  public static final LimeLight limeLight3g  = new LimeLight(Constants.Limelight.limelightName3g);
  
  // Xbox controller
  private final static CommandXboxController xboxController = new CommandXboxController(0);
  private static final GenericHID hid = xboxController.getHID();

  // Joystick buttons
  public static final Joystick BUTTON_JOYSTICK = new Joystick(1);
  /*
  swerveSubsystem
  public static JoystickButton IntakeAlgaeButton;
  public static JoystickButton OuttakeAlgaeButton;

  public static JoystickButton ArmLevelOne;
  public static JoystickButton ArmLevelTwo;
  public static JoystickButton ArmLevelThree;
  
  */


// Create joystick buttons
  public static JoystickButton MoveElevator;
  public static JoystickButton MoveCoralArm;
  public static JoystickButton FaceTag;
  public static JoystickButton BrakeClimber;
  public static JoystickButton MoveClimber;
  
  
  public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(limeLight3g); //////// ADD LIMELIGHT3 BACK IN!!!!!

  public static final AlgaeArm algaeArm = new AlgaeArm();
  public static final AlgaeControl algaeControl = new AlgaeControl();
  public static final Climber climber = new Climber();
  public static final CoralControl coralControl = new CoralControl();
  public static final CoralArm coralArm = new CoralArm();
  public static final CoralIntake coralIntakeControl = new CoralIntake(coralControl);
  public static final Elevator elevator = new Elevator();
  public static final Brake brake = new Brake();
  public static final FaceTag faceTag = new FaceTag(swerveSubsystem, limeLight3g);
  public static final JoyElevatorControl elevatorController = new JoyElevatorControl(elevator, BUTTON_JOYSTICK);
  public static final JoyCoralArm coralArmController = new JoyCoralArm(coralArm, BUTTON_JOYSTICK);
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, xboxController, Constants.isFieldCentric));

   BrakeClimber = new JoystickButton(BUTTON_JOYSTICK, 5);
   MoveCoralArm = new JoystickButton(BUTTON_JOYSTICK, 2);

    MoveElevator = new JoystickButton(BUTTON_JOYSTICK, 1);
    MoveClimber = new JoystickButton(BUTTON_JOYSTICK, 3);
    

    CameraServer.startAutomaticCapture(0);


    //Face April Tag Button (Might be spagehhti)

    FaceTag = new JoystickButton(BUTTON_JOYSTICK, 10);

       // Register Named Commands
      NamedCommands.registerCommand("AutoPlaceCoralThree", new AutoPlaceCoralThree(elevator, coralArm,  coralControl));
      NamedCommands.registerCommand("CoralPickup", new CoralPickup(elevator, coralArm,  coralControl));

    // Configure the trigger bindings
    configureBindings();

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
                
              ///////////////// ZERO GYRO \\\\\\\\\\\\\\\\\

    xboxController.start().whileTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro())); // 
    xboxController.start().whileTrue(new InstantCommand(() -> hid.setRumble(GenericHID.RumbleType.kBothRumble, 1)));
    xboxController.start().whileFalse(new InstantCommand(() -> hid.setRumble(GenericHID.RumbleType.kBothRumble, 0)));

              ///////////////// NEW INPUTS: CORAL \\\\\\\\\\\\\\\\\
 
    xboxController.a().toggleOnTrue(new CoralPickup(elevator, coralArm, coralControl));
    xboxController.b().toggleOnTrue(new CoralPositionOne(elevator, coralArm, coralControl));
    xboxController.x().toggleOnTrue(new CoralPositionTwo(elevator, coralArm, coralControl));
    xboxController.y().toggleOnTrue(new CoralPositionThree(elevator, coralArm, coralControl));

    xboxController.rightTrigger().whileTrue(new RunCommand(() -> coralControl.placeCoral(), coralControl));

    xboxController.povUp().whileTrue(new RunCommand(() -> coralControl.manualCoralMovement(-0.1), coralControl));
    xboxController.povUp().whileFalse(new RunCommand(() -> coralControl.manualCoralMovement(0), coralControl));

    xboxController.povDown().whileTrue(new RunCommand(() -> coralControl.manualCoralMovement(0.1), coralControl));
    xboxController.povDown().whileFalse(new RunCommand(() -> coralControl.manualCoralMovement(0), coralControl));

              ///////////////// NEW INPUTS: ALGAE \\\\\\\\\\\\\\\\\
    
    xboxController.leftTrigger().toggleOnTrue(new RunCommand(() -> algaeArm.setAlgaeRotations(Constants.Algae.ARM_LEVEL_THREE), algaeArm));
    xboxController.leftTrigger().toggleOnFalse(new RunCommand(() -> algaeArm.setAlgaeRotations(Constants.Algae.ARM_HOME_POSITION), algaeArm));

    xboxController.leftBumper().whileTrue(new InstantCommand(() -> algaeControl.setSpeeds(Constants.Algae.intakeCurrentAlgaeControl), algaeControl)); // INTAKE
    xboxController.leftBumper().whileFalse(new InstantCommand(() -> algaeControl.setSpeeds(1))); // INTAKE

    xboxController.leftStick().whileTrue(new InstantCommand(() -> algaeControl.setSpeeds(Constants.Algae.outtakeCurrentAlgaeControl), algaeControl)); // OUTTAKE
    xboxController.leftStick().whileFalse(new InstantCommand(() -> algaeControl.setSpeeds(0))); // OUTTAKE

              ///////////////// SLOW MODE TOGGLE \\\\\\\\\\\\\\\\\

    xboxController.button(7).toggleOnTrue(new InstantCommand(() -> swerveSubsystem.slowModeToggle()));
    xboxController.button(7).whileTrue(new InstantCommand(() -> hid.setRumble(GenericHID.RumbleType.kBothRumble, 1)));
    xboxController.button(7).whileFalse(new InstantCommand(() -> hid.setRumble(GenericHID.RumbleType.kBothRumble, 0)));


              ///////////////// JOYSTICK BUTTONS \\\\\\\\\\\\\\\\\
   
    FaceTag.whileTrue(new FollowTag(limeLight3g, swerveSubsystem));

    BrakeClimber.whileTrue(new RunCommand(() -> brake.ActivateBrake()));

    MoveClimber.whileTrue(new JoyClimb(climber, algaeArm, BUTTON_JOYSTICK));

    MoveElevator.whileTrue(new JoyElevatorControl(elevator, BUTTON_JOYSTICK));

    MoveCoralArm.whileTrue(new JoyCoralArm(coralArm, BUTTON_JOYSTICK));




   /*  

    xboxController.leftTrigger().whileFalse(new InstantCommand(() -> hid.setRumble(GenericHID.RumbleType.kBothRumble, 0)));

   xboxController.povDown().toggleOnTrue(new RunCommand(() -> algaeArm.setAlgaeRotations(Constants.Algae.ARM_HOME_POSITION), algaeArm));
   
   xboxController.povDown().toggleOnTrue(new RunCommand(() -> algaeArm.setAlgaeRotations(Constants.Algae.ARM_LEVEL_THREE), algaeArm));

    xboxController.povLeft().toggleOnTrue(new RunCommand(() -> algaeArm.setAlgaeRotations(Constants.Algae.ARM_LEVEL_ONE), algaeArm));

    xboxController.povRight().toggleOnTrue(new RunCommand(() -> algaeArm.setAlgaeRotations(Constants.Algae.ARM_LEVEL_TWO), algaeArm));
   
    xboxController.povUp().toggleOnTrue(new RunCommand(() -> algaeArm.setAlgaeRotations(Constants.Algae.ARM_HOME_POSITION), algaeArm));
    */
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
