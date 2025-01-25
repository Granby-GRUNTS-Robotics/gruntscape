// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeControl extends SubsystemBase {
  private final PIDController AlgaePIDControllerRight;
  private final PIDController AlgaePIDControllerLeft;

  /** Creates a new AlgaeControl. */
  private final static SparkMax AlgaeControlMotorRight = new SparkMax(Constants.OperatorConstants.RIGHT_ALGAE_CONTROL_MOTOR_ID,MotorType.kBrushless);
  private final static SparkMax AlgaeControlMotorLeft = new SparkMax(Constants.OperatorConstants.LEFT_ALGAE_CONTROL_MOTOR_ID,MotorType.kBrushless);

  private static final RelativeEncoder RIGHT_ALGAE_ENCODER = AlgaeControlMotorRight.getEncoder();
  private static final RelativeEncoder LEFT_ALGAE_ENCODER = AlgaeControlMotorLeft.getEncoder();
  
  private static double intakeSpeedAlgae;
  private static double placementSpeedAlgae;


 // private static final PIDController RIGHT_ALGAE_CONTROLLER = AlgaePIDController;

  public AlgaeControl() { 
    
    //AlgaeControlMotorRight.setSmartCurrentLimit(40);

     AlgaePIDControllerRight = new PIDController(0.015, 0, 0);
     AlgaePIDControllerLeft = new PIDController(0.015, 0, 0);
     
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
