// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeControl extends SubsystemBase {

  /** Creates a new AlgaeControl. */
  private final static SparkMax AlgaeControlMotorRight = new SparkMax(Constants.Algae.RIGHT_ALGAE_CONTROL_MOTOR_ID,MotorType.kBrushless);
  private final static SparkMax AlgaeControlMotorLeft = new SparkMax(Constants.Algae.LEFT_ALGAE_CONTROL_MOTOR_ID,MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  SparkClosedLoopController algaeRightPid = AlgaeControlMotorRight.getClosedLoopController();
  SparkClosedLoopController algaeLeftPid = AlgaeControlMotorLeft.getClosedLoopController();


  private static double intakeSpeedAlgae;
  private static double placementSpeedAlgae;


 // private static final PIDController RIGHT_ALGAE_CONTROLLER = AlgaePIDController;

 public AlgaeControl() { 
  config
  //.inverted(false)
  .idleMode(IdleMode.kBrake);
 config.encoder
  .positionConversionFactor(1)
  .velocityConversionFactor(1);
 config.closedLoop
  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
  .pid(0.02, 0.0, 0.0);
    
  //ElevatorDirectionMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  
  AlgaeControlMotorRight.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  AlgaeControlMotorLeft.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  

  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Intake Amps", AlgaeControlMotorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Right Intake Amps", AlgaeControlMotorRight.getOutputCurrent());

    SmartDashboard.putNumber("Left Intake Temp", AlgaeControlMotorLeft.getMotorTemperature());
    SmartDashboard.putNumber("Right Intake Temp", AlgaeControlMotorRight.getMotorTemperature());


  }

  public void setSpeeds(double amperage) {
    //AlgaeControlMotorRight.set(speed * -1);
    //AlgaeControlMotorLeft.set(speed);

    algaeRightPid.setReference(amperage, ControlType.kCurrent);
    algaeLeftPid.setReference(amperage * -1, ControlType.kCurrent);
  }
}
