// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArm extends SubsystemBase {
  /** Creates a new AlgaeArm. */

  private final static SparkMax AlgaeArmRotationMotor = new SparkMax(Constants.OperatorConstants.ALGAE_ARM_CONTROL_MOTOR_ID, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  SparkClosedLoopController algaeArmPid = AlgaeArmRotationMotor.getClosedLoopController();

  private static final RelativeEncoder ALGAE_ARM_POSITION_ENCODER = AlgaeArmRotationMotor.getEncoder();

  public AlgaeArm() {
    
    config
    .inverted(true)
    .idleMode(IdleMode.kBrake);
   config.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
   config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.02, 0.0, 0.0);
      
    //ElevatorDirectionMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  
    AlgaeArmRotationMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    ALGAE_ARM_POSITION_ENCODER.setPosition(0);

  }

  public void setAlgaeRotations(double wantedRotations) {
    algaeArmPid.setReference(wantedRotations, SparkMax.ControlType.kPosition);
  } 


    public double getCurrentArmRotation() {
    return ALGAE_ARM_POSITION_ENCODER.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Arm Position", ALGAE_ARM_POSITION_ENCODER.getPosition());
  }
}
