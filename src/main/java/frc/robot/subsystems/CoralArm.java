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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import com.revrobotics.AbsoluteEncoder;

public class CoralArm extends SubsystemBase {
  /** Creates a new CoralArm. */

   private final static SparkMax CoralArm = new SparkMax(Constants.Coral.CORAL_ARM_MOTOR_ID, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  SparkClosedLoopController CoralArmPID = CoralArm.getClosedLoopController();  
 // private static final RelativeEncoder CORAL_ARM_ENCODER = CoralArm.getEncoder();
  private static final AbsoluteEncoder CORAL_ARM_ENCODER_ABSOLUTE = CoralArm.getAbsoluteEncoder();


  public CoralArm() {
    double kMinOutput = -1;
    double kMaxOutput = 1;
   config
    .inverted(false)
    .idleMode(IdleMode.kBrake);
   config.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    // Set MAXMotion parameters
   config.closedLoop.maxMotion
    .maxVelocity(1)
    .maxAcceleration(1)
    .allowedClosedLoopError(0.01);
    config.closedLoop
    .pid(1, 0.0, 0.0)
    .outputRange(kMinOutput, kMaxOutput)
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
   //config.closedLoop
   // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
   // 
      
    //ElevatorDirectionMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  
    CoralArm.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    //CORAL_ARM_ENCODER.setPosition(0);
  }

  public void setCoralArmPosition(double wantedPosition) {
    CoralArmPID.setReference(wantedPosition, SparkMax.ControlType.kPosition);
  } 


    public double getCurrentCoralArmRotation() {
    return CORAL_ARM_ENCODER_ABSOLUTE.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Coral Arm Position", CORAL_ARM_ENCODER.getPosition());
    SmartDashboard.putNumber("Coral Arm Absolute Position", getAbsolutePosition());

  }
      ///// Absolute Encoder position  /////////////
      public static double getAbsolutePosition() {
        return CORAL_ARM_ENCODER_ABSOLUTE.getPosition();
      }
}
