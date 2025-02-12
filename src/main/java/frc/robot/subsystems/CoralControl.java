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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralControl extends SubsystemBase {
  /** Creates a new CoralControl. */
    private final static SparkMax CoralControlLeft = new SparkMax(Constants.Coral.CORAL_DIRECTION_LEFT_MOTOR_ID, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  SparkClosedLoopController CoralControlLeftPID = CoralControlLeft.getClosedLoopController();

  private final static SparkMax CoralControlRight = new SparkMax(Constants.Coral.CORAL_DIRECTION_RIGHT_MOTOR_ID, MotorType.kBrushless);
  SparkClosedLoopController CoralControlRightPID = CoralControlRight.getClosedLoopController();

  private static final RelativeEncoder CORAL_CONTROL_LEFT_ENCODER = CoralControlLeft.getEncoder();
  private static final RelativeEncoder CORAL_CONTROL_RIGHT_ENCODER = CoralControlRight.getEncoder();

  public CoralControl() {
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
    .maxVelocity(0.25)
    .maxAcceleration(1)
    .allowedClosedLoopError(0.01);
    config.closedLoop
    .pid(0.02, 0.0, 0.0)
    .outputRange(kMinOutput, kMaxOutput);
   //config.closedLoop
   // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
   // 
      
    //ElevatorDirectionMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  
    CoralControlLeft.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    CoralControlRight.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    CORAL_CONTROL_LEFT_ENCODER.setPosition(0);
    CORAL_CONTROL_RIGHT_ENCODER.setPosition(0);

  }

  public void setCoralControlRotations(double wantedPosition) {
    CORAL_CONTROL_LEFT_ENCODER.setPosition(0);
    CORAL_CONTROL_RIGHT_ENCODER.setPosition(0);

    //ELEVATOR_DIRECTION_ENCODER.setPosition(pos);
    CoralControlRightPID.setReference(wantedPosition * -1, SparkMax.ControlType.kPosition);
    CoralControlLeftPID.setReference(wantedPosition, SparkMax.ControlType.kPosition);

  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
