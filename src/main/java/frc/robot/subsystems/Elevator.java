// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.PIDController;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  //private final PIDController ElevatorDirection;
  private final static SparkMax ElevatorDirectionMotor = new SparkMax(Constants.OperatorConstants.ELEVATOR_DIRECTION_MOTOR_ID, MotorType.kBrushless);
  //private static final SparkClosedLoopController LEAD_HOOK_CONTROLLER = ElevatorDirectionMotor.getClosedLoopController();
  SparkMaxConfig config = new SparkMaxConfig();
  SparkClosedLoopController elevatorPid = ElevatorDirectionMotor.getClosedLoopController();

  private static final RelativeEncoder ELEVATOR_DIRECTION_ENCODER = ElevatorDirectionMotor.getEncoder();

 
   

  public Elevator() {
   
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
    ElevatorDirectionMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    ELEVATOR_DIRECTION_ENCODER.setPosition(0); // keep for testing

  }

  public double getCurrentElevatorPosition() {
    return ELEVATOR_DIRECTION_ENCODER.getPosition();
  }

  public void setElevatorPosition(double wantedPosition) {
    //ELEVATOR_DIRECTION_ENCODER.setPosition(pos);
    elevatorPid.setReference(wantedPosition, SparkMax.ControlType.kPosition);
  } 


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", ELEVATOR_DIRECTION_ENCODER.getPosition());
  }
}

