// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import java.time.chrono.ThaiBuddhistChronology;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Hook. */
private static final SparkMax LEAD_HOOK_MOTOR = new SparkMax(Constants.OperatorConstants.FIRST_HOOK_MOTOR_ID,MotorType.kBrushless);
private static final SparkMax FOLLOW_HOOK_MOTOR = new SparkMax(Constants.OperatorConstants.SECOND_HOOK_MOTOR_ID,MotorType.kBrushless);

private static final SparkClosedLoopController LEAD_HOOK_PID = LEAD_HOOK_MOTOR.getClosedLoopController();

static final DigitalInput AllowClimb = new DigitalInput(0);

SparkMaxConfig config = new SparkMaxConfig();

SparkMaxConfig FollowerConfig = new SparkMaxConfig();



  public Climber() {
    

   config
    .inverted(true)
    .idleMode(IdleMode.kBrake);
      
    // FOLLOWER MOTOR
    FollowerConfig
    .apply(config)
    .follow(LEAD_HOOK_MOTOR);


    LEAD_HOOK_MOTOR.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    FOLLOW_HOOK_MOTOR.configure(FollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  
  }

  @Override
  public void periodic() {

  }

  public void setClimberSpeed(double percent)
  {
    if (percent > 0.05)  // This should be trying to climb
    {
      LEAD_HOOK_PID.setReference(percent, SparkMax.ControlType.kDutyCycle);
    Brake.setBrakePosition(Constants.BrakeConstants.BRAKE_ON);
    }
   else if (percent < -0.05)
    {
    Brake.setBrakePosition(Constants.BrakeConstants.BRAKE_OFF);
    LEAD_HOOK_PID.setReference(percent, SparkMax.ControlType.kDutyCycle);
    }
    else{LEAD_HOOK_PID.setReference(0, SparkMax.ControlType.kDutyCycle);}

   }


   public void manualClimber(double percent) {
    setClimberSpeed(percent);
   }

 /*   
   public boolean AllowClimb() {
    return !AllowClimb.get();
   } 

 */



}