// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */

public class SwerveModule {

  private final SparkMax speedMotor;
  private final SparkMax directionMotor;

  private final RelativeEncoder speedEncoder;
  private final RelativeEncoder directionEncoder;
  private final PIDController turningPidController;
    
  

  private final CANCoder cancoderDirectionEncoder;  // CANCoder is an absolute encoder

  private final boolean encoderReversed;
  private final double encoderOffsetRad;

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
                      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed)
  {
    
    encoderOffsetRad = absoluteEncoderOffset;
    encoderReversed = absoluteEncoderReversed;
    cancoderDirectionEncoder = new CANCoder(absoluteEncoderId);


    speedMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    SparkMaxConfig speedMotorConfig = new SparkMaxConfig();
    
    directionMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
    SparkMaxConfig directionMotorConfig = new SparkMaxConfig();


speedMotorConfig
    .inverted(driveMotorReversed)
    .idleMode(IdleMode.kBrake);
speedMotorConfig.encoder
    .positionConversionFactor(Constants.speedEncoderRotationToMeter)
    .velocityConversionFactor(Constants.speedEncoderRPMToMPS);
speedMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
    
    speedMotor.configure(speedMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  directionMotorConfig
    .inverted(turningMotorReversed)
    .idleMode(IdleMode.kBrake);
  directionMotorConfig.encoder
    .positionConversionFactor(Constants.directionEncoderRotationToRadian)
    .velocityConversionFactor(Constants.directionEncoderRPMToRadsPS);
  directionMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(Constants.kPTurning, 0, 0);
    
    directionMotor.configure(directionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    /*speedEncoder = speedMotor.getEncoder();
    directionEncoder = directionMotor.getEncoder();

    // Change neo drive motor encoder conversion factors
    speedEncoder.setPositionConversionFactor(Constants.speedEncoderRotationToMeter);
    speedEncoder.setVelocityConversionFactor(Constants.speedEncoderRPMToMPS);

    // Change conversion factors for neo turning encoders - Should be radians
    directionEncoder.setPositionConversionFactor(Constants.directionEncoderRotationToRadian);
    directionEncoder.setVelocityConversionFactor(Constants.directionEncoderRPMToRadsPS);*/
    speedEncoder = speedMotor.getEncoder();
    directionEncoder = directionMotor.getEncoder();
    turningPidController = new PIDController(Constants.kPTurning, 0, 0);
    //turningPidController = new PIDController(Constants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
    //resetToAbsolute();  //Todd

    stop(); //Todd - seeing if this sets wheels straight at start up 8-22-23
  }

  public double getSpeedPosition()
  {
    return speedEncoder.getPosition();
  }

  public double getDirectionPosition()
  {
    return directionEncoder.getPosition();//cancoderDirectionEncoder.getAbsolutePosition(); //directionEncoder.getPosition();
  }

  public double getSpeedVelocity()
  {
    return speedEncoder.getVelocity();
  }

  public double getDirectionVelocity()
  {
    return directionEncoder.getVelocity();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      //speedEncoder.getPosition(), getAngle());
      //speedEncoder.getPosition(), new Rotation2d(getDirectionPosition()));
      getSpeedPosition(), new Rotation2d(getDirectionPosition()));
  }

  public double getAbsoluteEncoderRad()
  {
    double angle = cancoderDirectionEncoder.getAbsolutePosition(); 
    //double angle = cancoderDirectionEncoder.getBusVoltage()/ RobotController.getVoltage5V(); 
    angle = Math.toRadians(angle);
    angle -= encoderOffsetRad;
    return angle * (encoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders()
  {
    speedEncoder.setPosition(0);
    directionEncoder.setPosition(getAbsoluteEncoderRad());
    //directionEncoder.setPosition(getAbsoluteEncoderRad() - encoderOffsetRad);  //Todd: To reset I think we need the offset not just current position
    //directionEncoder.setPosition(cancoderDirectionEncoder.getAbsolutePosition() - endocderJustAngleOffset);  //Todd: To reset I think we need the offset not just current position
  }

  public SwerveModuleState getState()
  {
    return new SwerveModuleState(getSpeedVelocity(), new Rotation2d(getDirectionPosition()));
    //return new SwerveModuleState(getSpeedVelocity(), getAngle());   //Todd
  }

  public void setDesiredState(SwerveModuleState state)
  {
    SmartDashboard.putNumber("speed", state.speedMetersPerSecond);
    if (Math.abs(state.speedMetersPerSecond) < 0.001)
    {
    //   stop();
    //   return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);

    // Scale motor down using robot max speed in Meters per second
    speedMotor.set(state.speedMetersPerSecond / Constants.physicalMaxSpeedMPS);

    directionMotor.set(turningPidController.calculate(getDirectionPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + (cancoderDirectionEncoder.getDeviceID() / 10 ) + "] state", state.toString());
  }

  // Todd
  public double getAbsoluteEncoderAngle()
  {
     double angleOnly = cancoderDirectionEncoder.getAbsolutePosition();
     return angleOnly;
  }
 
  public void stop()
  {
    speedMotor.set(0);
    directionMotor.set(0);
  }
}