// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Coral;

public class CoralControl extends SubsystemBase {
  
  /** Creates a new CoralControl. */

  private final static SparkMax CoralControlRight = new SparkMax(Constants.Coral.CORAL_DIRECTION_RIGHT_MOTOR_ID, MotorType.kBrushless);
  SparkClosedLoopController CoralControlRightPID = CoralControlRight.getClosedLoopController();
    
  private final static SparkMax CoralControlLeft = new SparkMax(Constants.Coral.CORAL_DIRECTION_LEFT_MOTOR_ID, MotorType.kBrushless);
  SparkClosedLoopController CoralControlLeftPID = CoralControlLeft.getClosedLoopController();


  SparkMaxConfig coralright = new SparkMaxConfig();
  SparkMaxConfig coralleft = new SparkMaxConfig();


  private static final RelativeEncoder CORAL_CONTROL_LEFT_ENCODER = CoralControlLeft.getEncoder();
  private static final RelativeEncoder CORAL_CONTROL_RIGHT_ENCODER = CoralControlRight.getEncoder();
  
   static final DigitalInput FirstBeam = new DigitalInput(2);

  private static final DigitalInput SecondBeam = new DigitalInput(9);
  

  public CoralControl() {

    double kMinOutput = -1;
    double kMaxOutput = 1;
    coralright
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    coralright.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    // Set MAXMotion parameters
    coralright.closedLoop.maxMotion
    .maxVelocity(0.25)
    .maxAcceleration(1)
    .allowedClosedLoopError(0.01);
    coralright.closedLoop
    .pid(0.02, 0.0, 0.0)
    .outputRange(kMinOutput, kMaxOutput);

    // Coral Left
    coralleft
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    coralleft.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

      // Set MAXMotion parameters
    coralleft.closedLoop.maxMotion
        .maxVelocity(0.25)
        .maxAcceleration(1)
       .allowedClosedLoopError(0.01);
    coralleft.closedLoop
        .pid(0.02, 0.0, 0.0)
        .outputRange(kMinOutput, kMaxOutput);
    

    CoralControlRight.configure(coralright, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    CoralControlLeft.configure(coralleft, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setCoralVelocity(double position) {

    setCoralWheelPosition(0);

    CoralControlLeftPID.setReference(position, SparkMax.ControlType.kMAXMotionPositionControl);

    CoralControlRightPID.setReference(position, SparkMax.ControlType.kMAXMotionPositionControl);

  } 

  /*public void stopCoralVelocity() {
    CoralControlRightPID.setReference(0, SparkMax.ControlType.kMAXMotionVelocityControl);
    CoralControlLeftPID.setReference(0, SparkMax.ControlType.kMAXMotionVelocityControl);

  }*/



    public double coralWheelPosition() {
      
      return CORAL_CONTROL_LEFT_ENCODER.getPosition();
    }

    public void setCoralWheelPosition(double position) {

       CORAL_CONTROL_RIGHT_ENCODER.setPosition(position);
       CORAL_CONTROL_LEFT_ENCODER.setPosition(position);

    }


   public void placeCoral() {
    if(SecondBeamBroken() || FirstBeamBroken()) {

     // CORAL_CONTROL_LEFT_ENCODER.setPosition(2);
      //CORAL_CONTROL_RIGHT_ENCODER.setPosition(2);  

      setCoralVelocity(10);

    }
    else {

      setCoralVelocity(0);
  
    }
  } 

  public void manualCoralMovement(double wantedRotations) {
    
      CoralControlRightPID.setReference(wantedRotations, SparkMax.ControlType.kPosition);
      CoralControlLeftPID.setReference(wantedRotations, SparkMax.ControlType.kPosition);
  }


/*   public void testCoralPosition(double wantedVelocity) {
    CoralControlLeftPID.setReference(wantedVelocity, SparkMax.ControlType.kDutyCycle);

    CoralControlRightPID.setReference(wantedVelocity, SparkMax.ControlType.kDutyCycle);
} */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("1st Beam Broken", FirstBeamBroken());
    SmartDashboard.putBoolean("2nd Beam Broken", SecondBeamBroken());
    SmartDashboard.putNumber("Coral Integer Position", CORAL_CONTROL_LEFT_ENCODER.getPosition());
    SmartDashboard.putNumber("Coral Velocity", CORAL_CONTROL_LEFT_ENCODER.getVelocity());
  }


  public boolean FirstBeamBroken()
  {
    return !FirstBeam.get();
  }
  public boolean SecondBeamBroken()
  {
    return !SecondBeam.get();
  }
}