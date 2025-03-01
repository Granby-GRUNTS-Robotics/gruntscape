// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BrakeConstants;


public class Brake extends SubsystemBase {
  /** Creates a new Hand. */
  public static final Servo BRAKE_SERVOS = new Servo(0);
  //private static final Servo BRAKE_SERVO_RIGHT = RobotMap.BRAKE_SERVOS_RIGHT;
  //private static final Servo BRAKE_SERVO_LEFT = RobotMap.BRAKE_SERVOS_LEFT;

  public Brake() {
    setBrakePosition(Constants.BrakeConstants.BRAKE_ON);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Brake Angle", getBrakeAngle());

    if (DriverStation.getMatchTime() < .1  && DriverStation.getMatchTime() >= 0 )
    {
      Brake.setBrakePosition(BrakeConstants.BRAKE_ON);
    }
  }

  /**
   * exception if distance is not within reasonable rage or not in correct state
   * 
   * @param position set the hands to postion
   */
  public static void setBrakePosition(double position) {
    BRAKE_SERVOS.setAngle(position); 
  }


  // Get the current position of one of the hands/servos
  public static double getBrakeAngle() {
    return BRAKE_SERVOS.getAngle();

  }

}