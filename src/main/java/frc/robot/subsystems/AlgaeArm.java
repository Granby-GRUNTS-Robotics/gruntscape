package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArm extends SubsystemBase {
  /** Creates a new AlgaeArm. */

  private final static SparkMax AlgaeArmRotationMotor = new SparkMax(Constants.Algae.ALGAE_ARM_CONTROL_MOTOR_ID, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  SparkClosedLoopController algaeArmPid = AlgaeArmRotationMotor.getClosedLoopController();

  private static final RelativeEncoder ALGAE_ARM_POSITION_ENCODER = AlgaeArmRotationMotor.getEncoder();


  public AlgaeArm() {
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
        .maxVelocity(1400)
        .maxAcceleration(7000)
        .allowedClosedLoopError(0.05);
    config.closedLoop
        .pid(0.25, 0.0, 0.0) // original value: (0.1, 0, 0)
        .outputRange(kMinOutput, kMaxOutput);

    AlgaeArmRotationMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // force position to be 0 at the start
    ALGAE_ARM_POSITION_ENCODER.setPosition(0);
  }

  public void setAlgaeRotations(double wantedRotations) {
      algaeArmPid.setReference(wantedRotations, SparkMax.ControlType.kMAXMotionPositionControl);
    }
    
  public double getCurrentAlgaeArmRotation() {
    return ALGAE_ARM_POSITION_ENCODER.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Arm Position", ALGAE_ARM_POSITION_ENCODER.getPosition());
  }
}
