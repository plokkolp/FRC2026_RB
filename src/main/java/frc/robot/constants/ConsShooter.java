package frc.robot.constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

public final class ConsShooter {


  public static final int LEFT_SHOOTER_ID   = 21;
  public static final int RIGHT_SHOOTER_ID  = 22;

  public static final int TRAIN_ID          = 23;

  public static final int ANGLE_MOTOR_ID    = 24;
  public static final int ANGLE_CANCODER_ID = 25;

  public static final int MINION_MOTOR_ID   = 20;


  public static final TalonFXConfiguration LEFT_SHOOTER_CONFIG  = new TalonFXConfiguration();
  public static final TalonFXConfiguration RIGHT_SHOOTER_CONFIG = new TalonFXConfiguration();

  private static final Slot0Configs SHOOTER_VEL_SLOT0 =
      new Slot0Configs()
          .withKP(0.5)
          .withKI(0.00)
          .withKD(0.00)
          .withKV(0.12);

  static {
    LEFT_SHOOTER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    LEFT_SHOOTER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    LEFT_SHOOTER_CONFIG.Slot0 = SHOOTER_VEL_SLOT0;

    RIGHT_SHOOTER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    RIGHT_SHOOTER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    RIGHT_SHOOTER_CONFIG.Slot0 = SHOOTER_VEL_SLOT0;
  }


  public static final TalonFXConfiguration TRAIN_CONFIG = new TalonFXConfiguration();

  static {
    TRAIN_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    TRAIN_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }


  public static final CANcoderConfiguration ANGLE_CANCODER_CONFIG = new CANcoderConfiguration();
  public static final TalonFXConfiguration  ANGLE_MOTOR_CONFIG    = new TalonFXConfiguration();

  private static final Slot0Configs ANGLE_POS_SLOT0 =
      new Slot0Configs()
          .withKP(10)
          .withKI(0)
          .withKD(1);

  static {

    ANGLE_CANCODER_CONFIG.MagnetSensor.MagnetOffset = 0.0;

    ANGLE_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ANGLE_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    ANGLE_MOTOR_CONFIG.Feedback.FeedbackRemoteSensorID = ANGLE_CANCODER_ID;
    ANGLE_MOTOR_CONFIG.Feedback.FeedbackSensorSource   = FeedbackSensorSourceValue.FusedCANcoder;

    ANGLE_MOTOR_CONFIG.Feedback.RotorToSensorRatio = 1.0;
    ANGLE_MOTOR_CONFIG.Feedback.SensorToMechanismRatio =10;

    ANGLE_MOTOR_CONFIG.Slot0 = ANGLE_POS_SLOT0;
  }

  public static final TalonFXSConfiguration MINION_MOTOR_CONFIG = new TalonFXSConfiguration();

  private static final VoltageConfigs MINION_VOLT_LIMIT =
      new VoltageConfigs()
          .withPeakForwardVoltage(8)
          .withPeakReverseVoltage(-8);

  private static final Slot0Configs MINION_POS_SLOT0 =
      new Slot0Configs()
          .withKP(1)
          .withKI(0)
          .withKD(0.5);

  static {
    MINION_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    MINION_MOTOR_CONFIG.Voltage = MINION_VOLT_LIMIT;

    MINION_MOTOR_CONFIG.Slot0 = MINION_POS_SLOT0;
  }

  private ConsShooter() {}
}
