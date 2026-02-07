package frc.robot.constants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ConsIntake {

  private ConsIntake() {}


  public static final int HAVE_ROLLING_ID  = 11;
  public static final int TRAIN_ROLLING_ID = 12;
  public static final int HAVE_ANGLE_ID    = 13;

  public static final TalonFXConfiguration ROLLING_CONFIG;

  static {
    ROLLING_CONFIG = new TalonFXConfiguration();
    ROLLING_CONFIG.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive);
  }

  public static final TalonFXConfiguration ANGLE_CONFIG;

  static {
    ANGLE_CONFIG = new TalonFXConfiguration();

    ANGLE_CONFIG.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    ANGLE_CONFIG.Slot0 = new Slot0Configs()
        .withKP(2)
        .withKI(0)
        .withKD(0);

    ANGLE_CONFIG.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(20)
        .withMotionMagicAcceleration(40);
  }
}
