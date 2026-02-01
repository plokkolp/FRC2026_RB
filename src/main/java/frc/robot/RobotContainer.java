package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.constants.ConsController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {

    private final double maxSpeed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // ===================== Joysticks (依 ConsController port) =====================
    private final Joystick driver   = new Joystick(ConsController.kDriveControllerPort);      // 原本 driver (Xbox)
    private final Joystick testJoy  = new Joystick(3);                                        // 你原本 Test = port 3（ConsController 沒定義，我照你現有保留）
    private final Joystick operator = new Joystick(ConsController.kOperatorControllerPort);   // 原本 operator

    // ===================== Subsystems =====================
    private final Shooter m_shooter = new Shooter();
    private final Intake  m_intake  = new Intake();
    private final LL4 m_Ll4 = new LL4("");

    public final CommandSwerveDrivetrain drivetrain =
            TunerConstants.createDrivetrain();

    private final Vision vision = new Vision(drivetrain);

    private final SendableChooser<Command> autoChooser;

    private final SwerveRequest.SwerveDriveBrake brake =
            new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idle =
            new SwerveRequest.Idle();

    public RobotContainer(Supplier<Boolean> m_isRedAlliance) {
        drivetrain.setAllianceSupplier(m_isRedAlliance);
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        setDefaultCommand();

        // driver A：煞車
        new JoystickButton(driver, ConsController.Button.BUTTON_A.id)
                .whileTrue(drivetrain.applyRequest(() -> brake));

        // driver LB：重設 Field Centric
        new JoystickButton(driver, ConsController.Button.BUTTON_LB.id)
                .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        new JoystickButton(testJoy, ConsController.Button.BUTTON_A.id)
                .toggleOnTrue(new Shooter_test(m_shooter, 0.0));

        new JoystickButton(testJoy, ConsController.Button.BUTTON_Y.id)
                .toggleOnTrue(new Shooter_test(m_shooter, 0.5));
    }

    private void setDefaultCommand() {

        // ===================== Drivetrain Default =====================
        drivetrain.setDefaultCommand(
                new Drive(
                        drivetrain,
                        () -> +driver.getRawAxis(ConsController.Axis.LEFT_STICK_Y.id) * maxSpeed,
                        () -> -driver.getRawAxis(ConsController.Axis.LEFT_STICK_X.id) * maxSpeed,
                        () -> -driver.getRawAxis(ConsController.Axis.RIGHT_STICK_X.id) * maxAngularRate
                )
        );

        // ===================== Shooter Default =====================
        m_shooter.setDefaultCommand(
                new ShooterTest(
                        m_shooter,
                        () -> MathUtil.applyDeadband(
                                -testJoy.getRawAxis(ConsController.Axis.RIGHT_STICK_Y.id),
                                0.1
                        )
                )
        );

        // ===================== Intake Default =====================
        m_intake.setDefaultCommand(
                new IntakeTest(
                        m_intake,
                        () -> driver.getRawAxis(ConsController.Axis.RIGHT_TRIGGER.id),
                        () -> driver.getRawAxis(ConsController.Axis.LEFT_TRIGGER.id),
                        () -> new JoystickButton(driver, ConsController.Button.BUTTON_RB.id).getAsBoolean()
                )
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
