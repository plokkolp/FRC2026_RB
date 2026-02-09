package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.constants.ConsController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

    private final double maxSpeed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final XboxController driver   = new XboxController(ConsController.kDriveControllerPort);      
    private final XboxController testJoy  = new XboxController(3);                                       
    private final XboxController operator = new XboxController(ConsController.kOperatorControllerPort);  
    private final XboxController TT  = new XboxController(4);                                       


    private final Shooter m_shooter = new Shooter();
    private final Intake  m_intake  = new Intake();
    private final LL4 m_Ll4 = new LL4("limelight-shoot");

    public final CommandSwerveDrivetrain drivetrain =
            TunerConstants.createDrivetrain();

//     private final Vision vision = new Vision(drivetrain);

    private final SendableChooser<Command> autoChooser;

    private final SwerveRequest.SwerveDriveBrake brake =
            new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idle =
            new SwerveRequest.Idle();

    public RobotContainer(Supplier<Boolean> m_isRedAlliance) {
        drivetrain.setAllianceSupplier(m_isRedAlliance);
        configureBindings();
        
        configureNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        setDefaultCommand();
                Trigger leftTrigger = new Trigger(() ->
          testJoy.getRawAxis(ConsController.Axis.LEFT_TRIGGER.id) > 1);
          
         new JoystickButton(driver, ConsController.Button.BUTTON_LB.id)
    .                   onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
// new JoystickButton(driver, ConsController.Button.BUTTON_LB.id)
//     .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // new JoystickButton(driver, ConsController.Button.BUTTON_A.id)
        //         .whileTrue(drivetrain.applyRequest(() -> brake));

        // new JoystickButton(driver, ConsController.Button.BUTTON_LB.id)
        //         .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // new JoystickButton(testJoy, ConsController.Button.BUTTON_A.id)
        //         .toggleOnTrue(new Shooter_test(m_shooter, 0.0));


        // new JoystickButton(testJoy, ConsController.Button.BUTTON_A.id)
        //         .toggleOnTrue(new Shoot0227(m_shooter,testJoy));

        // new JoystickButton(driver, ConsController.Button.BUTTON_A.id)
        //         .toggleOnTrue(new GetFuel(m_intake));


          new JoystickButton(driver, ConsController.Button.BUTTON_Y.id)
                .whileTrue(new ShooterEasy(m_shooter, driver
                ));
//                 new JoystickButton(testJoy, ConsController.Button.BUTTON_A.id)
//     .onTrue(edu.wpi.first.wpilibj2.command.Commands.print("A pressed"));

         
    }

    private void setDefaultCommand() {


        drivetrain.setDefaultCommand(
                new Drive(
                        drivetrain,
                        () -> +driver.getRawAxis(ConsController.Axis.LEFT_STICK_Y.id) * maxSpeed,
                        () -> -driver.getRawAxis(ConsController.Axis.LEFT_STICK_X.id) * maxSpeed,
                        () -> -driver.getRawAxis(ConsController.Axis.RIGHT_STICK_X.id) * maxAngularRate
                )
        );


        m_shooter.setDefaultCommand(
        new Shooter_test(
                m_shooter,
                () -> MathUtil.applyDeadband(
                -testJoy.getRawAxis(ConsController.Axis.RIGHT_STICK_Y.id),
                0.1
                ),
                () -> testJoy.getRawButton(ConsController.Button.BUTTON_Y.id) 
        )
        );



        m_intake.setDefaultCommand( 
                new IntakeTest(
                        m_intake,
                        () -> TT.getRawAxis(ConsController.Axis.RIGHT_TRIGGER.id),
                        () -> TT.getRawAxis(ConsController.Axis.LEFT_TRIGGER.id),
                        () -> TT.getRawAxis(ConsController.Axis.RIGHT_STICK_Y.id)
                )
        );

        //  m_shooter.setDefaultCommand(
        // new ShooterTest(
        //         m_shooter,
        //         () -> MathUtil.applyDeadband(
        //         -testJoy.getRawAxis(ConsController.Axis.RIGHT_STICK_Y.id),
        //         0.1
        //         )
        // )
        // );
    }

     

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    private void configureNamedCommands() {
        NamedCommands.registerCommand("Shoot",new MJ(m_shooter));
        NamedCommands.registerCommand("Shoot1",new MJ1(m_shooter));

    }
}
