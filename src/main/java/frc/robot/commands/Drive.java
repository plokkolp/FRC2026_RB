package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Drive extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier vX, vY, vOmega;

    private static final double kTransDeadband = 0.1;   
    private static final double kRotDeadband   = 0.10;  
    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public Drive(
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier vX,
        DoubleSupplier vY,
        DoubleSupplier vOmega
    ) {
        this.drivetrain = drivetrain;
        this.vX = vX;
        this.vY = vY;
        this.vOmega = vOmega;
        addRequirements(drivetrain);
    }
    @Override
    public void initialize() {}
    
    @Override
    public void execute() {

        double x = MathUtil.applyDeadband(vX.getAsDouble(), kTransDeadband);
        double y = MathUtil.applyDeadband(vY.getAsDouble(), kTransDeadband);
        double omega = MathUtil.applyDeadband(vOmega.getAsDouble(), kRotDeadband);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(x)
                .withVelocityY(y)
                .withRotationalRate(omega)
        );
    }
    
    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return false;
    }
    }
