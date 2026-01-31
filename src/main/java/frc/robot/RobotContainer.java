// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.TestMotorSubsystem;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final TestMotorSubsystem m_testMotorSubsystem = new TestMotorSubsystem();

    public RobotContainer() {
        NamedCommands.registerCommand("moveSmallKrakenForwards", m_testMotorSubsystem.moveSmallKraken(1));
        NamedCommands.registerCommand("moveTestForwards", m_testMotorSubsystem.moveTest(1));
        NamedCommands.registerCommand("moveSmallKrakenBackwards", m_testMotorSubsystem.moveSmallKraken(-1));
        NamedCommands.registerCommand("moveTestBackwards", m_testMotorSubsystem.moveTest(-1));
        NamedCommands.registerCommand("stopSmallKraken", m_testMotorSubsystem.moveSmallKraken(0));
        NamedCommands.registerCommand("stopTest", m_testMotorSubsystem.moveTest(0));
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        
        joystick.leftBumper().onTrue(new PathPlannerAuto("Spin"));
        joystick.rightBumper().onTrue(new PathPlannerAuto("Spin"));
        
        // joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
     
        // joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
        


        /*
        * Joystick Y = quasistatic forward
        * Joystick A = quasistatic reverse
        * Joystick B = dynamic forward
        * Joystick X = dyanmic reverse
        */
        // joystick.y().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // joystick.a().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // joystick.b().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // joystick.x().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // FOR TESTING PURPOSES (bindings can be overwritten):
        joystick.x().onTrue(m_testMotorSubsystem.moveSmallKraken(0.6767));
        joystick.x().onFalse(m_testMotorSubsystem.stopSmallKraken());
        joystick.y().onTrue(m_testMotorSubsystem.moveTest(0.6767));
        joystick.y().onFalse(m_testMotorSubsystem.stopTest());
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );
        return new PathPlannerAuto("Practice");
    }
}
