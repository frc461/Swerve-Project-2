package frc.robot.subsystems;

import frc.robot.generated.TunerConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;

public class TestMotorSubsystem extends SubsystemBase {
    private final TalonFX smallKrakenMotor;
    private final TalonFX testMotor;

    public TestMotorSubsystem() {
        smallKrakenMotor = new TalonFX(TunerConstants.SmallKrakenMotorId);
        testMotor = new TalonFX(TunerConstants.TestMotorId);
    }

    public Command moveSmallKraken(double speed) {
        return runOnce(() -> smallKrakenMotor.set(speed));
    }

    public Command moveTest(double speed) {
        return runOnce(() -> testMotor.set(speed));
    }

    public Command stopSmallKraken() {
        return runOnce(() -> smallKrakenMotor.set(0));
    }

    public Command stopTest() {
        return runOnce(() -> testMotor.set(0));
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}