package frc.robot.subsystems.conveyor;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class ConveyorSubsystem extends SubsystemBase {
    private final CANSparkMax shooterFeederMotor;
    // private final CANSparkMax conveyorMotorLeader;
    // private final CANSparkMax conveyorMotorFollower;

    public ConveyorSubsystem() {
        this.shooterFeederMotor = new CANSparkMax(PortConstants.CONVEYOR_FEED_TO_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    }

    public void setShooterFeederMotorSpeed(double speed) {
        this.shooterFeederMotor.set(speed);
    }
}
