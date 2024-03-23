package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.util.NetworkTablesUtil;

public class ClimberSubsystem extends SubsystemBase {
    private static final DoublePublisher lCurrentPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("lclimb_current").publish();
    private static final DoublePublisher rCurrentPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("rclimb_current").publish();

    private final CANSparkMax leftHook;
    private final CANSparkMax rightHook;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SparkPIDController leftPid;
    private final SparkPIDController rightPid;

    public ClimberSubsystem() {
        leftHook = new CANSparkMax(PortConstants.CLIMBER_LEFT_MOTOR_ID, MotorType.kBrushless);
        rightHook = new CANSparkMax(PortConstants.CLIMBER_RIGHT_MOTOR_ID, MotorType.kBrushless);

        leftEncoder = leftHook.getEncoder();
        rightEncoder = rightHook.getEncoder();

        leftPid = leftHook.getPIDController();
        rightPid = rightHook.getPIDController();

        leftHook.setIdleMode(IdleMode.kBrake);
        rightHook.setIdleMode(IdleMode.kBrake);

        leftHook.enableVoltageCompensation(10);
        rightHook.enableVoltageCompensation(10);

        leftHook.setSmartCurrentLimit(25);
        rightHook.setSmartCurrentLimit(25);

        configurePID(leftPid);
        configurePID(rightPid);
    }

    public void configurePID(SparkPIDController pid) {
        pid.setP(0);
        pid.setI(0);
        pid.setD(0);
        pid.setFF(0);
    }

    public double getLeftAmperage() {
        return leftHook.getOutputCurrent();
    }

    public double getRightAmperage() {
        return rightHook.getOutputCurrent();
    }

    public double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    public void setLeftPosition(double position) {
        leftEncoder.setPosition(position);
    }

    public double getRightPosition() {
        return rightEncoder.getPosition();
    }

    public void setRightPosition(double position) {
        rightEncoder.setPosition(position);
    }

    public void setLeftSpeed(double speed) {
        leftHook.set(speed);
    }

    public void setRightSpeed(double speed) {
        rightHook.set(speed);
    }

    @Override
    public void periodic() {
        lCurrentPublisher.set(getLeftAmperage());
        rCurrentPublisher.set(getRightAmperage());
    }
}