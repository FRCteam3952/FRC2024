package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final CANSparkMax motor1;
    private final CANSparkMax motor2;

    private final RelativeEncoder motor1Encoder;
    private final RelativeEncoder motor2Encoder;

    private final SparkPIDController motor1Pid;
    private final SparkPIDController motor2Pid;

    public ClimberSubsystem() {
        motor1 = new CANSparkMax(PortConstants.CLIMBER_MOTOR_1_ID, MotorType.kBrushless);
        motor2 = new CANSparkMax(PortConstants.CLIMBER_MOTOR_2_ID, MotorType.kBrushless);

        motor1Encoder = motor1.getEncoder();
        motor2Encoder = motor2.getEncoder();

        motor1Pid = motor1.getPIDController();
        motor2Pid = motor2.getPIDController();

        zeroPid(motor1Pid);
        zeroPid(motor2Pid);
    }

    public void zeroPid(SparkPIDController pid) {
        pid.setP(0);
        pid.setI(0);
        pid.setD(0);
        pid.setFF(0);
    }

    public double getMotor1Amperage() { return motor1.getOutputCurrent(); }
    public double getMotor2Amperage() { return motor2.getOutputCurrent(); }

    public double getMotor1Position() { return motor1Encoder.getPosition(); }
    public double getMotor2Position() { return motor2Encoder.getPosition(); }

    public void setMotor1Position(double position) { motor1Encoder.setPosition(position); }
    public void setMotor2Position(double position) { motor2Encoder.setPosition(position); }

    public void setMotor1Speed(double speed) { motor1.set(speed); }
    public void setMotor2Speed(double speed) { motor2.set(speed); }
}
