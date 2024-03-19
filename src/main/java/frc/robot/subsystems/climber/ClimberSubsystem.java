package frc.robot.subsystems.climber;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Flags;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class ClimberSubsystem extends SubsystemBase {

    private static final int LEFT_MOTOR = 1;
    private static final int RIGHT_MOTOR = 2;

    private final List<CANSparkMax> motors;
    private final List<RelativeEncoder> encoders;
    private final List<SparkPIDController> pids;

    public final DoubleConsumer setLeftMotorSpeed, setRightMotorSpeed,
                                setLeftMotorPosition, setRightMotorPosition;

    public final DoubleSupplier getLeftMotorPosition, getRightMotorPosition,
                                getLeftMotorAmperage, getRightMotorAmperage;

    public ClimberSubsystem() {
        if (!Flags.Climber.IS_ATTACHED) throw new IllegalStateException("Flags.Climber.IS_ATTACHED == false, so the ClimberSubsystem can't be initialized!");

        motors = Stream.of(PortConstants.CLIMBER_MOTOR_1_ID, PortConstants.CLIMBER_MOTOR_2_ID)
                .map((port) -> new CANSparkMax(port, MotorType.kBrushless))
                .collect(Collectors.toList());

        encoders = motors.stream().map(CANSparkBase::getEncoder)      .collect(Collectors.toList());
        pids     = motors.stream().map(CANSparkBase::getPIDController).collect(Collectors.toList());

        pids.forEach((pid) -> {
            pid.setP(0);
            pid.setI(0);
            pid.setD(0);
            pid.setFF(0);
        });

        // Dynamic method creation for left & right motors
        setLeftMotorSpeed     = motorSpeedSetter(LEFT_MOTOR);
        getLeftMotorAmperage  = motorAmperageGetter(LEFT_MOTOR);
        getLeftMotorPosition  = motorPositionGetter(LEFT_MOTOR);
        setLeftMotorPosition  = motorPositionSetter(LEFT_MOTOR);

        setRightMotorSpeed    = motorSpeedSetter(RIGHT_MOTOR);
        getRightMotorAmperage = motorAmperageGetter(RIGHT_MOTOR);
        getRightMotorPosition = motorPositionGetter(RIGHT_MOTOR);
        setRightMotorPosition = motorSpeedSetter(RIGHT_MOTOR);
    }

    public DoubleConsumer motorSpeedSetter(int motorId) {
        return Flags.Climber.ENABLED ?
               (double speed) -> motors.get(motorId).set(speed) :
               (double speed) -> {};
    }

    public DoubleConsumer motorPositionSetter(int motorId) {
        return Flags.Climber.ENABLED ?
                (double position) -> encoders.get(motorId).setPosition(position) :
                (double position) -> {};
    }

    public DoubleSupplier motorAmperageGetter(int motorId) {
        return () -> motors.get(motorId).getOutputCurrent();
    }

    public DoubleSupplier motorPositionGetter(int motorId) {
        return () -> encoders.get(motorId).getPosition();

    }
}
