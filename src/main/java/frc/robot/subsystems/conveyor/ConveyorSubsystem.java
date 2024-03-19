package frc.robot.subsystems.conveyor;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Flags;
import frc.robot.util.NetworkTablesUtil;

public class ConveyorSubsystem extends SubsystemBase {
    private static final DoublePublisher leftCurrentPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("left_conveyor_current").publish();
    private static final DoublePublisher rightCurrentPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("right_conveyor_current").publish();
    private final CANSparkMax shooterFeederMotor;
    private final CANSparkMax conveyorMotorLeader;
    private final CANSparkMax conveyorMotorFollower;

    public ConveyorSubsystem() {
        this.shooterFeederMotor = new CANSparkMax(PortConstants.CONVEYOR_TO_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        this.conveyorMotorFollower = new CANSparkMax(PortConstants.CONVEYOR_LEFT_MOTOR_ID, MotorType.kBrushless);
        this.conveyorMotorLeader = new CANSparkMax(PortConstants.CONVEYOR_RIGHT_MOTOR_ID, MotorType.kBrushless);

        // this.conveyorMotorFollower.follow(conveyorMotorLeader, true);
    }

    public void setConveyorMotorsSpeed(double speedLeft, double speedRight) {
        if(Flags.Conveyor.ENABLED) {
            this.conveyorMotorFollower.set(speedLeft);
            this.conveyorMotorLeader.set(speedRight);
        }
    }

    public void setShooterFeederMotorSpeed(double speed) {
        if(Flags.Conveyor.ENABLED) {
            this.shooterFeederMotor.set(speed);
        }
    }

    public void setConveyorMotorsSpeed(double speed) {
        if(Flags.Conveyor.ENABLED) {
            this.setConveyorMotorsSpeed(speed, speed);
        }
    }

    @Override
    public void periodic() {
        leftCurrentPublisher.set(this.conveyorMotorFollower.getOutputCurrent());
        rightCurrentPublisher.set(this.conveyorMotorLeader.getOutputCurrent());

        //if(Math.abs(conveyorMotorFollower.get()) < 0.02) {
        //    System.out.println("left conveyor motor is at 0");
        //}
    }
}
