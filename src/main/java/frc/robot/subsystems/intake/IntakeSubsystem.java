package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Flags;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.ThroughboreEncoder;

public class IntakeSubsystem extends SubsystemBase {
    private static final DoublePublisher leaderCurrentPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("leader_current").publish();
    private static final DoublePublisher followerCurrentPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("follower_current").publish();
    private static final DoublePublisher pivotCurrentPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("pivot_current").publish();

    private final CANSparkMax leaderMotor;
    private final CANSparkMax followerMotor;
    private final CANSparkMax pivotMotor;

    private final RelativeEncoder pivotEncoder;
    // private final SparkPIDController pivotPIDController;
    private final PIDController pivotUpPIDController;
    private final PIDController pivotDownPIDController;

    private final ThroughboreEncoder throughboreEncoder;

    private double pivotAngleSetpoint = 0;

    public IntakeSubsystem() {
        leaderMotor = new CANSparkMax(PortConstants.INTAKE_TOP_MOTOR_ID, MotorType.kBrushless);
        followerMotor = new CANSparkMax(PortConstants.INTAKE_BOTTOM_MOTOR_ID, MotorType.kBrushless);

        leaderMotor.setInverted(true);
        followerMotor.setInverted(true);
        // followerMotor.follow(leaderMotor, false);

        pivotMotor = new CANSparkMax(PortConstants.INTAKE_PIVOT_MOTOR_ID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();

        pivotMotor.setInverted(true);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        leaderMotor.enableVoltageCompensation(10);
        followerMotor.enableVoltageCompensation(10);
        pivotMotor.enableVoltageCompensation(10);
        // pivotMotor.setSmartCurrentLimit(20);
        // pivotMotor.setSecondaryCurrentLimit(100);

        // this.leaderMotor.setSmartCurrentLimit(20);
        // this.followerMotor.setSmartCurrentLimit(20); // These are NEO550s - DO NOT CHANGE THESE VALUES ABOVE 20.

        // this.leaderMotor.setSecondaryCurrentLimit(60);
        // this.followerMotor.setSecondaryCurrentLimit(60);

        pivotEncoder.setPositionConversionFactor(70 / 20d); // 70deg = 20rot
        pivotEncoder.setPosition(0);

        this.throughboreEncoder = new ThroughboreEncoder(PortConstants.INTAKE_ABSOLUTE_ENCODER_ABS_PORT, PortConstants.INTAKE_ABSOLUTE_ENCODER_A_PORT, PortConstants.INTAKE_ABSOLUTE_ENCODER_B_PORT, 0.955, true);

        this.pivotUpPIDController = new PIDController(9e-3, 0, 1.5e-4);
        this.pivotDownPIDController = new PIDController(5e-3, 0, 0);

        this.followerMotor.setOpenLoopRampRate(1);
        this.leaderMotor.setOpenLoopRampRate(1);
    }

    public void setIntakeSpeed(double speed) {
        if (Flags.Intake.ENABLED) {
            leaderMotor.set(speed);
            followerMotor.set(speed);
        }
    }

    public void setIntakeSpeed(double speedTop, double speedBottom) {
        if (Flags.Intake.ENABLED) {
            leaderMotor.set(speedTop);
            followerMotor.set(speedBottom);
        }
    }

    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    public void setPivotSpeed(double pivotSpeed) {
        if (Flags.Intake.ENABLED && Flags.Intake.PIVOT_ENABLED) {
            pivotMotor.set(pivotSpeed);
        }
    }

    public void setPivotEncoderPosition(double degrees) {
        pivotEncoder.setPosition(degrees);
    }

    public void resetPivotEncoder() {
        setPivotEncoderPosition(0.0);
    }

    public double getLeaderMotorCurrent() {
        return leaderMotor.getOutputCurrent();
    }

    public double getFollowerMotorCurrent() {
        return followerMotor.getOutputCurrent();
    }

    public double getPivotMotorCurrent() {
        return pivotMotor.getOutputCurrent();
    }

    /**
     * Set the PID controller to target a specific position in degrees. Degrees should be [-90, 0], with -90 being the "down" position.
     *
     * @param degrees The target setpoint, with -90 being the down position and 0 being the up position.
     */
    public void pivotToAngle(double degrees) {
        if (Flags.Intake.ENABLED && Flags.Intake.PIVOT_ENABLED && Flags.Intake.PIVOT_PID_CONTROL) {
            this.pivotAngleSetpoint = degrees;
        }
    }

    public double getPivotTargetAngle() {
        return this.pivotAngleSetpoint;
    }

    public ThroughboreEncoder getThroughboreEncoder() {
        return this.throughboreEncoder;
    }

    @Override
    public void periodic() {
        if (Flags.Intake.ENABLED && Flags.Intake.PIVOT_ENABLED && Flags.Intake.PIVOT_PID_CONTROL) {
            double deltaAngle = this.throughboreEncoder.getAbsoluteEncoderValue() - this.pivotAngleSetpoint;
            double speed = 0;
            if(Math.abs(deltaAngle) > 1) {
                boolean goingDown = deltaAngle > 0;
                if(goingDown) {
                    speed = this.pivotDownPIDController.calculate(this.throughboreEncoder.getAbsoluteEncoderValue(), this.pivotAngleSetpoint) + 0;
                } else {
                    speed = this.pivotUpPIDController.calculate(this.throughboreEncoder.getAbsoluteEncoderValue(), this.pivotAngleSetpoint) + 0;
                }
                // System.out.println("going to: " + this.pivotAngleSetpoint + " from " + this.throughboreEncoder.getAbsoluteEncoderValue() + ", desired pivot speed: " + speed);
                // System.out.println("estimated encoder pos: " + this.pivotEncoder.getPosition());
            }
            this.pivotMotor.set(speed);
        }

        leaderCurrentPublisher.set(this.getLeaderMotorCurrent());
        followerCurrentPublisher.set(this.getFollowerMotorCurrent());
        //pivotCurrentPublisher.set(this.getPivotMotorCurrent());
        // System.out.println("pivot angle: " + this.getPivotEncoder()); 
        // System.out.println("intake throughbore: " + this.throughboreEncoder.getAbsoluteEncoderValue());
        //System.out.println(this.getPivotPosition());
    }
}
