package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Flags;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.ThroughboreEncoder;

public class IntakeSubsystem extends SubsystemBase {
    private static final double LEADER_CURRENT_SPIKE_THRESH = 17.5;
    private static final double FOLLOWER_CURRENT_SPIKE_THRESH = 15;

    private static final DoublePublisher leaderCurrentPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("leader_current").publish();
    private static final DoublePublisher followerCurrentPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("follower_current").publish();
    private static final DoublePublisher pivotCurrentPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("pivot_current").publish();

    private final CANSparkMax leaderMotor;
    private final CANSparkMax followerMotor;
    private final CANSparkMax pivotMotor;

    private final RelativeEncoder pivotEncoder;
    // private final SparkPIDController pivotPIDController;
    private final PIDController pivotPIDController;
    private final DigitalInput pivotUpLimitSwitch;

    private final ThroughboreEncoder throughboreEncoder;

    private double pivotAngleSetpoint = 0;

    public IntakeSubsystem() {
        leaderMotor = new CANSparkMax(PortConstants.INTAKE_TOP_MOTOR_ID, MotorType.kBrushless);
        followerMotor = new CANSparkMax(PortConstants.INTAKE_BOTTOM_MOTOR_ID, MotorType.kBrushless);

        this.pivotUpLimitSwitch = new DigitalInput(PortConstants.INTAKE_UP_LIMIT_SWITCH_PORT);

        leaderMotor.setInverted(true);
        followerMotor.setInverted(true);
        // followerMotor.follow(leaderMotor, false);

        pivotMotor = new CANSparkMax(PortConstants.INTAKE_PIVOT_MOTOR_ID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();

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

        this.pivotPIDController = new PIDController(9e-3, 0, 1e-4);

        /*
        pivotPIDController = pivotMotor.getPIDController();

        pivotPIDController.setP(1.85e-2, 0); // DOWN
        pivotPIDController.setI(0, 0);
        pivotPIDController.setD(0, 0);
        pivotPIDController.setFF(0, 0);

        pivotPIDController.setOutputRange(-1, 1, 0);

        
        pivotPIDController.setP(0, 1); // UP
        pivotPIDController.setI(0, 1);
        pivotPIDController.setD(0, 1);
        pivotPIDController.setFF(0, 1);

        pivotPIDController.setOutputRange(-1, 1, 1); */

        this.followerMotor.setOpenLoopRampRate(0.5);
        this.leaderMotor.setOpenLoopRampRate(0.5);
    }

    public void setIntakeSpeed(double speed) {
        if (Flags.Intake.ENABLED) {
            leaderMotor.set(speed);
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

    public boolean isPivotUpLimitPressed() {
        return this.pivotUpLimitSwitch.get();
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

    public boolean isLeaderCurrentSpiked() {
        return this.getLeaderMotorCurrent() > LEADER_CURRENT_SPIKE_THRESH;
    }

    public boolean isFollowerCurrentSpiked() {
        return this.getFollowerMotorCurrent() > FOLLOWER_CURRENT_SPIKE_THRESH;
    }

    public ThroughboreEncoder getThroughboreEncoder() {
        return this.throughboreEncoder;
    }

    @Override
    public void periodic() {
        /*
        if(this.isPivotUpLimitPressed()) {
            this.setPivotEncoderPosition(70);
        }*/

        if (Flags.Intake.ENABLED && Flags.Intake.PIVOT_ENABLED && Flags.Intake.PIVOT_PID_CONTROL) {
            boolean goingDown = this.throughboreEncoder.getAbsoluteEncoderValue() - this.pivotAngleSetpoint > 0;
            double speed = this.pivotPIDController.calculate(this.throughboreEncoder.getAbsoluteEncoderValue(), this.pivotAngleSetpoint);
            // System.out.println("going to: " + this.intakeAngleSetpoint + ", desired pivot speed: " + speed);
            if (goingDown) {
                // this.pivotMotor.set(speed);
            } else if (!goingDown) { // going up
                this.pivotMotor.set(speed + 0.02); // feedforward
            } else {
                this.pivotMotor.set(0);
            }
        }

        leaderCurrentPublisher.set(this.getLeaderMotorCurrent());
        followerCurrentPublisher.set(this.getFollowerMotorCurrent());
        //pivotCurrentPublisher.set(this.getPivotMotorCurrent());
        // System.out.println("pivot angle: " + this.getPivotEncoder()); 
        // System.out.println("intake throughbore: " + this.throughboreEncoder.getAbsoluteEncoderValue());
        //System.out.println(this.getPivotPosition());
    }
}
