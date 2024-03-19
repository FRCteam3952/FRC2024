package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Flags;
import frc.robot.subsystems.staticsubsystems.ColorSensor;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.ThroughboreEncoder;

// Shoutout to Avni whose code I reverse engineered - Fox in a box(awesome sause)

public class ShooterSubsystem extends SubsystemBase {
    private static final DoublePublisher lAmp = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("lAmp").publish();
    private static final DoublePublisher rAmp = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("rAmp").publish();
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANSparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;
    private final RelativeEncoder bottomShooterEncoder;
    private final DigitalInput pivotDownLimitSwitch;
    private final PIDController pivotPidController;
    private final SparkPIDController rightPidController;
    private final SparkPIDController leftPidController;
    private final ThroughboreEncoder throughboreEncoder;

    private double pivotAngleSetpoint = 30;

    public ShooterSubsystem() {
        leftMotor = new CANSparkMax(PortConstants.SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(PortConstants.SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);

        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);
        pivotMotor = new CANSparkMax(PortConstants.SHOOTER_PIVOT_MOTOR_ID, MotorType.kBrushless);

        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotEncoder = pivotMotor.getEncoder();
        bottomShooterEncoder = rightMotor.getEncoder();
        rightMotor.setInverted(true);
        // leftMotor.follow(rightMotor, true);
        pivotMotor.setInverted(false);
        pivotEncoder.setPosition(0);

        pivotDownLimitSwitch = new DigitalInput(PortConstants.SHOOTER_DOWN_LIMIT_SWITCH_PORT);

        pivotPidController = new PIDController(7.3e-3, 0, 0);
        rightPidController = rightMotor.getPIDController();
        leftPidController = leftMotor.getPIDController();

        this.throughboreEncoder = new ThroughboreEncoder(PortConstants.SHOOTER_ABSOLUTE_ENCODER_ABS_PORT, PortConstants.SHOOTER_ABSOLUTE_ENCODER_A_PORT, PortConstants.SHOOTER_ABSOLUTE_ENCODER_B_PORT, 0.693, true);

        leftMotor.enableVoltageCompensation(10);
        rightMotor.enableVoltageCompensation(10);


        rightPidController.setP(5e-6, 0);
        rightPidController.setI(0, 0);
        rightPidController.setD(0, 0);
        rightPidController.setFF(2.2e-4, 0);

        rightPidController.setP(0, 1);
        rightPidController.setI(0, 1);
        rightPidController.setD(0, 1);
        rightPidController.setFF(0, 1);

        leftPidController.setP(5e-6, 0);
        leftPidController.setI(0, 0);
        leftPidController.setD(0, 0);
        leftPidController.setFF(2.2e-4, 0);

        leftPidController.setP(0, 1);
        leftPidController.setI(0, 1);
        leftPidController.setD(0, 1);
        leftPidController.setFF(0, 1);
    }

    public void pivotToAngle(double degrees) {
        if (Flags.Shooter.ENABLED && Flags.Shooter.PIVOT_ENABLED && Flags.Shooter.PIVOT_PID_CONTROL) {
            this.pivotAngleSetpoint = degrees;
        }
    }

    // Setting the speed of the motors
    public void setBottomMotorSpeed(double speed) {
        if (Flags.Shooter.ENABLED) {
            rightMotor.set(speed);
        }
    }

    public void setPivotMotorSpeed(double speed) {
        pivotMotor.set(speed);
    }

    // Getting
    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }
    
    /*
    public boolean getPivotLimitSwitch(){
        return pivotLimitSwitch.get();
    }*/

    public double getPivotVelocity() {
        return pivotEncoder.getVelocity();
    }

    /*
    public void setFlapMotorSpeed(double degrees){
        flapMotor.set(degrees);
    }

    public double getFlapPosition(){
        return flapEncoder.getPosition();
    }

    public boolean getFlapLimitSwitch(){
        return flapLimitSwitch.get();
    }
    
    public double getFlapVelocity(){
        return flapEncoder.getVelocity();
    }

    // Setting 
    public void setPivotEncoderPosition(double position){
        pivotEncoder.setPosition(position);
    }
    public void setFlapEncoderPosition(double position){
        flapEncoder.setPosition(position);
    }

    public void setFlapTargetPosition(double degree){
        flapPidController.setReference(degree, ControlType.kPosition);
    }*/
    public void setMotorRpm(double rpm) {
        if (Flags.Shooter.ENABLED && Flags.Shooter.SHOOTER_RPM_PID_CONTROL) {
            rightPidController.setReference(rpm, ControlType.kVelocity, 0);
            leftPidController.setReference(rpm, ControlType.kVelocity, 0);
        }
    }

    /**
     * This method should be used to stop the shooter rather than running setMotorRpm(0) since it chooses to not send voltage to the motors rather than actively trying to stop the shooter.
     */
    public void stopShooterPID() {
        if (Flags.Shooter.ENABLED && Flags.Shooter.SHOOTER_RPM_PID_CONTROL) {
            rightPidController.setReference(0, ControlType.kVelocity, 1);
            leftPidController.setReference(0, ControlType.kVelocity, 1);
        }
    }

    public double getShooterRpm() {
        return bottomShooterEncoder.getVelocity();
    }

    /*
    // Resetting 
    public void resetPivot(){
        pivotEncoder.setPosition(0.0);
    }
    public void resetFlap(){
        flapEncoder.setPosition(0.0);
    }*/
    @Override
    public void periodic() {
        if (Flags.Shooter.ENABLED && Flags.Shooter.PIVOT_ENABLED && Flags.Shooter.PIVOT_PID_CONTROL) {
            double speed = this.pivotPidController.calculate(this.throughboreEncoder.getAbsoluteEncoderValue(), this.pivotAngleSetpoint);
            // System.out.println("sending " + speed + " to go to " + pivotAngleSetpoint);
            this.pivotMotor.set(speed);
        }

        // System.out.println("current color: " + ColorSensor.colorAsRGBString(ColorSensor.getColor()));
        if (ColorSensor.isNoteColor()) {
            // System.out.println("omg its a note");
        }
        //lAmp.set(this.topMotor.getOutputCurrent());
        //rAmp.set(this.bottomMotor.getOutputCurrent());
        // System.out.println("shooter throughbore: " + this.throughboreEncoder.getAbsoluteEncoderValue());
        // System.out.println("rel encoder: " + this.getPivotPosition());
    }
}