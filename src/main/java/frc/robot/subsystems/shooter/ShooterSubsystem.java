package frc.robot.subsystems.shooter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Flags;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.ThroughboreEncoder;
import frc.robot.Constants.PortConstants;
import frc.robot.subsystems.staticsubsystems.ColorSensor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

// Shoutout to Avni whose code I reverse engineered - Fox in a box(awesome sause)

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;

    private final CANSparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;
    private final RelativeEncoder bottomShooterEncoder;
    
    private final DigitalInput pivotDownLimitSwitch;

    private final SparkPIDController pivotPidController;
    private final SparkPIDController bottomPidController;
    
    private static final DoublePublisher lAmp = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("lAmp").publish();
    private static final DoublePublisher rAmp = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("rAmp").publish();

    private final ThroughboreEncoder throughboreEncoder;

    public ShooterSubsystem() {
        topMotor = new CANSparkMax(PortConstants.SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(PortConstants.SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);

        topMotor.setIdleMode(IdleMode.kCoast);
        bottomMotor.setIdleMode(IdleMode.kCoast);
        pivotMotor = new CANSparkMax(PortConstants.SHOOTER_PIVOT_MOTOR_ID, MotorType.kBrushless);
        
        pivotEncoder = pivotMotor.getEncoder();
        bottomShooterEncoder = bottomMotor.getEncoder();
        bottomMotor.setInverted(true);
        topMotor.follow(bottomMotor, true);
        
        pivotDownLimitSwitch = new DigitalInput(PortConstants.SHOOTER_DOWN_LIMIT_SWITCH_PORT);

        pivotPidController = pivotMotor.getPIDController();
        bottomPidController = bottomMotor.getPIDController();

        this.throughboreEncoder = new ThroughboreEncoder(PortConstants.SHOOTER_ABSOLUTE_ENCODER_ABS_PORT, PortConstants.SHOOTER_ABSOLUTE_ENCODER_A_PORT, PortConstants.SHOOTER_ABSOLUTE_ENCODER_B_PORT);
        
        /*
        //intializen PID values to 0
        pivotPidController.setP(0);
        pivotPidController.setI(0);
        pivotPidController.setD(0);
        pivotPidController.setFF(0);
        */

        bottomPidController.setP(7.31415926e-4, 0);
        bottomPidController.setI(0, 0);
        bottomPidController.setD(1e-3, 0);
        bottomPidController.setFF(2e-4, 0);

        bottomPidController.setP(0, 1);
        bottomPidController.setI(0, 1);
        bottomPidController.setD(0, 1);
        bottomPidController.setFF(0, 1);
    }

    // Setting the speed of the motors
    public void setBottomMotorSpeed(double speed){
        if(Flags.Shooter.ENABLED) {
            bottomMotor.set(speed);
        }
    }
    
    public void setPivotMotorSpeed(double degrees){
        pivotMotor.set(degrees);
    }

    // Getting
    public double getPivotPosition(){
        return pivotEncoder.getPosition();
    }
    
    /*
    public boolean getPivotLimitSwitch(){
        return pivotLimitSwitch.get();
    }*/

    public double getPivotVelocity(){
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
    public void setPivotTargetPosition(double degree){
        pivotPidController.setReference(degree, ControlType.kPosition);
    }
    public void setFlapTargetPosition(double degree){
        flapPidController.setReference(degree, ControlType.kPosition);
    }*/
    public void setMotorRpm(double rpm){
        if(Flags.Shooter.ENABLED && Flags.Shooter.SHOOTER_RPM_PID_CONTROL) {
            bottomPidController.setReference(rpm, ControlType.kVelocity, 0);
        }
    }

    public void stopShooterPID() {
        if(Flags.Shooter.ENABLED && Flags.Shooter.SHOOTER_RPM_PID_CONTROL) {
            bottomPidController.setReference(0, ControlType.kVelocity, 1);
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
        //System.out.println(ColorSensor.colorAsRGBString(ColorSensor.getColor()));
        if(ColorSensor.isNoteColor()) {
            // System.out.println("omg its a note");
        }
        //lAmp.set(this.topMotor.getOutputCurrent());
        //rAmp.set(this.bottomMotor.getOutputCurrent());
        // System.out.println(this.throughboreEncoder.getAbsoluteEncoderValue());
    }
 }


