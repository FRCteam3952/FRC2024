package frc.robot.subsystems.shooter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Flags;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.Constants.PortConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

// Shoutout to Avni whose code I reverse engineered - Fox in a box(awesome sause)

 public class ShooterSubsystem extends SubsystemBase{
    //make the motor variables
    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;
    // private final CANSparkMax pivotMotor;
    // private final CANSparkMax flapMotor;
    //make the motor encoder variables
    // private final RelativeEncoder pivotEncoder;
    // private final RelativeEncoder flapEncoder;
    private final RelativeEncoder bottomShooterEncoder;
    //make the limit switch variables
    // private final DigitalInput pivotLimitSwitch;
    // private final DigitalInput flapLimitSwitch;
    //PID stuff
    // private final SparkPIDController pivotPidController;
    // private final SparkPIDController flapPidController;
    private final SparkPIDController bottomPidController;
    
    
    private static final DoublePublisher lAmp = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("lAmp").publish();
    private static final DoublePublisher rAmp = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("rAmp").publish();


    public ShooterSubsystem(){
        // intialize motors
        topMotor = new CANSparkMax(PortConstants.SHOOTER_TOP_MOTOR_ID, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(PortConstants.SHOOTER_BOTTOM_MOTOR_ID, MotorType.kBrushless);

        topMotor.setIdleMode(IdleMode.kCoast);
        bottomMotor.setIdleMode(IdleMode.kCoast);
        // pivotMotor = new CANSparkMax(PortConstants.SHOOTER_PIVOT_MOTOR_ID, MotorType.kBrushless);
        // flapMotor = new CANSparkMax(PortConstants.SHOOTER_FLAP_MOTOR_ID, MotorType.kBrushless);
        // intialize encoders
        // pivotEncoder = pivotMotor.getEncoder();
        // flapEncoder = flapMotor.getEncoder();
        //makes topMotor follow bottomMotor
        bottomShooterEncoder = bottomMotor.getEncoder();
        topMotor.follow(bottomMotor, true);
        //intialize limit switch
        // pivotLimitSwitch = new DigitalInput(PortConstants.SHOOTER_PIVOT_LIMIT_SWITCH);
        // flapLimitSwitch = new DigitalInput(PortConstants.SHOOTER_PIVOT_LIMIT_SWITCH);

        //intialize PIDs
        // pivotPidController = pivotMotor.getPIDController();
        // flapPidController = flapMotor.getPIDController();
        bottomPidController = bottomMotor.getPIDController();
        
        /*
        //intializen PID values to 0
        pivotPidController.setP(0);
        pivotPidController.setI(0);
        pivotPidController.setD(0);
        pivotPidController.setFF(0);

        flapPidController.setP(0);
        flapPidController.setI(0);
        flapPidController.setD(0);
        flapPidController.setFF(0);*/

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
    /*/
    public void setPivotMotorSpeed(double degrees){
        pivotMotor.set(degrees);
    }
    public void setFlapMotorSpeed(double degrees){
        flapMotor.set(degrees);
    } */

    /*
    // Getting
    public double getPivotPosition(){
        return pivotEncoder.getPosition();
    }
    public double getFlapPosition(){
        return flapEncoder.getPosition();
    }
    public boolean getPivotLimitSwitch(){
        return pivotLimitSwitch.get();
    }
    public boolean getFlapLimitSwitch(){
        return flapLimitSwitch.get();
    }
    public double getPivotVelocity(){
        return pivotEncoder.getVelocity();
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
        //lAmp.set(this.topMotor.getOutputCurrent());
        //rAmp.set(this.bottomMotor.getOutputCurrent());
    }
 }


