package frc.robot.subsystems.shooter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

// Shoutout to Avni whose code I reverse engineered - Fox in a box(awesome sause)

 public class ShooterSubsystem extends SubsystemBase{
    //make the motor variables
    private final CANSparkMax followerMotor;
    private final CANSparkMax leaderMotor;
    private final CANSparkMax pivotMotor;
    private final CANSparkMax flapMotor;
    //make the motor encoder variables
    private final RelativeEncoder followerEncoder; //Don't know what to do with this, but its there
    private final RelativeEncoder leaderEncoder;
    private final RelativeEncoder pivotEncoder;
    private final RelativeEncoder flapEncoder;
    //make the limit switch variables
    private final DigitalInput pivotLimitSwitch;
    private final DigitalInput flapLimitSwitch;
    //PID stuff
    private static SparkPIDController pivotPidController;
    private static SparkPIDController flapPidController;
    private static SparkPIDController leaderPidController;
    


    public ShooterSubsystem(){
        // intialize motors
        followerMotor = new CANSparkMax(PortConstants.SHOOTER_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        leaderMotor = new CANSparkMax(PortConstants.SHOOTER_LEADER_MOTOR_ID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(PortConstants.SHOOTER_PIVOT_MOTOR_ID, MotorType.kBrushless);
        flapMotor = new CANSparkMax(PortConstants.SHOOTER_FLAP_MOTOR_ID, MotorType.kBrushless);
        // intialize encoders
        followerEncoder = followerMotor.getEncoder();
        leaderEncoder = leaderMotor.getEncoder();
        pivotEncoder = pivotMotor.getEncoder();
        flapEncoder = flapMotor.getEncoder();
        //makes followMotor follow leaderMotor
        followerMotor.follow(leaderMotor);
        //intialize limit switch
        pivotLimitSwitch = new DigitalInput(PortConstants.SHOOTER_PIVOT_LIMIT_SWITCH);
        flapLimitSwitch = new DigitalInput(PortConstants.SHOOTER_PIVOT_LIMIT_SWITCH);

        //intialize PIDs
        pivotPidController = pivotMotor.getPIDController();
        flapPidController = flapMotor.getPIDController();
        leaderPidController = leaderMotor.getPIDController();

        //intializen PID values to 0
        pivotPidController.setP(0);
        pivotPidController.setI(0);
        pivotPidController.setD(0);
        pivotPidController.setFF(0);

        flapPidController.setP(0);
        flapPidController.setI(0);
        flapPidController.setD(0);
        flapPidController.setFF(0);

        leaderPidController.setP(0);
        leaderPidController.setI(0);
        leaderPidController.setD(0);
        leaderPidController.setFF(0);
    
        }

        // Setting the speed of the motors
        public void setLeaderMotorSpeed(double RPM){
            leaderMotor.set(RPM); 
        }
        public void setPivotMotorSpeed(double degrees){
            pivotMotor.set(degrees);
        }
        public void setFlapMotorSpeed(double degrees){
            flapMotor.set(degrees);
        }

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
        public void setPivotPosition(double position){
            pivotEncoder.setPosition(position);
        }
        public void setFlapPosition(double position){
            flapEncoder.setPosition(position);
        }
        public void setPivotPid(double degree){
            pivotPidController.setP(degree);
            pivotPidController.setI(degree);
            pivotPidController.setD(degree);
            pivotPidController.setFF(degree);
        }
        public void setFlapPid(double degree){
            flapPidController.setP(degree);
            flapPidController.setI(degree);
            flapPidController.setD(degree);
            flapPidController.setFF(degree);
        }
        public void setMotorPid(double RPM){
            leaderPidController.setP(RPM);
            leaderPidController.setI(RPM);
            leaderPidController.setD(RPM);
            leaderPidController.setFF(RPM);
        }
        // Resetting 
        public void resetPivot(){
            pivotEncoder.setPosition(0.0);
        }
        public void resetFlap(){
            flapEncoder.setPosition(0.0);
        }
 }


