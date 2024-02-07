package frc.robot.subsystems.shooter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import com.revrobotics.RelativeEncoder;

// Shoutout to Avni whose code I reverse engineered - Fox in a box(awesome sause)

 public class ShooterSubsystem extends SubsystemBase{
// this function should intialize things and set them to something
    //make the motor variables
    private CANSparkMax followerMotor;
    private CANSparkMax leaderMotor;
    private CANSparkMax pivotMotor;
    private CANSparkMax flapMotor;

    //make the motor encoder variables
    private RelativeEncoder followerEncoder;
    private RelativeEncoder leaderEncoder;
    private RelativeEncoder pivotEncoder;
    private RelativeEncoder flapEncoder;

    //make the limit switch variables
    private DigitalInput pivotLimitSwitch;
    private DigitalInput flapLimitSwitch;


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

        followerMotor.follow(leaderMotor);

        //intialize limit switch
        pivotLimitSwitch = new DigitalInput(PortConstants.SHOOTER_PIVOT_LIMIT_SWITCH);
        flapLimitSwitch = new DigitalInput(PortConstants.SHOOTER_PIVOT_LIMIT_SWITCH);
        }


        // Setting the speed of the motors I have no idea how to set the speed. So...
        public void setLeaderMotorSpeed(double speed){
            leaderMotor.set(speed); 
        }
        public void setPivotMotorSpeed(double pivotSpeed){
            pivotMotor.set(pivotSpeed);
        }
        public void setFlapMotorSpeed(double flapSpeed){
            flapMotor.set(flapSpeed);
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

        // Setting 
        public void setPivotPosition(double position){
            pivotEncoder.setPosition(position);
        }
        public void setFlapPosition(double position){
            flapEncoder.setPosition(position);
        }
        // Resetting 
        public void resetPivot(){
            pivotEncoder.setPosition(0.0);
        }
        public void resetFlap(){
            flapEncoder.setPosition(0.0);
        }
 }


