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
    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;
    private final CANSparkMax pivotMotor;
    private final CANSparkMax flapMotor;
    //make the motor encoder variables
    private final RelativeEncoder topEncoder; //Don't know what to do with this, but its there
    private final RelativeEncoder bottomEncoder;
    private final RelativeEncoder pivotEncoder;
    private final RelativeEncoder flapEncoder;
    //make the limit switch variables
    private final DigitalInput pivotLimitSwitch;
    private final DigitalInput flapLimitSwitch;
    //PID stuff
    private static SparkPIDController pivotPidController;
    private static SparkPIDController flapPidController;
    private static SparkPIDController bottomPidController;
    


    public ShooterSubsystem(){
        // intialize motors
        topMotor = new CANSparkMax(PortConstants.SHOOTER_TOP_MOTOR_ID, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(PortConstants.SHOOTER_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(PortConstants.SHOOTER_PIVOT_MOTOR_ID, MotorType.kBrushless);
        flapMotor = new CANSparkMax(PortConstants.SHOOTER_FLAP_MOTOR_ID, MotorType.kBrushless);
        // intialize encoders
        topEncoder = topMotor.getEncoder();
        bottomEncoder = bottomMotor.getEncoder();
        pivotEncoder = pivotMotor.getEncoder();
        flapEncoder = flapMotor.getEncoder();
        //makes topMotor follow bottomMotor
        topMotor.follow(bottomMotor);
        //intialize limit switch
        pivotLimitSwitch = new DigitalInput(PortConstants.SHOOTER_PIVOT_LIMIT_SWITCH);
        flapLimitSwitch = new DigitalInput(PortConstants.SHOOTER_PIVOT_LIMIT_SWITCH);

        //intialize PIDs
        pivotPidController = pivotMotor.getPIDController();
        flapPidController = flapMotor.getPIDController();
        bottomPidController = bottomMotor.getPIDController();

        //intializen PID values to 0
        pivotPidController.setP(0);
        pivotPidController.setI(0);
        pivotPidController.setD(0);
        pivotPidController.setFF(0);

        flapPidController.setP(0);
        flapPidController.setI(0);
        flapPidController.setD(0);
        flapPidController.setFF(0);

        bottomPidController.setP(0);
        bottomPidController.setI(0);
        bottomPidController.setD(0);
        bottomPidController.setFF(0);
    }

        // Setting the speed of the motors
        public void setBottomMotorSpeed(double rpm){
            bottomMotor.set(rpm); 
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
        public void setMotorPid(double rpm){
            bottomPidController.setP(rpm);
            bottomPidController.setI(rpm);
            bottomPidController.setD(rpm);
            bottomPidController.setFF(rpm);
    }
        // Resetting 
        public void resetPivot(){
            pivotEncoder.setPosition(0.0);
    }
        public void resetFlap(){
            flapEncoder.setPosition(0.0);
    }
 }


