package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class IntakeSubsytem extends SubsystemBase{

    private final CANSparkMax leaderMotor;
    private final CANSparkMax followerMotor;
    private final CANSparkMax pivotMotor;


    private final RelativeEncoder leaderEncoder;
    private final RelativeEncoder followerEncoder;
    private final RelativeEncoder pivotEncoder;

    private final DigitalInput intakeLimitSwitch;

    private final SparkPIDController pivotPIDController;


    public IntakeSubsytem(){
        leaderMotor = new CANSparkMax(PortConstants.LEADER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        followerMotor = new CANSparkMax(PortConstants.FOLLOWER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        followerMotor.setInverted(true);
        
        leaderEncoder = leaderMotor.getEncoder();
        followerEncoder = followerMotor.getEncoder();

        pivotMotor = new CANSparkMax(PortConstants.PIVOT_INTAKE_MOTOR_ID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();

        intakeLimitSwitch = new DigitalInput(PortConstants.INTAKE_LIMIT_SWITCH_CHANNEL);

        pivotPIDController = pivotMotor.getPIDController();
        //tune these later
        //also addison what do you mean by make pivotvpidcontroller a class variable on cansparkmax??? 
        //ik what a class variable is...im pretty sure at lest
        pivotPIDController.setP(0);
        pivotPIDController.setI(0);
        pivotPIDController.setD(0);
        pivotPIDController.setFF(0);
        

    }


    public void setLeaderSpeed(double leaderSpeed){
        leaderMotor.set(leaderSpeed);
    }

    public double getPivotPosition(){
        return pivotEncoder.getPosition();
    }

    public void setPivotSpeed(double pivotSpeed){
        pivotMotor.set(pivotSpeed);
    }
   
    public void resetPivotEncoder(){
        setPivotSpeed(0.0);
    }

    public boolean getLimitSwitchPosition(){
        return intakeLimitSwitch.get();
    }

    public double getLeaderMotorCurrent(){
        return leaderMotor.getOutputCurrent();
    }

    public double getFollowerMotorCurrent(){
        return followerMotor.getOutputCurrent();
    }

    public double getPivotMotorCurrent(){
        return pivotMotor.getOutputCurrent();
    }

    public void setPivotPIDController(double pivotAngle){
        pivotPIDController.setReference(pivotAngle, ControlType.kPosition);
    }
}
