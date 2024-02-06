package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Flags;
import frc.robot.util.RobotMathUtil;


import frc.robot.Constants.PortConstants;

public class IntakeSubsytem extends SubsystemBase{

    private final CANSparkMax leaderMotor;
    private final CANSparkMax followerMotor;
    private final CANSparkMax pivotMotor;


    private final RelativeEncoder leaderEncoder;
    private final RelativeEncoder followerEncoder;
    private final RelativeEncoder pivotEncoder;

    private final DigitalInput intakeLimitSwitch;


    public IntakeSubsytem(){
        leaderMotor = new CANSparkMax(PortConstants.LEADER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        followerMotor = new CANSparkMax(PortConstants.FOLLOWER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        followerMotor.setInverted(true);
        
        leaderEncoder = leaderMotor.getEncoder();
        followerEncoder = followerMotor.getEncoder();

        pivotMotor = new CANSparkMax(PortConstants.PIVOT_INTAKE_MOTOR_ID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();

        intakeLimitSwitch = new DigitalInput(0); //change this stupid AVNI

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
}
