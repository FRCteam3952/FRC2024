package frc.robot.subsystems.shooter;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
/* 2 motors top down top is follower bottom is leader
 * one motor pivots the shooter
 * the other for the flap
 * 2 limit switchs 1 for each (shooter, flap)
 */

public class Shooter extends SubsystemBase {

    private final CANSparkMax followerMotor;
    private final CANSparkMax leaderMotor;

    private final RelativeEncoder followerEncoder;
    private final RelativeEncoder leaderEncoder;

    public void intializeMotors(){
        followerMotor = new CANSparkMax(PortConstants.SHOOTER_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        leaderMotor = new CANSparkMax(PortConstants.SHOOTER_LEADER_MOTOR_ID, MotorType.kBrushless);
        }
    public void resetMotors(){
        followerMotor
        leaderMotor 
    }
    public 
}
