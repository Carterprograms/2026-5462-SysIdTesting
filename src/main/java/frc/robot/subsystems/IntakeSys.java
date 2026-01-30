package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.CANDevices;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;


public class IntakeSys extends SubsystemBase {

    private final SparkFlex rollerMtr;
    private final RelativeEncoder rollerEnc;
    private final SparkClosedLoopController rollerController;

    public IntakeSys() {
        
        rollerMtr = new SparkFlex(CANDevices.rollerMtrId, MotorType.kBrushless);
        rollerEnc = rollerMtr.getEncoder();
        rollerController = rollerMtr.getClosedLoopController();

        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(RollerConstants.stallLimitAmps, RollerConstants.freeLimitAmps, RollerConstants.maxRPM);
        rollerConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        rollerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(RollerConstants.rollerkP, RollerConstants.rollerkI, RollerConstants.rollerkD);

        rollerMtr.configure(rollerConfig, ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

    }

    public void setRollerRPM(double rpm) {
        rollerController.setReference(rpm, ControlType.kVelocity);
    }

    public double getRollerRPM() {
        return rollerEnc.getVelocity();
    }

    public void stop() {
        rollerController.setReference(0, ControlType.kVelocity);
    }
    
}
