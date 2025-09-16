package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Motor {
    MotorType type;
    SparkMax sparkMax;
    TalonFX talon;
    boolean inverted;
    idleMode idlMode = idleMode.Brake;

    public enum MotorType{
        SparkMax,
        Talon,
        Ghost // this is for motors that dont exist yet
    }
    public enum idleMode{
        Coast,
        Brake
    }

    public Motor(int motorID, MotorType type) {
        this.type = type;
        switch (type) {
            case SparkMax:
                sparkMax = new SparkMax(motorID, SparkLowLevel.MotorType.kBrushless);
            break;
            //      :)
            case Talon:
                talon = new TalonFX(motorID);
            break;
        
            default:
                break;
        }
    }
    public void set(double output) {
        switch (type) {
            case SparkMax:
                sparkMax.set(inverted?-output:output);
                break;
            case Talon:
                talon.set(inverted?-output:output);
                break;
            default:
                break;
        }
    }
    public double getVoltage() {
        switch (type) {
            case SparkMax:
                return sparkMax.get();
            case Talon:
                return talon.get();
            default:
                return 0;
        }
    }
    public void invert() {
        inverted = !inverted;
    }
    public void setNeutralMode(idleMode mode){
        try {
            switch (type) {
            case Talon:
                switch (idlMode) {
                    case Brake:
                        talon.setNeutralMode(NeutralModeValue.Brake);
                        break;
                    case Coast:
                        talon.setNeutralMode(NeutralModeValue.Coast);
                        break;
                    default:
                        break;
                }
            case SparkMax:
                switch (idlMode) {
                    case Brake:
                        SparkBaseConfig conb = new SparkMaxConfig();
                        conb.idleMode(IdleMode.kBrake);
                        sparkMax.configure(conb, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                        break;
                    case Coast:
                        SparkBaseConfig conc = new SparkMaxConfig();
                        conc.idleMode(IdleMode.kBrake);
                        sparkMax.configure(conc, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                    default:
                        break;
                }
                break;
            default:
                break;
        }
            
        } catch (Exception e) {
            // TODO: handle exception
        }
        
    }

}