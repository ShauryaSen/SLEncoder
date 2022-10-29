package com.stuypulse.robot.subsystems;

import static com.stuypulse.robot.constants.Ports.Shooter.*;
import static com.stuypulse.robot.constants.Settings.Shooter.*;
import static com.stuypulse.robot.constants.Motors.Shooter.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.robot.util.SLEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter extends SubsystemBase {

    private SmartNumber targetRPM;

    private CANSparkMax shooterMotor;
    private CANSparkMax shooterFollower;

    // rev encoders
    private RelativeEncoder shooterMotorEncoder;
    private RelativeEncoder shooterFollowerEncoder;

    // Stuylib encoders
    private SLEncoder slShooterFollowerEncoder;
    private SLEncoder slShooterMotorEncoder;
    private SmartBoolean usingSLEncoder;

    private Controller shooterController;

    public Shooter(SmartBoolean usingSLEncoder) {
        this.usingSLEncoder = usingSLEncoder;
        targetRPM = new SmartNumber("Shooter/TargetRPM", 0.0);

        shooterMotor = new CANSparkMax(SHOOTER_MOTOR, MotorType.kBrushless);
        ShooterMotorConfig.configure(shooterMotor);
        
        shooterFollower = new CANSparkMax(SHOOTER_FOLLOWER, MotorType.kBrushless);
        ShooterFollowerConfig.configure(shooterFollower);

        // create SLEncoders
        slShooterMotorEncoder = new SLEncoder(shooterMotor);
        slShooterFollowerEncoder = new SLEncoder(shooterFollower);

        // create rev encoders
        shooterMotorEncoder = shooterMotor.getEncoder();
        shooterFollowerEncoder = shooterFollower.getEncoder();
                        
        shooterController = new PIDController(ShooterPID.kP, ShooterPID.kI, ShooterPID.kD)
            .add(new Feedforward.Flywheel(ShooterFF.kS, ShooterFF.kV, ShooterFF.kA ).velocity());
    }

    public void setTargetRPM(double RPM){
        targetRPM.set(RPM);
    }

    public double getRPM() {
        double shooterRPM;
        if (usingSLEncoder.get()) {
            shooterRPM = (slShooterMotorEncoder.getVelocity() + slShooterFollowerEncoder.getVelocity()) / 2;
        } else {
            shooterRPM = (shooterMotorEncoder.getVelocity() + shooterFollowerEncoder.getVelocity()) / 2;
        }

        return shooterRPM;
    }

    // private double getShooterVoltage() {
    //     return shooterController.update(targetRPM.get(), getShooterRPM());
    // }

    public void setVoltage(double voltage) {
        shooterMotor.setVoltage(voltage);
        shooterFollower.setVoltage(voltage);
    }    

    @Override
    public void periodic() {
        setVoltage(controller.update(targetRPM.get(), getRPM());
        SmartDashboard.putNumber("Shooter/Applied Voltage", controller.getOutput());

        SmartDashboard.putNumber("Shooter/Shooter Voltage", shooterVoltage);
        SmartDashboard.putNumber("Shooter/Shooter RPM", getShooterRPM());

    }
}