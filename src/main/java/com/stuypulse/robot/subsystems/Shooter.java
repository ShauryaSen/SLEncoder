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
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.robot.util.SLEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter extends SubsystemBase {

    private SmartNumber targetRPM;

    private CANSparkMax motorA;
    private CANSparkMax motorB;

    // rev encoders
    private RelativeEncoder encoderMotorA;
    private RelativeEncoder encoderMotorB;

    // Stuylib encoders
    private SLEncoder slEncoderMotorA;
    private SLEncoder slEncoderMotorB;
    private SmartBoolean usingSLEncoder;

    private Controller controller;

    public Shooter(SmartBoolean usingSLEncoder) {
        this.usingSLEncoder = usingSLEncoder;
        targetRPM = new SmartNumber("Shooter/TargetRPM", 0.0);

        motorA = new CANSparkMax(SHOOTER_MOTOR, MotorType.kBrushless);
        ShooterMotorConfig.configure(motorA);
        
        motorB = new CANSparkMax(SHOOTER_FOLLOWER, MotorType.kBrushless);
        ShooterFollowerConfig.configure(motorB);

        // create SLEncoders
        slEncoderMotorA = new SLEncoder(motorA);
        slEncoderMotorB = new SLEncoder(motorB);

        // create rev encoders
        encoderMotorA = motorA.getEncoder();
        encoderMotorB = motorB.getEncoder();
                        
        controller = new PIDController(ShooterPID.kP, ShooterPID.kI, ShooterPID.kD)
            .add(new Feedforward.Flywheel(ShooterFF.kS, ShooterFF.kV, ShooterFF.kA ).velocity());
    }

    public void setTargetRPM(double RPM){
        targetRPM.set(RPM);
    }

    public double getRPM() {
        double shooterRPM;
        if (usingSLEncoder.get()) {
            shooterRPM = (slEncoderMotorA.getVelocity() + slEncoderMotorB.getVelocity()) / 2;
        } else {
            shooterRPM = (encoderMotorA.getVelocity() + encoderMotorB.getVelocity()) / 2;
        }
        return shooterRPM;
    }

    // private double getShooterVoltage() {
    //     return shooterController.update(targetRPM.get(), getShooterRPM());
    // }

    public void setVoltage(double voltage) {
        motorA.setVoltage(voltage);
        motorB.setVoltage(voltage);
    }    

    @Override
    public void periodic() {
        setVoltage(controller.update(targetRPM.get(), getRPM()));
        SmartDashboard.putNumber("Shooter/Shooter Voltage", controller.getOutput());
        SmartDashboard.putNumber("Shooter/Shooter RPM", getRPM());

    }
}