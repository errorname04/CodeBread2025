package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RealIntake implements SubSystem {

    private final Config config;
    private DcMotor intakeMotor;

    public RealIntake(Config cfg){
        this.config = cfg;
    }

    @Override
    public void init() {
        intakeMotor = config.hardwareMap.get(DcMotor.class, Config.INTAKE_MOTOR);
        // Reset the encoder and set it to be in RUN_TO_POSITION
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void update() {
        if (config.gamePad2.left_stick_y>0.05) {
            intakeMotor.setPower(config.gamePad2.left_stick_y);
        }
        else if (config.gamePad2.left_stick_y<-0.05) {
            intakeMotor.setPower(config.gamePad2.left_stick_y);
        }
        else if (config.gamePad2.a){
            intakeMotor.setPower(0);
        }
        else{
            intakeMotor.setPower(0);
        }
    }
}