package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Launcher implements SubSystem {

    private final Config config;
    private DcMotor launchMotor;

    public Launcher(Config cfg){
        this.config = cfg;
    }

    @Override
    public void init() {
        launchMotor = config.hardwareMap.get(DcMotor.class, Config.SLIDE_MOTOR);
        // Reset the encoder and set it to be in RUN_TO_POSITION
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void update() {
        double slidepower = config.gamePad2.right_stick_y;  // Note: pushing stick forward gives negative value
        launchMotor.setPower(slidepower*2/3.5);
    }
}