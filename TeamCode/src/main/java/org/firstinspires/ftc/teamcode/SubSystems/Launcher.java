package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Launcher implements SubSystem {

    private final Config config;
    private DcMotor launchMotor1;
    private DcMotor launchMotor2;

    public Launcher(Config cfg){
        this.config = cfg;
    }

    @Override
    public void init() {
        launchMotor1 = config.hardwareMap.get(DcMotor.class, Config.LAUNCH_MOTOR);
        // Reset the encoder and set it to be in RUN_TO_POSITION
        launchMotor1.setDirection(DcMotor.Direction.FORWARD);
        launchMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launchMotor2 = config.hardwareMap.get(DcMotor.class, Config.LAUNCH_MOTOR2);
        // Reset the encoder and set it to be in RUN_TO_POSITION
        launchMotor2.setDirection(DcMotor.Direction.FORWARD);
        launchMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void update() {
        if (config.gamePad1.a) {
            launchMotor1.setPower(1.0);
        }
        else if (config.gamePad1.b) {
            launchMotor1.setPower(-1.0);
        }
        //launchMotor2.setPower(launchpower*-2/3.5);
    }
}