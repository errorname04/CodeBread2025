package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake implements SubSystem {

    private final Config config;
    private CRServo test_servo;

    public Intake(Config cfg){
        this.config = cfg;
    }

    @Override
    public void init() {
        test_servo = config.hardwareMap.get(CRServo.class, Config.INTAKE);
        // Reset the encoder and set it to be in RUN_TO_POSITION
        test_servo.setDirection(CRServo.Direction.FORWARD);
    }

    public void update() {
        if (config.gamePad1.right_bumper) {
            test_servo.setPower(1.0);
            System.out.println("running forwards");
        }
        else if (config.gamePad1.left_bumper) {
            test_servo.setPower(-1.0);
            System.out.println("running backwards");
        }
        else {
            test_servo.setPower(0.0);
        }
    }
}
