package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Config;

// RR-specific imports

//import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

//@Config
@Autonomous(name = "Move 3 Centimeters Left Auto", group = "Autonomous")
public class Move3CentimetersLeftAuto extends LinearOpMode {

    public void runOpMode() {

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose, telemetry);
        drive.pinpoint.resetPosAndIMU();
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-3, 0))
                //.strafeTo(new Vector2d(-18, 0))
                ;
        Pose2D pos = drive.pinpoint.getPosition();
        Config config = new Config(telemetry, hardwareMap, gamepad1, gamepad2);

//        config.telemetry.addData("Position", data);
//        config.telemetry.update();


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        tab1.build()
                )
        );

    }
}