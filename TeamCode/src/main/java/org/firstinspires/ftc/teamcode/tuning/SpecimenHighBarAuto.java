package org.firstinspires.ftc.teamcode;

// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@Autonomous(name = "Specimen Auto test", group = "Autonomous")
public class SpecimenHighBarAuto extends LinearOpMode {

    public class Lift {
        private DcMotorEx lift;
        private DcMotorEx slide;
        private CRServo cervo;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, org.firstinspires.ftc.teamcode.Config.ARM_MOTOR);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            slide = hardwareMap.get(DcMotorEx.class, org.firstinspires.ftc.teamcode.Config.SLIDE_MOTOR);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setDirection(DcMotorSimple.Direction.REVERSE);
            cervo = hardwareMap.get(CRServo.class, org.firstinspires.ftc.teamcode.Config.INTAKE);
            cervo.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3900) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.3);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 5.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
        public class slideOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slide.setPower(0.8);
                    initialized = true;
                }

                double pos = slide.getCurrentPosition();
                packet.put("slidePos", pos);
                if (pos < 750) {
                    return true;
                } else {
                    slide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideOut(){
            return new slideOut();
        }

        public class slideIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slide.setPower(-0.8);
                    initialized = true;
                }

                double pos = slide.getCurrentPosition();
                packet.put("slidePos", pos);
                if (pos > -50) {
                    return true;
                } else {
                    slide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideIn(){
            return new slideIn();
        }
        public class spitOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    cervo.setPower(0.8);
                    initialized = true;
                }

                double pos = 1;
                packet.put("slidePos", pos);
                if (pos < 7000) {
                    return true;
                } else {
                    cervo.setPower(0);
                    return false;
                }
            }
        }
        public Action spitOut(){
            return new spitOut();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, 20))
        //.strafeTo(new Vector2d(-18, 0))
        ;
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(6, 0))
                //.strafeTo(new Vector2d(-18, 0));
;
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        lift.liftUp(),
                        lift.slideOut(),
                        tab1.build(),
                        lift.liftDown(),
                        lift.slideIn(),
                        lift.spitOut()

                )
        );

    }
}




