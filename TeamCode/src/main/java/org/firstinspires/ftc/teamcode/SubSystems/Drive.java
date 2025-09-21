package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


import java.util.Locale;


// first commit to bot
public class Drive implements SubSystem {
    private Config config;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private GoBildaPinpointDriver odo = null;

    public Drive(Config cfg) {
        this.config = cfg;
    }

    public void init() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = config.hardwareMap.get(DcMotor.class, Config.LEFT_FRONT_DRIVE);
        rightFrontDrive = config.hardwareMap.get(DcMotor.class, Config.RIGHT_FRONT_DRIVE);
        leftBackDrive = config.hardwareMap.get(DcMotor.class, Config.LEFT_BACK_DRIVE);
        rightBackDrive = config.hardwareMap.get(DcMotor.class, Config.RIGHT_BACK_DRIVE);

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // DO NOT CHANGE
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // DO NOT CHANGE
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD); // DO NOT CHANGE
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD); // DO NOT CHANGE

        odo = config.hardwareMap.get(GoBildaPinpointDriver.class, Config.ODOM);
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
    }

    public void update() {

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -config.gamePad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = config.gamePad1.left_stick_x;
        double yaw = config.gamePad1.right_stick_x;
        // Take the average of the 2 trigger
        double speed = 1 - (config.gamePad1.right_trigger + config.gamePad1.left_trigger) / 2;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = (axial + yaw + lateral) * speed; // DO NOT CHANGE
        double rightFrontPower = (axial - yaw - lateral) * speed; // DO NOT CHANGE
        double leftBackPower = (axial - lateral + yaw) * speed; // DO NOT CHANGE
        double rightBackPower = (axial + lateral - yaw) * speed; // DO NOT CHANGE

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = 0.0;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels`
        leftFrontDrive.setPower(leftFrontPower*9/16);
        rightFrontDrive.setPower(rightFrontPower*9/16);
        leftBackDrive.setPower(leftBackPower*9/16);
        rightBackDrive.setPower(rightBackPower*9/16);

        // leftBackDrive.getCurrentPosition();

        // Show the elapsed game time and wheel power.
        config.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        config.telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

        odo.update();
        if (config.gamePad1.a){
            odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        }

        if (config.gamePad1.b){
            odo.recalibrateIMU(); //recalibrates the IMU without resetting position
        }

            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
//        double newTime = getRuntime();
//        double loopTime = newTime-oldTime;
//        double frequency = 1/loopTime;
//        oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        config.telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        config.telemetry.addData("Velocity", velocity);


            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
            */
        config.telemetry.addData("Status", odo.getDeviceStatus());

        config.telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
        //config.telemetry.addData("REV Hub Frequency: ", config.frequency); //prints the control system refresh rate

    }
}