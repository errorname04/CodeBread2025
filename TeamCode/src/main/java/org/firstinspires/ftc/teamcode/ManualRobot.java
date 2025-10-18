package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.SubSystems.Drive;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.RealIntake;
import org.firstinspires.ftc.teamcode.SubSystems.SubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.Launcher;

import java.util.LinkedList;
import java.util.List;
enum state {
    IDLE,
    DRIVING,
    SHOOTING
};
public class ManualRobot {
    // Config class to all hardware controls
    Config config;
    List<SubSystem> subSystems = new LinkedList<SubSystem>();
    state curstate = state.IDLE;

    // Constructor
    public ManualRobot(Config cfg) {
        config = cfg;

        subSystems.add(new Drive(config));
        subSystems.add(new Launcher(config));
        subSystems.add(new Intake(config));
        subSystems.add(new RealIntake(config));
//        subSystems.add(new Arm(config));
//        subSystems.add(new Intake(config));
    }

    //
    public void init() {
        for (SubSystem sub : subSystems) {
            sub.init();
        }
    }

    public void update() {
        for (SubSystem sub : subSystems) {
            sub.update();
        }
    }
}

// this file is to create connect all of the subsystems, so that way in the update loop, we can update each subsytem with one function
