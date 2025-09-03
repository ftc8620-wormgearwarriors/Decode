package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public HardwareMap m_hwMap = null;

    public Intake intake = null;

    public Output output = null;

    public Robot(HardwareMap hardwareMap, Pose2d initialPose){

        intake = new Intake(hardwareMap);

        output = new Output(hardwareMap);

        m_hwMap = hardwareMap;
    }
}
