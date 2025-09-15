package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robot {
    public HardwareMap m_hwMap = null;

    public Intake intake = null;

    public Output output = null;

    public MecanumDrive driveTrain = null;

    public Robot(HardwareMap hardwareMap, Pose2d initialPose) {

        intake = new Intake(hardwareMap);

        output = new Output(hardwareMap);

        m_hwMap = hardwareMap;
    }
        /**
         * expDrive() sets up exponential driving for teleOp.
         */
        double joyDead = 0.01;         // joystick range in which movement is considered accidental
        double motorMin = 0.05;         // minimum drive motor power

        public double expDrive(double joyVal, double maxVel) {
            if (Math.abs(joyVal) > joyDead) {
                double mOut = ((joyVal * joyVal) * (maxVel - motorMin)) / (1.0 - joyDead);
                double finalOutput = Math.copySign(mOut, joyVal);
                return finalOutput;
            }
            return 0;
        }

        private double offset = 0;

        public double getHeading() {
            double rawHeading = driveTrain.getRawExternalHeading();
            double heading = rawHeading - offset;
            while (heading < 0)
                heading = heading + (2 * Math.PI);
            while (heading > (2 * Math.PI))
                heading = heading - (2 * Math.PI);
            return heading;
        }

        public double resetHeading(double heading) {
            offset = driveTrain.getRawExternalHeading() + heading;

            return offset;
    }
}
