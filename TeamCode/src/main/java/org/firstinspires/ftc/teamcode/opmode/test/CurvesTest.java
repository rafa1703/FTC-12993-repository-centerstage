package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;
import org.firstinspires.ftc.teamcode.system.paths.splines.BezierCurve;
import org.opencv.core.Point;

import java.util.ArrayList;

@Autonomous
public class CurvesTest extends LinearOpMode
{
    TelemetryPacket packet;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    BezierCurve curve = new BezierCurve(
            new Point[]{
                    new Point(0, 0),
                    new Point(-10, 20),
                    new Point(20, 20)
            });

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        ArrayList<Point> curvePoints = curve.returnCurve();
        ArrayList<Point> derivedPoints = curve.returnDerivedCurve();
        ArrayList<Vector> headingPoints = curve.returnTangentialVectors();
        waitForStart();
        while(opModeIsActive())
        {
            packet = new TelemetryPacket();

            for (Point point: curvePoints)
            {
                //telemetry.addData("POINT", point);
                //packet.fieldOverlay().setFill("black").fillCircle(point.x, point.y, 1);
            }
            for (Point point: derivedPoints)
            {
                //telemetry.addData("POINT", point);
                //packet.fieldOverlay().setFill("red").fillCircle(point.x, point.y, 1);
            }
            for (Vector point: headingPoints)
            {
                //telemetry.addData("Heading", point * 180/Math.PI);
                telemetry.addData("Heading", point);
            }


            //dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
