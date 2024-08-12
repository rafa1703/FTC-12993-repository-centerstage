package org.firstinspires.ftc.teamcode.system.paths.splines;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.normalizeRadians;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;
import org.opencv.core.Point;

public class GVFLogic
{
    public boolean followTangentially = false;
    public boolean reverse = false;
    @NonNull
    public Vector calculate(BelzierCurve curve, Pose pose)
    {
        // correction is like the go back to the fucking line and then path shit is follow the bitch spline
        Point robot = pose.toPoint();
        Vector endPoint = curve.getEndPoint();
        Vector closestPoint = curve.returnClosestPointOnCurve(robot); //TODO: there are two full loop iterations of all points in the curve
        double t = curve.returnClosesT(robot);
        Vector derivative = curve.getTangentialVector(t);
        Vector robotToPoint = new Vector(closestPoint.getX() - robot.x, closestPoint.getY() - robot.y);
        Vector robotToEnd = endPoint.subtract(robot);

        double CORRECTION_DIS = 30; // tune
        double SAVING_THROW_DIS = 24;
        // this values are like for a visualizer so like robot smaller ig, this is effectively 10 because magSqr

        double directPursuitThreshold = 1;

        // TODO: go from above or use O(log2) alg
        for(double i = 0; i <= 1; i += 1 / curve.getInterval()) // okay so this is like at the point lets iterate trough all points and find out if the distance of the point to the end point if its lower than the threshold
        {
            double dist = endPoint.subtract(curve.parametric(i)).getMagnitude();
            // if we are closer than the threshold we can start dirctly following the curve
            if (dist < SAVING_THROW_DIS)
            {
                directPursuitThreshold = i;
                break;
            }
        }

        double correctionScale = Math.min(1, robotToPoint.getMagnitude() / CORRECTION_DIS);
        double direction = headingInterpolation(derivative.getAngle(), robotToPoint.getAngle(), correctionScale);

        Vector tempRobot = new Vector(robot.x, robot.y);
        // this just say if the closest point and the robot is tangent or t is greater than the follow the path threshold we follow the last point
        if ((t == 1 && Math.abs(tempRobot.subtract(closestPoint).getAngle() - derivative.getAngle()) <= 0.5 * Math.PI) //TODO: is the first condition necessary?
                || t >= directPursuitThreshold)
        {
            direction = endPoint.subtract(robot).getAngle();
        }

        Vector movementVector = new Vector(Math.cos(direction), Math.sin(direction));
        double speed = 1;
        if (robotToEnd.getMagnitude() < 34) // this value
        {
            // like a weighted average for the speed
            speed = interpolation(0.15, speed, robotToEnd.getMagnitude() / 34);
        }
        movementVector.scaleBy(speed);

        if (followTangentially) // reverse only affects if we following tangentially so it can be nested here
        {
            if(t == 1)
            {
                derivative = curve.getTangentialVector(0.99); // this is because at 1 the derivative is NaN
            }
                double angle = derivative.getAngle();
                if (reverse) angle += Math.toRadians(180);
                double headingScale = Math.abs(Math.min(normalizeRadians(angle - pose.getHeading()) / Math.toRadians(70), 1)); // rn i think 70 is the best angle to consider max thing
                double headingDiff = headingInterpolation(pose.getHeading(), angle, headingScale) - pose.getHeading();  //Math.min(normalizeRadians(derivative.getAngle() - pose.getHeading()) / Math.toRadians(30), 1); // Who the fuck knows if this is gonna work
                // we want to remove the current heading because interpolation at t = 0 returns the current heading
                movementVector = new Vector(movementVector.getX(), movementVector.getY(), headingDiff);

        }
        return movementVector;

    }

    private double headingInterpolation(double theta1, double theta2, double t)
    {
        //Normalize radians with a scaled (t)
        double diff = theta2 - theta1;
        diff %= 2 * Math.PI;
        if (Math.abs(diff) > Math.PI) {
            if (diff > 0) {
                diff -= 2 * Math.PI;
            } else {
                diff += 2 * Math.PI;
            }
        }
        return theta1 + t * diff;
    }

    private double interpolation(double p1, double p2, double t) {
        return (1 - t) * p1 + t * p2;
    }


    private double binomalSearch(BelzierCurve curve, Point point, double start, double end)
    {
        Vector robot = new Vector(point);
        double middle = (start + end) / 2;

        //Point p = curve.parametric(middle);
        Point startPoint = curve.parametric(start);
        Point endPoint = curve.parametric(end);
        //Vector robotToPoint = new Vector(p.x - robot.getX(), p.y - robot.getY());
        Vector robotToEnd = new Vector(endPoint.x - robot.getX(), endPoint.y - robot.getY());
        Vector robotToStart = new Vector(startPoint.x - robot.getX(), startPoint.y - robot.getY());

        if (robotToStart.getMagSqr() < robotToEnd.getMagSqr())
        {
            return binomalSearch(curve, point, start, middle);
        }
        else if (robotToStart.getMagSqr() > robotToEnd.getMagSqr())
        {
            return binomalSearch(curve, point, middle, end);
        }
        else return middle;
        //if ()

    }

    public void setFollowTangentially(boolean followTangentially)
    {
        this.followTangentially = followTangentially;
    }

    public void setReverse(boolean reverse)
    {
        this.reverse = reverse;
    }

    public boolean isFollowTangentially()
    {
        return followTangentially;
    }

    public boolean isReverse()
    {
        return reverse;
    }
}
