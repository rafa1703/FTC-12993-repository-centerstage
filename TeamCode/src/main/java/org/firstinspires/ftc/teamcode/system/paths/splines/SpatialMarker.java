package org.firstinspires.ftc.teamcode.system.paths.splines;

import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;

public class SpatialMarker
{
    Callback callback;
    Pose spatialPoint;
    double t, u;
    public SpatialMarker(Pose pose, Callback callback)
    {
        this.callback = callback;
        spatialPoint = pose;
    }

}

