#pragma once

struct PID_Coefficients {

    // Constructor to initialize all values
    PID_Coefficients(double kF, double kP, double kI, double kD);

    double kF;
    double kP;
    double kI;
    double kD;

};
