#include <iostream>
#include <fstream>
#include <sstream>
//#include <bits/stdc++.h>
#include <vector>

using namespace std;

int main(){
    int Ndel;
    cout << "Number of values to leave at the start... ";
    cin >> Ndel;
    cout.flush();

    int N;
    cout << "Number of values to take... ";
    cin >> N;
    cout.flush();

    double *Vx = new double[N], *Vy = new double[N];
    double *x = new double[N], *y = new double[N];
    double A00, A01, A10, A11;

    double Sum_Vx;
    double Sum_VxVx;
    double Sum_Vy;
    double Sum_VyVy;
    double Sum_x;
    double Sum_y;
    double Sum_xVx;
    double Sum_yVx;
    double Sum_xVy;
    double Sum_yVy;

    string Line, Value;
    stringstream Row;
    ifstream VxOnly("XVoltageOnly.csv");
    ifstream VyOnly("YVoltageOnly.csv");
    cout << "XVoltageOnly.csv" << " is loaded" << endl;
    cout << "YVoltageOnly.csv" << " is loaded" << endl;

    Sum_Vx = 0;
    Sum_VxVx = 0;
    Sum_Vy = 0;
    Sum_VyVy = 0;
    Sum_x = 0;
    Sum_y = 0;
    Sum_xVx = 0;
    Sum_yVx = 0;
    Sum_xVy = 0;
    Sum_yVy = 0;

    for(int i=0; i<Ndel; i++){
        getline(VxOnly, Line, '\n');
    }
    for(int i=0; i<N; i++) {
        // Read a line
        getline(VxOnly, Line, '\n');
        Row = stringstream(Line);
        // Read values from line
        getline(Row, Value, ',');
        Vx[i] = stod(Value);
        getline(Row, Value, ',');
        // Vy[i] = stod(Value);
        getline(Row, Value, ',');
        x[i] = stod(Value);
        getline(Row, Value, ',');
        y[i] = stod(Value);

        // Sums
        Sum_Vx += Vx[i];
        Sum_VxVx += Vx[i]*Vx[i];
        Sum_x += x[i];
        Sum_xVx += x[i]*Vx[i];
        Sum_y += y[i];
        Sum_yVx += y[i]*Vx[i] ;
    }
    double Slope_xVx = (Sum_x*Sum_Vx-N*Sum_xVx)/(Sum_Vx*Sum_Vx-N*Sum_VxVx);
    double Slope_yVx = (Sum_y*Sum_Vx-N*Sum_yVx)/(Sum_Vx*Sum_Vx-N*Sum_VxVx);
    cout << "Slope of XPixShift vs. XVoltage is : " << Slope_xVx << endl;
    cout << "Slope of YPixShift vs. XVoltage is : " << Slope_yVx << endl;

    Sum_Vx = 0;
    Sum_VxVx = 0;
    Sum_Vy = 0;
    Sum_VyVy = 0;
    Sum_x = 0;
    Sum_y = 0;
    Sum_xVx = 0;
    Sum_yVx = 0;
    Sum_xVy = 0;
    Sum_yVy = 0;

    for(int i=0; i<Ndel; i++){
        getline(VyOnly, Line, '\n');
    }
    for(int i=0; i<N; i++) {
        // Read a line
        getline(VyOnly, Line, '\n');
        Row = stringstream(Line);
        // Read values from line
        getline(Row, Value, ',');
        // Vx[i] = stod(Value);
        getline(Row, Value, ',');
        Vy[i] = stod(Value);
        getline(Row, Value, ',');
        x[i] = stod(Value);
        getline(Row, Value, ',');
        y[i] = stod(Value);
        // Sums
        Sum_Vy += Vy[i];
        Sum_VyVy += Vy[i]*Vy[i];
        Sum_x += x[i];
        Sum_xVy += x[i]*Vy[i];
        Sum_y += y[i];
        Sum_yVy += y[i]*Vy[i] ;
    }
    double Slope_xVy = (Sum_x*Sum_Vy-N*Sum_xVy)/(Sum_Vy*Sum_Vy-N*Sum_VyVy);
    double Slope_yVy = (Sum_y*Sum_Vy-N*Sum_yVy)/(Sum_Vy*Sum_Vy-N*Sum_VyVy);
    cout << "Slope of XPixShift vs. YVoltage is : " << Slope_xVy << endl;
    cout << "Slope of YPixShift vs. YVoltage is : " << Slope_yVy << endl;

    double idetCM = 1.0/(Slope_xVx*Slope_yVy - Slope_xVy*Slope_yVx);
    A00 = idetCM*Slope_yVy;
    A01 = -idetCM*Slope_xVy;
    A10 = -idetCM*Slope_yVx;
    A11 = idetCM*Slope_xVx;

    ofstream cm("CalibrationMatrix.csv");
    cm << A00 << "," << A10 << "," << A01 << "," << A11 << endl;
    cm << Slope_xVx << "," << Slope_yVx << "," << Slope_xVy << "," << Slope_yVy << endl;
    cm << "0.0, 0.0, 5.0" << endl; // Default Vxoffset, Vyoffset, Slew rate,
    cm << "0.5, 0.5, 0.0" << endl; // Kp, Kd and Ki
    cm << "300, 5" << endl; // Ni, nd
    cm << "3, 5" << endl; // COM ports
    cm.close();

    ifstream calmat("CalibrationMatrix.csv");
    double CM[4];

    getline(calmat, Line, '\n');
    Row = stringstream(Line);
    getline(Row, Value, ',');
    CM[0] = stod(Value);
    getline(Row, Value, ',');
    CM[1] = stod(Value);
    getline(Row, Value, ',');
    CM[2] = stod(Value);
    getline(Row, Value, ',');
    CM[3] = stod(Value);
    calmat.close();
    cout << "Calibration matrix : " << CM[0] << "," << CM[1] << "," << CM[2] << "," << CM[3] << endl;

    return 0;
}