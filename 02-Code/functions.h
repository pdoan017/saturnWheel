#ifndef FUNCTIONS_H
#define FUNCTIONS_H

namespace SaturnWheel
{
  // Macros

  // Linear Interpolation
  // It is assumed that x2 > x > x1
  #define INTERP(x1, x2, y1, y2, x) (((y2-y1)/(x2-x1))*(x-x1)+y1)

  // Max Value
  #define MAX(x, y) ((y>x)?y:x)

  // Min Value
  #define MIN(x, y) ((y<x)?y:x)

  // Limit Value
  // Assumes that x2 > x1
  #define LIMIT(x1, x2, y) ((y>x2)?x2:((y<x1)?x1:y))

  // Calculation of acceleration due to drag
  //
  // The drag force is determined using the equation
  // F=0.5*rho*u^2*area_ref*c_drag
  //
  // The resultant force is equivalent to F=m*a
  #define ACCEL_DRAG(rho, u, c_drag) ((0.5*rho*u^2*area_ref*c_drag)/m)
  #define COEFF_DRAG(rho, u, a)      ((m*a)/(0.5*rho*u^2*area_ref))

  class Model
  {
    static const float c_drag[2] = [0.12, 0.34]; // Table of Drag constants at closed and open positions
    static float m; // Mass of spacecraft                                                 [kg]
    static float u_air; // Velocity of surrounding air                                    [m/s]
    static float u_sc_cur; // Current velocity of spacecraft                              [m/s]
    static float u_sc_req; // Requested velocity of spacecraft                            [m/s]
    static float a_sc_cur; // Current acceleration of spacecraft                          [m/s^2]
    static float a_sc_req; // Requested acceleration of spacecraft                        [m/s^2]
    static float t_air; // Temperature of surrounding air                                 [Celsius]
    static const float area_ref = 2.356; // Reference area for drag equation              [m^2]
    static float d_shutter_act_cur; // Current position of shutter actuator semi-halves   [%]
    static float d_shutter_act_req; // Requested position of shutter actuator semi-halves [%]
    static float r_shutter_act_req; // Requested rate for shutter to open/close           [%/s]
    static float c_drag_cur; // Current drag coefficient                                  [-]
    static float c_drag_req; // Requested drag coefficient                                [-]

  public:
    void update();
    void getAtm();
    void calcCurrIter();
    void calcReq();
    void calcNextIter();
  }
}

#endif