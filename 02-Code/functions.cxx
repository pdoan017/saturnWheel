#include functions.h

using namespace SaturnWheel;

Model::Model() : dt(0.5), m(1.0), totTime(0)
{
}

Model::~Model()
{
}

// Calculation of air density
float density(float t)
{
  // Values from source
}

// Update surrounding atmospheric values and execute functionalities
void Model::update()
{
  // Update with new values for start of time step
  getAtm();
  calcCurrIter();

  // Calculate new target values, commanded internally or externally
  calcReq();

  // Calculate values for next time iteration
  calcNextIter();
}

// Get atmospheric values and update atmospheric variables for this time iteration
void Model::getAtm()
{
  u_air = getU();
  t_air = getTemp();
  rho   = density(temp);
}

// Calculates velocity and acceleration of spacecraft
void Model::calcCurrIter()
{
  totTime += dt;
  m = getMass();
}

// Calculations of requested variables
void Model::calcReq()
{
  static const float t_vel_change = 1.0; // Tune for acceleration extremeties

  // If requested from user, use inputs from user rather than calculated inputs
  if( req_vel_cntr_prev > req_vel_cntr )
  {
    u_sc_req = user_input_vel;
  }
  else
  {
    u_sc_req = u_sc_cur;
  }

  a_sc_req = (u_sc_req - u_sc_cur) / t_vel_change;

  // Drag forces
  c_drag_req = LIMIT(c_drag[0], c_drag[1], calcDragCoeff(rho, u_sc_req, a_sc_req));
  d_shutter_act_req = INTERP(c_drag[0], c_drag[1], 0.0, 1.0, c_drag_req);
}

// Calculations for next iteration
void Model::calcNextIter()
{
  static const float r_shutter_act = 0.05; // Rate of speed shutter opens/closes [%/s]

  u_sc_cur = a_sc_cur * dt;
  a_sc_cur = a_sc_req;
  c_drag_cur = calcDragCoeff(rho, u_sc_cur, a_sc_cur);

  if( d_shutter_act_req > d_shutter_act_cur)
  {
    d_shutter_act_cur = MIN(d_shutter_act_req, d_shutter_act_cur+r_shutter_act*dt);
  }
  else
  {
    d_shutter_act_cur = MAX(d_shutter_act_req, d_shutter_act_cur-r_shutter_act*dt);
  }
}