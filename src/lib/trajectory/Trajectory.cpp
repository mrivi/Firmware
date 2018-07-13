#include "Trajectory.hpp"

//Two simple helper function to make testing easier
const char* GetInputFeasibilityResultName(RapidTrajectoryGenerator::InputFeasibilityResult fr)
{
  switch(fr)
  {
  case RapidTrajectoryGenerator::InputFeasible:             return "Feasible";
  case RapidTrajectoryGenerator::InputIndeterminable:       return "Indeterminable";
  case RapidTrajectoryGenerator::InputInfeasibleThrustHigh: return "InfeasibleThrustHigh";
  case RapidTrajectoryGenerator::InputInfeasibleThrustLow:  return "InfeasibleThrustLow";
  case RapidTrajectoryGenerator::InputInfeasibleRates:      return "InfeasibleRates";
  }
  return "Unknown!";
};

const char* GetStateFeasibilityResultName(RapidTrajectoryGenerator::StateFeasibilityResult fr)
{
  switch(fr)
  {
  case RapidTrajectoryGenerator::StateFeasible:   return "Feasible";
  case RapidTrajectoryGenerator::StateInfeasible: return "Infeasible";
  }
  return "Unknown!";
};

Trajectory::Trajectory()
{
	_sub_vehicle_local_position = orb_subscribe(ORB_ID(vehicle_local_position));
}

void Trajectory::updateSubscritpions()
{
	bool updated;
	orb_check(_sub_vehicle_local_position, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _sub_vehicle_local_position, &_local_position);
	}

	// std::cout << "\033[1;31m _local_position: " << _local_position.x << " " << _local_position.y << " " << _local_position.z << "\033[0m\n";

	generate();
}

void Trajectory::generate()
{
	//Define the trajectory starting state:
  Vec3 pos0 = Vec3(_local_position.x, _local_position.y, _local_position.z); //position
  Vec3 vel0 = Vec3(_local_position.vx, _local_position.vy, _local_position.vz); //velocity
  Vec3 acc0 = Vec3(_local_position.ax, _local_position.ay, _local_position.az); //acceleration

  //define the goal state:
  Vec3 posf = Vec3(2, 0, -2); //position
  Vec3 velf = Vec3(0, 0, -1); //velocity
  Vec3 accf = Vec3(0, 0, 0); //acceleration

  //define the duration:
  double Tf = 1.3;

  double fmin = 5;//[m/s**2]
  double fmax = 25;//[m/s**2]
  double wmax = 20;//[rad/s]
  double minTimeSec = 0.02;//[s]

  //Define how gravity lies in our coordinate system
  Vec3 gravity = Vec3(0,0,-9.81);//[m/s**2]

  //Define the state constraints. We'll only check that we don't fly into the floor:
  Vec3 floorPos = Vec3(0,0,0);//any point on the boundary
  Vec3 floorNormal = Vec3(0,0,-1);//we want to be in this direction of the boundary

  RapidTrajectoryGenerator traj(pos0, vel0, acc0, gravity);
  traj.SetGoalPosition(posf);
  traj.SetGoalVelocity(velf);
  traj.SetGoalAcceleration(accf);

  traj.Generate(Tf);
  std::vector<float> alpha;
  std::vector<float> beta;
  std::vector<float> gamma;


  for(int i = 0; i < 3; i++)
  {
    std::cout << "Axis #" << i << "\n";
    std::cout << "\talpha = " << traj.GetAxisParamAlpha(i);
    std::cout << "\tbeta = "  << traj.GetAxisParamBeta(i);
    std::cout << "\tgamma = " << traj.GetAxisParamGamma(i);
    std::cout << "\n";

    alpha.push_back(traj.GetAxisParamAlpha(i));
    beta.push_back(traj.GetAxisParamBeta(i));
    gamma.push_back(traj.GetAxisParamGamma(i));
  }
  std::cout << "Total cost = " << traj.GetCost() << "\n";
  std::cout << "Input feasible? " << GetInputFeasibilityResultName(traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec)) << "\n";
  std::cout << "Position feasible? " << GetStateFeasibilityResultName(traj.CheckPositionFeasibility(floorPos, floorNormal)) << "\n";


  if (GetInputFeasibilityResultName(traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec)) == "Feasible" &&
  	GetStateFeasibilityResultName(traj.CheckPositionFeasibility(floorPos, floorNormal)) == "Feasible")
  {
  	float t = 0.0f;
    while (t < 1.0f) {
      std::vector<float> pos_t;
      std::vector<float> vel_t;
      std::vector<float> acc_t;

      for (int k = 0; k < 3; ++k)
      {
        float acc_i = acc0[k];
        float vel_i = vel0[k];
        float pos_i = pos0[k];

        pos_t.push_back(alpha[k] / 120.0f * pow(t, 5) + beta[k] / 24.0f * pow(t, 4) + gamma[k] / 6.0f * pow(t, 3) + acc_i / 2.0f * pow(t, 2) + vel_i * t + pos_i);
        vel_t.push_back(alpha[k] / 24.0f * pow(t, 4) + beta[k] / 6.0f * pow(t, 3) + gamma[k] / 2.0f * pow(t, 2) + acc_i *t + vel_i);
        acc_t.push_back(alpha[k] / 6.0f * pow(t, 3) + beta[k] / 2.0f * pow(t, 2) + gamma[k] * t + acc_i);
      }

      local_position_setpoint.timestamp = hrt_absolute_time();
      local_position_setpoint.x = pos_t[0];
      local_position_setpoint.y = pos_t[1];
      local_position_setpoint.z = pos_t[2];
      local_position_setpoint.vx = NAN;
      local_position_setpoint.vy = NAN;
      local_position_setpoint.vz = NAN;
      local_position_setpoint.acc_x = NAN;
      local_position_setpoint.acc_y = NAN;
      local_position_setpoint.acc_z = NAN;
      matrix::Vector3f(NAN, NAN, NAN).copyTo(local_position_setpoint.thrust);
      std::cout << "\033[1;32m local_position_setpoint: " << local_position_setpoint.x << " " << local_position_setpoint.y << " " << local_position_setpoint.z << "\033[0m\n";
      updateLocalPositionSetpoint();
      t += 0.01f;
  	}
  }
}

const vehicle_local_position_setpoint_s Trajectory::updateLocalPositionSetpoint()
{
	return local_position_setpoint;
}