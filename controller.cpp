#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>

using namespace Eigen;
using namespace std;

class control_system 
{
public:
	double mass;
	double mu;
	Matrix3f Ib;

	double roll_p;
	double roll_d;
	double pitch_p;
	double pitch_d;
	double yaw_p;
	double yaw_d;
	
	Matrix4f A;

	Vector3f integral_error;

	double velocity_p;
	double velocity_i;
	double velocity_d;

	control_system(){
		mass = 3; // kg
		mu = 0.1; // Drag Coefficient [N/(m/s)]
		Ib << 0.0241, 0, 0,
			  0, 0.0232, 0,
			  0, 0, 0.0451;
		
		roll_p = 5;
		roll_d =-3;
		pitch_p = 5;
		pitch_d =-3;
		yaw_p = 5;
		yaw_d =-3;
		
		A = geometry();

		integral_error <<0,0,0;
		
		velocity_p = 2.5;
		velocity_i = 0.001;
		velocity_d = 5;
	}
	~control_system(){}

	Matrix4f geometry(){
		Matrix4f thruster1;
		thruster1 << 1,0,0, 0.0707,
			     0,1,0,-0.0707,
    			     0,0,1,0,
        	             0,0,0,1;
		Matrix4f thruster2;
		thruster2 << 1,0,0,-0.0707,
			     0,1,0,-0.0707,
			     0,0,1,0,
        	             0,0,0,1;

		Matrix4f thruster3;
		thruster3 << 1,0,0,-0.0707,
			   		 0,1,0, 0.0707,
					 0,0,1,0,
   		             0,0,0,1;
		Matrix4f thruster4;
		thruster4 << 1,0,0, 0.0707,
					 0,1,0, 0.0707,
					 0,0,1,0,
                	 0,0,0,1;
		Vector3f orientation;
		orientation << 0,0,-1;
		Vector3f a = thruster1.topRightCorner<3,1>().cross(orientation);
		Vector3f b = thruster2.topRightCorner<3,1>().cross(orientation);
		Vector3f c = thruster3.topRightCorner<3,1>().cross(orientation);
		Vector3f d = thruster4.topRightCorner<3,1>().cross(orientation);

		MatrixXf geometry(4,4);
		geometry << a,b,c,d,1,1,1,1;
		MatrixXf yaw(4,4);
		yaw << 0,0,0,0,
			   0,0,0,0,
			  -0.13,0.13,-0.13,0.13,
			   0,0,0,0;
		MatrixXf A(4,4);
		A = geometry+yaw;
		//cout << A.determinant() << endl;
		return A;
	}

	void reset_integrat(){
		integral_error.resize(3);
		integral_error << 0,0,0;
	}
	Vector4f inner_loop(Vector4f commands, VectorXf state){
		Vector4f thrust;
		// Calculate attitude error
		double roll_err = commands(0)-state(3);
		double pitch_err = commands(1)-state(4);
		double yaw_err = commands(2)-state(5);
		// Calculate desired angular accelerations;
		double roll_accel = roll_p*roll_err + roll_d*state(9);
		double pitch_accel = pitch_p*pitch_err + pitch_d*state(10);
		double yaw_accel = yaw_p*yaw_err + yaw_d*state(11);

		Vector3f ang_accel;
		ang_accel << roll_accel,pitch_accel,yaw_accel;
		Vector3f moments;
		moments = Ib*ang_accel;
		Vector4f moments_and_force;
		moments_and_force << moments,commands(3);
		thrust = A.ldlt().solve(moments_and_force);
		
		for (int i=0;i<4;i++){
			if (thrust(i) > 15);
				thrust(i) = 15;
			if (thrust(i) < 0);
				thrust(i) = 0;
		}
		return thrust;
	}
	Vector4f outer_loop(Vector4f inputs,VectorXf states){
		Vector4f commands;
		Vector3f Or = states.segment<3>(3);
		Vector3f An = states.segment<3>(9);

		Matrix3f Rot;
		Rot << cos(Or(1))*cos(Or(2)),sin(Or(0))*sin(Or(1))*cos(Or(2))-cos(Or(0))*sin(Or(2)),cos(Or(0))*sin(Or(1))*cos(Or(2))-sin(Or(0))*sin(Or(2)),
			-cos(Or(1))*sin(Or(2)),-sin(Or(0))*sin(Or(1))*sin(Or(2))+cos(Or(0))*cos(Or(2)),sin(Or(0))*cos(Or(2))-cos(Or(0))*sin(Or(1))*sin(Or(2)),
			-sin(Or(1)),sin(Or(0))*cos(Or(1)),cos(Or(0))*cos(Or(1));
		
		Vector3f world_vel = Rot*states.segment<3>(6);

		Vector3f proportional_error = inputs.head<3>()-world_vel;
		integral_error = integral_error + proportional_error*0.01;
		Vector3f derivative_error = -An.cross(states.segment<3>(6));

		// Calculate nominal required force in world_frame
		Vector3f world_force << inputs(0)*mu,inputs(1)*mu,-9.81*mass;
		// Add corrective control terms
		world_force = world_force + velocity_p*proportional_error + velocity_i*integral_error + velocity_d*derivative_error;

		Vector3f wf_unit = world_force/world_force.norm();

		Vector3f thrust_orientation;
		thrust_orientation << 0,0,-1;
		double d = thrust_orientation.dot(wf_unit);
		double q = acos(d);
		Vector3f c = thrust_orientation.cross(wf_unit);
		c = sin(q/2)*c/c.norm();

		// This next part is spelled out for simplicity and readability but might effect runtime
		double w = cos(q/2);
		double x = c(0);
		double y = c(1);
		double z = c(2);

		double w2 = pow(w,2);
		double x2 = pow(x,2);
		double y2 = pow(y,2);
		double z2 = pow(z,2);

		Matrix3f Rd;
		Rd << w2+x2-y2-z2,2*(x*y-w*z,2*(w*y+x*z),
			  2*(x*y+w*z),w2-x2+y2-z2,2*(y*z-w*x),
			  2*(x*z-w*y),2*(w*x+y*z),w2-x2-y2+z2;

		float setYaw = Or(2) + 0.01*inputs(4);
		float setRoll = atan2(Rd(2,1),Rd(2,2);
		// This quite possibly should drop the setYaw term
		float setPitch = atan2(-Rd(2,0),Rd(2,2)/cos(setYaw);

		commands << setRoll,setPtich,setYaw,world_force.norm();
		return commands;
	}

};
int main(int argc, char **argv){
	control_system control;
	Vector4f cmnd;
	VectorXf state;
	state.resize(12);
	cmnd << 0,0,0,0;
	state << 0,0,0,0,1.75,0,20,0,0,0,0,0;

control.outer_loop(cmnd,state);
	return 0;
}
