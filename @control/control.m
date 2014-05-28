classdef control < handle
    methods
        function sys = control(m,Ib,mu,inner_loop_gains)
            sys.m = m;
            sys.Ib = Ib;
            sys.mu = mu;
            % PD attitude control
            % P velocity control
            % Steady-state error exists in velocity term, needs accounted for later
            sys.roll_p = inner_loop_gains(1);
            sys.roll_d = inner_loop_gains(3);
            sys.pitch_p = inner_loop_gains(4);
            sys.pitch_d = inner_loop_gains(6);
            sys.yaw_p = inner_loop_gains(7);
            sys.yaw_d = inner_loop_gains(9);
            sys.velocity_p = inner_loop_gains(10);
        end
        function  geometry(sys)
        	% Define 4 HTMs describing propeller location
            thruster1 = [1 0 0 .0707;0 1 0 -.0707;0 0 1 0;0 0 0 1];
            thruster2 = [1 0 0 -.0707;0 1 0 -.0707;0 0 1 0;0 0 0 1];
            thruster3 = [1 0 0 -.0707;0 1 0 .0707;0 0 1 0;0 0 0 1];
            thruster4 = [1 0 0 .0707;0 1 0 .0707;0 0 1 0;0 0 0 1];

            % A matrix converts thrust commands to moments and total thrust
            a = cross(thruster1(1:3,4),[0;0;-1]);
            b = cross(thruster2(1:3,4),[0;0;-1]);
            c = cross(thruster3(1:3,4),[0;0;-1]);
            d = cross(thruster4(1:3,4),[0;0;-1]);
            sys.A = [a b c d];
            sys.A(4,:) = [-1 -1 -1 -1];
            sys.A = sys.A + [0 0 0 0;0 0 0 0;.013 -.013 .013 -.013 ;0 0 0 0];


            % Thrusters 1 and 3 rotate: CCW (posative yaw)
            % Thrusters 2 and 4 rotate: CW (negative yaw)
        end
        function thrust = inner_loop(sys, set_points, State)

            % This function takes the phi, theta, psi and w set points
            % and the current state to determine the desired thrust

            % Calculate attitude error
            roll_err = set_points(1)-State(4);
            pitch_err = set_points(2)-State(5);
            yaw_err = set_points(3)-State(6);
    
            % Calculate rate of change of attitude error
            roll_accel = sys.roll_p*roll_err + sys.roll_d*State(10);
            pitch_accel = sys.pitch_p*pitch_err + sys.pitch_d*State(11);
            yaw_accel = sys.yaw_p*yaw_err + sys.yaw_d*State(12);
    
            % Calculate desired moments
            moments = sys.Ib*[roll_accel;pitch_accel;yaw_accel];
    
            % Calculate desired force along body Z
            force = sys.m*sys.velocity_p*State(9);

            % Calculate required thrust
            thrust = sys.A\[moments;force];

    
            % Apply thrust limiting (20 N / 4.5 lbf) 
            for j = 1:4
                if thrust(j)>20
                    thrust(j) = 20;
                elseif thrust(j) < 0;
                    thrust(j) = 0;
                end
            end
        end
        function inner_set_points = outer_loop(sys,state, set_points)
            inner_x_vel_p = .15;
            inner_y_vel_p = -.15;
            %inner_z_vel_p = -.5;
            inner_x_vel_d = 03;
            inner_y_vel_d = 03;
            Or = state(4:6);
            An = state(10:12);
            world_vel = [1 0 0;0 -1 0;0 0 1]*[cos(Or(2))*cos(Or(3)), sin(Or(1))*sin(Or(2))*cos(Or(3))-cos(Or(1))*sin(Or(3)), cos(Or(1))*sin(Or(2))*cos(Or(3))-sin(Or(1))*sin(Or(3));
                              cos(Or(2))*sin(Or(3)), sin(Or(1))*sin(Or(2))*sin(Or(3))-cos(Or(1))*cos(Or(3)), cos(Or(1))*sin(Or(2))*sin(Or(3))-sin(Or(1))*cos(Or(3));
                            -sin(Or(2)), sin(Or(1))*cos(Or(2)), cos(Or(1))*cos(Or(2))]*state(7:9);
  
            setRoll = state(5)+inner_y_vel_p*(world_vel(2)-set_points(2));
            setPitch = state(4)+inner_x_vel_p*(world_vel(1)-set_points(1));

            inner_set_points = [setRoll;setPitch;0;20];
        end
    end
    properties
        m
        mu
        Ib
        roll_p
        roll_d
        pitch_p
        pitch_d
        yaw_p
        yaw_d
        velocity_p
        A
    end
end
        