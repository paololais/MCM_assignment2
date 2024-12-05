%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

            % TO DO
             % Get the transformation matrix of the end-effector with respect to the base
            bTe = self.gm.getTransformWrtBase(self.gm.jointNumber);
        
            % Extract the position of the end-effector
            r_ee = bTe(1:3, 4);
            for i=1:self.gm.jointNumber
                % Get the transformation matrix for the current joint
                bTi = self.gm.getTransformWrtBase(i);
        
                % Extract joint position and z-axis direction for the current joint
                r_i = bTi(1:3, 4);  % Position of the current joint
                k_i = bTi(1:3, 3);  % z-axis of the current joint (rotation axis)
                %rotational joint
                if (self.gm.jointType(i) == 0)
                    % Angular velocity contribution (aligned with the z-axis of the current joint)
                    self.J(1:3, i) = k_i;

                    % Linear velocity contribution (cross product of z-axis and position vector)
                    self.J(4:6, i) = cross(k_i, (r_ee - r_i));
                    
                %prismatic
                elseif (self.gm.jointType(i) == 1)
                    % Angular velocity contribution (zero for prismatic joints)
                    self.J(1:3, i) = [0; 0; 0];

                    % Linear velocity contribution (aligned with the z-axis of the current joint)
                    self.J(4:6, i) = k_i;
                else
                    error('jointType must be 0 (rotational) or 1 (prismatic)')
                end
            end
        end
    end
end

