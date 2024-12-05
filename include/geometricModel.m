%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType)
            if nargin > 1
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end
        function updateDirectGeometry(self, q)
            %%% GetDirectGeometryFunction
            % This method update the matrices iTj.
            % Inputs:
            % q : joints current position ;

            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)
           
            for i = 1:self.jointNumber 
                %Rotational Joint
                if(self.jointType(i) == 0)
                    Rz_qi = [cos(q(i)) -sin(q(i)) 0; 
                            sin(q(i)) cos(q(i)) 0; 
                            0 0 1];
                    self.iTj(1:3,1:3,i) = self.iTj_0(1:3,1:3,i) * Rz_qi;
                % Prismatic Joint
                elseif (self.jointType(i) == 1)
                    self.   iTj(1:3,4,i) = self.iTj_0(1:3,4,i) + [0;0;q(i)];
                else
                    error('jointType must be 0 (rotational) or 1 (prismatic)')
                end
            end
            
        end
        function [bTk] = getTransformWrtBase(self,k)
            %% GetTransformatioWrtBase function
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.

            bTk = eye(4); % base
            for i=1:k
                bTk = bTk * self.iTj(:,:,i);
            end
        end

    end
end


