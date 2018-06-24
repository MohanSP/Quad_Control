function quadcopterDynamicsSFunction(block)
setup(block); 

function setup(block)

  % Register the number of ports.
  block.NumInputPorts  = 5;
  block.NumOutputPorts = 12;
  
  % Set up the port properties to be inherited or dynamic.
  for i = 1:4; % These are the motor inputs
  block.InputPort(i).Dimensions        = 1;
  block.InputPort(i).DirectFeedthrough = false;
  block.InputPort(i).SamplingMode      = 'Sample';
  end

  % This is the disturbance input
  block.InputPort(5).Dimensions        = 6; % torques x,y,z; forces x,y,z.
  block.InputPort(5).DirectFeedthrough = false;
  block.InputPort(5).SamplingMode      = 'Sample';

  for i = 1:12;
  block.OutputPort(i).Dimensions       = 1;
  block.OutputPort(i).SamplingMode     = 'Sample';
  end

  % Register the parameters.
  block.NumDialogPrms     = 2;
  
  % Set up the continuous states.
  block.NumContStates = 12;

  % Register the sample times.
  block.SampleTimes = [0 0];
  
  % Specify if Accelerator should use TLC or call back to the MATLAB file
  block.SetAccelRunOnTLC(false);
  % Specify the block simStateCompliance. The allowed values are:
  %    'UnknownSimState', < The default setting; warn and assume DefaultSimState
  %    'DefaultSimState', < Same SimState as a built-in block
  %    'HasNoSimState',   < No SimState
  %    'CustomSimState',  < Has GetSimState and SetSimState methods
  %    'DisallowSimState' < Errors out when saving or restoring the SimState
  block.SimStateCompliance = 'DefaultSimState';
    
  % CheckParameters:
  %   Functionality    : Called in order to allow validation of the
  %                      block dialog parameters. You are 
  %                      responsible for calling this method
  %                      explicitly at the start of the setup method.
  block.RegBlockMethod('CheckParameters', @CheckPrms);

  % InitializeConditions:
  %   Functionality    : Called in order to initialize the
  %                      conditions of the quad.
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  
  % Outputs:
  %   Functionality    : Call to generate the block outputs during a
  %                      simulation step.
  block.RegBlockMethod('Outputs', @Outputs);
 
  % Derivatives:
  %   Functionality    : Call to update the derivatives of the
  %                      continuous states during a simulation step.
  %   C-Mex counterpart: mdlDerivatives
  block.RegBlockMethod('Derivatives', @Derivatives);
  
 function CheckPrms(block)
     quad   = block.DialogPrm(1).Data;
     IC     = block.DialogPrm(2).Data;

function InitializeConditions(block)
% Initialize 12 States
IC = block.DialogPrm(2).Data;

% IC.P, IC.Q, IC.R converted to rad/s
P = IC.P*pi/180; Q = IC.Q*pi/180; R = IC.R*pi/180; 

% IC.Phi, IC.The, IC.Psi converted to rads
Phi = IC.Phi*pi/180; The = IC.The*pi/180; Psi = IC.Psi*pi/180;

U = IC.U; V = IC.V; W = IC.W; 
X = IC.X; Y = IC.Y; Z = IC.Z;

init = [P,Q,R,Phi,The,Psi,U,V,W,X,Y,Z];

for i=1:12
block.OutputPort(i).Data = init(i);
block.ContStates.Data(i) = init(i);
end

function Outputs(block)
for i = 1:12;
  block.OutputPort(i).Data = block.ContStates.Data(i);
end

function Derivatives(block)

% Load model data selected in parameter block
quad = block.DialogPrm(1).Data;

% P Q R in units of rad/sec
P = block.ContStates.Data(1);
Q = block.ContStates.Data(2);
R = block.ContStates.Data(3);

% Phi The Psi in radians
Phi = block.ContStates.Data(4);
The = block.ContStates.Data(5);
Psi = block.ContStates.Data(6);
% U V W in units of m/s
%Uncomment for position control and team_37 control
%U = block.ContStates.Data(7);
%V = block.ContStates.Data(8);
%W = block.ContStates.Data(9);

%Set to 0 because we are looking at only orientation of the quad
%Comment if not using attitude control
U = 0;
V = 0;
W = 0;

% X Y Z in units of m
%Uncomment for position control and team_37 control
%X = block.ContStates.Data(10);
%Y = block.ContStates.Data(11);
%Z = block.ContStates.Data(12);

%Set to 0 because we are looking at only orientation of the quad
%Comment if not using attitude control
X = 0;
Y = 0;
Z = 0;

% w values in rev/min
w1 = block.InputPort(1).Data;
w2 = block.InputPort(2).Data;
w3 = block.InputPort(3).Data;
w4 = block.InputPort(4).Data;
w  = [w1; w2; w3; w4];

Dist_tau = block.InputPort(5).Data(1:3);
Dist_F   = block.InputPort(5).Data(4:6);

% Calculate moment and thrust forces
tau_motorGyro = [Q*quad.Jm*2*pi/60*(-w1-w3+w2+w4); P*quad.Jm*2*pi/60*(w1+w3-w2-w4); 0]; % Note: 2*pi/60 required to convert from RPM to radians/s
Mb = (quad.dctcq*(w.^2))+ tau_motorGyro + (Dist_tau);  % Mb = [tau1 tau2 tau3]'
% Thrust due to motor speed
Fb = [0; 0; sum(quad.ct*(w.^2))];
% Obtain dP dQ dR
omb_bi = [P; Q; R];
OMb_bi = [ 0,-R, Q; R, 0,-P; -Q, P, 0];
      
b_omdotb_bi = quad.Jbinv*(Mb-(cross(omb_bi,(quad.Jb*omb_bi))));
H_Phi = [1,tan(The)*sin(Phi), tan(The)*cos(Phi); 0, cos(Phi), -sin(Phi); 0, sin(Phi)/cos(The), cos(Phi)/cos(The)];   
Phidot = H_Phi*omb_bi;

% We use a Z-Y-X rotation matrix
Rib = [cos(Psi)*cos(The) cos(Psi)*sin(The)*sin(Phi)-sin(Psi)*cos(Phi) cos(Psi)*sin(The)*cos(Phi)+sin(Psi)*sin(Phi);
       sin(Psi)*cos(The) sin(Psi)*sin(The)*sin(Phi)+cos(Psi)*cos(Phi) sin(Psi)*sin(The)*cos(Phi)-cos(Psi)*sin(Phi);
       -sin(The)         cos(The)*sin(Phi)                            cos(The)*cos(Phi)];
Rbi = Rib';
ge = [0; 0; -quad.g];
gb = Rbi*ge;
Dist_Fb = Rbi*Dist_F;

% Compute Velocity and Position derivatives of body frame
vb = [U;V;W];
% Acceleration in body frame (For velocity)
b_dv = (1/quad.mass)*Fb+gb+Dist_Fb-OMb_bi*vb;
% Units OK SI: Velocity of body frame w.r.t inertial frame (FOR POSITION)
i_dp = Rib*vb;
dP = b_omdotb_bi(1);
dQ = b_omdotb_bi(2);
dR = b_omdotb_bi(3);
dPhi = Phidot(1);
dTheta = Phidot(2);
dPsi = Phidot(3);
dU = b_dv(1);
dV = b_dv(2);
dW = b_dv(3);
dX = i_dp(1);
dY = i_dp(2);
dZ = i_dp(3);

%This is the state derivative vector
f = [dP dQ dR dPhi dTheta dPsi dU dV dW dX dY dZ].';
block.Derivatives.Data = f;