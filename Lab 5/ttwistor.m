% Flight conditions and atmospheric parameters derived from AVL and other 
% sources for the University of Colorado's Ttwistor Unmanned Aircraft, an
% twin-engine version of the Tempest UAS.
%
%   File created by: Eric Frew, eric.frew@colorado.edu
%
%   Data taken from files generated by Jason Roadman.
%       - Derivatives come from AVL analysis
%       - Inertias from Solidworks model
%
%   Data further modified from CFD analysis by Roger Laurence and by
%   adjustments from Eric Frew
%       - modified with new engine model, aircraft geometry and drag model
%
%
% If using this data for published work please reference:
%
% Jason Roadman, Jack Elston, Brian Argrow, and Eric W. Frew. 
%   "Mission Performance of the Tempest UAS in Supercell Storms".� 
%   AIAA Journal of Aircraft, 2012.
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% All dimensional parameters in SI units
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

aircraft_parameters.g = 9.81;           % Gravitational acceleration [m/s^2]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Aircraft geometry parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.S = 0.6282; %[m^2]
aircraft_parameters.b = 3.067; %[m]
aircraft_parameters.c = 0.208; %[m]
aircraft_parameters.AR = aircraft_parameters.b*aircraft_parameters.b/aircraft_parameters.S;

aircraft_parameters.m = 5.74; %[kg]
aircraft_parameters.W = aircraft_parameters.m*aircraft_parameters.g; %[N]


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inertias from Solidworks model of Tempest
% These need to be validated, especially for Ttwistor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SLUGFT2_TO_KGM2 = 14.5939/(3.2804*3.2804);
aircraft_parameters.Ix = SLUGFT2_TO_KGM2*4106/12^2/32.2; %[kg m^2]
aircraft_parameters.Iy = SLUGFT2_TO_KGM2*3186/12^2/32.2; %[kg m^2]
aircraft_parameters.Iz = SLUGFT2_TO_KGM2*7089/12^2/32.2; %[kg m^2]
aircraft_parameters.Ixz = SLUGFT2_TO_KGM2*323.5/12^2/32.2; %[kg m^2]


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Drag terms determined by curve fit to CFD analysis performed by Roger
% Laurence. Assumes general aircraft drag model
%       CD = CDmin + K(CL-CLmin)^2
% or equivalently
%       CD = CD0 + K1*CL + K*CL^2
% where
%       CD0 = CDmin + K*CLmin^2
%       K1  = -2K*CLmin
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.CDmin = 0.0240;
aircraft_parameters.CLmin = 0.2052;
aircraft_parameters.K = 0.0549;
aircraft_parameters.e = 1/(aircraft_parameters.K*aircraft_parameters.AR*pi);

aircraft_parameters.CD0 = aircraft_parameters.CDmin+aircraft_parameters.K*aircraft_parameters.CLmin*aircraft_parameters.CLmin;
aircraft_parameters.K1 = -2*aircraft_parameters.K*aircraft_parameters.CLmin;
aircraft_parameters.CDpa = aircraft_parameters.CD0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Engine parameters, assuming model from Beard and Mclain that gives zero
% thrust for zero throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.Sprop = 0.0707;
aircraft_parameters.Cprop = 1;
aircraft_parameters.kmotor = 30;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Zero angle of attack aerodynamic forces and moments
% - some sources (like text used for ASEN 3128) define the body
%   coordinate system as the one that gives zero total lift at 
%   zero angle of attack
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.CL0 = 0.2219;
aircraft_parameters.Cm0 = 0.0519;

aircraft_parameters.CY0 = 0;
aircraft_parameters.Cl0 = 0;
aircraft_parameters.Cn0 = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Longtidunal nondimensional stability derivatives from AVL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.CLalpha = 6.196683; 
aircraft_parameters.Cmalpha = -1.634010; 
aircraft_parameters.CLq = 10.137584; 
aircraft_parameters.Cmq = -24.376066;

% Neglected parameters, check units below if incorporated later
aircraft_parameters.CLalphadot = 0; 
aircraft_parameters.Cmalphadot = 0; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lateral-directional nondimensional stability derivatives from AVL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.CYbeta = -0.367231; 
aircraft_parameters.Clbeta = -0.080738; 
aircraft_parameters.Cnbeta = 0.080613; 
aircraft_parameters.CYp = -0.064992;
aircraft_parameters.Clp = -0.686618;
aircraft_parameters.Cnp = -0.039384;
aircraft_parameters.Clr = 0.119718;
aircraft_parameters.Cnr = -0.052324;
aircraft_parameters.CYr = 0.213412;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control surface deflection parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % Elevator
  aircraft_parameters.CLde =   0.006776;
  aircraft_parameters.Cmde =  -0.06; 

  % Aileron
  aircraft_parameters.CYda =  -0.000754;
  aircraft_parameters.Clda =  -0.02; 
  aircraft_parameters.Cnda =  -0.000078;
 
  % Rudder
  aircraft_parameters.CYdr =   0.003056;
  aircraft_parameters.Cldr =   0.000157;
  aircraft_parameters.Cndr =  -0.000856;

  
