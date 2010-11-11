% second_model.m
% Objective: get values to match up with Realpaver results and get the code
%            closer to CSP model code.

clear all
clc

% -------------------------------
% Set up
% -------------------------------

% Drive cycle data
dt = 1.0;
vels = [0, 0, 0, 1.1, 1.2, 3.0, 3.4, 4.0, 4.1, 4.2, 5.0, 5.1, 5.6, 5.9, 4.7, 3.9, 2.9, 1.3, 0, 0, 0];
N = length(vels);

% System constants
vehicleMass = 750;
wheelDia = 0.619;
airDensity = 1.29;
g = 9.81;
gradeAngle = 0.0;
drag_coeff = 0.5;
fo = 0.0095;
fs = 0.0035;
frontArea = 1.784;

pumpmotorEdisp = (2.8e-5)/(2*pi); 
pumpmotorBdisp = (2.8e-5)/(2*pi);
accumPrecharge = 17500000;
initOilVol = 0.03;
accumVol = 0.04;
accumMaxRatio = 2;
accumP_full = accumPrecharge*accumMaxRatio;
accumP_max = 0.9*accumP_full;
accumP_min = 0.6*accumP_full;
initGasVol = accumVol - initOilVol;

enginePower = 13700; 
engineSpeed_rpm = 2200;
pumpmotorBreduct = 1.3;
restartPenalty = 500;
transmissionReduct = 2.66;
% gear1 = 7.49;
% gear2 = 2.89;
gear1 = 5.0;
gear2 = 5.0;

% Calculation of parameters
wheelsVelocity = 2.0 * vels / wheelDia;
vehicleAcceleration = [0, (vels(2:end)-vels(1:end-1)) / dt];
vehicleForceaccel = vehicleMass*vehicleAcceleration;
vehicleForcedrag = 0.5*airDensity*drag_coeff*frontArea*(vels.^2);
fr = (fo+(3.24*fs*((vels*2.23693629/100).^2.5)));
fr(vels == 0) = 0;
vehicleForcerolling =   fr*vehicleMass*g;
vehicleForceslope = ones(1,N) * vehicleMass*g*(sin(gradeAngle)/cos(gradeAngle));
vehicleForcetotal = vehicleForceaccel  + vehicleForcedrag + vehicleForcerolling + vehicleForceslope;
wheelsPower = vehicleForcetotal.*vels;
wheelsTorque = wheelsPower./wheelsVelocity;
wheelsTorque(wheelsVelocity == 0) = 0.0;

% -------------------------------
% Run the drive cycle
% -------------------------------

hydshaftReducts = 1:.1:10;
fake_losses = [];
max_disps = [];
all_results = {};

for hydshaftReduct=hydshaftReducts
  
  % Initial conditions
  gear(1) = gear1
  w1 = (engineSpeed_rpm/gear1)/transmissionReduct;
  w2 = (engineSpeed_rpm/gear2)/transmissionReduct;
  wp1(1) = hydshaftReduct*(2*w1-(wheelsVelocity(1)*60/(2*pi)));
  wp2(1) = hydshaftReduct*(2*w2-(wheelsVelocity(1)*60/(2*pi)));
    
  carriershaftSpeed(1) = (engineSpeed_rpm/(gear(1)*transmissionReduct+eps))*(2*pi/60);
  hyddifferentialSpeed(1) = 2.*carriershaftSpeed(1) - wheelsVelocity(1);
  pumpmotorESpeed(1) = hyddifferentialSpeed(1) * hydshaftReduct;
  hydbranchPower(1) = wheelsTorque(1) * hyddifferentialSpeed(1);
  mechbranchPower(1) = -2 * wheelsTorque(1) * carriershaftSpeed(1);
  
  accumPressure(1) = accumPrecharge;
  accumVolumeOil(1) = initOilVol;
  accumVolumeGas(1) = initGasVol;
  
  engineState(1) = 1;
  pumpmotorBPower(1) = enginePower*engineState(1);
  pumpmotorETrueflowrate(1) = hydbranchPower(1) / accumPressure(1);
  pumpmotorBTrueflowrate(1) = pumpmotorBPower(1) / accumPressure(1);
  pumpmotorEMaxfractdisp(1) = hydbranchPower(1)/(hyddifferentialSpeed(1)*accumPressure(1)*pumpmotorEdisp*hydshaftReduct+eps);
  pumpmotorBMaxfractdisp(1) = pumpmotorBPower(1)/(engineSpeed_rpm*(2*pi/60)*pumpmotorBreduct*accumPressure(1)*pumpmotorBdisp);
  
  % March forward in time
  for t=2:N
    w1 = (engineSpeed_rpm/gear1)/transmissionReduct;
    w2 = (engineSpeed_rpm/gear2)/transmissionReduct;
    wp1(t) = hydshaftReduct*(2*w1-(wheelsVelocity(t)*60/(2*pi)));
    wp2(t) = hydshaftReduct*(2*w2-(wheelsVelocity(t)*60/(2*pi)));
    
    if (abs(wp2(t))<abs(wp1(t))) && (wheelsVelocity(t)*60/(2*pi))>2*w2
        gear(t) = gear2;
    else
        gear(t) = gear1;
    end
    
    carriershaftSpeed(t) = (engineSpeed_rpm / (gear(t)*transmissionReduct+eps))*(2*pi/60);
    hyddifferentialSpeed(t) = 2 * carriershaftSpeed(t) - wheelsVelocity(t);
    pumpmotorESpeed(t) = hyddifferentialSpeed(t) * hydshaftReduct;
    hydbranchPower(t) = wheelsTorque(t) * hyddifferentialSpeed(t);
    mechbranchPower(t) = -2 * wheelsTorque(t) * carriershaftSpeed(t);
    
    accumVolumeOil(t) = accumVolumeOil(t-1) + ((2*pumpmotorETrueflowrate(t-1) + pumpmotorBTrueflowrate(t-1))*dt);
    accumVolumeGas(t) = accumVol - accumVolumeOil(t);
    accumPressure(t) = accumPrecharge * initGasVol / (accumVolumeGas(t));
    
    % switch engine on and off
    engineState(t) = ((accumPressure(t)<accumP_min && engineState(t-1)==0)||(accumPressure(t)<accumP_max && engineState(t-1)==1));
                        
    pumpmotorBPower(t) = (enginePower)*(engineState(t)) + mechbranchPower(t);
    pumpmotorBTrueflowrate(t) = pumpmotorBPower(t) /(accumPressure(t));
    pumpmotorETrueflowrate(t) =  hydbranchPower(t) / (accumPressure(t));
    pumpmotorEMaxfractdisp(t) = hydbranchPower(t) / (hyddifferentialSpeed(t)*accumPressure(t)*pumpmotorEdisp*hydshaftReduct);
    pumpmotorBMaxfractdisp(t) = pumpmotorBPower(t) / (engineSpeed_rpm*(2*pi/60)*pumpmotorBreduct*accumPressure(t)*pumpmotorBdisp);
  end
  
  fake_losses = [fake_losses, sum(pumpmotorEMaxfractdisp.^2)];
  max_disps = [max_disps, max(abs(pumpmotorEMaxfractdisp))];
end

% -------------------------------
% So, what happened?
% -------------------------------

plot(hydshaftReducts, fake_losses, 'bo')
hold on

before = find(max_disps > 1.0);
before = before(end);

plot(hydshaftReducts(1:before), max_disps(1:before), 'rx')
plot(hydshaftReducts(before:end), max_disps(before:end), 'kx')
legend('Objective function to be maximized', 'Maximum Displacement')
xlabel('Gear Ratio')
hold off

figure

subplot(3, 3, 1)
plot(wheelsTorque)
ylabel('Wheel Torque')

subplot(3, 3, 2)
plot(vels)
ylabel('Velocity Avg')

subplot(3, 3, 3)
plot(engineState, 'bo')
ylabel('Engine State')

subplot(3, 3, 4)
plot(accumPressure)
ylabel('Accumulator P')

subplot(3, 3, 5)
plot(pumpmotorEMaxfractdisp)
ylabel('Pump E Max fract disp')

subplot(3, 3, 6)
plot(gear)
ylabel('Gear')

subplot(3, 3, 7)
plot(wp1, 'bo')
hold on
plot(wp2, 'rx')
hold off
ylabel('wp')
