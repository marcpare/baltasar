# second_model.py
# by Marc Pare
# 10/9/2010
#
# ------------------------------------------------------------
#
# Implement a more complex drive cycle
#
# Add engine switching on and off
#
# verification: see second_model.m
#
# Notes:
# The engine switching works without a hitch. However, adding
# more points makes the solver unable to find a solution. This
# was found to be an error with the translation from the MATLAB
# model to this model. Once it was fixed, the values matched
# the MATLAB code *exactly*. 
# ------------------------------------------------------------

from realpaver_api import *
from math import pi, sin, cos

if __name__ == "__main__":

  # Drive cycle data
  # ----------------

  dt = 1.0
  vels = [0, 0, 0, 1.1, 1.2, 3.0, 3.4, 4.0, 4.1, 4.2, 5.0, 5.1, 5.6, 5.9, 4.7, 3.9, 2.9, 1.3, 0, 0, 0]
  
  N = len(vels)
  times = range(N)
  
  # System Constants
  # ----------------

  vehicleMass = 750.0
  wheelDia = 0.619
  airDensity = 1.29
  g = 9.81
  gradeAngle = 0.0
  drag_coeff = 0.5
  fo = 0.0095
  fs = 0.0035
  frontArea = 1.784
  gear1 = 7.49
  gear2 = 2.89
  
  accumPrecharge = 17500000
  initOilVol = 0.03
  accumVol = 0.04
  initGasVol = accumVol - initOilVol
  pumpmotorEdisp= (2.8e-5)/(2*pi)
  pumpmotorBdisp= (2.8e-5)/(2*pi)
  accumMaxRatio = 2
  accumP_full = accumPrecharge*accumMaxRatio
  accumP_max = 0.9*accumP_full
  accumP_min = 0.6*accumP_full
  
  enginePower = 13700
  engineSpeed_rpm = 2200
  pumpmotorBreduct=1.3
  restartPenalty = 500
  transmissionReduct = 2.66
  
  gear = 5.0
    
  # Precalculation of parameters
  # ----------------------------
  #
  # TODO: import numpy as np
  # a = np.arange(10000000)
  # b = np.arange(10000000)
  # c = a + b
  
  wheelVelocities = [0]*N
  accs = [0]*N
  vehicleForceaccels= [0]*N
  vehicleForcedrags = [0]*N
  frs = [0]*N
  vehicleForcerollings = [0]*N
  vehicleForceslopes = [0]*N
  vehicleForcetotals = [0]*N
  wheelsPowers = [0]*N
  wheelsTorques = [0]*N
  
  for i in range(N):
    v = vels[i]
    wheelVelocities[i] = 2.0*v/wheelDia
    accs[i] = 0 if i==0 else (vels[i] - vels[i-1])/dt
    vehicleForceaccels[i] = vehicleMass*accs[i]
    vehicleForcedrags[i] = 0.5*airDensity*drag_coeff*frontArea*(v**2)
    frs[i] = 0.0 if v < 1e-9 else fo+(3.24*fs*((v*2.23693629/100)**2.5))      
    vehicleForcerollings[i] = frs[i]*vehicleMass*g
    vehicleForceslopes[i] = vehicleMass*g*(sin(gradeAngle)/cos(gradeAngle))  
    vehicleForcetotals[i] = vehicleForceaccels[i] + vehicleForcedrags[i] + vehicleForcerollings[i] + vehicleForceslopes[i]
    wheelsPowers[i] = vehicleForcetotals[i]*v
    try:
      wheelsTorques[i] = wheelsPowers[i]/wheelVelocities[i]
    except:
      wheelsTorques[i] = 0.0
      
  add_constant("wheelsTorques", wheelsTorques, times)
  add_constant("wheelVelocities", wheelVelocities, times)
  add_constant("wheelsPowers", wheelsPowers, times)
  add_constant("initGasVol", initGasVol)
  add_constant("initOilVol", initOilVol)
  add_constant("accumVol", accumVol)
  add_constant("w", (engineSpeed_rpm/gear)/transmissionReduct)
  add_constant("pumpmotorEdisp", pumpmotorEdisp)
  add_constant("pumpmotorBdisp", pumpmotorEdisp)
  add_constant("pumpmotorBreduct", pumpmotorBreduct)
  add_constant("accumPrecharge", accumPrecharge)
  add_constant("dt", dt)
  add_constant("enginePower", enginePower)
  add_constant("engineSpeed_rpm", engineSpeed_rpm)
  add_constant("transmissionReduct", transmissionReduct)

  add_variable("gear", 5.0, 5.0, times)
  add_variable("hydshaftReduct", 10.0, 10.0)
  add_variable("pumpmotorEMaxfractdisp", -1, 1, times)
  add_variable("accumPressure", 0.0, "+oo", times)
  add_variable("accumVolumeGas", 0.0, "+oo", times)
  add_variable("accumVolumeOil", 0.0, "+oo", times)
  add_variable("carriershaftSpeed", "-oo", "+oo", times)
  add_variable("hyddifferentialSpeed", "-oo", "+oo", times)
  add_variable("pumpmotorESpeed", "-oo", "+oo", times)
  add_variable("hydbranchPower", "-oo", "+oo", times)
  add_variable("mechbranchPower", "-oo", "+oo", times)
  add_variable("pumpmotorBMaxfractdisp", -1, 1, times)
  add_variable("pumpmotorBPower", "-oo", "+oo", times)
  add_variable("pumpmotorBTrueflowrate", "-oo", "+oo", times)
  add_variable("pumpmotorETrueflowrate", "-oo", "+oo", times)
  add_variable("engineState", 0, 1, times, "int")
  
  # Initial conditions  
  quick_constraint("accumPressure[0] = accumPrecharge")
  quick_constraint("accumVolumeOil[0] = initOilVol")
        
  # After initial condition  
  add_set("after_start", times[1:])
  quick_constraint("accumVolumeOil[t] = accumVolumeOil[t-1] + dt*(2.0*pumpmotorETrueflowrate[t-1] + pumpmotorBTrueflowrate[t-1]) for t in after_start")
    
  # Defined over entire time range  
  add_set("times", times)
  
  quick_constraint("accumVolumeGas[t] = accumVol - accumVolumeOil[t] for t in times")  
  quick_constraint("accumPressure[t] * accumVolumeGas[t] = initGasVol * accumPrecharge for t in times")
  quick_constraint("carriershaftSpeed[t] = (engineSpeed_rpm / (gear[t] * transmissionReduct))*%s for t in times" % ((2*pi/60)))
  quick_constraint("hyddifferentialSpeed[t] = 2.0*carriershaftSpeed[t] - wheelVelocities[t] for t in times")
  quick_constraint("pumpmotorESpeed[t] = hyddifferentialSpeed[t] * hydshaftReduct for t in times")  
  quick_constraint("hydbranchPower[t] = wheelsTorques[t] * hyddifferentialSpeed[t] for t in times")
  quick_constraint("mechbranchPower[t] = -2.0 * wheelsTorques[t] * carriershaftSpeed[t] for t in times")
  quick_constraint("pumpmotorBPower[t] = enginePower * engineState[t] + mechbranchPower[t] for t in times")
  quick_constraint("pumpmotorETrueflowrate[t] * accumPressure[t] = hydbranchPower[t] for t in times")
  quick_constraint("pumpmotorBTrueflowrate[t] * accumPressure[t] = pumpmotorBPower[t] for t in times")
  quick_constraint("hydbranchPower[t] = pumpmotorEMaxfractdisp[t] * hyddifferentialSpeed[t] * accumPressure[t] * pumpmotorEdisp * hydshaftReduct for t in times")
  calc_speed = engineSpeed_rpm*(2.0*pi/60.0)
  quick_constraint("pumpmotorBPower[t] = pumpmotorBMaxfractdisp[t] * %s * pumpmotorBreduct * accumPressure[t] * pumpmotorBdisp for t in times" % calc_speed)
  
  # Cost function
  add_constraint(
    lt(var_sum(times, power("pumpmotorEMaxfractdisp", 2)), 500.0))

  # ---------------------------
  # Engine switching
  # ---------------------------
  
  # i = 1 if a > b
  # a <= b + bigM * i
  # a > b - bigM * (1-i)
  
  # accumPressure < accumP_min
  add_variable("below_min", 0, 1, times, "int")
  add_constant("accumP_min", accumP_min)
  quick_constraint("accumP_min < accumPressure[t] + 300000000.0 * below_min[t] for t in times")
  quick_constraint("accumP_min > accumPressure[t] - 300000000.0 * (1-below_min[t]) for t in times")
  
  # accumPressure < accumP_max
  add_variable("below_max", 0, 1, times, "int")
  add_constant("accumP_max", accumP_max)
  quick_constraint("accumP_max < accumPressure[t] + 300000000.0 * below_max[t] for t in times")
  quick_constraint("accumP_max > accumPressure[t] - 300000000.0 * (1-below_max[t]) for t in times")
  
  quick_constraint("engineState[0] = 1")
  quick_constraint("engineState[t] = (below_min[t] * (1 - engineState[t-1])) + (below_max[t] * engineState[t-1]) for t in after_start")

  # ---------------------------
  # Gear switching
  # ---------------------------
  # add_constant("gear1", gear1)
  # add_constant("gear2", gear2)
  # add_constant("w1", (engineSpeed_rpm/gear1)/transmissionReduct)
  # add_constant("w2", (engineSpeed_rpm/gear2)/transmissionReduct)
  
  # add_variable("wp1", "-oo", "+oo", times)
  # add_variable("wp2", "-oo", "+oo", times)
  # quick_constraint("wp1[t]*1000.0 = hydshaftReduct*(2*w1-(wheelVelocities[t]*%s)) for t in times" % (60/(2*pi)))
  # quick_constraint("wp2[t]*1000.0 = hydshaftReduct*(2*w2-(wheelVelocities[t]*%s)) for t in times" % (60/(2*pi)))
  
  # wp1 > wp2
  # add_variable("second_is_less", 0, 1, times, "int")
  # quick_constraint("wp1[t]*wp1[t] < wp2[t]*wp2[t] + 1000.0 * second_is_less[t] for t in times")
  # quick_constraint("wp1[t]*wp1[t] > wp2[t]*wp2[t] - 1000.0 * (1-second_is_less[t]) for t in times")
  
  # 2*w2 < wheelsVelocity(t)*60/(2*pi)
  # add_variable("above_w2", 0, 1, times, "int")
  # quick_constraint("wheelVelocities[t]*%s < 2*w2 + 1000.0 * above_w2[t] for t in times" % (60/(2*pi)))
  # quick_constraint("wheelVelocities[t]*%s > 2*w2 - 1000.0 * (1-above_w2[t]) for t in times" % (60/(2*pi)))
  
  # add_variable("choose_gear_2", 0, 1, times, "int")
  # quick_constraint("choose_gear_2[t] = second_is_less[t] * above_w2[t] for t in times")
  # quick_constraint("gear[t] = gear1 * (1 - choose_gear_2[t]) + gear2 * choose_gear_2[t] for t in times")

  # Dump output to text file
  rendered = render()
  foo = open("foo.txt", "w")
  foo.write(rendered)
  foo.close()

  # print rendered
  print "Finished rendering to foo.txt"