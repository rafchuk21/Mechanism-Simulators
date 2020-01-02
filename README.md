# Mechanism-Simulators
Mechanism Simulators for FRC

Simulates mechanisms using characterization constants as described in the FRC Drivetrain Characterization Paper by Noa Gleason and Eli Barnett.

To use:

MechanismTimestep.m takes a state and an input and returns the new state.

DrivetrainSimulator.m is the file that runs the drivetrain simulation using MechanismTimestep.m. This file acts as the "driver."

SimulateDrive.m runs DrivetrainSimulator.m for a single set of robot specs and creates the plots for it. This is what you want to use to evaluate a specific drive you're considering.

SprintDistVSGearings.m makes a plot of sprint distance time vs low gear and high gear using DrivetrainSimulator.m, and then uses SimulateDrive.m to give specific plots for the optimal DS and SS gearings.

To switch between no stopping and bang-bang stopping, go into DrivetrainSimulator.m and comment and uncomment the appropriate blocks for Bang-Bang Stopping and No Stopping (currently between lines 104 and 110).

Drivetrain specs are inputs to all files except SprintDistVSGearings. To change those, go to the top of SprintDistVSGearings and adjust the values. Reference SprintDistVSGearings for how to run SimulateDrive.m.
