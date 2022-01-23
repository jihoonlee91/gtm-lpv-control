%%%%%%%%%%%%%%%%%%%%%%%%%%% General Description %%%%%%%%%%%%%%%%%%%%%%%%%%%
This project is designed to simulate the longitudinal dynamics of morphing
aircraft which is capable of varying span and sweep.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ver1.0 (20170926)
[1] The first version of this simulator has been distributed.

ver1.1 (20170928)
[1] In aero_data.m, CF_S(2,1) has been corrected from '0.874' to '0.987'.
[2] In aircraft_model.slx, the signal 'cmd' going from the Guidance block
    to the Configuration block has been removed to fix an algebraic loop
    error.