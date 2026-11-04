classdef SmokeTest < matlab.unittest.TestCase
    % Fast checks that a fresh MATLAB can load, trim, and simulate the morphing GTM.
    % Run from the repository root:  results = runtests("tests")
    % CI runs this on every push (.github/workflows/matlab.yml). The full design
    % (lpv_design.m: 63-point trim + LPV synthesis) takes far longer and is not run in CI.

    properties (Constant)
        DesignDir = fullfile(fileparts(fileparts(mfilename("fullpath"))), "GTM_DesignSim", "gtm_design")
    end

    methods (TestClassSetup)
        function runSetup(testCase)
            here = pwd;
            testCase.addTeardown(@() cd(here));
            testCase.addTeardown(@() bdclose("all"));
            cd(testCase.DesignDir);
            evalin("base", "setup");
        end
    end

    methods (Test)
        function savedDesignLoads(testCase)
            % setup.m loads the saved trim/linearization and controller results.
            testCase.verifyTrue(evalin("base", "exist('sch_tas_trim','var') == 1"), ...
                "lpv_design_result.mat did not provide the scheduled trim data");
        end

        function trimsOneDesignPoint(testCase)
            cd(testCase.DesignDir);
            rng(0);          % trimgtm_morph draws a random initial guess
            alt = 5000;      % ft
            mor = -12.5;     % symmetric morphing parameter, %
            tas = sas2tas(90, alt);
            [~, ~, trimcond, err] = trimgtm_morph_try(tas, alt, mor, 3);
            testCase.verifyLessThan(err, 1e-3, "trim residual too large");
            testCase.verifyEqual(trimcond.altitude, alt, "AbsTol", 1);
            testCase.verifyEqual(trimcond.morphing, mor, "AbsTol", 1e-6);
        end

        function fliesAutopilotTwoSeconds(testCase)
            % Same setup as plot/autopilot/plot_autopilot_straight.m (the paper's
            % straight-climb autopilot scenario), flown for 2 s instead of 180 s.
            cd(testCase.DesignDir);
            rng(0);
            evalin("base", strjoin([
                "alt0 = 1000; etas0 = -12.5; tas0 = 75; t_final = 2; etasc_feedthrough = 0;"
                "FlightMode = 3.0; ControlMode = 4; RateofClimb = 10; phi_to_eta = 0;"
                "LookaheadDistance = 700; TransitionRadius = 1000*0.3048; Radius = 2000; TurnDirection = 1;"
                "Waypoints = [0 0 -alt0; 3000 0 -alt0; 3000 3000 -alt0; 0 3000 -alt0; 0 0 -alt0];"
                "MWS = init_design('GTM',1); MWS.bias.morphing = etas0; MWS.StatesInp(9) = alt0;"
                "MWS.SurfaceDeadbandOn = 0; loadmws(MWS,'gtm_design');"
                "[MWS,Xtrim,Trimcond,Err] = trimgtm_morph_try(tas0,alt0,etas0,3);"
                "MWS.SurfaceDeadbandOn = 1; loadmws(MWS,'gtm_design');"
                "deltc = MWS.bias.throttle;"
                ], " "));
            for blk = ["Control/Controller", "Guidance", "Monitor", "Guidance/Autopilot Step"]
                set_param("gtm_design/" + blk, "commented", "off");
            end
            for blk = ["Guidance/CAS Maneuver", "Guidance/Autopilot Trajectory Tracking", ...
                       "Guidance/Heading Control", "Guidance/CAS Step"]
                set_param("gtm_design/" + blk, "commented", "on");
            end
            set_param("gtm_design", "MaxStep", "auto", "RelTol", "1e-3", ...
                "MinStep", num2str(evalin("base", "MWS.Timestep") - 0.00001));
            [t, x] = evalin("base", "sim('gtm_design', [0 t_final])");
            testCase.verifyGreaterThanOrEqual(t(end), 2 - 1e-9);
            testCase.verifyTrue(all(isfinite(x(:))), "simulation produced non-finite states");
        end
    end
end
