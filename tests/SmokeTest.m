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

        function simulatesTwoSeconds(testCase)
            cd(testCase.DesignDir);
            % trimgtm_morph comments these blocks out for trimming; fly with them in.
            for blk = ["Control/Controller", "Guidance", "Monitor"]
                set_param("gtm_design/" + blk, "commented", "off");
            end
            out = sim("gtm_design", "StopTime", "2", "ReturnWorkspaceOutputs", "on");
            testCase.verifyGreaterThanOrEqual(out.tout(end), 2 - 1e-9);
        end
    end
end
