clc; clear; close all;
%% Load COMSOL Model
import com.comsol.model.*
import com.comsol.model.util.*

model = mphload("G:\공유 드라이브\BSL-Data\Processed_data\Hyundai_dataset\OCV\es_ex_1C\JYR_cell_echem_parachange.mph");

ModelUtil.showProgress(true);

%% Load Data
data_folder = 'G:\공유 드라이브\BSL-Data\Processed_data\Hyundai_dataset\OCV\es_ex_1C';
filename_data = "results_f.mat";
data_now = load(fullfile(data_folder, filename_data));
SOC_now = data_now.results_f(3).SOC_vec;
V_now = data_now.results_f(3).V_vec; % 3rd entry for C-rate == 1 이 부분 수정생각중

%% Initial guess

[~,~,base{1},~] = mphevaluate(model, 'n_am1_i0'); %**
[~,~,base{2},~] = mphevaluate(model, 'n_am1_Ds'); %**

para_0 = [1, 1];  % [i0, Ds]
para_lb = [-1e5, -1e5];
para_ub = [1e5, 1e5];

% Set initial parameters in COMSOL model
model.param.set('n_am1_i0', [num2str(para_0(1)) '*' base{1}]); %**
model.param.set('n_am1_Ds', [num2str(para_0(2)) '*' base{2}]); %**
model.study('std1').run;

SOC_init = mphglobal(model, 'SOC', 'dataset', 'dset1');
V_init = mphglobal(model, 'E_cell', 'dataset', 'dset1');

%% Cost function and optimization using lsqcurvefit
options = optimoptions(@lsqcurvefit, 'Display', 'iter', 'MaxIterations', 1);

% Objective function for lsqcurvefit
fhandle_cost = @(para, SOC_exp)func_cost_lsqcurvefit(para, SOC_exp, model, base);

tic2 = tic;
[para_hat, resnorm] = lsqcurvefit(fhandle_cost, para_0, SOC_now, V_now, para_lb, para_ub, options);
toc(tic2);

% Update model with fitted parameter
model.param.set('n_am1_i0', [num2str(para_hat(1)) '*' base{1}]); %**
model.param.set('n_am1_Ds', [num2str(para_hat(2)) '*' base{2}]); %**
model.study('std1').run;

SOC_fit = mphglobal(model, 'SOC', 'dataset', 'dset1');
V_fit = mphglobal(model, 'E_cell', 'dataset', 'dset1');

%% Plot fitted result
figure(2)
plot(SOC_now, V_now, 'ko'); hold on
plot(SOC_init, V_init, 'b-');
plot(SOC_fit, V_fit, 'r-');
legend({'Target Data', 'Initial Guess', 'Model Fit'})
xlabel('SOC')
ylabel('V')
grid on
title('Model Fitting Results')

%% Cost Function Definition for lsqcurvefit
function V_model_interp = func_cost_lsqcurvefit(para, SOC_exp, model, base)

    model.param.set('n_am1_i0', [num2str(para(1)) '*' base{1}]); %**
    model.param.set('n_am1_Ds', [num2str(para(2)) '*' base{2}]); %**
   
    disp(para); % Debugging to see the parameters

    tic1 = tic; % Measure time for each model run
    model.study('std1').run;
    toc(tic1); % Display time taken
    
    SOC_model = mphglobal(model, 'SOC', 'dataset', 'dset1');
    V_model = mphglobal(model, 'E_cell', 'dataset', 'dset1');

    % 중복값 제거
    [SOC_model_unique, unique_idx] = unique(SOC_model);
    V_model_unique = V_model(unique_idx);

    % Interpolate model voltage to match experimental SOC points
    V_model_interp = interp1(SOC_model_unique, V_model_unique, SOC_exp, 'linear', 'extrap');
end
