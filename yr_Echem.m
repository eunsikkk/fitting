clc; clear; close all;
%% Load COMSOL Model
import com.comsol.model.*
import com.comsol.model.util.*

model = mphload("G:\공유 드라이브\BSL-Data\Processed_data\Hyundai_dataset\OCV\es_ex_1C\JYR_cell_echem_parachange.mph");

ModelUtil.showProgress(true);

% mphnavigator %**
%model.study('std1').feature('time').set('tlist', 0:100:5000);%**


% model.study('std1').run; % ** 중복
% 
% % Initial model
% SOC = mphglobal(model, 'SOC', 'dataset', 'dset1');
% V = mphglobal(model, 'E_cell', 'dataset', 'dset1');


%% Load Data
data_folder = 'G:\공유 드라이브\BSL-Data\Processed_data\Hyundai_dataset\OCV\es_ex_1C';
filename_data = "results_f.mat";
data_now = load(fullfile(data_folder, filename_data));
SOC_now = data_now.results_f(3).SOC_vec;
V_now = data_now.results_f(3).V_vec; % 3rd entry for C-rate == 1 이 부분 수정생각중

% subplot 하기 (initial comparison)

% figure(1)
% subplot(2,1,1)
% plot(SOC, V)
% xlabel('SOC')
% ylabel('V')
% grid on
% subplot(2,1,2)
% plot(SOC_now,V_now)
% xlabel('SOC')
% ylabel('V')
% grid on

%% Initial guess

[~,~,base{1},~] = mphevaluate(model, 'n_am1_i0'); %**
[~,~,base{2},~] = mphevaluate(model, 'n_am1_Ds'); %**
% [~,~,base{3},~] = mphevaluate(model, 'p_am1_i0'); %**
% [~,~,base{4},~] = mphevaluate(model, 'p_am1_Ds'); %**

para_0 = [1, 1];  % [i0, Ds]
% para_lb = [-1e3,-1e5];
% para_ub = [1e3,1e5];
para_lb = [-Inf,-Inf];
para_ub = [Inf,Inf];


% Set initial parameters in COMSOL model
model.param.set('n_am1_i0', [num2str(para_0(1)) '*' base{1}]); %**
model.param.set('n_am1_Ds', [num2str(para_0(2)) '*' base{2}]); %**
% model.param.set('p_am1_i0', [num2str(para_0(3)) '*' base{3}]); %**
% model.param.set('p_am1_Ds', [num2str(para_0(4)) '*' base{4}]); %**
model.study('std1').run;


SOC_init = mphglobal(model, 'SOC', 'dataset', 'dset1');
V_init = mphglobal(model, 'E_cell', 'dataset', 'dset1');


% case 1 cose
% cost_0 =func_cost(para_0, SOC_now, V_now, model,base);  
% cost_1 = func_cost(para_0*16, SOC_now, V_now, model,base);

%model.study('std1').feature('time').set('tlist', 'range(0,0.01,10)');  % 타임스텝을 0.001 단위로 시작하여 10까지 계산

%% Cost function
options = optimoptions(@fmincon,'Display','iter','MaxIterations',2);
 
fhandle_cost = @(para)func_cost(para, SOC_now, V_now, model,base);
tic2=tic;% **
para_hat = fmincon(fhandle_cost, para_0, [], [], [], [], para_lb, para_ub, [], options);


% para_hat = para_0*16 ;

toc(tic2)% **
% Update model with fitted parameter
model.param.set('n_am1_i0', [num2str(para_hat(1)) '*' base{1}]); %**
model.param.set('n_am1_Ds', [num2str(para_hat(2)) '*' base{2}]); %**
% model.param.set('p_am1_i0', [num2str(para_hat(3)) '*' base{3}]); %**
% model.param.set('p_am1_Ds', [num2str(para_hat(4)) '*' base{4}]); %**

model.study('std1').run;

SOC_fit = mphglobal(model, 'SOC', 'dataset', 'dset1');
V_fit = mphglobal(model, 'E_cell', 'dataset', 'dset1');

%% plot fitted result
figure(2)
plot(SOC_now, V_now, 'ko'); hold on
plot(SOC_init, V_init, 'b-');
plot(SOC_fit, V_fit, 'r-');
legend({'Target Data','initial guess','Model Fit'})
xlabel('SOC')
ylabel('V')
grid on
title('Model Fitting Results')
%% Cost Function Definition
function cost = func_cost(para, SOC_exp, V_exp, model,base)

    model.param.set('n_am1_i0', [num2str(para(1)) '*' base{1}]); %**
    model.param.set('n_am1_Ds', [num2str(para(2)) '*' base{2}]); %**
    % model.param.set('p_am1_i0', [num2str(para(3)) '*' base{3}]); %**
    % model.param.set('p_am1_Ds', [num2str(para(4)) '*' base{4}]); %**
   
   %pause% **
   disp(para)% ** % 왜 여러번 이터 되는지 확인 필요 
   % 

   tic1 = tic; % **
   model.study('std1').run;
   toc(tic1)% **
    
   SOC_model = mphglobal(model, 'SOC', 'dataset', 'dset1');
   V_model = mphglobal(model, 'E_cell', 'dataset', 'dset1');
   % 중복값 제거
   [SOC_model_unique, unique_idx] = unique(SOC_model);
   V_model_unique = V_model(unique_idx);

   V_model_interp = interp1(SOC_model_unique, V_model_unique, SOC_exp, 'spline', 'extrap');

   cost = sqrt(mean((V_exp - V_model_interp).^2));
end