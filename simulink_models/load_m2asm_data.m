%
% load_m2asm_data.m
%
% Load variables to compile M2ASM controller
% 
% Fev, 2023: Segment-wise implementation

% Flag to save/update controller data file
update_test_dt = true;
% Controller sampling period
Ts = 1/8e3;


%%

% st variable

% Load parameters of controllers and other subsystems

% File with controller and interface parameters
fprintf('Loading the controller TFs using getcontrol_asm()\n');
asm_script_folder = '/Users/rromano/Workspace/GMT-IMS/controllers';
addpath(asm_script_folder)
st = getcontrol_asm();
if(1)
    fc=44.62;   % [Hz] controller unit-magnitude crossover
    f2=22.4;    % [Hz] low-freq-lag
    st.ngao.fao=tf(2*pi*fc*1,[1 0])*tf([1 2*pi*f2],[1 0]);
end

rmpath(asm_script_folder)

%% ASM inner loop controller preshape filter
%%
if(1)
    % Discretization settings
    c2d_opts = c2dOptions('Method','foh');

    % Set to 1 to use original pre shape filter (fc=2200Hz)
    if(true)
        fpre = st.asm.fpre;
    else
        fpre_bw = 2200; % [Hz] prefilter break frequency
        dd = [1.00000000 3.12393994 4.39155033 3.20108587 1.00000000]; % for wc=1
        dd = dd.*(2*pi*fpre_bw).^(0:4); % for wc=2*pi*fpre_bw
        nn = [0 0 0 0 dd(end)]; % for unity dc gain
        fpre = tf(nn,dd);
    end
    % FF contributions
    [a,b,c]=ssdata(fpre);
    flag=ss(a,b,[c;c*a;c*(a^2)],[0;c*b;c*a*b]); % [1;s;s^2]*fpre
    % Discrete-time FF controller TFs - 4th-order Bessel approx
    flag_d = c2d(flag,Ts,c2d_opts);
else
    % ASM Shaping filter - AdOptica' implementation
    sm.tSF = 1/2200; %#ok<*UNRCH>
    sm.d1SF = 60/sm.tSF^3;
    sm.d2SF = -180/sm.tSF^4;
    sm.d3SF = 120/sm.tSF^5;
    sm.TCmd = st.ngao.T;
end


%% ASM inner loop feedbaack (PI+D) controller TFs
%%

% ASM inner loop controller discretization method
c2d_opts = c2dOptions('Method','foh');%,'PrewarpFrequency',fms_wc/2); %

% PI compensator
Cpi_d = c2d(st.asm.fpi,Ts,c2d_opts);
% Numerical differentiation
Hpd_d = c2d(st.asm.fpd,Ts,c2d_opts);

G_fb_fd = [-Cpi_d-st.asm.Kd*Hpd_d;-st.asm.Kfd*Hpd_d];

%% ASM feedforward (FF) modal controller

ModelFolder = fullfile(im.lfFolder,"20210611_1336_MT_mount_v202104_ASM_full_epsilon");

FileName = fullfile(ModelFolder,"modal_state_space_model_2ndOrder_postproc_45m1_66m2Modes_.mat");
if(~exist('FEM_IO','var') || 0)
    load(FileName,'FEM_IO','Phi','Phim','eigenfrequencies','proportionalDampingVec');
end
fprintf('Model from %s loaded.\n', FileName);

% Matrix with initial (col1) and final (col2) indices of each output
get_IO_ranges = @(x)[cumsum(x)-x+1,cumsum(x)];

FEM_output_ind_dt = get_IO_ranges(FEM_IO.outputs_size);
FEM_input_ind_dt = get_IO_ranges(FEM_IO.inputs_size);

% Compute ASM modal stiffness matrices
VC_modal_stiff = cell(7,1);
for iseg = 1:7
    % VC IO indexes
    idx = contains(FEM_IO.inputs_name,sprintf('M2_S%d_FS-CP_modal_F',iseg));
    in_idxs = FEM_input_ind_dt(idx,:);
    idx = contains(FEM_IO.outputs_name,sprintf('M2_S%d_FS-RB_modal_D',iseg));
    out_idxs = FEM_output_ind_dt(idx,:);
    
    % Compute the VC modal stiffness matrix
%     DCg = -C(out_idxs(1):out_idxs(2),:)*(A\B(:,in_idxs(1):in_idxs(2)));
    
    DCg = Phi(out_idxs(1):out_idxs(2),4:end) *...
        diag(1./((2*pi*eigenfrequencies(4:end)).^2)) *...
        Phim(in_idxs(1):in_idxs(2),4:end)';
    VC_modal_stiff{iseg} = inv(DCg);
end

% Update Ks with ASM modal stiffness
Ks = VC_modal_stiff;


%% Load simulink model
ModelFName = 'm2asm_2_rust';
open(sprintf('%s.slx',ModelFName));
deltaT = Ts;    % Fixed-step solver sampling period


%% Calibration-dependent matrices
%%

Km = st.asm.Km; Kb = st.asm.Kb;
KsS1_66 = Ks{1}; KsS2_66 = Ks{2}; KsS3_66 = Ks{3};
KsS4_66 = Ks{4}; KsS5_66 = Ks{5}; KsS6_66 = Ks{6}; KsS7_66 = Ks{7};

save('../calib_dt/m2asm_ctrl_dt.mat',...
    'KsS1_66', 'KsS2_66', 'KsS3_66', 'KsS4_66',...
    'KsS5_66', 'KsS6_66', 'KsS7_66', 'Km', 'Kb');

% Test/verification step data
% Columns of preshapeBessel_step_y: [cmd_f, dot_cmd_f, ddot_cmd_f]
[preshapeBessel_step_y,preshapeBessel_step_t] = step(flag_d);
G_fb_fd = [-Cpi_d-st.asm.Kd*Hpd_d;-st.asm.Kfd*Hpd_d];
% Columns of asm_fb_y:
[asm_fb_y, asm_fb_t] = impulse(G_fb_fd);

if (update_test_dt && ~exist('m2asm_tests','var'))
    save('m2asm_tests','preshapeBessel_step_y','preshapeBessel_step_t',...
        'asm_fb_y','asm_fb_t');
end


%% Clutter
%%
% 
% hplc_label = sprintf('%s/M1_HP_loadcells', ModelFName);
% hp_RBMtoD_label = sprintf('%s/M1RBM_to_HP_relD', ModelFName);
% hp_dyn_label = sprintf('%s/M1_HP_Dynamics', ModelFName);
% hp_stiff_label = sprintf('%s/HPk', ModelFName);
% m1SA_C_CS_label = sprintf('%s/M1SA_Control_CS' ,ModelFName);
% m1SA_C_OA_label = sprintf('%s/M1SA_Control_OA', ModelFName);
% 
% switch build_subsys
%     case 'M1_SA'
%         n_bm = 27; %size(m1sys{2}.m1BM2F,2);
%         % Solver sampling period
%         deltaT = m1sys{1}.ofl.Ts;
%         set_param(hplc_label,'Commented','on');
%         set_param(hp_RBMtoD_label,'Commented','on');
%         set_param(hp_dyn_label,'Commented','on');
%         set_param(hp_stiff_label,'Commented','on');
%         set_param(m1SA_C_OA_label,'Commented','off');
%         set_param(m1SA_C_CS_label,'Commented','off');
%         % Test data
%         for i1 = 1:numel(m1sys{1}.ofl.SSdtC)
%             OActrl(i1,i1) = m1sys{1}.ofl.SSdtC{i1}; %#ok<*SAGROW>
%             CSctrl(i1,i1) = m1sys{7}.ofl.SSdtC{i1};
%         end
%         OAsys = m1sys{1}.SAdyn*m1sys{1}.Kbal*OActrl;
%         CSsys = m1sys{7}.SAdyn*m1sys{7}.Kbal*CSctrl;
%         [OAact_imp_y,OAact_imp_t] = impulse(OAsys);
%         [CSact_imp_y,CSact_imp_t] = impulse(CSsys);
% 
%         if (update_test_dt && ~exist('m1_act_impulse_test','var'))
%             save m1_act_impulse_test OAact_imp_t OAact_imp_y CSact_imp_t CSact_imp_y
%         end
%         
%     case 'HP_dyn'
%         % Solver sampling period
%         deltaT = m1sys{1}.HPdtf.Ts;
%         set_param(hplc_label,'Commented','off');
%         set_param(hp_RBMtoD_label,'Commented','off');
%         set_param(hp_dyn_label,'Commented','off');
%         set_param(hp_stiff_label,'Commented','off');
%         set_param(m1SA_C_OA_label,'Commented','on');
%         set_param(m1SA_C_CS_label,'Commented','on');
%         
%         % Test data
%         [hp_dyn_step_y,hp_dyn_step_t] = step(m1sys{1}.HPdtf);
%         if (update_test_dt && ~exist('hp_dyn_step_test','var'))
%             save hp_dyn_step_test hp_dyn_step_t hp_dyn_step_y
%         end
% end
% 
% 
