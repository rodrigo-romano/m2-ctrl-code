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
c2d_opts = c2dOptions('Method','foh');

% PI compensator
Cpi_d = c2d(st.asm.fpi,Ts,c2d_opts);
% Numerical differentiation
Hpd_d = c2d(st.asm.fpd,Ts,c2d_opts);


%% ASM feedforward (FF) modal controller

ModelFolder = fullfile(im.lfFolder,"20210611_1336_MT_mount_v202104_ASM_full_epsilon");
% ModelFolder = fullfile(im.lfFolder,"20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111");

FileName = "modal_state_space_model_2ndOrder.mat";
    load(fullfile(ModelFolder,FileName),'inputs2ModalF','modalDisp2Outputs',...
        'eigenfrequencies','proportionalDampingVec','inputTable','outputTable');

fprintf('Loading model file %s\n from folder \n%s\n', FileName, ModelFolder);

%%





% % INPUT MATRIX
% in1 = inputTable{sprintf('MC_M2_S%d_VC_delta_F',m2_seg),"indices"}{1}(:);
% in2 = inputTable{sprintf('MC_M2_S%d_fluid_damping_F',m2_seg),"indices"}{1}(:);
% b_ = [B(:,in1)*XiFS(:,1:n_Zmodes),B(:,in2)*XiFS(:,1:n_Zmodes)];
% nu = size(b_,2);
% 
% % OUTPUT MATRIX
% out1 = outputTable{sprintf('MC_M2_S%d_VC_delta_D',m2_seg),"indices"}{1}(:);
% out2 = outputTable{sprintf('M2_segment_%d_axial_d',m2_seg),"indices"}{1}(:);
% c_ = [XiFS(:,1:n_Zmodes)'*C(out1,:); XiFS(:,1:n_Zmodes)'*C(out2,:)];


% Loop over the segments to:
% 1) Create modal transformation matrix
XiFS = cell(7,1);
% 2) Compute ASM modal stiffness matrices
VC_modal_stiff = cell(7,1);

for iseg = 1:7
    in_label = sprintf('MC_M2_S%d_VC_delta_F',iseg);
    % Take coordinates of the ASM FS nodes
    % (not necessarily the same in all segments)
    nDim = inputTable(in_label,1).size;
    xFS = zeros(nDim,1); yFS = zeros(nDim,1);
    
    for ii = 1:nDim
        props = inputTable(in_label,:).properties{1}{ii};
        xFS(ii) = props.location(2,1); yFS(ii) = props.location(2,2);
    end
    
    % Check for location differences between segments
    if(iseg>1 && (~isequal(xFS_,xFS) || ~isequal(yFS_,yFS)))
        fprintf('FS node coordinates of S%d and S%d are different!\n',iseg,iseg-1);
    end
    
    
    % Modal transformation matrix
    pmax = 10;  % 66 modes
    [XiFS{iseg},~,~,n_Zmodes] = utils.zernike(complex(xFS,yFS),pmax);
    if iseg==1, fprintf('Number of vector basis vectors:%d \n',n_Zmodes);
    end
    
    % VC IO indexes
    in_ = inputTable{sprintf('MC_M2_S%d_VC_delta_F',iseg),"indices"}{1}(:);
    out_ = outputTable{sprintf('MC_M2_S%d_VC_delta_D',iseg),"indices"}{1}(:);
    
    % Compute the VC modal stiffness matrix    
    DCg = XiFS{iseg}' * modalDisp2Outputs(out_,4:end) *...
        diag(1./((2*pi*eigenfrequencies(4:end)).^2)) *...
        inputs2ModalF(4:end,in_) * XiFS{iseg};
    VC_modal_stiff{iseg} = eye(n_Zmodes) / DCg;
    
    % Save coordinates to check for differences between segments
    xFS_ = xFS; yFS_ = yFS;
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
V_S1 = XiFS{1}; V_S2 = XiFS{2}; V_S3 = XiFS{3}; V_S4 = XiFS{4};
V_S5 = XiFS{5}; V_S6 = XiFS{6}; V_S7 = XiFS{7};
KsS1_66 = Ks{1}; KsS2_66 = Ks{2}; KsS3_66 = Ks{3};
KsS4_66 = Ks{4}; KsS5_66 = Ks{5}; KsS6_66 = Ks{6}; KsS7_66 = Ks{7};

save('../calib_dt/m2asm_ctrl_dt.mat','Km', 'Kb',...
    'KsS1_66', 'KsS2_66', 'KsS3_66', 'KsS4_66',...
    'KsS5_66', 'KsS6_66', 'KsS7_66',...
    'V_S1','V_S2','V_S3','V_S4','V_S5','V_S6','V_S7');

% Test/verification step data
% Columns of preshapeBessel_step_y: [cmd_f, dot_cmd_f, ddot_cmd_f]
[preshapeBessel_step_y,preshapeBessel_step_t] = step(flag_d);
G_fb_fd = [-Cpi_d-st.asm.Kd*Hpd_d;-st.asm.Kfd*Hpd_d];
% Columns of asm_fb_y:
[asm_fb_imp_y, asm_fb_imp_t] = impulse(G_fb_fd);

if (update_test_dt && ~exist('m2asm_tests','var'))
    save('m2asm_tests','preshapeBessel_step_y','preshapeBessel_step_t',...
        'asm_fb_imp_y','asm_fb_imp_t');
end







%%
%%

% FileName = fullfile(ModelFolder,"modal_state_space_model_2ndOrder_postproc_45m1_66m2Modes_.mat");
% if(~exist('FEM_IO','var') || 0)
%     load(FileName,'FEM_IO','Phi','Phim','eigenfrequencies','proportionalDampingVec');
% end
% 
% fprintf('Model from %s loaded.\n', FileName);
% 
% % Matrix with initial (col1) and final (col2) indices of each output
% get_IO_ranges = @(x)[cumsum(x)-x+1,cumsum(x)];
% 
% FEM_output_ind_dt = get_IO_ranges(FEM_IO.outputs_size);
% FEM_input_ind_dt = get_IO_ranges(FEM_IO.inputs_size);
% 
% % Compute ASM modal stiffness matrices
% VC_modal_stiff = cell(7,1);
% for iseg = 1:7
%     % VC IO indexes
%     idx = contains(FEM_IO.inputs_name,sprintf('M2_S%d_FS-CP_modal_F',iseg));
%     in_idxs = FEM_input_ind_dt(idx,:);
%     idx = contains(FEM_IO.outputs_name,sprintf('M2_S%d_FS-RB_modal_D',iseg));
%     out_idxs = FEM_output_ind_dt(idx,:);
%     
%     % Compute the VC modal stiffness matrix
% %     DCg = -C(out_idxs(1):out_idxs(2),:)*(A\B(:,in_idxs(1):in_idxs(2)));
%     
%     DCg = Phi(out_idxs(1):out_idxs(2),4:end) *...
%         diag(1./((2*pi*eigenfrequencies(4:end)).^2)) *...
%         Phim(in_idxs(1):in_idxs(2),4:end)';
%     VC_modal_stiff{iseg} = inv(DCg);
% end
% 
% % Update Ks with ASM modal stiffness
% Ks = VC_modal_stiff;

