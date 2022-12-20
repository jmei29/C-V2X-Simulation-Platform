%% suburban deployment modified by ZYK 2011/04/14 delete femto cell
%% suburban femto deployment changed by WWJ 2010/06/15
%% suburban femto deployment by XLL 2010/05/20
%% 19 Macro eNBs / Macro with Femto
%% delete the wrap round part in node deployment, but add this in SINR
%% calculation ---- WWJ
warning off MATLAB:DeprecatedLogicalAPI
clear;
clc;
addpath('../data_deploy');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Simulation Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
drop_num = 1;
drop_duration=1000; % in ms
update_duration=100; % in ms
sub_drop_num=drop_duration/update_duration; % in ms
tic
%% *****        scenario parameters         ********%%
ISD = 1732;% Inter Sites Distance is 1732 m
R = ISD/sqrt(3); % Macro cell radius
Road_length=2*ISD; %Simulation region is 2*ISD>2000m
Lane_width=4; %Lane width is 4 m
Lane_num=6; %3 in each direction (6 lanes in total in the freeway)
loca_x_max=Road_length/2+ISD/2;
loca_x_min=-Road_length/2+ISD/2;
%% parameters before wrap-around
MeNB_num = 2;% MeNB number
sector_num_per_cell = 3;% sector number per cell
Total_sec_num = sector_num_per_cell*MeNB_num;% total sector number
vehicle_speed=70;%Absolute vehicle speed is 70 or 140 km/h
vehicle_distance=2.5*(vehicle_speed*5/18);%Average inter-vehicle distance in the same lane is 2.5(sec)*absolute vehicle speed(m/s). 
lambda = floor(Road_length/vehicle_distance);% MUE number per lane,MJ

%% minimum distance between two nodes
min_distance_VUE_MeNB = 35;% in m , Minimum distance between V-UE and MeNB
min_distance_VUE_VUE_1 = vehicle_distance;% in m, Minimum separation of two V-UEs
min_distance_VUE_VUE_2 = 0.8*vehicle_distance;% in m, Minimum separation of two V-UEs
min_distance_VUE_VUE_3 = 0.4*vehicle_distance;% in m, Minimum separation of two V-UEs
min_distance_VUE_VUE_4 = 0.1*vehicle_distance;% in m, Minimum separation of two V-UEs

%% cell splitting radius
% theta_min = pi-atan(ISD/2/25);
% theta_max = pi-atan(min_distance_VUE_MeNB/25);
% theta_scope = theta_max-theta_min;
% r_half_theta = 25*tan(pi-(theta_scope/2+theta_min));
% 
% r_half_size=(UE_deploy_size*0.5/pi+0.5*min_distance_VUE_MeNB^2)^0.5;
%% shadow fading parameters
Shadow_std_V2I = 8;% in dB, V2I scenario: the standard deviation of shadow fading of all links except links between a HeNB and a UE served by this eNB
Shadow_std_V2V_LOS=3;% in dB, V2I scenario: LOS
Shadow_std_V2V_NLOS=4;% in dB, V2I scenario: NLOS
correlation_matrix=0.5*ones(MeNB_num,MeNB_num)+0.5*diag(ones(1,MeNB_num)); % A Shadowing correlation factor of 0.5 for the shadowing between eNB sites  
R_matrix=sqrt(correlation_matrix);
D_corr_V2I=50; %V2I: Decorrelation distance: 50 m
D_corr_V2V=25; %V2V: Decorrelation distance: 50 m 
%and of 1.0 between sectors of the same eNB site are used 
% save parameters
savefile = sprintf('../data_deploy/node_deployment_Freeway_parameters_vehicle_speed=%d.mat',vehicle_speed);
save(savefile,'drop_num','sub_drop_num','ISD','R', ...
    'MeNB_num','sector_num_per_cell','Total_sec_num','Road_length','loca_x_max','loca_x_min',...
    'min_distance_VUE_MeNB','vehicle_speed','Lane_width','Lane_num',...
    'min_distance_VUE_VUE_1','min_distance_VUE_VUE_2','min_distance_VUE_VUE_3','min_distance_VUE_VUE_4',...
    'Shadow_std_V2I','Shadow_std_V2V_LOS','Shadow_std_V2V_NLOS','D_corr_V2I','D_corr_V2V','R_matrix','drop_duration');

%% **************    Simulation Start!!!    ******************* %%
fprintf('%% **************    Simulation Start!!! %d drops.   ******************* %%\n',drop_num);
fprintf('VANET Freeway case. Node deployment.\n %d MeNBs.\n %d sectors per Macro cell. %d Lane. \n ',...
    MeNB_num,sector_num_per_cell,Lane_num);
fprintf('\n!!!!!!!!!!!! shadow fading correlation between cells is 0.5 , between sectors is 1.0 !!!!!!!!!!!!!!\n\n');
%% statistics
UE_indoor_ratio=0.8;
cyclic_count_limit = 200;% the limit of cyclic times
cyclic_over_limit = 0;
break_limit = 1000;% if cyclic_count >= 1000, start a new drop
break_flag = 1;% 0: need to start a new drop
snapshot = 1;
real_snapshot = 0;
for snapshot = 1:drop_num
    % Spatial Poisson Process
    VUE_num_per_lane=poissrnd(lambda,Lane_num,1);% MUE number per sector, modified from 20 to 30,ZYK
    Total_VUE_num = sum(VUE_num_per_lane);% MUE number of 19 Macro Cells
    break_flag = 1;% 0: need to start a new drop

    %% statistics
    cyclic_over_limit = 0;
    
    %% ***********   scenario deployment    ****************%%
    % -------------------    memory preallocation    ---------------------%
    MeNB_loca = zeros(MeNB_num,1);
    % ********************   MeNB deployment     *********************** %
    % consider 19 MeNBs
    MeNB_loca(1,1) = 0+0j;% set MeNB1 as (0,0)
    MeNB_loca(2,1) = sqrt(3)*R+1i*0;
    %--------------------MUE deployment in 19 Macro cells--------------------%
    % 19 cells, deploy MUEs randomly and uniformly cell by cell
    VUE_info_start= zeros(Total_VUE_num,4);%[1.VUEs' location;2.VUE direction(=1 North,2 West,3 South,4 East);3.height;4.Lane_index]
    for loop_lane = 1:Lane_num
        VUE_info_per_lane = zeros(VUE_num_per_lane(loop_lane),3);%[1.VUEs' location;2.VUE direction(=1 North,2 West,3 South,4 East);3.height;4.Lane_index]
        for loop_VUE = 1:VUE_num_per_lane(loop_lane)
            loca_temp = 0;
            flag_loca = 1;  
        %% MUE is not inside Femto houses
            cyclic_count = 0;
            while flag_loca == 1
                cyclic_count = cyclic_count + 1;
                if cyclic_count >= break_limit
                    fprintf('\n BREAK! \n');
                    break_flag = 0;break;% to start a new drop
                end
                %% deploy VUE in each lane
                loca_x = Road_length*(rand-0.5); %%rand is between 0 and 1
                loca_y = 10-4*loop_lane;
                %% relocate MUE which is in the two triangle areas
                loca_x=loca_x+ISD/2;
                loca_y=loca_y-min_distance_VUE_MeNB-12;
                loca_temp = loca_x + 1j*loca_y;% temporary location of MUE
                %% MUE is not inside a femto house
                %% check if MUE is far away enough from MeNB(0,0)
                if abs(loca_temp) < min_distance_VUE_MeNB
                    continue;
                end
                if cyclic_count <= cyclic_count_limit
                    %% check if MUE is far away enough from other MUEs in current cell
                    if loop_VUE > 1 &&  min(abs(loca_temp-VUE_info_per_lane(1:loop_VUE-1,1))) < min_distance_VUE_VUE_1
                        continue;
                    end
                elseif cyclic_count <= 2*cyclic_count_limit
                    %% check if MUE is far away enough from other MUEs in current cell
                    if loop_VUE > 1 &&  min(abs(loca_temp-VUE_info_per_lane(1:loop_VUE-1,1))) < min_distance_VUE_VUE_2
                        continue;
                    end
                end
                VUE_info_per_lane(loop_VUE,1) = loca_temp;% MUE location
                if loop_lane<=3
                VUE_info_per_lane(loop_VUE,2) = 1;% left  
                else
                VUE_info_per_lane(loop_VUE,2) = 2;% right      
                end
                VUE_info_per_lane(loop_VUE,3) = 1.5;%outdoor UE height  
                VUE_info_per_lane(loop_VUE,4) = loop_lane;% lane index 
                flag_loca = 0;% the location has been found
            end
            if cyclic_count > cyclic_count_limit
                cyclic_over_limit = cyclic_over_limit + 1;
                %fprintf('\ncyclic_count=%d',cyclic_count);
            end
            if break_flag == 0  %added by bin
                break;% to start a new drop
            end
        end % end of loop_MUE  
        if break_flag == 0
            break;% to start a new drop
        end
        %% relocate MUEs in current cell
        VUE_info_start([1:VUE_num_per_lane(loop_lane)]+sum(VUE_num_per_lane(1:loop_lane-1,1)),1) = VUE_info_per_lane(1:VUE_num_per_lane(loop_lane),1); % location
        VUE_info_start([1:VUE_num_per_lane(loop_lane)]+sum(VUE_num_per_lane(1:loop_lane-1,1)),2) = VUE_info_per_lane(1:VUE_num_per_lane(loop_lane),2); % direction
        VUE_info_start([1:VUE_num_per_lane(loop_lane)]+sum(VUE_num_per_lane(1:loop_lane-1,1)),3) = VUE_info_per_lane(1:VUE_num_per_lane(loop_lane),3); % Vehicle height
        VUE_info_start([1:VUE_num_per_lane(loop_lane)]+sum(VUE_num_per_lane(1:loop_lane-1,1)),4) = VUE_info_per_lane(1:VUE_num_per_lane(loop_lane),4); % lane index
    end % end of loop_cell    
    if break_flag == 0
        continue;% to start a new drop
    end
    real_snapshot = real_snapshot + 1;    
    %% update location and shadowing
    for sub_snapshot=1:sub_drop_num    
        VUE_loca_WRAP=zeros(Total_VUE_num,2);
        if sub_snapshot==1
         VUE_info = zeros(Total_VUE_num,6); %[1.MUEs' location;2.UE direction(=1 left,2 right);3.height;4.lane_index;5.sector(=1,2,3);6.Service Type]
         VUE_info(:,2) = VUE_info_start(:,2); % direction
         VUE_info(:,3) = VUE_info_start(:,3); % Vehicle height
         VUE_info(:,4) = VUE_info_start(:,4); % lane_index
        % update location
        VUE_info(:,1)=VUE_info_start(:,1); %location
        %% distance between eNB and VUE after wrapround,MeNB_num*Total_VUE_num matrix
        [Distance_eNB2VUE, eNB_loca_WRAP]=WRAP_eNB2VUE_distance(VUE_info(:,1),MeNB_loca,ISD,Total_VUE_num,MeNB_num);
        Distance_eNB2VUE_pre=Distance_eNB2VUE;
        %% distance between VUE and VUE after wrapround,MeNB_num*Total_VUE_num matrix
        [Distance_VUE2VUE, VUE_loca_WRAP]=WRAP_VUE2VUE_distance(VUE_info(:,1),ISD,Total_VUE_num);
        Distance_VUE2VUE_pre=Distance_VUE2VUE;
        % update shadowing fading V2V
        Shadowing_VUE2VUE_LOS=Shadow_std_V2V_LOS*randn(Total_VUE_num,Total_VUE_num);
        Shadowing_VUE2VUE_LOS=triu(Shadowing_VUE2VUE_LOS,0)+tril(Shadowing_VUE2VUE_LOS',-1);
        Shadowing_VUE2VUE_LOS=Shadowing_VUE2VUE_LOS+diag(-diag(Shadowing_VUE2VUE_LOS));% shadow fading, std=3 dB
        Shadowing_VUE2VUE_NLOS=Shadow_std_V2V_NLOS*randn(Total_VUE_num,Total_VUE_num);
        Shadowing_VUE2VUE_NLOS=triu(Shadowing_VUE2VUE_NLOS,0)+tril(Shadowing_VUE2VUE_NLOS',-1);
        Shadowing_VUE2VUE_NLOS=Shadowing_VUE2VUE_NLOS+diag(-diag(Shadowing_VUE2VUE_NLOS));% shadow fading, std=4 dB
        % update shadowing fading V2I
        Shadowing_eNB2VUE=R_matrix*Shadow_std_V2I*randn(MeNB_num,Total_VUE_num); % Initialization at time 0 of every drop, MeNB_num*Total_VUE_num matrix 
        else %sub_snapshot>1
      %% update location
        for loop_VUE=1:Total_VUE_num
        if VUE_info(loop_VUE,2)==1 %left
           VUE_info(loop_VUE,1)=VUE_info(loop_VUE,1)-(vehicle_speed*5/18)*0.1;
           if VUE_info(loop_VUE,1)<loca_x_min %vehicle wrapround
               VUE_info(loop_VUE,1)=VUE_info(loop_VUE,1)+Road_length;
           end
        else
           VUE_info(loop_VUE,1)=VUE_info(loop_VUE,1)+(vehicle_speed*5/18)*0.1;
           if VUE_info(loop_VUE,1)>loca_x_max %vehicle wrapround
               VUE_info(loop_VUE,1)=VUE_info(loop_VUE,1)-Road_length;
           end
        end
        end %end of loop_VUE  
       %% change of distance after wrapround 
       [Distance_eNB2VUE, eNB_loca_WRAP]=WRAP_eNB2VUE_distance(VUE_info(:,1),MeNB_loca,ISD,Total_VUE_num,MeNB_num);
       Distance_eNB2VUE_change=abs(Distance_eNB2VUE-Distance_eNB2VUE_pre); %change in distance of the UE to the eNB site
       Distance_eNB2VUE_pre=Distance_eNB2VUE;
       [Distance_VUE2VUE, VUE_loca_WRAP]=WRAP_VUE2VUE_distance(VUE_info(:,1),ISD,Total_VUE_num);
%        Distance_VUE2VUE_0=abs(repmat(VUE_info(:,1),1,Total_VUE_num)-VUE_loca_WRAP);
%        Distance_VUE2VUE_DELTA=Distance_VUE2VUE-Distance_VUE2VUE_0;
       Distance_VUE2VUE_change=abs(Distance_VUE2VUE-Distance_VUE2VUE_pre); %change in distance of the UE to the eNB site
       Distance_VUE2VUE_pre=Distance_VUE2VUE;
       %% update shadowing fading V2I
        Shadowing_eNB2VUE=exp(-Distance_eNB2VUE_change/D_corr_V2I).*Shadowing_eNB2VUE+...
           sqrt(1-exp(-2*Distance_eNB2VUE_change/D_corr_V2I)).*(R_matrix*Shadow_std_V2I*randn(MeNB_num,Total_VUE_num));
       %% update shadowing fading V2V
        Y_LOS=Shadow_std_V2V_LOS*randn(Total_VUE_num,Total_VUE_num);
        Y_LOS=triu(Y_LOS,0)+tril(Y_LOS',-1);
        Y_LOS=Y_LOS+diag(-diag(Y_LOS));% shadow fading, std=3 dB
        Shadowing_VUE2VUE_LOS=exp(-Distance_VUE2VUE_change/D_corr_V2V).*Shadowing_VUE2VUE_LOS+sqrt(1-exp(-2*Distance_VUE2VUE_change/D_corr_V2V)).*Y_LOS;
        %% update shadowing fading V2V:NLOS
           Y_VUE2VUE_NLOS=Shadow_std_V2V_NLOS*randn(Total_VUE_num,Total_VUE_num);
           Y_VUE2VUE_NLOS=triu(Y_VUE2VUE_NLOS,0)+tril(Y_VUE2VUE_NLOS',-1);
           Y_VUE2VUE_NLOS=Y_VUE2VUE_NLOS+diag(-diag(Y_VUE2VUE_NLOS));% NLOS shadow fading, std=4 dB
           Shadowing_VUE2VUE_NLOS=exp(-Distance_VUE2VUE_change/D_corr_V2V).*Shadowing_VUE2VUE_NLOS+...
               sqrt(1-exp(-2*Distance_VUE2VUE_change/D_corr_V2V)).*Y_VUE2VUE_NLOS;
        end %end of if sub_snapshot==1
    %% save data
    savefile = sprintf('../data_deploy/node_deployment_Freeway_vehicle_speed=%d_No%d_subdrop_ID%d.mat',vehicle_speed,real_snapshot,sub_snapshot);
    save(savefile,'VUE_num_per_lane','Total_VUE_num','MeNB_loca','VUE_info','Shadowing_eNB2VUE','Shadowing_VUE2VUE_LOS','Shadowing_VUE2VUE_NLOS','real_snapshot',...
        'Distance_VUE2VUE','Distance_eNB2VUE','VUE_loca_WRAP','eNB_loca_WRAP');
    %% print
    fprintf('\n real_snapshot=%d, sub_snapshot=%d,',real_snapshot,sub_snapshot);
    fprintf(' cyclic_over_limit=%d.\n',cyclic_over_limit);
    end %end of sub_snapshot
end % end of snapshot

toc

%% print
fprintf('%% **************    Simulation End!!! %d drops(real).   ******************* %%\n',real_snapshot);
fprintf(' snapshot break = %d times.\n',snapshot-real_snapshot);