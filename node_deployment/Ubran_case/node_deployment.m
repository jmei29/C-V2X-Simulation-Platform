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
%% *****        scenario parameters-street block         ********%%
ISD = 500;% Inter Sites Distance is 1732 m
R = ISD/sqrt(3); % Macro cell radius
Lane_width=3.5; %Lane width is 3.5 m
Sidewalk_width=3;%Sidewalk width is 3 m
Road_grid_num=14;% 14 raod grid in the simulation
Lane_num=4; %2 in each direction (horizon/vertical)
grid_length=433; % the length of grid
grid_width=250; %the width of grid
Road_length_horizon=grid_width-Lane_width*4;
Road_length_vertical=grid_length-Lane_width*4;
road_grid_offset=[-grid_width+1i*grid_length 1i*grid_length grid_width+1i*grid_length 2*grid_width+1i*grid_length...
    -2*grid_width -grid_width 0 grid_width 2*grid_width 3*grid_width...
    -grid_width-1i*grid_length -1i*grid_length grid_width-1i*grid_length 2*grid_width-1i*grid_length];
MeNB_loca_offset=-R/(2*sqrt(3)) + 1i*R/2; %
intersection_loca=[];% the location of intersection
for loop_road_grid=1:Road_grid_num
    intersection_per_grid=[0 1i*grid_length -grid_width+1i*grid_length -grid_width]+road_grid_offset(loop_road_grid);
    intersection_loca=[intersection_loca intersection_per_grid];
end
intersection_loca=unique(intersection_loca);
intersection_num=length(intersection_loca);
Total_CUE_num_block = 50; %the number of CUE in each block
Total_CUE_num = Total_CUE_num_block*Road_grid_num;
%% parameters before wrap-around
MeNB_num = 7;% MeNB number
sector_num_per_cell = 3;% sector number per cell
Total_sec_num = sector_num_per_cell*MeNB_num;% total sector number
vehicle_speed=60;%Absolute vehicle speed is 15 or 60 km/h
vehicle_distance=2.5*(vehicle_speed*5/18);%Average inter-vehicle distance in the same lane is 2.5(sec)*absolute vehicle speed(m/s).
% Total_VUE_num = sum(VUE_num_per_lane);% MUE number of 19 Macro Cells

%% minimum distance between two nodes
min_distance_VUE_MeNB = 35;% in m , Minimum distance between V-UE and MeNB
min_distance_VUE_VUE_1 = vehicle_distance;% in m, Minimum separation of two V-UEs
min_distance_VUE_VUE_2 = 0.8*vehicle_distance;% in m, Minimum separation of two V-UEs
min_distance_VUE_VUE_3 = 0.2*vehicle_distance;% in m, Minimum separation of two V-UEs
min_distance_VUE_VUE_4 = 0.1*vehicle_distance;% in m, Minimum separation of two V-UEs
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
savefile = sprintf('../data_deploy/node_deployment_Urban_parameters_vehicle_speed=%d.mat',vehicle_speed);
save(savefile,'drop_num','sub_drop_num','ISD','R', ...
    'MeNB_num','sector_num_per_cell','Total_sec_num','grid_length','grid_width','MeNB_loca_offset','road_grid_offset',...
    'intersection_loca','intersection_num','Road_grid_num','min_distance_VUE_MeNB','vehicle_speed','vehicle_distance','Road_length_horizon','Road_length_vertical',...
    'min_distance_VUE_VUE_1','min_distance_VUE_VUE_2','min_distance_VUE_VUE_3','min_distance_VUE_VUE_4','Lane_width','Sidewalk_width',...
    'Shadow_std_V2I','Shadow_std_V2V_LOS','Shadow_std_V2V_NLOS','D_corr_V2I','D_corr_V2V','R_matrix','drop_duration');

%% **************    Simulation Start!!!    ******************* %%
fprintf('%% **************    Simulation Start!!! %d drops.   ******************* %%\n',drop_num);
fprintf('VANET Freeway case. Node deployment.\n %d MeNBs.\n %d sectors per Macro cell. %d road grids. \n ',...
    MeNB_num,sector_num_per_cell,Road_grid_num);
%% statistics
UE_indoor_ratio=0.8;
cyclic_count_limit = 200;% the limit of cyclic times
cyclic_over_limit = 0;
break_limit = 1000;% if cyclic_count >= 1000, start a new drop
break_flag = 1;% 0: need to start a new drop
real_snapshot = 0;
for snapshot = 1:drop_num
    
    break_flag = 1;% 0: need to start a new drop
    
    %% statistics
    cyclic_over_limit = 0;
    
    %% ***********   scenario deployment    ****************%%
    % -------------------    memory preallocation    ---------------------%
    MeNB_loca = zeros(MeNB_num,1);
    % ********************   MeNB deployment     *********************** %
    % consider 7 MeNBs
    MeNB_loca(1,1) = 0+0j;% set MeNB1 as (0,0)
    MeNB_loca(2,1) = -sqrt(3)*R+0j;
    MeNB_loca(3,1) = -sqrt(3)*R/2+1i*1.5*R;
    MeNB_loca(4,1) = sqrt(3)*R/2+1i*1.5*R;
    MeNB_loca(5,1) = sqrt(3)*R+1i*0;
    MeNB_loca(6,1) = sqrt(3)/2*R-1i*1.5*R;
    MeNB_loca(7,1) = -sqrt(3)/2*R-1i*1.5*R;
    MeNB_loca(:,1) = MeNB_loca(:,1)+MeNB_loca_offset;
    %--------------------CUE and VUE deployment in 14 Road grid--------------------%
    % 19 cells, deploy MUEs randomly and uniformly cell by cell
    VUE_info_start= [];
    CUE_info = zeros(Total_CUE_num,8);%[1.MUEs' location;2.VUE direction(=1 North,2 West,3 South,4 East);3.height;4.lane_index; 5.Serving sector(=1,2,3);
        %6.if the vehilce is turning(>0,it is turning, 0 it is driving directly); 7.Service Type; 8.Road grid index]
    for loop_road_grid=1:Road_grid_num
        %% parameter set of road grid
        lambda_horizon = floor(Road_length_horizon/vehicle_distance);% VUE number per lane in horizon, MJ
        VUE_num_per_lane_horizon=poissrnd(lambda_horizon,Lane_num,1);% VUE number per sector
        lambda_vertical= floor(Road_length_vertical/vehicle_distance);% VUE number per lane in horizon, MJ
        VUE_num_per_lane_vertical=poissrnd(lambda_vertical,Lane_num,1);% VUE number per sector
        VUE_info_per_grid = zeros(sum(VUE_num_per_lane_horizon)+sum(VUE_num_per_lane_vertical),3); % VUE information in each road grid
        %[1.VUEs' location;2.VUE direction(=1 North,2 West,3 South,4 East);3.height;4.Lane_index]
        %% node deployment in horizon
        for loop_lane_horizon = 1:Lane_num
            VUE_info_per_lane = zeros(VUE_num_per_lane_horizon(loop_lane_horizon),4); % VUE information in each road lane
            for loop_VUE = 1:VUE_num_per_lane_horizon(loop_lane_horizon)
                loca_temp = 0;
                flag_loca = 1;
                cyclic_count = 0;
                while flag_loca == 1
                    cyclic_count = cyclic_count + 1;
                    if cyclic_count >= break_limit
                        fprintf('\n BREAK! \n');
                        break_flag = 0;break;% to start a new drop
                    end
                    %% deploy VUE in each lane
                    loca_x = Road_length_horizon*(rand-1)-2*Lane_width; %%rand is between -1 and 0
                    if loop_lane_horizon<=2
                        loca_y = grid_length-Lane_width/2-(loop_lane_horizon-1)*Lane_width;
                    else
                        loca_y = Lane_width/2+(loop_lane_horizon-3)*Lane_width;
                    end  % temporary location of VUE
                    %% relocate MUE which is in the two triangle areas
                    loca_temp = loca_x + 1j*loca_y+road_grid_offset(loop_road_grid);% location of VUE
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
                    VUE_info_per_lane(loop_VUE,4) = loop_lane_horizon;% lane index
                    if loop_lane_horizon<=2 %loop_lane
                        VUE_info_per_lane(loop_VUE,2) = 4;% East
                    else
                        VUE_info_per_lane(loop_VUE,2) = 2;% West
                    end
                    VUE_info_per_lane(loop_VUE,3) = 1.5;%outdoor UE height
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
            % relocate MUEs in current cell
            VUE_info_start= [VUE_info_start;VUE_info_per_lane];
        end % end of loop_lane_horizon
        %% node deployment in vertical
        for loop_lane_vertical = 1:Lane_num
            VUE_info_per_lane = zeros(VUE_num_per_lane_vertical(loop_lane_vertical),3); % VUE information in each road lane
            for loop_VUE = 1:VUE_num_per_lane_vertical(loop_lane_vertical)
                loca_temp = 0;
                flag_loca = 1;
                cyclic_count = 0;
                while flag_loca == 1
                    cyclic_count = cyclic_count + 1;
                    if cyclic_count >= break_limit
                        fprintf('\n BREAK! \n');
                        break_flag = 0;break;% to start a new drop
                    end
                    %% deploy VUE in each lane
                    loca_y = Road_length_vertical*rand+2*Lane_width; %%rand is between 0 and 1
                    if loop_lane_vertical<=2
                        loca_x = -grid_width+Lane_width/2+(loop_lane_vertical-1)*Lane_width;
                    else
                        loca_x = -Lane_width/2-(loop_lane_vertical-3)*Lane_width;
                    end  % temporary location of VUE
                    %% relocate MUE which is in the two triangle areas
                    loca_temp = loca_x + 1j*loca_y+road_grid_offset(loop_road_grid);% location of VUE
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
                    VUE_info_per_lane(loop_VUE,4) = loop_lane_vertical+4;% lane index
                    if loop_lane_vertical<=2
                        VUE_info_per_lane(loop_VUE,2) = 1;% North
                    else
                        VUE_info_per_lane(loop_VUE,2) = 3;% South
                    end
                    VUE_info_per_lane(loop_VUE,3) = 1.5; %outdoor UE height
                    flag_loca = 0; % the location has been found
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
            % relocate MUEs in current cell
            VUE_info_start= [VUE_info_start;VUE_info_per_lane];
        end % end of loop_lane_vertical
        %% definition of CUEs
        CUE_info((loop_road_grid-1)*Total_CUE_num_block+1:(loop_road_grid-1)*Total_CUE_num_block+9,1) =...
            1i*(grid_length-2*Lane_width-Sidewalk_width/2)+(-8.5-(0:25.89:233))...
            + road_grid_offset(loop_road_grid);
        CUE_info((loop_road_grid-1)*Total_CUE_num_block+1:(loop_road_grid-1)*Total_CUE_num_block+9,2) = 4; %East
        CUE_info((loop_road_grid-1)*Total_CUE_num_block+10:(loop_road_grid-1)*Total_CUE_num_block+18,1) = ...
            1i*(2*Lane_width+Sidewalk_width/2)+(-8.5-(0:25.89:233))...
            + road_grid_offset(loop_road_grid);
        CUE_info((loop_road_grid-1)*Total_CUE_num_block+10:(loop_road_grid-1)*Total_CUE_num_block+18,2) = 2;%East
        CUE_info((loop_road_grid-1)*Total_CUE_num_block+19:(loop_road_grid-1)*Total_CUE_num_block+34,1) = ...
            (-grid_width+2*Lane_width+Sidewalk_width/2)+1i*(8.5+(26:26:416))...
            + road_grid_offset(loop_road_grid);
        CUE_info((loop_road_grid-1)*Total_CUE_num_block+19:(loop_road_grid-1)*Total_CUE_num_block+34,2) = 1;% North
        CUE_info((loop_road_grid-1)*Total_CUE_num_block+35:(loop_road_grid-1)*Total_CUE_num_block+50,1) = ...
            (-2*Lane_width-Sidewalk_width/2)+1i*(8.5+(26:26:416))...
            + road_grid_offset(loop_road_grid);
        CUE_info((loop_road_grid-1)*Total_CUE_num_block+35:(loop_road_grid-1)*Total_CUE_num_block+50,2) = 3;%South
    end % end of loop_road_grid
    [Total_VUE_num,aa]=size(VUE_info_start);
    if break_flag == 0
        continue;% to start a new drop
    end
    real_snapshot = real_snapshot + 1;
    %% update location and shadowing
    for sub_snapshot=1:sub_drop_num
        %% update location
        if sub_snapshot==1
            VUE_info = zeros(Total_VUE_num,8); %[1.MUEs' location;2.VUE direction(=1 North,2 West,3 South,4 East);3.height;4.lane_index; 5.Serving sector(=1,2,3);
            %6.if the vehilce is turning(>0,it is turning, 0 it is driving directly); 7.Service Type; 8.Road grid index]
            %PS: 6.if the vehilce is turning(>0,it is turning(=1 North,2 West,3 South,4 East), and =6 already turn);
            VUE_info(:,2) = VUE_info_start(:,2); % direction
            VUE_info(:,3) = VUE_info_start(:,3); % Vehicle height
            VUE_info(:,4) = VUE_info_start(:,4); % lane_index
            VUE_info(:,1)=VUE_info_start(:,1); %location initialzation
            %% Road grid index
            Coordinate_matrix=repmat(VUE_info(:,1),1,Road_grid_num)-repmat(road_grid_offset,Total_VUE_num,1);
            distance_x=real(Coordinate_matrix);
            distance_y=imag(Coordinate_matrix);
            if_in_road_grid=(distance_x<=0).*(distance_x>=-grid_width).*...
                (distance_y>=0).*(distance_y<=grid_length);
            for loop_VUE=1:Total_VUE_num
                VUE_info(loop_VUE,8)=find(if_in_road_grid(loop_VUE,:)==1);%road_grid_index of VUE
            end
            %% Shadowing fading
            [Distance_eNB2VUE, eNB_loca_WRAP]=WRAP_eNB2VUE_distance(VUE_info,MeNB_loca,grid_length,grid_width,Total_VUE_num,MeNB_num,1,R);
            %            Distance_eNB2VUE_1=abs(repmat(VUE_info(:,1).',MeNB_num,1)-eNB_loca_WRAP);
            Distance_eNB2VUE_pre=Distance_eNB2VUE;%distance between eNB and VUE,MeNB_num*Total_VUE_num matrix
            [Distance_VUE2VUE, VUE_loca_WRAP]=WRAP_VUE2VUE_distance(VUE_info,grid_length,grid_width,Total_VUE_num,1);
            Distance_VUE2VUE_pre=Distance_VUE2VUE;%Total_VUE_num*Total_VUE_num matrix
            % initializate shadowing fading V2I
            Shadowing_eNB2VUE=R_matrix*Shadow_std_V2I*randn(MeNB_num,Total_VUE_num); % Initialization at time 0 of every drop
            % initializate shadowing fading V2V LOS/NLOS
            Shadowing_VUE2VUE_LOS=Shadow_std_V2V_LOS*randn(Total_VUE_num,Total_VUE_num);
            Shadowing_VUE2VUE_LOS=triu(Shadowing_VUE2VUE_LOS,0)+tril(Shadowing_VUE2VUE_LOS',-1);
            Shadowing_VUE2VUE_LOS=Shadowing_VUE2VUE_LOS+diag(-diag(Shadowing_VUE2VUE_LOS));% LOS shadow fading, std=3 dB
            Shadowing_VUE2VUE_NLOS=Shadow_std_V2V_NLOS*randn(Total_VUE_num,Total_VUE_num);
            Shadowing_VUE2VUE_NLOS=triu(Shadowing_VUE2VUE_NLOS,0)+tril(Shadowing_VUE2VUE_NLOS',-1);
            Shadowing_VUE2VUE_NLOS=Shadowing_VUE2VUE_NLOS+diag(-diag(Shadowing_VUE2VUE_NLOS));% NLOS shadow fading, std=4 dB
        else
            Coordinate_matrix=repmat(VUE_info(:,1),1,intersection_num)-repmat(intersection_loca,Total_VUE_num,1);
            distance_x=abs(real(Coordinate_matrix));
            distance_y=abs(imag(Coordinate_matrix));
            if_in_intersection=(distance_x<=2*Lane_width).*(distance_y<=2*Lane_width);
            Index_0=find(sum(if_in_intersection,2)==0);
            VUE_info(Index_0,6)=0; %6.if the vehilce is turning(=1,it is turning, 0 it is driving directly)
            %% check if the vehicle in intersection needs to turn
            Index_1=find(sum(if_in_intersection,2)>0);
            Index_1_num=length(Index_1);
            if Index_1_num~=0
                for loop_Index_1=1:Index_1_num
                    VUE_index=Index_1(loop_Index_1);
                    if VUE_info(VUE_index,6)==0 %change direction
                        if VUE_info(VUE_index,2)==1 %North
                            VUE_info(VUE_index,6)=randsrc(1,1,[1 2 4; 0.5 0.25 0.25]);
                        elseif VUE_info(VUE_index,2)==3 %South
                            VUE_info(VUE_index,6)=randsrc(1,1,[3 2 4; 0.5 0.25 0.25]);
                        elseif VUE_info(VUE_index,2)==2 %West
                            VUE_info(VUE_index,6)=randsrc(1,1,[2 1 3; 0.5 0.25 0.25]);
                        elseif VUE_info(VUE_index,2)==4 %East
                            VUE_info(VUE_index,6)=randsrc(1,1,[4 1 3; 0.5 0.25 0.25]);
                        end
                    end
                    intersection_index=find(if_in_intersection(VUE_index,:)>0);
                    distance_min=(vehicle_speed*5/18)*0.1;
                    % Case1: Turn East to North, lane_index=1 to 8
                    if VUE_info(VUE_index,4)==1 && abs(real(intersection_loca(intersection_index))+3*Lane_width/2-real(VUE_info(VUE_index,1)))<=distance_min ...
                            && VUE_info(VUE_index,6)==1 %lane_index=1
                        VUE_info(VUE_index,4)=8; % new lane_index
                        VUE_info(VUE_index,2)=1; % change direction to north
                        VUE_info(VUE_index,1)=real(intersection_loca(intersection_index))+3*Lane_width/2+1i*imag(VUE_info(VUE_index,1));
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case2: Turn East to North, lane_index=2 to 7
                    elseif VUE_info(VUE_index,4)==2 && abs(real(intersection_loca(intersection_index))+Lane_width/2-real(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==1 %lane_index=2
                        VUE_info(VUE_index,4)=7; % new lane_index
                        VUE_info(VUE_index,2)=1; % change direction to north
                        VUE_info(VUE_index,1)=real(intersection_loca(intersection_index))+Lane_width/2+1i*imag(VUE_info(VUE_index,1));
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case3: Turn East to South, lane_index=1 to 6
                    elseif VUE_info(VUE_index,4)==1 && abs(real(intersection_loca(intersection_index))-Lane_width/2-real(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==3 %lane_index=1
                        VUE_info(VUE_index,4)=6; % new lane_index
                        VUE_info(VUE_index,2)=3; % change direction to south
                        VUE_info(VUE_index,1)=real(intersection_loca(intersection_index))-Lane_width/2+1i*imag(VUE_info(VUE_index,1));
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case4: Turn East to South, lane_index=2 to 5
                    elseif VUE_info(VUE_index,4)==2 && abs(real(intersection_loca(intersection_index))-3*Lane_width/2-real(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==3 %lane_index=2
                        VUE_info(VUE_index,4)=5; % new lane_index
                        VUE_info(VUE_index,2)=3; % change direction to south
                        VUE_info(VUE_index,1)=real(intersection_loca(intersection_index))-3*Lane_width/2+1i*imag(VUE_info(VUE_index,1));
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case5: Turn West to North, lane_index=3 to 8
                    elseif VUE_info(VUE_index,4)==3 && abs(real(intersection_loca(intersection_index))+3*Lane_width/2-real(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==1 %lane_index=3
                        VUE_info(VUE_index,4)=8; % new lane_index
                        VUE_info(VUE_index,2)=1; % change direction to north
                        VUE_info(VUE_index,1)=real(intersection_loca(intersection_index))+3*Lane_width/2+1i*imag(VUE_info(VUE_index,1));
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case6: Turn West to North, lane_index=4 to 7
                    elseif VUE_info(VUE_index,4)==4 && abs(real(intersection_loca(intersection_index))+Lane_width/2-real(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==1 %lane_index=4
                        VUE_info(VUE_index,4)=7; % new lane_index
                        VUE_info(VUE_index,2)=1; % change direction to north
                        VUE_info(VUE_index,1)=real(intersection_loca(intersection_index))+Lane_width/2+1i*imag(VUE_info(VUE_index,1));
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case7:Turn West to South, lane_index=3 to 5
                    elseif VUE_info(VUE_index,4)==3 && abs(real(intersection_loca(intersection_index))-3*Lane_width/2-real(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==3 %lane_index=3
                        VUE_info(VUE_index,4)=5; % new lane_index
                        VUE_info(VUE_index,2)=3; % change direction to south
                        VUE_info(VUE_index,1)=real(intersection_loca(intersection_index))-3*Lane_width/2+1i*imag(VUE_info(VUE_index,1));
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case8:Turn West to South, lane_index=4 to 6
                    elseif VUE_info(VUE_index,4)==4 && abs(real(intersection_loca(intersection_index))-Lane_width/2-real(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==3 %lane_index=4
                        VUE_info(VUE_index,4)=6; % new lane_index
                        VUE_info(VUE_index,2)=3; % change direction to south
                        VUE_info(VUE_index,1)=real(intersection_loca(intersection_index))-Lane_width/2+1i*imag(VUE_info(VUE_index,1));
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case9:Turn South to East, lane_index=5 to 2
                    elseif VUE_info(VUE_index,4)==5 && abs(imag(intersection_loca(intersection_index))-3*Lane_width/2-imag(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==4
                        VUE_info(VUE_index,4)=2; % new lane_index
                        VUE_info(VUE_index,2)=4; % change direction to east
                        VUE_info(VUE_index,1)=real(VUE_info(VUE_index,1))+1i*(imag(intersection_loca(intersection_index))-3*Lane_width/2);
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case10:Turn South to East, lane_index=6 to 1
                    elseif VUE_info(VUE_index,4)==6 && abs(imag(intersection_loca(intersection_index))-Lane_width/2-imag(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==4
                        VUE_info(VUE_index,4)=1; % new lane_index
                        VUE_info(VUE_index,2)=4; % change direction to east
                        VUE_info(VUE_index,1)=real(VUE_info(VUE_index,1))+1i*(imag(intersection_loca(intersection_index))-Lane_width/2);
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case11:Turn South to West, lane_index=5 to 3
                    elseif VUE_info(VUE_index,4)==5 && abs(imag(intersection_loca(intersection_index))+3*Lane_width/2-imag(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==2
                        VUE_info(VUE_index,4)=3; % new lane_index
                        VUE_info(VUE_index,2)=2; % change direction to west
                        VUE_info(VUE_index,1)=real(VUE_info(VUE_index,1))+1i*(imag(intersection_loca(intersection_index))+3*Lane_width/2);
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case12:Turn South to West, lane_index=6 to 4
                    elseif VUE_info(VUE_index,4)==6 && abs(imag(intersection_loca(intersection_index))+Lane_width/2-imag(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==2
                        VUE_info(VUE_index,4)=4; % new lane_index
                        VUE_info(VUE_index,2)=2; % change direction to west
                        VUE_info(VUE_index,1)=real(VUE_info(VUE_index,1))+1i*(imag(intersection_loca(intersection_index))+Lane_width/2);
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case13:Turn North to East, lane_index=7 to 1
                    elseif VUE_info(VUE_index,4)==7 && abs(imag(intersection_loca(intersection_index))-Lane_width/2-imag(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==4
                        VUE_info(VUE_index,4)=1; % new lane_index
                        VUE_info(VUE_index,2)=4; % change direction to east
                        VUE_info(VUE_index,1)=real(VUE_info(VUE_index,1))+1i*(imag(intersection_loca(intersection_index))-Lane_width/2);
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case14:Turn North to East, lane_index=8 to 2
                    elseif VUE_info(VUE_index,4)==8 && abs(imag(intersection_loca(intersection_index))-3*Lane_width/2-imag(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==4
                        VUE_info(VUE_index,4)=2; % new lane_index
                        VUE_info(VUE_index,2)=4; % change direction to east
                        VUE_info(VUE_index,1)=real(VUE_info(VUE_index,1))+1i*(imag(intersection_loca(intersection_index))-3*Lane_width/2);
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case15:Turn North to West, lane_index=7 to 3
                    elseif VUE_info(VUE_index,4)==7 && abs(imag(intersection_loca(intersection_index))+3*Lane_width/2-imag(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==2
                        VUE_info(VUE_index,4)=3; % new lane_index
                        VUE_info(VUE_index,2)=2; % change direction to east
                        VUE_info(VUE_index,1)=real(VUE_info(VUE_index,1))+1i*(imag(intersection_loca(intersection_index))+3*Lane_width/2);
                        VUE_info(VUE_index,6)=6; % note already change direction
                        % Case16:Turn North to West, lane_index=8 to 4
                    elseif VUE_info(VUE_index,4)==8 && abs(imag(intersection_loca(intersection_index))+Lane_width/2-imag(VUE_info(VUE_index,1)))<=distance_min...
                            && VUE_info(VUE_index,6)==2
                        VUE_info(VUE_index,4)=4; % new lane_index
                        VUE_info(VUE_index,2)=2; % change direction to east
                        VUE_info(VUE_index,1)=real(VUE_info(VUE_index,1))+1i*(imag(intersection_loca(intersection_index))+Lane_width/2);
                        VUE_info(VUE_index,6)=6; % note already change direction
                    end
                end %end of loop_Index in intersection
            end
            %% update the location of vehicle and change of distance between VUE2VUE and eNB2VUE
            VUE_info(:,1)=VUE_info(:,1)+(vehicle_speed*5/18)*0.1*((1i).^(VUE_info(:,2)));
            % vehicle location wrapround
            over_boundary_case=cell(8,1);
            over_boundary_case_size=zeros(8,1);
            over_boundary_case{1,1}=find(imag(VUE_info(:,1))>2*grid_length);
            over_boundary_case_size(1,1)=length(over_boundary_case{1,1});
            if over_boundary_case_size(1,1)~=0
                VUE_info(over_boundary_case{1,1},1)=VUE_info(over_boundary_case{1,1},1)-1i*3*grid_length;
            end
            over_boundary_case{2,1}=find(imag(VUE_info(:,1))<-grid_length);
            over_boundary_case_size(2,1)=length(over_boundary_case{2,1});
            if over_boundary_case_size(2,1)~=0
                VUE_info(over_boundary_case{2,1},1)=VUE_info(over_boundary_case{2,1},1)+1i*3*grid_length;
            end
            % Case 3
            over_boundary_case{3,1}=(real(VUE_info(:,1))>2*grid_width).*((imag(VUE_info(:,1))>grid_length));%+...
            % (imag(VUE_info(:,1))<-grid_length+2*Lane_width));
            over_boundary_case{3,1}=find(over_boundary_case{3,1}>0);
            over_boundary_case_size(3,1)=length(over_boundary_case{3,1});
            if over_boundary_case_size(3,1)~=0
                VUE_info(over_boundary_case{3,1},1)=VUE_info(over_boundary_case{3,1},1)-4*grid_width;
            end
            % Case 4
            over_boundary_case{4,1}=(real(VUE_info(:,1))<-2*grid_width).*(imag(VUE_info(:,1))<0);
            over_boundary_case{4,1}=find(over_boundary_case{4,1}>0);
            over_boundary_case_size(4,1)=length(over_boundary_case{4,1});
            if over_boundary_case_size(4,1)~=0
                VUE_info(over_boundary_case{4,1},1)=VUE_info(over_boundary_case{4,1},1)+4*grid_width;
            end
            % Case 5
            over_boundary_case{5,1}=find((real(VUE_info(:,1))>3*grid_width));
            over_boundary_case_size(5,1)=length(over_boundary_case{5,1});
            if over_boundary_case_size(5,1)~=0
                VUE_info(over_boundary_case{5,1},1)=VUE_info(over_boundary_case{5,1},1)-6*grid_width;
            end
            % Case 6
            over_boundary_case{6,1}=find((real(VUE_info(:,1))<-3*grid_width));
            over_boundary_case_size(6,1)=length(over_boundary_case{6,1});
            if over_boundary_case_size(6,1)~=0
                VUE_info(over_boundary_case{6,1},1)=VUE_info(over_boundary_case{6,1},1)+6*grid_width;
            end
            % Case 7
            over_boundary_case{7,1}=(imag(VUE_info(:,1))>grid_length).*(real(VUE_info(:,1))<-2*grid_width);
            over_boundary_case{7,1}=find(over_boundary_case{7,1}>0);
            over_boundary_case_size(7,1)=length(over_boundary_case{7,1});
            if over_boundary_case_size(7,1)~=0
                VUE_info(over_boundary_case{7,1},1)=VUE_info(over_boundary_case{7,1},1)-1i*grid_length;
            end
            % Case 8
            over_boundary_case{8,1}=(imag(VUE_info(:,1))<0).*(real(VUE_info(:,1))>2*grid_width);
            over_boundary_case{8,1}=find(over_boundary_case{8,1}>0);
            over_boundary_case_size(8,1)=length(over_boundary_case{8,1});
            if over_boundary_case_size(8,1)~=0
                VUE_info(over_boundary_case{8,1},1)=VUE_info(over_boundary_case{8,1},1)+1i*grid_length;
            end
            %% Road grid index
            Coordinate_matrix=repmat(VUE_info(:,1),1,Road_grid_num)-repmat(road_grid_offset,Total_VUE_num,1);
            distance_x=real(Coordinate_matrix);
            distance_y=imag(Coordinate_matrix);
            if_in_road_grid=(distance_x<0).*(distance_x>=-grid_width).*...
                (distance_y>=0).*(distance_y<grid_length);
            for loop_VUE=1:Total_VUE_num
                VUE_info(loop_VUE,8)=find(if_in_road_grid(loop_VUE,:)==1);%road_grid_index of VUE
            end
            %% calculate distance change
            [Distance_eNB2VUE,eNB_loca_WRAP]=WRAP_eNB2VUE_distance(VUE_info,MeNB_loca,grid_length,grid_width,Total_VUE_num,MeNB_num,1,R);
            Distance_eNB2VUE_change=abs(Distance_eNB2VUE-Distance_eNB2VUE_pre); %change in distance of the UE to the eNB site
            Distance_eNB2VUE_pre=Distance_eNB2VUE;
            [Distance_VUE2VUE, VUE_loca_WRAP]=WRAP_VUE2VUE_distance(VUE_info,grid_length,grid_width,Total_VUE_num,1);
            Distance_VUE2VUE_change=abs(Distance_VUE2VUE-Distance_VUE2VUE_pre); %change in distance of the UE to the eNB site
            Distance_VUE2VUE_pre=Distance_VUE2VUE;
            %% update shadowing fading V2I
            Shadowing_eNB2VUE=exp(-Distance_eNB2VUE_change/D_corr_V2I).*Shadowing_eNB2VUE+...
                sqrt(1-exp(-2*Distance_eNB2VUE_change/D_corr_V2I)).*(R_matrix*Shadow_std_V2I*randn(MeNB_num,Total_VUE_num));
            %% update shadowing fading V2V:LOS
            Y_VUE2VUE_LOS=Shadow_std_V2V_LOS*randn(Total_VUE_num,Total_VUE_num);
            Y_VUE2VUE_LOS=triu(Y_VUE2VUE_LOS,0)+tril(Y_VUE2VUE_LOS',-1);
            Y_VUE2VUE_LOS=Y_VUE2VUE_LOS+diag(-diag(Y_VUE2VUE_LOS));% LOS shadow fading, std=3 dB
            Shadowing_VUE2VUE_LOS=exp(-Distance_VUE2VUE_change/D_corr_V2V).*Shadowing_VUE2VUE_LOS+...
                sqrt(1-exp(-2*Distance_VUE2VUE_change/D_corr_V2V)).*Y_VUE2VUE_LOS;
            %% update shadowing fading V2V:NLOS
            Y_VUE2VUE_NLOS=Shadow_std_V2V_NLOS*randn(Total_VUE_num,Total_VUE_num);
            Y_VUE2VUE_NLOS=triu(Y_VUE2VUE_NLOS,0)+tril(Y_VUE2VUE_NLOS',-1);
            Y_VUE2VUE_NLOS=Y_VUE2VUE_NLOS+diag(-diag(Y_VUE2VUE_NLOS));% NLOS shadow fading, std=4 dB
            Shadowing_VUE2VUE_NLOS=exp(-Distance_VUE2VUE_change/D_corr_V2V).*Shadowing_VUE2VUE_NLOS+...
                sqrt(1-exp(-2*Distance_VUE2VUE_change/D_corr_V2V)).*Y_VUE2VUE_NLOS;
        end %end of if sub_snapshot==1
        %% save data
        savefile = sprintf('../data_deploy/node_deployment_Urban_vehicle_speed=%d_No%d_subdrop_ID%d.mat',vehicle_speed,real_snapshot,sub_snapshot);
        save(savefile,'Total_VUE_num','MeNB_loca','VUE_info','Shadowing_eNB2VUE','Shadowing_VUE2VUE_LOS','Shadowing_VUE2VUE_NLOS','real_snapshot',...
            'Distance_VUE2VUE','Distance_eNB2VUE','VUE_loca_WRAP','eNB_loca_WRAP','CUE_info');
        %% print
        fprintf('\n real_snapshot=%d, sub_snapshot=%d,',real_snapshot,sub_snapshot);
        fprintf(' cyclic_over_limit=%d.\n',cyclic_over_limit);
    end %end of sub_snapshot
end % end of snapshot

toc

%% print
fprintf('%% **************    Simulation End!!! %d drops(real).   ******************* %%\n',real_snapshot);
fprintf(' snapshot break = %d times.\n',snapshot-real_snapshot);