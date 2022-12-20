clear;clc;
vehicle_speed = 70;
%% load nodedeployment file
openfile = sprintf('data_deploy/node_deployment_Freeway_parameters_vehicle_speed=%d.mat',vehicle_speed);
load(openfile);
%% draw eNB
site_MarkerEdgeColor = 'black';
site_MarkerFaceColor = 'red';
eNB_marker = 'o';
antenna_LineColor = 'blue';
angle_per_sector = [0 120 240]; % sector_index= 1,2,3. the angle of major lobe in each sector
vector_length = 6;
eNB_name = sprintf('eNB-'); % the name of eNB
% Cellular Layout:Hexagonal grid, 3 sectors per site
A_site = zeros(13,13);
for ii= 1:12
    A_site(ii,ii+1)=1;
end
A_site(1,6)=1;   A_site(1,10)=1;      A_site(2,13)=1;
% plot margin coordinates
coordinates_site = R*[0,0;-1/2/sqrt(3),1/2;0,1;1/sqrt(3),1;sqrt(3)/2,1/2;1/sqrt(3),0;...
    sqrt(3)/2,-1/2;1/sqrt(3),-1;0,-1;-1/2/sqrt(3),-1/2;-sqrt(3)/2,-1/2;-2/sqrt(3),0;-sqrt(3)/2,1/2];
coordinates_site = -real(coordinates_site) + imag(coordinates_site);
%coordinates_site = rot90(coordinates_site);
%% draw Freeway Scenario
% plot block coordinates
% Lane_mid_Color = [1 1 1]; % white
% Lane_edge_Color = [0 0 0]; % black
Lane_face_Color = [.86 .86 .86]; % gray
coordinates_Freeway_scenario = [(loca_x_max) + 1i*(-min_distance_VUE_MeNB-4),...
    (loca_x_max) + 1i*(-min_distance_VUE_MeNB-4-6*Lane_width),...
    (loca_x_min) + 1i*(-min_distance_VUE_MeNB-4-6*Lane_width),...
    (loca_x_min) + 1i*(-min_distance_VUE_MeNB-4)];
figure('units','normalized','position',[0.06,0.06,0.8,0.5]); hold on;
fill(real(coordinates_Freeway_scenario),imag(coordinates_Freeway_scenario),Lane_face_Color);
hold on;
for loop_lane = 2:Lane_num
    if loop_lane ~= 4
    plot([loca_x_min loca_x_max],(-min_distance_VUE_MeNB-loop_lane*Lane_width)*[1 1],'w--');
    else
    plot([loca_x_min loca_x_max],(-min_distance_VUE_MeNB-loop_lane*Lane_width)*[1 1],...
        'Linestyle','-','LineWidth',1.5,'Color','y');   
    end
end
VUE_DotColor_blue = [0 0 1]; % blue
%% loop of draw
for loop_drop = 1 : drop_num
    for sub_loop_drop = 1 : sub_drop_num
        openfile_drop = sprintf('data_deploy/node_deployment_Freeway_vehicle_speed=%d_No%d_subdrop_ID%d.mat',vehicle_speed,...
            loop_drop,sub_loop_drop);
        if exist(openfile_drop,'file') == 0
            continue;
        end % check the if the m-file exist.
        load(openfile_drop,'Total_VUE_num','MeNB_loca','VUE_info','VUE_loca_WRAP','eNB_loca_WRAP');
        if sub_loop_drop == 1
            %% draw eNB
            % Plot all of the sites' positions
            scatter(real(MeNB_loca),imag(MeNB_loca),'Marker',eNB_marker,'MarkerEdgeColor',...
                site_MarkerEdgeColor,'MarkerFaceColor',site_MarkerFaceColor);
            hold on;
            % Print eNB site number
            %         figure_small_distance = 40; % in m
            for site_number = 1:MeNB_num
                text(real(MeNB_loca(site_number))+20,imag(MeNB_loca(site_number))-5,...
                    [eNB_name,num2str(site_number)],'Color','k','FontSize',8);
            end % loop of site_number
            axis([loca_x_min loca_x_max -65 -35]);
            xlabel('x pos [m]');
            ylabel('y pos [m]');
        end % end of sub_loop_drop == 1
        title_str = sprintf('eNodeB and UE positions, drop = %d, subdrop = %d',loop_drop,sub_loop_drop);
        title(title_str,'FontSize',12);
        % VUE postition
        VUE_pos = scatter(real(VUE_info(:,1)),imag(VUE_info(:,1)),...
            'Marker','.','MarkerFaceColor',VUE_DotColor_blue,'MarkerEdgeColor',VUE_DotColor_blue);
        pause(1);
        if loop_drop ~= drop_num || sub_loop_drop ~= sub_drop_num
            set(VUE_pos,'visible','off');
        end
        %hold on;
    end % end of sub_loop_drop
end % end of loop_drop