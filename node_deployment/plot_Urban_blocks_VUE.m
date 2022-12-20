clear;clc;
vehicle_speed = 60; %km/h
%% load nodedeployment file
openfile = sprintf('data_deploy/node_deployment_Urban_parameters_vehicle_speed=%d.mat',vehicle_speed);
load(openfile);
%% draw eNB
site_MarkerEdgeColor = 'black';
site_MarkerFaceColor = 'red';
eNB_marker = 'o';
antenna_LineColor = 'blue';
angle_per_sector = [0 120 240]; % sector_index= 1,2,3. the angle of major lobe in each sector
vector_length = 40;
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
%% draw block
% plot block coordinates
block_Color = [.86 .86 .86]; % gray
block_name = 'Block-';
coordinates_building = [(2*Lane_width+Sidewalk_width) + 1i*(2*Lane_width+Sidewalk_width),...
    (grid_width-2*Lane_width-Sidewalk_width) + 1i*(2*Lane_width+Sidewalk_width),...
    (grid_width-2*Lane_width-Sidewalk_width) + 1i*(grid_length-2*Lane_width-Sidewalk_width),...
    (2*Lane_width+Sidewalk_width) + 1i*(grid_length-2*Lane_width-Sidewalk_width)];
figure('units','normalized','position',[0.06,0.06,0.6,0.8]); hold on;
for loop_road_grid = 1:Road_grid_num
    fill(real(coordinates_building+road_grid_offset(loop_road_grid))-grid_width,imag(coordinates_building+road_grid_offset(loop_road_grid)),...
        block_Color);
    text(grid_width/2-50+real(road_grid_offset(loop_road_grid))-grid_width,...
        grid_length/2+imag(road_grid_offset(loop_road_grid)),...
        [block_name,num2str(loop_road_grid)],'Color','k','FontSize',8);
    hold on;
end % loop_road_grid
VUE_DotColor_blue = [0 0 1]; % blue
CUE_DotColor_red = [1 0 0]; %red
%% loop of draw
for loop_drop = 1 : drop_num
    for sub_loop_drop = 1 : sub_drop_num
        openfile_drop = sprintf('data_deploy/node_deployment_Urban_vehicle_speed=%d_No%d_subdrop_ID%d.mat',vehicle_speed,...
            loop_drop,sub_loop_drop);
        if exist(openfile_drop,'file') == 0
            continue;
        end % check the if the m-file exist.
        load(openfile_drop,'Total_VUE_num','MeNB_loca','VUE_info','VUE_loca_WRAP','eNB_loca_WRAP','CUE_info');
        if sub_loop_drop == 1
            %% draw eNB
            % Plot all of the sites' positions
            scatter(real(MeNB_loca),imag(MeNB_loca),'Marker',eNB_marker,'MarkerEdgeColor',...
                site_MarkerEdgeColor,'MarkerFaceColor',site_MarkerFaceColor);
            hold on;
            % Print eNB site number
            %         figure_small_distance = 40; % in m
            for site_number = 1:MeNB_num
                text(real(MeNB_loca(site_number))-60,imag(MeNB_loca(site_number))-50,...
                    [eNB_name,num2str(site_number)],'Color','k','FontSize',8);
                temp = [coordinates_site(:,1)+real(MeNB_loca(site_number)) coordinates_site(:,2)+imag(MeNB_loca(site_number))];
                gplot(A_site,temp,'k-');
                hold on;
                for loop_sector = 1:sector_num_per_cell
                    % Plot a line that tells where the antenna major lobe is pointing,
                    angle = angle_per_sector(loop_sector);
                    vector = vector_length*[ cosd(angle) sind(angle) ];
                    destiny = vector + [real(MeNB_loca(site_number)) imag(MeNB_loca(site_number))];
                    plot([real(MeNB_loca(site_number)) destiny(1)],[imag(MeNB_loca(site_number)) destiny(2)],...
                        antenna_LineColor);
                    hold on;
                end
            end % loop of site_number
            axis([-760 760 -600 900]);
            xlabel('x pos [m]');
            ylabel('y pos [m]');
        end % end of sub_loop_drop == 1
        title_str = sprintf('eNodeB and UE positions, drop = %d, subdrop = %d',loop_drop,sub_loop_drop);
        title(title_str,'FontSize',12);
        % VUE postition
        VUE_pos = scatter(real(VUE_info(:,1)),imag(VUE_info(:,1)),...
            'Marker','.','MarkerFaceColor',VUE_DotColor_blue,'MarkerEdgeColor',VUE_DotColor_blue,'LineWidth',0.1);
        %         % CUE postition
        %         CUE_pos = scatter(real(CUE_info(:,1)),imag(CUE_info(:,1)),...
        %             'Marker','.','MarkerFaceColor',CUE_DotColor_red,'MarkerEdgeColor',CUE_DotColor_red,'LineWidth',0.1);
        pause(1);
        if loop_drop ~= drop_num || sub_loop_drop ~= sub_drop_num
            set(VUE_pos,'visible','off');
            %set(CUE_pos,'visible','off');
        end
        %hold on;
    end % end of sub_loop_drop
end % end of loop_drop