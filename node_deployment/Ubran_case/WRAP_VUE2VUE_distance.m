function [Distance_VUE2VUE, VUE_loca_WRAP]=WRAP_VUE2VUE_distance(VUE_info,grid_length,grid_width,Total_VUE_num,method)
%% input

%% output

%% Write by MJ 2016-05-22 V1.0.0
VUE_loca=VUE_info(:,1);
if method==1
WRAP_VUE_offset=zeros(14,14);
WRAP_VUE_offset(1,[9 10 11 14])=[-4*grid_width+1i*2*grid_length -4*grid_width+1i*2*grid_length grid_width+1i*3*grid_length -4*grid_width+1i*2*grid_length];
WRAP_VUE_offset(2,[10 11 12])=[-4*grid_width+1i*2*grid_length grid_width+1i*3*grid_length grid_width+1i*3*grid_length];
WRAP_VUE_offset(3,[11 12 13])=[1 1 1]*(grid_width+1i*3*grid_length);
WRAP_VUE_offset(4,[5 12 13 14])=[5*grid_width+1i*grid_length grid_width+1i*3*grid_length grid_width+1i*3*grid_length grid_width+1i*3*grid_length];
WRAP_VUE_offset(5,[13 14 4 9 10])=[-4*grid_width+1i*2*grid_length -4*grid_width+1i*2*grid_length -5*grid_width-1i*grid_length...
    -5*grid_width-1i*grid_length -5*grid_width-1i*grid_length];
WRAP_VUE_offset(6,[14 10])=[-4*grid_width+1i*2*grid_length -5*grid_width-1i*grid_length];
WRAP_VUE_offset(9,[1 5])=[4*grid_width-1i*2*grid_length 5*grid_width+1i*grid_length];
WRAP_VUE_offset(10,[5 6 11 2 1])=[5*grid_width+1i*grid_length 5*grid_width+1i*grid_length 5*grid_width+1i*grid_length...
    4*grid_width-1i*2*grid_length 4*grid_width-1i*2*grid_length];
WRAP_VUE_offset(11,[10 1 2 3])=[-5*grid_width-1i*grid_length -grid_width-1i*3*grid_length -grid_width-1i*3*grid_length -grid_width-1i*3*grid_length];
WRAP_VUE_offset(12,[2 3 4])=[1 1 1]*(-grid_width-1i*3*grid_length);
WRAP_VUE_offset(13,[3 4 5])=[-grid_width-1i*3*grid_length -grid_width-1i*3*grid_length 4*grid_width-1i*2*grid_length];
WRAP_VUE_offset(14,[1 4 5 6])=[4*grid_width-1i*2*grid_length -grid_width-1i*3*grid_length 4*grid_width-1i*2*grid_length 4*grid_width-1i*2*grid_length];
VUE_loca_WRAP=zeros(Total_VUE_num,Total_VUE_num);
for loop_VUE=1:Total_VUE_num
    for road_grid_index=1:14
        VUE_Index_road_gird= VUE_info(:,8)==road_grid_index;
        VUE_loca_WRAP(loop_VUE,VUE_Index_road_gird)=VUE_info(VUE_Index_road_gird,1)+WRAP_VUE_offset(VUE_info(loop_VUE,8),road_grid_index);
    end
end
Distance_VUE2VUE=abs(repmat(VUE_loca,1,Total_VUE_num)-VUE_loca_WRAP);
elseif method==2
wrapround_VUE_offset=[grid_width+1i*3*grid_length;5*grid_width+1i*grid_length;4*grid_width-1i*2*grid_length;
    -grid_width-1i*3*grid_length;-5*grid_width-1i*grid_length;-4*grid_width+1i*2*grid_length;];
Wrapround_VUE_loca=zeros(Total_VUE_num,6);
Distance_VUE2VUE=abs(repmat(VUE_loca,1,Total_VUE_num)-repmat(VUE_loca.',Total_VUE_num,1));
VUE_loca_WRAP=repmat(VUE_loca.',Total_VUE_num,1);
for ii=1:6
    Wrapround_VUE_loca(:,ii)=VUE_loca+wrapround_VUE_offset(ii);
    Distance_VUE2VUE_wrap=abs(repmat(VUE_loca,1,Total_VUE_num)-repmat(Wrapround_VUE_loca(:,ii).',Total_VUE_num,1));
    pointer_matrix=Distance_VUE2VUE>Distance_VUE2VUE_wrap;
    Distance_VUE2VUE=min(Distance_VUE2VUE,Distance_VUE2VUE_wrap);
    VUE_loca_WRAP=repmat(Wrapround_VUE_loca(:,ii).',Total_VUE_num,1).*pointer_matrix+...
        VUE_loca_WRAP.*(~pointer_matrix);
end
end
