function [Distance_eNB2VUE, eNB_loca_WRAP]=WRAP_eNB2VUE_distance(VUE_info,MeNB_loca,grid_length,grid_width,Total_VUE_num,MeNB_num,method,R)
VUE_loca=VUE_info(:,1);
if method==1
WRAP_loca_change = R*[-2*sqrt(3)+1i*3;% 9,10 index after wrap-around
    sqrt(3)/2+1i*4.5;% 11,12
    2.5*sqrt(3)+1i*1.5;% 13,14
    2*sqrt(3)-1i*3;% 15,16
    -sqrt(3)/2-1i*4.5;% 17,18
    -2.5*sqrt(3)-1i*1.5];% 8,19 location change
wlc = WRAP_loca_change;
% MeNB_num_WRAP = 61;
wrapround_MeNB_offset =[0 0 0 0 0 0 0;
    0 0 0 wlc(6) wlc(6) wlc(1) 0;
    0 0 0 0 wlc(1) wlc(1) wlc(2);
    0 wlc(3) 0 0 0 wlc(2) wlc(2);
    0 wlc(3) wlc(4) 0 0 0 wlc(3);
    0 wlc(4) wlc(4) wlc(5) 0 0 0;
    0 0 wlc(5) wlc(5) wlc(6) 0 0];  
eNB_loca_WRAP=zeros(MeNB_num,Total_VUE_num);
WRAP_VUE_offset=[3 3 4 4 2 2 1 1 5 5 7 7 6 6];
for loop_VUE=1:Total_VUE_num
        eNB_loca_WRAP(:,loop_VUE)=MeNB_loca+wrapround_MeNB_offset(WRAP_VUE_offset(VUE_info(loop_VUE,8)),:).';
end
Distance_eNB2VUE=abs(repmat(VUE_loca.',MeNB_num,1)-eNB_loca_WRAP);
elseif method==2
wrapround_eNB_offset=[grid_width+1i*3*grid_length;5*grid_width+1i*grid_length;4*grid_width-1i*2*grid_length;
    -grid_width-1i*3*grid_length;-5*grid_width-1i*grid_length;-4*grid_width+1i*2*grid_length;];
Wrapround_eNB_loca=zeros(MeNB_num,6);
Distance_eNB2VUE=abs(repmat(VUE_loca.',MeNB_num,1)-repmat(MeNB_loca,1,Total_VUE_num));
eNB_loca_WRAP=repmat(MeNB_loca,1,Total_VUE_num);
for ii=1:6
    Wrapround_eNB_loca(:,ii)=MeNB_loca+wrapround_eNB_offset(ii);
    Distance_eNB2VUE_wrap=abs(repmat(VUE_loca.',MeNB_num,1)-repmat(Wrapround_eNB_loca(:,ii),1,Total_VUE_num));
    pointer_matrix=Distance_eNB2VUE>Distance_eNB2VUE_wrap;
    Distance_eNB2VUE=min(Distance_eNB2VUE,Distance_eNB2VUE_wrap);
    eNB_loca_WRAP=repmat(Wrapround_eNB_loca(:,ii),1,Total_VUE_num).*pointer_matrix+...
        eNB_loca_WRAP.*(~pointer_matrix);
end
end