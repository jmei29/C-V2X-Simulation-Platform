function [Distance_VUE2VUE, VUE_loca_WRAP]=WRAP_VUE2VUE_distance(VUE_loca,ISD,Total_VUE_num)
wrapround_VUE_offset=ISD*[-2 2];
Wrapround_VUE_loca=zeros(Total_VUE_num,2);
Distance_VUE2VUE=abs(repmat(VUE_loca,1,Total_VUE_num)-repmat(VUE_loca.',Total_VUE_num,1));
VUE_loca_WRAP=repmat(VUE_loca.',Total_VUE_num,1);
for ii=1:2
    Wrapround_VUE_loca(:,ii)=VUE_loca+wrapround_VUE_offset(ii);
    Distance_VUE2VUE_wrap=abs(repmat(VUE_loca,1,Total_VUE_num)-repmat(Wrapround_VUE_loca(:,ii).',Total_VUE_num,1));
    pointer_matrix=Distance_VUE2VUE>Distance_VUE2VUE_wrap;
    Distance_VUE2VUE=min(Distance_VUE2VUE,Distance_VUE2VUE_wrap);
    VUE_loca_WRAP=repmat(Wrapround_VUE_loca(:,ii).',Total_VUE_num,1).*pointer_matrix+...
        VUE_loca_WRAP.*(~pointer_matrix);
end
