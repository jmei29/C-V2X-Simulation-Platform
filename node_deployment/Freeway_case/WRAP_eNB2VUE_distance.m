function [Distance_eNB2VUE, eNB_loca_WRAP]=WRAP_eNB2VUE_distance(VUE_loca,MeNB_loca,ISD,Total_VUE_num,MeNB_num)
wrapround_eNB_offset=ISD*[-2 2];
Wrapround_eNB_loca=zeros(MeNB_num,2);
Distance_eNB2VUE=abs(repmat(VUE_loca.',MeNB_num,1)-repmat(MeNB_loca,1,Total_VUE_num));
eNB_loca_WRAP=repmat(MeNB_loca,1,Total_VUE_num);
for ii=1:2
    Wrapround_eNB_loca(:,ii)=MeNB_loca+wrapround_eNB_offset(ii);
    Distance_eNB2VUE_wrap=abs(repmat(VUE_loca.',MeNB_num,1)-repmat(Wrapround_eNB_loca(:,ii),1,Total_VUE_num));
    pointer_matrix=Distance_eNB2VUE>Distance_eNB2VUE_wrap;
    Distance_eNB2VUE=min(Distance_eNB2VUE,Distance_eNB2VUE_wrap);
    eNB_loca_WRAP=repmat(Wrapround_eNB_loca(:,ii),1,Total_VUE_num).*pointer_matrix+...
        eNB_loca_WRAP.*(~pointer_matrix);
end