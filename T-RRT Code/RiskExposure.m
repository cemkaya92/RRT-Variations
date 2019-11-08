% This file is created by U. Cem Kaya - 2018
%% FUNCTION TO CALCULATE RISK EXPOSURE
function riskExposure = RiskExposure(prem,finalPath)

cf_nmi_to_m = 1852;             % 1 nmi = 1852 m


riskExposure = 0;
for jj = 1:length(finalPath)-1
    x1 = finalPath(jj).x/cf_nmi_to_m;  y1 = finalPath(jj).y/cf_nmi_to_m;
    x2 = finalPath(jj+1).x/cf_nmi_to_m;  y2 = finalPath(jj+1).y/cf_nmi_to_m;
    length_segment(jj) = Dist([x1 y1]',[x2 y2]');
    PREM_value = 0;
    for ii = 1:length(prem.pdfs)    % For each PDF in the PREM
    
    m1 = prem.pdfs(ii).m(1);
    m2 = prem.pdfs(ii).m(2);
    s1 = sqrt(prem.pdfs(ii).k(1,1));
    s2 = sqrt(prem.pdfs(ii).k(2,2));
    
    % Evaluate and superimpose each PDF
    PREM_value = PREM_value + exp(-1/2*(((x1 - m1)/s1).^2 + ((y1 - m2)/s2).^2))/(2*pi*s1*s2);
   
    end
    riskExposure(jj) = PREM_value*length_segment(jj);
end
riskExposure = sum(riskExposure(:));
end