clearvars;
clc;



%Input
panel_l=[6,6,6,6];
% curve3_testworld1
% centerx=[2.97508,8.84992,13.3541];
% centery=[2.10791,0.651104,-3.44297];
% angle=[0.01051,-0.530655,-1.05051];
% curve3_testworld2
% centerx=[2.97128,8.77201,13.0267];
% centery=[2.11122,3.68286,7.94115];
% angle=[0.01051,0.530655,1.05051];
% curve3_testworld3
% centerx=[2.97121,7.75027,7.74356];
% centery=[2.11122,4.84725,10.3955];
% angle=[0.010515,1.05052,2.10052];
% curve3_testworld4
centerx=[2.96864,7.74872,7.74416,2.98568];
centery=[2.11276,4.84989,10.3999,13.1186];
angle=[0.009372,1.05199,2.10031,0.009615];
a=zeros(1,3);
c=zeros(1,3);
%Outputs
for i=1:size(panel_l,2)
    tempa=tan(angle(i));
    tempc=centery(i)-tan(angle(i))*centerx(i);
    a(i)=tempa;
    c(i)=tempc;
end
% Line parameters printing
string1="{";
string2="{";
for i=1:size(panel_l,2)
 string1=string1+a(i);
 string2=string2+c(i);
 if i<size(panel_l,2)
     string1=string1+",";
     string2=string2+",";
 end
end
string1=string1+"}";
string2=string2+"}";
disp(string1)
disp(string2)
