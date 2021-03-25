clc;
clearvars;
close all

n_worlds=4;
n_trials=5;
error_char=zeros(2,n_worlds);
a_char=zeros(2,n_worlds);
c_char=zeros(2,n_worlds);
errors=zeros(2,n_worlds);
world1_coeff=[0.01051,-0.5868,-1.7454; 2.0766,5.8442,19.865];
world2_coeff=[0.01051,0.5868,1.7454; 2.08,-1.4645,-14.7954];
world3_coeff=[0.010515 ,1.7454,-1.7078;2.08,-8.6802,23.62];
world4_coeff=[0.0093723,1.7514,-1.7086,0.0096153; 2.08,-8.7211,23.6318,13.0899];
worlds={{world1_coeff} {world2_coeff} {world3_coeff} {world4_coeff}};
syms x; syms y;
% For each world
for i=1:n_worlds
    string="World"+i;
    temp=[];
    atemp=0;
    ctemp=0;
    % for each trial
    for j=1:n_trials
        err=load(string+"/error_dist_line"+j+".txt");
        mat=load(string+"/estimated_line"+j+".txt");
        index=mat(:,1);
        a=mat(:,2);
        c=mat(:,3);
        trial(i,j).error=[mean(err);std(err)];
        temp=[temp trial(i,j).error];
        trial(i,j).n_panel=max(index);
        trial(i,j).ang_coeff=[];
        trial(i,j).inter=[];
        for k=1:trial(i,j).n_panel
            inda=find(index==k);
            a_char(:,i)=[mean(a(inda)) std(a(inda))]';
            c_char(:,i)=[mean(c(inda)) std(c(inda))]';
            trial(i,j).ang_coeff=[trial(i,j).ang_coeff a_char(:,i)];
            trial(i,j).inter=[trial(i,j).inter c_char(:,i)];
        end
        atemp=atemp+trial(i,j).ang_coeff;
        ctemp=ctemp+trial(i,j).inter;
    end
    

    atemp=atemp/n_trials;
    ctemp=ctemp/n_trials;
    figure
    world_coeff=cell2mat(worlds{i});
    plot(world_coeff(1,:),world_coeff(2,:),'go')
    hold on
    plot(atemp(1,:),ctemp(1,:),'mo')
    for p=1:size(atemp,2)
        text(atemp(1,p)-0.1,ctemp(1,p)+1.1,"panel "+p)
        plotEllipse(atemp(1,p),ctemp(1,p),atemp(2,p),ctemp(2,p))
    end
    title("World "+i+" real and estimated line parameters of the panels")
    xlabel("angular coefficients [adimensionsional]")
    ylabel("World y-axis-intercept[m]")
    legend("real line coefficients","estimated coefficients")
    xlim([min(atemp(1,:))-0.4,max(atemp(1,:))+0.4])
    ylim([min(ctemp(1,:))-2,max(ctemp(1,:))+2])
    errors(:,i)=[mean( temp(1,:)); mean( temp(2,:))];
end
figure
errorbar(1:2:8,errors(1,:),errors(2,:),'gs','MarkerSize',10,...
    'MarkerEdgeColor','black','MarkerFaceColor','magenta')
xlim([0 9])
ylim([-1 2])
title('Error mean and variance over the worlds')
ylabel('error mean and variance[m]')
set(gca,'xtick',1:2:8);
set(gca,'XTickLabel',["world1","world2","world3","world4"],'fontsize',8);

function []=plotEllipse(x_center,y_center,x_length,y_length)
     t = linspace(0,2*pi);
     x = x_center + x_length*cos(t);
     y = y_center + y_length*sin(t);
     hold on
     plot(x,y,'b--')
end