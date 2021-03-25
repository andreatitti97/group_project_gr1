n_worlds=1;
n_trials=2;
error_char=zeros(2,n_worlds);
a_char=zeros(2,n_worlds);
c_char=zeros(2,n_worlds);
errors=zeros(2,n_worlds);
% For each world
for i=1:n_worlds
    string="World"+i;
    temp=[];
    % for each trial
    for j=1:n_trials
        err=load(string+"/error_dist_line"+j+".txt");
        mat=load(string+"/estimated_line"+j+".txt");
        index=mat(:,1);
        a=mat(:,2);
        c=mat(:,3);
        trial(i,j).error=[mean(err);var(err)];
        temp=[temp trial(i,j).error];
        trial(i,j).n_panel=max(index);
        trial(i,j).ang_coeff=[];
        trial(i,j).inter=[];
        for k=1:trial(i,j).n_panel
            inda=find(index==k);
            a_char(:,i)=[mean(a(inda)) var(a(inda))]';
            c_char(:,i)=[mean(c(inda)) var(c(inda))]';
            trial(i,j).ang_coeff=[trial(i,j).ang_coeff a_char];
            trial(i,j).inter=[trial(i,j).inter c_char];
        end
    end
    errors(:,i)=[mean(temp(1,:)); var(temp(2,:))];
end
errors=[errors [1 2 3;4 5 5]];
errorbar(errors(1,:),errors(2,:),'o')