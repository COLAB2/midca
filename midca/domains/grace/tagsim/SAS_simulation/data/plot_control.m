function [  ] = plot_control(  )
%t_vec and x_vec are the full trajectories from initial to final time

    global master
    global param

    
    t0 = param.t0;
    tf = param.tf;
    
    %filter actions that are out of tf (because receding horizon goes up to tf+T)
    a=(tf >= master.all_actions(:,1));
        
    all_actions = master.all_actions(1:sum(a),:);
    %check if t_f of last action is larger that tf
    if all_actions(end,2) > tf
        all_actions(end,2) = tf;
    end
    
    for k=1:param.ulen %repeat loop for all controls
    
        figure,

        hold on

        for i=1:size(all_actions,1)
            if(i==1)
                if(all_actions(i,1) > t0)%first iteration, compare t_i with t0
                    if nargin==2%checks whether we want to plot wrt to u_nominal or zero
                        temp = t_vec(t_vec<all_actions(i,1));
                        time = [temp;all_actions(i,1)];
                        x_now = interp1(t_vec,x_vec,all_actions(i,1)); %interpolate
                        x = [x_vec(1:length(temp),:);x_now];
                        u = []; %populate control u
                        for j=1:size(x,1)
                            u_nom_temp = param.u_nominal(x(j,:));
                            u(j) = u_nom_temp(k);
                        end
                    else
                        time = [t0;all_actions(i,1)];
                        u = [];
                        u = zeros(length(time),1);
                    end
                    plot(time, u) %plot nominal
                end    
            else%for the rest iterations
                if(all_actions(i,1) > all_actions(i-1,2))
                    if nargin==2%checks whether we want to plot wrt to u_nominal or zero
                        condition = (t_vec<all_actions(i,1))&(t_vec>all_actions(i-1,2));%index state and time between actions
                        temp = t_vec(condition);
                        time = [all_actions(i-1,2);temp;all_actions(i,1)];  
                        x_final = interp1(t_vec,x_vec,all_actions(i,1)); %interpolate
                        x_init = interp1(t_vec,x_vec,all_actions(i-1,2)); %interpolate
                        x = [x_init;x_vec(condition,:);x_final];
                        u = []; %populate control u
                        for j=1:size(x,1)                        
                            u_nom_temp = param.u_nominal(x(j,:));
                            u(j) = u_nom_temp(k);
                        end
                    else
                        time = [all_actions(i-1,2);all_actions(i,1)];
                        u = [];
                        u = zeros(length(time),1);
                    end
                    plot(time, u) %plot nominal
                end 

            end

            if(i==size(all_actions,1))% for last iteration compare tf with t_f
                if(all_actions(i,2) < tf) 
                    if nargin==2%checks whether we want to plot wrt to u_nominal or zero
                        temp = t_vec(t_vec>all_actions(i,2));
                        time = [all_actions(i,2);temp];
                        x_now = interp1(t_vec,x_vec,all_actions(i,2)); %interpolate
                        x = [x_now;x_vec(end+1-length(temp):end,:)];
                        u = []; %populate control u
                        for j=1:size(x,1)
                            u_nom_temp = param.u_nominal(x(j,:));
                            u(j) = u_nom_temp(k);
                        end
                    else
                        time = [all_actions(i,2);tf];
                        u = [];
                        u = zeros(length(time),1);
                    end
                    plot(time, u) %plot nominal
                end 
            end


            %checks whether we want to plot wrt to u_nominal or zero -------
            if nargin==2
                x_now = interp1(t_vec,x_vec,all_actions(i,2)); %interpolate
                u_nom_temp = param.u_nominal(x_now);%nominal
                unext = u_nom_temp(k);
                x_prev = interp1(t_vec,x_vec,all_actions(i,1)); %interpolate
                u_nom_temp = param.u_nominal(x_prev);%nominal;
                uprev = u_nom_temp(k);
            else
                unext = 0;
                uprev = 0;
            end

            if (i~=size(all_actions,1))
                if(all_actions(i,2) == all_actions(i+1,1))
                    unext = [];
                    unext = all_actions(i+1,2+k);%action
                end
            end

            if (i~=1)
                if(all_actions(i,1) == all_actions(i-1,2))
                    uprev = [];
                    uprev = all_actions(i-1,2+k);%action
                end
            end

            %--------------------------------------------------------------

            %plot action
            stairs([all_actions(i,1)-param.epsilon, all_actions(i,1),all_actions(i,2)],[uprev, all_actions(i,2+k), unext]);
        end

        leg = {['Control ' num2str(k)]};
        xlabel('time (s)')
        legend(leg);
        hold off
        
    end


end

