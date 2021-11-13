%% generate random obstacles: 
clear all;
no_ob = 5;
pos_ob_array_pre_store = zeros(2,no_ob,50);
radius_pre_store = zeros(no_ob, 50);
for i_page=1:50
    
    no_ob = 5;
    flag_ok = 0; 
    radius = ones(no_ob,1);
    %any two of the obstcles should not overlap with another 
    while(flag_ok ==0)
        for i=1:no_ob
            radius(i) = 1+ 1.5*rand(1);
%             radius(i) = 4.184491324312271; %20180814, for test
            pos_ob(:,i) = [16+60*rand(1,1);  -1.7+ 3.4*rand(1,1) ];
            
%             pos_ob(:,i) = [80+100*rand(1,1);  -50+  3.4*rand(1,1) ];
%             pos_ob(:,i) = [33.7240534640672;1.56622368103891]; %20180814, for test
        end        
        pos_ob(1,:) = sort(pos_ob(1,:));
        
        if(no_ob>1)
            for i=1:(no_ob-1)
                flagin = 0;
                for j = (i+1):no_ob  
                    norm_test = norm(pos_ob(:,i) -  pos_ob(:,j));
                    if(norm_test <= radius(i)+radius(j))
    %                 if(norm_test <= 2)
                        flagin= 1;
                        break;
                    end
                end  
                if(flagin==1)
                       break;
                end
                if(i==no_ob-1) && (j==no_ob)
                    flag_ok=1;
                end
            end
        else
            flag_ok = 1;
        end
    end        
    
    pos_ob_array_pre_store(:,:,i_page) = pos_ob;
    radius_pre_store(:, i_page)= radius;
end

save pos_ob_array_pre_store20181122.mat pos_ob_array_pre_store radius_pre_store;

