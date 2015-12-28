int N //Number of agents
int S//Number of partitions in the circle. Pick something between 5-10
R_danger//Radius of the collision circle. Pick twice the radius of the spiders


R_v=1;
offset=2*R_v/sin(pi/S)//This offset coefficient guarantees that no collision will take place. R_v should be <R_danger


xs=poses(:,1); //Agents x measurments (N agents)
ys=poses(:,2); //Agents y measurments (N agents)
thetas=poses(:,3); //Agents theta measurments (N agents)

xs_t=targets(:,1); //Target x coordinate for each agent (N agents)
ys_t=targets(:,2); //Target y coordinate for each agent (N agents)


//Generate the distance matrix (NxN matrix)
distances=zeros(N,N);
I=1;
for i=1:N
    for j=I:N
        distances(i,j)=sqrt( (xs(i)-xs(j))^2 + (ys(i)-ys(j))^2 );
        distances(j,i)=distances(i,j);
    end
    I=I+1;
end


//Generate the angle matrix (NxN matrix)
I=1;
angles=zeros(N,N);
for i=1:N
    for j=I:N
        if i~=j
            angles(i,j)=atan2( ys(j)-ys(i),xs(j)-xs(i) );
            angles(j,i)=angles(i,j)-pi;
        end
    end
    I=I+1;
end


//For each agent calculate the temporary target way point coordinates
for i=1:N 
    agent_distance=distances(i,:); //Take the row of the distance matrix distances that correpsonds to the i agent
    [r,c]=find(agent_distance>0 & agent_distance<R_danger); //Identify the robots that violate the collision condition with the i agent
    if isempty(c)==1 // If there is not a collision identify then the set-points for the i agent should be the targets
        set_points(i,:)=[xs_t(i) ys_t(i)]; %This means that there is no conflicts in the cone
    else //This is what happens when a collsion is identified
        //Figure out how many conflicts do you have
        J=length(c);
        phi_t=atan2 ((ys_t(i) - ys(i)) , (xs_t(i) - xs(i))); //Find the angle of the target
        rho_t= sqrt( (xs_t(i)-xs(i))^2+ (ys_t(i)-ys(i))^2 ); //Find the distance of the target
        //With these agents you have a conflict
        conflict_agents=[];
        conflict_agents=c;
        
        lc=length(conflict_agents); //I think this is the same with J      
        for j=1:S //Check all the particions of the circle
            x_t_temp=xs(i)+R_danger*cos(phi_t+(j-1)*((2*pi)/S));
            y_t_temp=ys(i)+R_danger*sin(phi_t+(j-1)*((2*pi)/S));  //Find the coordinates of the center point in each particion of the circle. If there is no conflict in this cone then proceed
            phi_t_temp=atan2 ((y_t_temp - ys(i)) , (x_t_temp - xs(i))); //Identify this angle 
            
            x_off=xs(i)-offset*cos(phi_t+(j-1)*((2*pi)/S));  
            y_off=ys(i)-offset*sin(phi_t+(j-1)*((2*pi)/S)); //Introduce the offset coordinates that guarantee that no collison will take palce
            
            theta_offs=zeros(1,lc);
            angle_checkI=zeros(1,lc);
            angle_checkII=zeros(1,lc);
            for js=1:lc //You will need to repeat this loop with all the agents that violate the collision condition
                theta_offs(1,js)=atan2(ys(conflict_agents(js))-y_off,xs(conflict_agents(js))-x_off); //Find the angle of the offset coordinate with the angle of the conflict agents
                angle_checkI(1,js)=cos(angles(i,conflict_agents(js))-phi_t_temp); //Is there an agent in the cone
                angle_checkII(1,js)=cos(theta_offs(1,js)-phi_t_temp);//If there is an agent in the cone is it in the back?
            end
            [rows,checkI]=find(angle_checkI>0);
            [rows,checkII]=find(angle_checkII>cos(pi/S)); //Check for the follwing conflicts
                        
            //%if isempty(checkI)==1 || (isempty(checkI)==0 && isempty(checkII)==1)
            if isempty(checkI)==1 || isempty(checkII)==1
                set_points(i,:)=[x_t_temp y_t_temp]; %This means that there is no conflicts in the cone //This means all clear. Set the temporatry coordinate as the center of the cone and break
                break
            else
                set_points(i,:)=[xs(i) ys(i)]; //If all the cones are occupied just stay were you are
            end 
        end
    end
end