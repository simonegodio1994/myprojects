#include <iostream>
#include <math.h>
#define R  162
#define C  288
#define NU 10
using namespace std;

// Define the UAV structure:
typedef struct UAV{
    float x,y,z;
    int id;
}UAV;

// Define the target structure:
typedef struct target{
    float x,y,z;
}target;

// Define the neuron structure:
typedef struct neuron{
    int id, penalty = 0, facil = 0;
    float cost, neighbors_cost[8];
    // ...
}neuron;


int main()
{
    // Define Resolution of the cost map on the map:
    int xd, yd, dir_chosen[NU], id_chosen[NU], Resolution = 1;
    
    // Define variables to save the previous position:
    int id_previous[NU]; 
    
    // Define variables useful for the path planning:
    float  nx, ny, m, q, d[8], dlim, cost_max;
    
    // Declare the weights for each parameter:
    // Weight of the cost map:
    float   w_cm = 0.0;
    // Weight of the mean distance among UAVs:
    float   w_md = 1.0;
    // Weight of the variance of the distance among UAVs:
    float   w_vr = 1.0;
    // Weight of the facilitated path parameter:
    float   w_pn = 1.0;
    
    // Declare the cost map and the map matrix:
    float  cost_map[R*Resolution][C*Resolution];
    int    map[R][C], cp=0, n_o=0;
    
    // Declare the matrix that avoid collisions between trajectories:
    int    traj_check[R][C], traj[R][C];
    
    // Declare the Area covered size variable [m] for each UAV:
    float  CS = 28;
    
    // Define penalization parameters variables:
    int xnuav, ynuav, xmov, ymov, idwp, idwp_prec;
    float perc_cov, perc_lim = 0.99, duav, dwpuav, dref, dmin = R*C;

    // Declare moves counter:
    int    move_count = 0, count_prec = 0;
    
    // Declare variables useful for the iso-distance lines definition:
    float  n_lines, flag_left, flag_right, flag_up, flag_down;
    
    // Declare the neuron coordinates map & neurons list:
    neuron neuron_map[R*Resolution][C*Resolution];
    
    // Declare the vectors containing info about the corners 
    // of the areas covered by each drone:
    float  x_c[NU][2], y_c[NU][2];
    
    // Simple index for loops:
    int    i, j, k, h, z;
    
    // Declare drones and target variables:
    target target;
    UAV    drone[NU];
    
    // Declare max point dist variables:
    float  max_point_dist = 0, dm, min_mpd = sqrt(R*R + C*C);
    
    // Declare the flag to avoid collisions:
    int    flag_coll[8];
    
    // Declare the matrix containing the relative distances
    // among drones:
    float  dist[NU-1], mean_d, sum_d, var_d, max_var, max_mean = sqrt(R*R + C*C), max_d = sqrt(R*R + C*C);
    
    // Declare useful counters & flag:
    int   c_dist, c_neurons, all_covered = 0, move_all, count_all = 0;
    
    // Print results variables:
    int tot_p[R*Resolution][C*Resolution], count_tot = 0;
    
    // Declare file pointers:
    FILE *fp, *fp2;
    
    // Read the 2D input map from "map.txt" and save into map[R][C]:
    fp=fopen("map.txt","r");
    for (i = 0; i < R; i++){ 
        for (j = 0; j < C; j++){ 
            fscanf(fp, "%d ", &map[i][j]);
            if(map[i][j] == 1)
                n_o++;
        }
    }
    fclose(fp);

    
    // Initialize the cost map and fill the obstacle coordinates with a -1:
    for(i=0; i < R; i++){
        for(j=0; j < C; j++){
            if(map[i][j] == 1)
                cost_map[i][j] = -1;
            else
                cost_map[i][j] = 0;
            tot_p[i][j] = 0;
            neuron_map[i][j].facil = 0;
        }
    }
    
    // Initialize the previous id vector:
    for(i=0; i < NU; i++)
        id_previous[i] = -1;
        
    // Define the initial position and their id for each UAV [m]:
    
    drone[0].id = 0;
    drone[0].x  = 8;
    drone[0].y  = 8;
    drone[0].z  = 0;
    
    drone[1].id = 1;
    drone[1].x  = 156;
    drone[1].y  = 8;
    drone[1].z  = 0;
    
    drone[2].id = 2;
    drone[2].x  = 276;
    drone[2].y  = 8;
    drone[2].z  = 0;

    drone[3].id = 3;
    drone[3].x  = 204;
    drone[3].y  = 62;
    drone[3].z  = 0;    
    
    drone[4].id = 4;
    drone[4].x  = 220;
    drone[4].y  = 90;
    drone[4].z  = 0;
    
    drone[5].id = 5;
    drone[5].x  = 12;
    drone[5].y  = 155;
    drone[5].z  = 0;  
    
    drone[6].id = 6;
    drone[6].x  = 100;
    drone[6].y  = 50;
    drone[6].z  = 0; 
    
    drone[7].id = 7;
    drone[7].x  = 262;
    drone[7].y  = 148;
    drone[7].z  = 0; 
    
    drone[8].id = 8;
    drone[8].x  = 160;
    drone[8].y  = 90;
    drone[8].z  = 0; 
    
    drone[9].id = 9;
    drone[9].x  = 100;
    drone[9].y  = 120;
    drone[9].z  = 0; 
    
    // Initialize neuron id chosen vector for the next move:
    for(i=0; i < NU; i++){
        id_chosen[i]   = -1;
    }
    
    // Find the corners coordinates for each area covered:
    while (move_count < 250){
        for(i=0; i < NU; i++){
            // Left check:
            if(drone[i].x-int(CS/2) >= 0)
                x_c[i][0]                     = drone[i].x-int(CS/2);
            else
                x_c[i][0]                     = 0;
            
            // Right check:
            if(drone[i].x+int(CS/2) < C)
                x_c[i][1]                     = drone[i].x+int(CS/2);
            else
                x_c[i][1]                     = C-1;
            
            // Down map check:
            if(drone[i].y - int(CS/2) >= 0)
                y_c[i][0]                     = drone[i].y-int(CS/2);
            else
                y_c[i][0]                     = 0;
                
            // Up map check:
            if(drone[i].y + int(CS/2)< R)
                y_c[i][1]                     = drone[i].y+int(CS/2);
            else
                y_c[i][1]                     = R-1;
        }    
    
        // Penalize the already covered point:
        for (k=0; k < NU; k++){
            for (i=y_c[k][0]; i <= y_c[k][1]; i++){
                for (j=x_c[k][0]; j <= x_c[k][1]; j++){
                    if(cost_map[i][j] != -1){
                        neuron_map[i][j].penalty = 1;
                        tot_p[i][j] = 1;
                    }                        
                }
            }
        }  
  
  
        // Create a facilitated path toward the uncovered points:
        for (i=0; i < R; i++){
            for(j=0; j < C; j++){
                if(neuron_map[i][j].penalty == 0 && cost_map[i][j] != -1){
                    dmin = R*C;
                    // Find the nearest UAV:
                    xmov   = j;
                    ymov   = i;
                    // Select the nearest UAV:
                    for(z=0; z < NU; z++){
                        duav = sqrt(pow((drone[z].x - xmov),2)+pow((drone[z].y - ymov),2));
                        if(duav < dmin){
                            dmin  = duav;
                            xnuav = drone[z].x;  
                            ynuav = drone[z].y;
                        }  
                    }
                    dref   = dmin;
                    while(dmin > Resolution){
                        dmin = R*C;
                        // First neighbor (up-left):
                        dwpuav = sqrt(pow((xnuav - (xmov - Resolution)),2)+pow((ynuav - (ymov - Resolution)),2));
                        if(ymov-Resolution >= 0 && xmov-Resolution >= 0){
                            if(dwpuav < dmin){
                                idwp = 0;
                                dmin = dwpuav;
                            }
                        }
                        // Second neighbor (up):    
                        dwpuav = sqrt(pow((xnuav - xmov),2)+pow((ynuav - (ymov - Resolution)),2));
                        if(ymov-Resolution >= 0){
                            if(dwpuav < dmin){
                                idwp = 1;
                                dmin = dwpuav;
                            }
                        }                             
                        // Third neighbor (up-right):
                        dwpuav = sqrt(pow((xnuav - (xmov + Resolution)),2)+pow((ynuav - (ymov - Resolution)),2));
                        if(ymov-Resolution >= 0 && xmov + Resolution < C){
                            if(dwpuav < dmin){
                                idwp = 2;
                                dmin = dwpuav;
                            }
                        }
                        // Fourth neighbor (right):    
                        dwpuav = sqrt(pow((xnuav - (xmov + Resolution)),2)+pow((ynuav - ymov),2));
                        if(xmov + Resolution < C){
                            if(dwpuav < dmin){
                                idwp = 3;
                                dmin = dwpuav;
                            }
                        }
                        // Fifth neighbor (down-right):  
                        dwpuav = sqrt(pow((xnuav - (xmov + Resolution)),2)+pow((ynuav - (ymov + Resolution)),2));
                        if(ymov + Resolution < R && xmov + Resolution < C){
                            if(dwpuav < dmin){
                                idwp = 4;
                                dmin = dwpuav;
                            }
                        }
                        // Sixth neighbor (down):    
                        dwpuav = sqrt(pow((xnuav - xmov),2)+pow((ynuav - (ymov + Resolution)),2));
                        if(ymov + Resolution < R){
                            if(dwpuav < dmin){
                                idwp = 5;
                                dmin = dwpuav;
                            }
                        }
                        // Seventh neighbor (down-left):    
                        dwpuav = sqrt(pow((xnuav - (xmov - Resolution)),2)+pow((ynuav - (ymov + Resolution)),2));
                        if(ymov + Resolution < R && xmov - Resolution >= 0){
                            if(dwpuav < dmin){
                                idwp = 6;
                                dmin = dwpuav;
                            }
                        }
                        // Eigth neighbor (left):    
                        dwpuav = sqrt(pow((xnuav - (xmov - Resolution)),2)+pow((ynuav - ymov),2));
                        if(xmov - Resolution >= 0){
                            if(dwpuav < dmin){
                                idwp = 7;
                                dmin = dwpuav;
                            }
                        }
                        if(idwp == 0){
                            xmov  = xmov - Resolution;
                            ymov  = ymov - Resolution;
                            if(cost_map[ymov][xmov] != -1 && neuron_map[ymov][xmov].facil != 1)
                                cost_map[ymov][xmov] = cost_map[ymov][xmov] + dmin / max_d * w_pn;
                            neuron_map[ymov][xmov].facil = 1;
                        }
                        if(idwp == 1){
                            xmov  = xmov;
                            ymov  = ymov - Resolution;
                            if(cost_map[ymov][xmov] != -1 && neuron_map[ymov][xmov].facil != 1)
                                cost_map[ymov][xmov] = cost_map[ymov][xmov] + dmin / max_d * w_pn;
                            neuron_map[ymov][xmov].facil = 1;    
                        }
                        if(idwp == 2){
                            xmov  = xmov + Resolution;
                            ymov  = ymov - Resolution;
                            if(cost_map[ymov][xmov] != -1 && neuron_map[ymov][xmov].facil != 1)
                                cost_map[ymov][xmov] = cost_map[ymov][xmov] + dmin / max_d * w_pn;
                            neuron_map[ymov][xmov].facil = 1;    
                        }
                        if(idwp == 3){
                            xmov  = xmov + Resolution;
                            ymov  = ymov;
                            if(cost_map[ymov][xmov] != -1 && neuron_map[ymov][xmov].facil != 1)
                                cost_map[ymov][xmov] = cost_map[ymov][xmov] + dmin / max_d * w_pn;
                            neuron_map[ymov][xmov].facil = 1;    
                        }
                        if(idwp == 4){
                            xmov  = xmov + Resolution;
                            ymov  = ymov + Resolution;
                            if(cost_map[ymov][xmov] != -1 && neuron_map[ymov][xmov].facil != 1)
                                cost_map[ymov][xmov] = cost_map[ymov][xmov] + dmin / max_d * w_pn;
                            neuron_map[ymov][xmov].facil = 1;    
                        }
                        if(idwp == 5){
                            xmov  = xmov;
                            ymov  = ymov + Resolution;
                            if(cost_map[ymov][xmov] != -1 && neuron_map[ymov][xmov].facil != 1)
                                cost_map[ymov][xmov] = cost_map[ymov][xmov] + dmin / max_d * w_pn;
                            neuron_map[ymov][xmov].facil = 1;    
                        }
                        if(idwp == 6){
                            xmov  = xmov - Resolution;
                            ymov  = ymov + Resolution;
                            if(cost_map[ymov][xmov] != -1 && neuron_map[ymov][xmov].facil != 1)
                                cost_map[ymov][xmov] = cost_map[ymov][xmov] + dmin / max_d * w_pn;
                            neuron_map[ymov][xmov].facil = 1;    
                        }
                        if(idwp == 7){
                            xmov  = xmov - Resolution;
                            ymov  = ymov;
                            if(cost_map[ymov][xmov] != -1 && neuron_map[ymov][xmov].facil != 1)
                                cost_map[ymov][xmov] = cost_map[ymov][xmov] + dmin / max_d * w_pn;
                            neuron_map[ymov][xmov].facil = 1;    
                        }
                    idwp_prec = idwp;    
                    }
                }    
            }
        }
        
        for (i=0; i < R; i++){
            for(j=0; j < C; j++){
                if(cost_map[i][j] != -1 && neuron_map[i][j].penalty == 0){
                    dmin = R*C;
                    // Find the nearest UAV:
                    for(z=0; z < NU; z++){
                        duav = sqrt(pow((drone[z].x - j),2)+pow((drone[z].y - i),2));
                        if(duav < dmin)
                            dmin  = duav;
                    }
                    cost_map[i][j] = cost_map[i][j] + dmin / max_d * w_pn;
                }
                neuron_map[i][j].facil = 0;
            }
        }

        for(i=0; i < R; i++){
            for(j=0; j < C; j++){
                    traj_check[i][j] = 0;
                    traj[i][j] = 0;
                }
        }
                
        // Start the cycle to find the next waypoint for each UAV:
        for (z=0; z < NU; z++){
            
            // Iso-distances lines:
            // Set parameters for iso-distance lines:
            flag_left                   =  0;
            flag_right                  =  0;
            flag_up                     =  0;
            flag_down                   =  0;
            n_lines                     =  0;
            
            // Fill the cost map matrix for each UAV contribution:
            for(k=0; k < NU; k++){
                while(flag_left == 0 || flag_right==0 || flag_up==0 || flag_down==0){
                    for (i=(y_c[k][0]-n_lines); i <= (y_c[k][1]+n_lines); i++){  
                        if(i==(y_c[k][0]-n_lines) || i==(y_c[k][1]+n_lines)){
                            for(j = (x_c[k][0]-n_lines); j <= (x_c[k][1]+n_lines); j++){
                                if(j >= 0 && j < C && i >= 0 && i < R && cost_map[i][j] != -1){
                                    cost_map[i][j]  = cost_map[i][j] + n_lines / max_d * w_cm;    
                                }    
                            }
                        }
                        else{
                            for(j = (x_c[k][0]-n_lines); j <= (x_c[k][1]+n_lines); j=j+(x_c[k][1]-x_c[k][0]+2*n_lines)){
                                if(j >= 0 && j < C && i >= 0 && i < R && cost_map[i][j] != -1){
                                    cost_map[i][j]  = cost_map[i][j] + n_lines / max_d * w_cm;    
                                }    
                            }
                        }
                    }
                    
                    if(x_c[k][0]-n_lines <= 0)
                        flag_left   =  1;
            
                    if(x_c[k][1]+n_lines >= C-1)
                        flag_right  =  1;
            
                    if(y_c[k][0]-n_lines <= 0)
                        flag_down   =  1;
            
                    if(y_c[k][1]+n_lines >= R-1)
                        flag_up     =  1;
                    n_lines                 =  n_lines + 1;
                }
                flag_left                   =  0;
                flag_right                  =  0;
                flag_up                     =  0;
                flag_down                   =  0;
                n_lines                     =  0;
            }
        
            // Assignments to each neuron:
            c_neurons=0;
            for(i=0; i < R; i++){
                for(j=0; j < C; j++){
                    
                    // Fill the neuron id and cost:
                    neuron_map[i][j].id                    = c_neurons;
                    neuron_map[i][j].cost                  = cost_map[i][j];
                    
                    // First neighbor (up-left):
                    if(i-int(CS/2+1) >= 0 && j-int(CS/2+1) >= 0)
                        neuron_map[i][j].neighbors_cost[0] = cost_map[i-int(CS/2+1)][j-int(CS/2+1)];
                    else
                        neuron_map[i][j].neighbors_cost[0] = -3;  // -3 for not existing neighbor
                         
                    // Second neighbor (up):    
                    if(i-int(CS/2+1) >= 0)
                        neuron_map[i][j].neighbors_cost[1] = cost_map[i-int(CS/2+1)][j-1];
                    else
                        neuron_map[i][j].neighbors_cost[1] = -3;  // -3 for not existing neighbor     
                         
                    // Third neighbor (up-right):    
                    if(i-int(CS/2+1) >= 0 && j+int(CS/2+1) < C)
                        neuron_map[i][j].neighbors_cost[2] = cost_map[i-int(CS/2+1)][j+int(CS/2+1)];
                    else
                        neuron_map[i][j].neighbors_cost[2] = -3;  // -3 for not existing neighbor 
                        
                    // Fourth neighbor (right):    
                    if(j+int(CS/2+1) < C)
                        neuron_map[i][j].neighbors_cost[3] = cost_map[i+1][j+int(CS/2+1)];
                    else
                        neuron_map[i][j].neighbors_cost[3] = -3;  // -3 for not existing neighbor 
                        
                    // Fifth neighbor (down-right):    
                    if(i+int(CS/2+1) <= R && j+int(CS/2+1) < C)
                        neuron_map[i][j].neighbors_cost[4] = cost_map[i+int(CS/2+1)][j+int(CS/2+1)];
                    else
                        neuron_map[i][j].neighbors_cost[4] = -3;  // -3 for not existing neighbor  
                        
                    // Sixth neighbor (down):    
                    if(i+int(CS/2+1) <= R)
                        neuron_map[i][j].neighbors_cost[5] = cost_map[i+int(CS/2+1)][j];
                    else
                        neuron_map[i][j].neighbors_cost[5] = -3;  // -3 for not existing neighbor 
                        
                    // Seventh neighbor (down-left):    
                    if(i+int(CS/2+1) <= R && j-int(CS/2+1) >= 0)
                        neuron_map[i][j].neighbors_cost[6] = cost_map[i+int(CS/2+1)][j-int(CS/2+1)];
                    else
                        neuron_map[i][j].neighbors_cost[6] = -3;  // -3 for not existing neighbor
                        
                    // Eigth neighbor (left):    
                    if(j-int(CS/2+1) >= 0)
                        neuron_map[i][j].neighbors_cost[7] = cost_map[i][j-int(CS/2+1)];
                    else
                        neuron_map[i][j].neighbors_cost[7] = -3;  // -3 for not existing neighbor                 
        
                    c_neurons++;
                }
            }    
            
            // Find the goal id for each UAV:
            // Verify if there are obstacles among the neuron and its neighbors:
            yd            = drone[z].y;
            xd            = drone[z].x;
            dlim          = 2;                     // [m]
            cost_max      = -3;
            for (i=0; i < 8; i++)
                flag_coll[i] = 0;
            dir_chosen[z] = 8;
            id_chosen[z]  = neuron_map[yd][xd].id;
            
            // 1st neighbor and first half of the 2nd and the eight neighbor:
            if(neuron_map[yd][xd].neighbors_cost[0] != -3){
        
                // Calculate lines coefficients:
                nx        = xd - int(CS/2+1);
                ny        = yd - int(CS/2+1);
                m         = (ny - yd)/(nx - xd);
                q         = yd - xd * (ny - yd)/(nx - xd);
    
                for(i=ny; i <= yd; i++){
                    for(j=nx; j <= xd; j++){
                        if(cost_map[i][j] == -1 && i >= 0 && i < R && j >= 0 && j < C){
                            d[0] = abs((i - m * j - q) / sqrt(1 + m * m));
                            if(d[0] <= dlim)
                                flag_coll[0] = 1;
                            // First half of the 2nd neighbor:   
                            d[1] = abs(j - xd);
                            if(d[1] <= dlim)
                                flag_coll[1] = 1;      
                            // First half of the 8th neighbor:   
                            d[7] = abs(i - yd);
                            if(d[7] <= dlim)
                                flag_coll[7] = 1;          
                        }       
                    }
                }
                if(flag_coll[0] == 0 && id_previous[z] != neuron_map[int(drone[z].y-CS/2)][int(drone[z].x-CS/2)].id){
                    for(i=0; i < NU; i++){
                        if(i!=z && neuron_map[int(drone[z].y-CS/2)][int(drone[z].x-CS/2)].id == id_chosen[i])
                            flag_coll[0] = 1;
                    }
                    c_dist = 0;
                    sum_d  = 0;
                    var_d  = 0;
                    for(j=0; j < NU; j++){
                        if(z!=j){
                            dist[c_dist] = sqrt(pow(((drone[z].x-CS/2) - drone[j].x),2)+pow(((drone[z].y-CS/2) - drone[j].y),2)+pow((drone[z].z - drone[j].z),2));
                            sum_d = sum_d + dist[c_dist];
                            c_dist++;
                        }
                    }    
                    mean_d  = sum_d / (NU-1);
                    max_var = mean_d*mean_d*(NU-1);
                    for(j = 0; j < NU-1; j++){
                        var_d = var_d + pow((dist[j] - mean_d),2) / (NU-1);
                    }
                    var_d = sqrt(var_d);
                    if(neuron_map[yd][xd].neighbors_cost[0] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr > cost_max && flag_coll[0] == 0){
                        for(i=0; i < R; i++)
                            for(j=0; j < C; j++)
                                traj_check[i][j] = 0;
                            for(i=ny; i <= yd-1; i++){
                                for(j=nx; j <= xd-1; j++){
                                if(traj[i][j] == 1){
                                    flag_coll[0] = 1;
                                }
                                else
                                    traj_check[i][j] = 1;
                            }
                        }
                        if(flag_coll[0] == 0){
                            cost_max      = neuron_map[yd][xd].neighbors_cost[0] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr;
                            id_chosen[z]  = neuron_map[int(drone[z].y-CS/2)][int(drone[z].x-CS/2)].id;
                            dir_chosen[z] = 0;
                        }
                    }    
                }
            }
        
            // 3rd neighbor, 2nd half of the 2nd neighbor and 1st half of the 4th:
            if(neuron_map[yd][xd].neighbors_cost[2] != -3){
            
                // Calculate lines coefficients:
                nx        = xd + int(CS/2+1);
                ny        = yd - int(CS/2+1);
                m         = (ny - yd)/(nx - xd);
                q         = yd - xd * (ny - yd)/(nx - xd);
        
                for(i=ny; i <= yd; i++){
                    for(j=xd; j <= nx; j++){
                        if(cost_map[i][j] == -1 && i >= 0 && i < R && j >= 0 && j < C){
                            d[2] = abs((i - m * j - q) / sqrt(1 + m * m));
                            if(d[2] <= dlim)
                                flag_coll[2] = 1;
                            // 2nd half of the 2nd neighbor:   
                            d[1] = abs(j - xd);
                            if(d[1] <= dlim)
                                flag_coll[1] = 1;    
                            // 1st half of the 4th neighbor:   
                            d[3] = abs(i - yd);
                            if(d[3] <= dlim)
                                flag_coll[3] = 1;                            
                        }       
                    }
                }
                if(flag_coll[1] == 0 && id_previous[z] != neuron_map[int(drone[z].y-CS/2)][int(drone[z].x)].id){
                    for(i=0; i < NU; i++){
                        if(i!=z && neuron_map[int(drone[z].y-CS/2)][int(drone[z].x)].id == id_chosen[i])
                            flag_coll[1] = 1;
                    }            
                    c_dist = 0;
                    sum_d  = 0;
                    var_d  = 0;
                    for(j=0; j < NU; j++){
                        if(z!=j){
                            dist[c_dist] = sqrt(pow(((drone[z].x) - drone[j].x),2)+pow(((drone[z].y-CS/2) - drone[j].y),2)+pow((drone[z].z - drone[j].z),2));
                            sum_d = sum_d + dist[c_dist];
                            c_dist++;
                        }
                    }    
                    mean_d  = sum_d / (NU-1);
                    max_var = mean_d*mean_d*(NU-1);
                    for(j = 0; j < NU-1; j++){
                        var_d = var_d + pow((dist[j] - mean_d),2) / (NU-1);
                    }
                    var_d = sqrt(var_d);
                    if(neuron_map[yd][xd].neighbors_cost[1] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr > cost_max  && flag_coll[1]==0){
                        for(i=0; i < R; i++)
                            for(j=0; j < C; j++)
                                traj_check[i][j] = 0;
                        for(i=ny; i <= yd-1; i++){
                            for(j=xd; j <= nx-1; j++){
                                if(traj[i][j] == 1){
                                    flag_coll[1] = 1;
                                }
                                else
                                    traj_check[i][j] = 1;
                            }
                        }           
                        if(flag_coll[1] == 0){
                            cost_max      = neuron_map[yd][xd].neighbors_cost[1] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr;
                            id_chosen[z]  = neuron_map[int(drone[z].y-CS/2)][int(drone[z].x)].id;
                            dir_chosen[z] = 1;
                        }
                    }    
                }
                if(flag_coll[2] == 0 && id_previous[z] != neuron_map[int(drone[z].y-CS/2)][int(drone[z].x + CS/2)].id){
                    for(i=0; i < NU; i++){
                        if(i!=z && neuron_map[int(drone[z].y-CS/2)][int(drone[z].x + CS/2)].id == id_chosen[i])
                            flag_coll[2] = 1;
                    }               
                    c_dist = 0;
                    sum_d  = 0;
                    var_d  = 0;
                    for(j=0; j < NU; j++){
                        if(z!=j){
                            dist[c_dist] = sqrt(pow(((drone[z].x+CS/2) - drone[j].x),2)+pow(((drone[z].y-CS/2) - drone[j].y),2)+pow((drone[z].z - drone[j].z),2));
                            sum_d = sum_d + dist[c_dist];
                            c_dist++;
                        }
                    }    
                    mean_d  = sum_d / (NU-1);
                    max_var = mean_d*mean_d*(NU-1);            
                    for(j = 0; j < NU-1; j++){
                        var_d = var_d + pow((dist[j] - mean_d),2) / (NU-1);
                    }
                    var_d = sqrt(var_d);
                    if(neuron_map[yd][xd].neighbors_cost[2] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr > cost_max  && flag_coll[2]==0){
                        for(i=0; i < R; i++)
                            for(j=0; j < C; j++)
                                traj_check[i][j] = 0;
                        for(i=ny; i <= yd-1; i++){
                            for(j=xd; j <= nx-1; j++){
                                if(traj[i][j] == 1){
                                    flag_coll[2] = 1;
                                }
                                else
                                    traj_check[i][j] = 1;
                            }
                        }
                        if(flag_coll[2] == 0){
                            cost_max      = neuron_map[yd][xd].neighbors_cost[2] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr;
                            id_chosen[z]  = neuron_map[int(drone[z].y-CS/2)][int(drone[z].x + CS/2)].id;
                            dir_chosen[z] = 2;
                        }
                    }    
                }
                    
            }
        
            // 5th neighbor, 2nd half of the 4th neighbor and 1st half of the 6th:
            if(neuron_map[yd][xd].neighbors_cost[4] != -3){
                 
                // Calculate lines coefficients:
                nx        = xd + int(CS/2+1);
                ny        = yd + int(CS/2+1);
                m         = (ny - yd)/(nx - xd);
                q         = yd - xd * (ny - yd)/(nx - xd);
        
                for(i=yd; i <= ny; i++){
                    for(j=xd; j <= nx; j++){
                        if(cost_map[i][j] == -1 && i >= 0 && i < R && j >= 0 && j < C){
                            d[4] = abs((i - m * j - q) / sqrt(1 + m * m));
                            if(d[4] <= dlim)
                                flag_coll[4] = 1;
                            // 2nd half of the 4th neighbor:   
                            d[3] = abs(i - yd);
                            if(d[3] <= dlim)
                                flag_coll[3] = 1;    
                            // 1st half of the 6th neighbor:   
                            d[5] = abs(j - xd);
                            if(d[5] <= dlim)
                                flag_coll[5] = 1;                            
                        }       
                    }
                }
                if(flag_coll[3] == 0 && id_previous[z] != neuron_map[int(drone[z].y)][int(drone[z].x + CS/2)].id){
                    for(i=0; i < NU; i++){
                        if(i!=z && neuron_map[int(drone[z].y)][int(drone[z].x + CS/2)].id == id_chosen[i])
                            flag_coll[3] = 1;
                    }              
                    c_dist = 0;
                    sum_d  = 0;
                    var_d  = 0;
                    for(j=0; j < NU; j++){
                        if(z!=j){
                            dist[c_dist] = sqrt(pow(((drone[z].x+CS/2) - drone[j].x),2)+pow(((drone[z].y) - drone[j].y),2)+pow((drone[z].z - drone[j].z),2));
                            sum_d = sum_d + dist[c_dist];
                            c_dist++;
                        }
                    }    
                    mean_d  = sum_d / (NU-1);
                    max_var = mean_d*mean_d*(NU-1);            
                    for(j = 0; j < NU-1; j++){
                        var_d = var_d + pow((dist[j] - mean_d),2) / (NU-1);
                    }
                    var_d = sqrt(var_d);
                    if(neuron_map[yd][xd].neighbors_cost[3] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr > cost_max  && flag_coll[3]==0){
                        for(i=0; i < R; i++)
                            for(j=0; j < C; j++)
                                traj_check[i][j] = 0;
                        for(i=yd; i <= ny-1; i++){
                            for(j=xd; j <= nx-1; j++){
                                if(traj[i][j] == 1){
                                    flag_coll[3] = 1;
                                }
                                else
                                    traj_check[i][j] = 1;
                            }
                        }  
                        if(flag_coll[3] == 0){
                            cost_max      = neuron_map[yd][xd].neighbors_cost[3] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr;
                            id_chosen[z]  = neuron_map[int(drone[z].y)][int(drone[z].x + CS/2)].id;
                            dir_chosen[z] = 3;
                        }
                    }    
                }
                if(flag_coll[4] == 0 && id_previous[z]!=neuron_map[int(drone[z].y+CS/2)][int(drone[z].x + CS/2)].id){
                    for(i=0; i < NU; i++){
                        if(i!=z && neuron_map[int(drone[z].y+CS/2)][int(drone[z].x + CS/2)].id == id_chosen[i])
                            flag_coll[4] = 1;
                    }                
                    c_dist = 0;
                    sum_d  = 0;
                    var_d  = 0;
                    for(j=0; j < NU; j++){
                        if(z!=j){
                            dist[c_dist] = sqrt(pow(((drone[z].x+CS/2) - drone[j].x),2)+pow(((drone[z].y+CS/2) - drone[j].y),2)+pow((drone[z].z - drone[j].z),2));
                            sum_d = sum_d + dist[c_dist];
                            c_dist++;
                        }
                    }    
                    mean_d  = sum_d / (NU-1);
                    max_var = mean_d*mean_d*(NU-1);
                    for(j = 0; j < NU-1; j++){
                        var_d = var_d + pow((dist[j] - mean_d),2) / (NU-1);
                    }
                    var_d = sqrt(var_d);            
                    if(neuron_map[yd][xd].neighbors_cost[4] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr > cost_max && flag_coll[4]==0){
                        for(i=0; i < R; i++)
                            for(j=0; j < C; j++)
                                traj_check[i][j] = 0;
                        for(i=yd; i <= ny-1; i++){
                            for(j=xd; j <= nx-1; j++){
                                if(traj[i][j] == 1){
                                    flag_coll[4] = 1;
                                }
                                else
                                    traj_check[i][j] = 1;
                            }
                        }   
                        if(flag_coll[4] == 0){
                            cost_max      = neuron_map[yd][xd].neighbors_cost[4] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr;
                            id_chosen[z]  = neuron_map[int(drone[z].y+CS/2)][int(drone[z].x + CS/2)].id;
                            dir_chosen[z] = 4;
                        }
                    }    
                }
            }    
    
            // 7th neighbor, 2nd half of the 6th neighbor and 2nd half of the 8th:
            if(neuron_map[yd][xd].neighbors_cost[6] != -3){
        
                // Calculate lines coefficients:
                nx        = xd - int(CS/2+1);
                ny        = yd + int(CS/2+1);
                m         = (ny - yd)/(nx - xd);
                q         = yd - xd * (ny - yd)/(nx - xd);
        
                for(i=yd; i <= ny; i++){
                    for(j=nx; j <= xd; j++){
                        if(cost_map[i][j] == -1 && i >= 0 && i < R && j >= 0 && j < C){
                            d[6] = abs((i - m * j - q) / sqrt(1 + m * m));
                            if(d[6] <= dlim)
                                flag_coll[6] = 1;
                            // 2nd half of the 6th neighbor:   
                            d[5] = abs(j - xd);
                            if(d[5] <= dlim)
                                flag_coll[5] = 1;    
                            // 2nd half of the 8th neighbor:   
                            d[7] = abs(i - yd);
                            if(d[7] <= dlim)
                                flag_coll[7] = 1;                            
                        }       
                    }
                }
                if(flag_coll[6] == 0 && id_previous[z] != neuron_map[int(drone[z].y+CS/2)][int(drone[z].x - CS/2)].id){
                    for(i=0; i < NU; i++){
                        if(i!=z && neuron_map[int(drone[z].y+CS/2)][int(drone[z].x - CS/2)].id == id_chosen[i])
                            flag_coll[6] = 1;
                    }              
                    c_dist = 0;
                    sum_d  = 0;
                    var_d  = 0;
                    for(j=0; j < NU; j++){
                        if(z!=j){
                            dist[c_dist] = sqrt(pow(((drone[z].x-CS/2) - drone[j].x),2)+pow(((drone[z].y+CS/2) - drone[j].y),2)+pow((drone[z].z - drone[j].z),2));
                            sum_d = sum_d + dist[c_dist];
                            c_dist++;
                        }
                    }    
                    mean_d  = sum_d / (NU-1);
                    max_var = mean_d*mean_d*(NU-1);
                    for(j = 0; j < NU-1; j++){
                        var_d = var_d + pow((dist[j] - mean_d),2) / (NU-1);
                    }
                    var_d = sqrt(var_d);            
                    if(neuron_map[yd][xd].neighbors_cost[6] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr > cost_max && flag_coll[6]==0){
                        for(i=0; i < R; i++)
                            for(j=0; j < C; j++)
                                traj_check[i][j] = 0;
                        for(i=yd; i <= ny-1; i++){
                            for(j=nx; j <= xd-1; j++){
                                if(traj[i][j] == 1){
                                    flag_coll[6] = 1;
                                }
                                else
                                    traj_check[i][j] = 1;
                            }
                        }           
                        if(flag_coll[6] == 0){
                            cost_max      = neuron_map[yd][xd].neighbors_cost[6] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr;
                            id_chosen[z]  = neuron_map[int(drone[z].y+CS/2)][int(drone[z].x - CS/2)].id;
                            dir_chosen[z] = 6;
                        }
                    }    
                }
                if(flag_coll[5] == 0 && id_previous[z] != neuron_map[int(drone[z].y+CS/2)][int(drone[z].x)].id){
                    for(i=0; i < NU; i++){
                        if(i!=z && neuron_map[int(drone[z].y+CS/2)][int(drone[z].x)].id == id_chosen[i])
                            flag_coll[5] = 1;
                    }                
                    c_dist = 0;
                    sum_d  = 0;
                    var_d  = 0;
                    for(j=0; j < NU; j++){
                        if(z!=j){
                            dist[c_dist] = sqrt(pow(((drone[z].x) - drone[j].x),2)+pow(((drone[z].y+CS/2) - drone[j].y),2)+pow((drone[z].z - drone[j].z),2));
                            sum_d = sum_d + dist[c_dist];
                            c_dist++;
                        }
                    }    
                    mean_d  = sum_d / (NU-1);
                    max_var = mean_d*mean_d*(NU-1);
                    for(j = 0; j < NU-1; j++){
                        var_d = var_d + pow((dist[j] - mean_d),2) / (NU-1);
                    }
                    var_d = sqrt(var_d);            
                    if(neuron_map[yd][xd].neighbors_cost[5] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr > cost_max && flag_coll[5]==0){
                        for(i=0; i < R; i++)
                            for(j=0; j < C; j++)
                                traj_check[i][j] = 0;
                        for(i=yd; i <= ny-1; i++){
                            for(j=nx; j <= xd-1; j++){
                                if(traj[i][j] == 1){
                                    flag_coll[5] = 1;
                                }
                                else
                                    traj_check[i][j] = 1;
                            }
                        } 
                        if(flag_coll[5] == 0){
                            cost_max      = neuron_map[yd][xd].neighbors_cost[5] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr;
                            id_chosen[z]  = neuron_map[int(drone[z].y+CS/2)][int(drone[z].x)].id;
                            dir_chosen[z] = 5;
                        }
                    }  
                }    
                if(flag_coll[7] == 0 && neuron_map[int(drone[z].y)][int(drone[z].x-CS/2)].id != id_previous[z]){
                    for(i=0; i < NU; i++){
                        if(i!=z && neuron_map[int(drone[z].y)][int(drone[z].x-CS/2)].id == id_chosen[i])
                            flag_coll[7] = 1;
                    }             
                    c_dist = 0;
                    sum_d  = 0;
                    var_d  = 0;
                    for(j=0; j < NU; j++){
                        if(z!=j){
                            dist[c_dist] = sqrt(pow(((drone[z].x-CS/2) - drone[j].x),2)+pow(((drone[z].y) - drone[j].y),2)+pow((drone[z].z - drone[j].z),2));
                            sum_d = sum_d + dist[c_dist];
                            c_dist++;
                        }
                    }    
                    mean_d  = sum_d / (NU-1);
                    max_var = mean_d*mean_d*(NU-1);
                    for(j = 0; j < NU-1; j++){
                        var_d = var_d + pow((dist[j] - mean_d),2) / (NU-1);
                    }
                    var_d = sqrt(var_d);             
                    if(neuron_map[yd][xd].neighbors_cost[7] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr > cost_max && flag_coll[7]==0){
                        for(i=0; i < R; i++)
                            for(j=0; j < C; j++)
                                traj_check[i][j] = 0;
                        for(i=yd; i <= ny-1; i++){
                            for(j=nx; j <= xd-1; j++){
                                if(traj[i][j] == 1){
                                    flag_coll[7] = 1;
                                }
                                else
                                    traj_check[i][j] = 1;
                            }
                        }               
                        if(flag_coll[7] == 0){
                            cost_max      = neuron_map[yd][xd].neighbors_cost[7] + mean_d/max_mean * w_md + sqrt(var_d/ max_var) * w_vr;
                            id_chosen[z]  = neuron_map[int(drone[z].y)][int(drone[z].x-CS/2)].id;
                            dir_chosen[z] = 7;
                        }
                    }    
                }
            }    
            
            /*
            if(dir_chosen[z] == 8){
                for(i=0; i < R; i++)
                    for(j=0; j < C; j++)
                        traj_check[i][j] = 0;
                for(i=(yd - int(CS/2)); i <= (yd + int(CS/2)); i++){
                    for(j=(xd - int(CS/2)); j <= (xd + int(CS/2)); j++){
                        if(i >= 0 && i < R && j >= 0 && j < C)
                            traj_check[i][j] = 1;
                    }
                }                   
            }
            */
            
            id_previous[z] = neuron_map[yd][xd].id;
            
            // Update the trajectories control matrices:
            for(i=0; i < R; i++)
                for(j=0; j < C; j++)
                    traj[i][j] = traj[i][j] + traj_check[i][j];
            
            // Convert id_chosen into drones coordinates:
            if(dir_chosen[z] == 0){
                drone[z].x  = drone[z].x - int(CS/2);
                drone[z].y  = drone[z].y - int(CS/2);
                drone[z].z  = 0;
            }    
            if(dir_chosen[z] == 1){
                drone[z].x  = drone[z].x;
                drone[z].y  = drone[z].y - int(CS/2);
                drone[z].z  = 0;
            }     
            if(dir_chosen[z] == 2){
                drone[z].x  = drone[z].x + int(CS/2);
                drone[z].y  = drone[z].y - int(CS/2);
                drone[z].z  = 0;
            } 
            if(dir_chosen[z] == 3){
                drone[z].x  = drone[z].x + int(CS/2);
                drone[z].y  = drone[z].y;
                drone[z].z  = 0;
            } 
            if(dir_chosen[z] == 4){
                drone[z].x  = drone[z].x + int(CS/2);
                drone[z].y  = drone[z].y + int(CS/2);
                drone[z].z  = 0;
            } 
            if(dir_chosen[z] == 5){
                drone[z].x  = drone[z].x;
                drone[z].y  = drone[z].y + int(CS/2);
                drone[z].z  = 0;
            } 
            if(dir_chosen[z] == 6){
                drone[z].x  = drone[z].x - int(CS/2);
                drone[z].y  = drone[z].y + int(CS/2);
                drone[z].z  = 0;
            } 
            if(dir_chosen[z] == 7){
                drone[z].x  = drone[z].x - int(CS/2);
                drone[z].y  = drone[z].y;
                drone[z].z  = 0;
            }  
            if(dir_chosen[z] == 8){
                drone[z].x  = drone[z].x;
                drone[z].y  = drone[z].y;
                drone[z].z  = 0;
            }  
        }
        
        // Print drones positions:
        for(h=0; h < NU; h++){    
            printf("%f  %f  %f ", drone[h].x, drone[h].y, drone[h].z);    
           }
        printf("\n");   
        
        // Update the moves counter:
        move_count++;   
        
        /*
        if(move_count == 249){
        // Print matrix:
            fp2=fopen("output.txt","w");
            for(i=0; i < R; i++){
                for(j=0; j < C; j++){
                    fprintf(fp2, "%f ", cost_map[i][j]);
                }
               fprintf(fp2, "\n"); 
            }
            fclose(fp2);
        }
        */
        
        // Check max_point_dist && Initialize the cost map and fill the obstacle coordinates with a -1 
        // && Count penalized points, in order to calculate the coverage points density:
        for(i=0; i < R; i ++){
            for(j=0; j < C; j++){
                if(cost_map[i][j] != -1){
                    min_mpd = sqrt(R*R + C*C);
                    for(z=0; z < NU; z++){
                        dm = sqrt(pow((drone[z].x-j),2)+pow((drone[z].y - i),2));
                        if(dm < min_mpd)
                            min_mpd = dm;
                    }
                    if(min_mpd > max_point_dist)
                        max_point_dist = min_mpd;
                    cost_map[i][j] = 0;  
                }
                if(neuron_map[i][j].penalty == 1)
                    cp++;
                if(tot_p[i][j] == 1)
                    count_tot ++;                
            }
        } 

        // Reinitialize the penalty matrix when a percentage of the total point is covered:
        perc_cov = float(cp)/float(R*C-n_o);
        if(perc_cov > perc_lim){            
            for (i=0; i < R; i++){
                for(j=0; j < C; j++){
                    neuron_map[i][j].penalty = 0;
                }
            }
            all_covered = 1;
            move_all    = move_count;
            count_all ++;
        }
        cp = 0;        
    }
    
    if(all_covered == 1)
        printf("last: %d\n", move_all);
    printf("tot: %d\n", count_all);
    printf("max_point_dist: %f\n", max_point_dist);
    return 0;
}

