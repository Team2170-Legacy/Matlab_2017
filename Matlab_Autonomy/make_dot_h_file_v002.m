function status = make_dot_h_file_v002(start_pos,end_pos,all_omega_R,all_omega_L,all_t,Robot,t_auto_end, i_auto_end)

%Wheel position [rotations], velocity [rotations/s],	Ts = 10 [ms];

N =length(all_omega_R);

rotation_per_sec_R  = all_omega_R/2/pi;
rotation_per_sec_L  = all_omega_L/2/pi;
rotation_R = zeros(size(all_omega_R));
rotation_L = zeros(size(all_omega_L));
rotation_R(1) = 0;
for i=2:N
    rotation_R(i) = rotation_R(i-1)+ all_omega_R(i-1)*Robot.Ts;
    rotation_L(i) = rotation_L(i-1)+ all_omega_L(i-1)*Robot.Ts;
    
end

traj_dir    = 'Trajectories_dot_h\';

t_final     = max(all_t);
i_final     = length(all_t);

traj_name = [ start_pos '_to_' end_pos ];

dot_h_file_name = [ traj_dir traj_name '.h' ];
fh = fopen( dot_h_file_name ,'w');

formatSpec = [ 'const double t_final_' traj_name ' = %10.2f;\n' ];
fprintf(fh,formatSpec,t_final);

formatSpec = [ 'const int i_final_' traj_name ' = %10.0f;\n' ];
fprintf(fh,formatSpec,i_final);

formatSpec = [ 'const double t_auto_end_' traj_name ' = %10.2f;\n' ];
fprintf(fh,formatSpec,t_auto_end);

formatSpec = [ 'const int i_auto_end_' traj_name ' = %10.0f;\n' ];
fprintf(fh,formatSpec,i_auto_end);

%   Right motor code
formatSpec = [ 'const ProfileData AutoMove_' traj_name '_R = {{\n' ];
fprintf(fh,formatSpec);
formatSpec = '{ %13.6f , %13.6f , %13.6f },\n';

for i=1:N
    if i == N
        formatSpec = '{ %13.6f , %13.6f , %13.6f }\n'  ;
    end
    fprintf( fh, formatSpec, [rotation_R(i), rotation_per_sec_R(i), Robot.Ts*1000]);
end

formatSpec = '}};\n';
fprintf(fh,formatSpec);

%   Left motor code

formatSpec = [ 'const ProfileData AutoMove_' traj_name '_L = {{\n'];
fprintf(fh,formatSpec);
formatSpec = '{ %13.6f , %13.6f , %13.6f },\n';

for i=1:N
    if i == N
        formatSpec = '{ %13.6f , %13.6f , %13.6f }\n'  ;
    end
    fprintf( fh, formatSpec, [rotation_L(i), rotation_per_sec_L(i), Robot.Ts*1000]);
    
end

formatSpec = '}};\n';
fprintf(fh,formatSpec);

status = ~fclose(fh);

if status,
    disp([ dot_h_file_name '        written successfully...' ]);
else
    disp([ dot_h_file_name '        ERROR!' ]);
end




