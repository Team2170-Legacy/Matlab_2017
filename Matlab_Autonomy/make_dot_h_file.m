function status = make_dot_h_file(all_omega_R,all_omega_L,all_t,Robot,t3_end,i3_end)

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

%const ProfileData AutonomousMove_S1Red_to Boiler_Red {{

fh = fopen('RS3_to_RB.h','w');
formatSpec = 'const double t_end_RS3_to_RB = %10.2f;\n';fprintf(fh,formatSpec,t3_end);
formatSpec = 'const int t_i_RS3_to_RB = %10.0f;\n';
fprintf(fh,formatSpec,i3_end);
formatSpec = 'const ProfileData AutoMove_RS3_to_RB_R = {{\n';
fprintf(fh,formatSpec);
formatSpec = '{ %13.6f , %13.6f , %13.6f },\n';

for i=1:N
    if i ==N
      formatSpec = '{ %13.6f , %13.6f , %13.6f }\n'  ;
    end
   fprintf( fh, formatSpec, [rotation_R(i), rotation_per_sec_R(i), Robot.Ts*1000]);
   %fprintf( fh, formatSpec, [rotation_L, rotation_per_sec_L, Robot.Ts*1000]);
  
    
   

end

formatSpec = '}};\n';
fprintf(fh,formatSpec);
%Left motor code
formatSpec = 'const ProfileData AutoMove_RS3_to_RB_L = {{\n';
fprintf(fh,formatSpec);
formatSpec = '{ %13.6f , %13.6f , %13.6f },\n';

for i=1:N
    if i ==N
      formatSpec = '{ %13.6f , %13.6f , %13.6f }\n'  ;
    end
   fprintf( fh, formatSpec, [rotation_L(i), rotation_per_sec_L(i), Robot.Ts*1000]);

end

formatSpec = '}};\n';
fprintf(fh,formatSpec);







status = 1





