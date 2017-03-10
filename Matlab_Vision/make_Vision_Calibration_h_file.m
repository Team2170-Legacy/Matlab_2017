function status = make_Vision_Calibration_h_file(Calib)

%const ProfileData AutonomousMove_S1Red_to Boiler_Red {{

fh = fopen('Vision_Calibration.h','w');

names = fieldnames(Calib);
for i = 1:length(names)
    formatSpec = [ '#define ' names{i} ' %12.4f\n' ];
    fprintf(fh,formatSpec, eval([ 'Calib.' names{i} ]));
end

fclose(fh);

status = 1;

