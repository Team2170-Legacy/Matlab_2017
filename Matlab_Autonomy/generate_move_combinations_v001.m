%
%   generate_move_combinations_v001.m
%   Written by Jacob and Martin Krucinski on 3/31/17 at 19:50
%
Combo1 = {'RS3' , 'RP3' , 'RB' , 'F'};
n_combo = length(Combo1);
n_pairs = n_combo - 1;

total_time = 0;
peg_time = 1.0;
boiler_time = 2.0;

temp1 = size(Auto_Table);
n_rows = temp1(1);

for i = 1:n_pairs,
    start_pos   = Combo1{i};
    end_pos     = Combo1{i+1};
    
    for j = 2:n_rows,
        
        if strcmp(start_pos , Auto_Table{j}{1} ) && strcmp(end_pos , Auto_Table{j}{2} ),
            move_time = Auto_Table{j}{3};
            total_time = total_time + move_time;
            
            if regexp(end_pos , '.P.'),
                total_time = total_time + peg_time;
            end
            
            if regexp(end_pos , '.B'),
                total_time = total_time + boiler_time;
            end
            
        end
    end
end

final_result = { Combo1{:} , total_time };
cell2table(final_result)

    