%
%   starting_pos_texts.m
%
%   Add labels for all starting positions

roffset     = 1.7;
tRS1 = text(Field.RS1.x - roffset, Field.RS1.y, 'RS1');
tRS2 = text(Field.RS2.x - roffset, Field.RS2.y, 'RS2');
tRS3 = text(Field.RS3.x - roffset, Field.RS3.y, 'RS3');
% W    = 1.5;
% set(hRS1,'LineWidth',W);
% set(hRS2,'LineWidth',W);
% set(hRS3,'LineWidth',W);

boffset     = 0.5;
tBS1 = text(Field.BS1.x + boffset, Field.BS1.y, 'BS1');
tBS2 = text(Field.BS2.x + boffset, Field.BS2.y, 'BS2');
tBS3 = text(Field.BS3.x + boffset, Field.BS3.y, 'BS3');
% set(hBS1,'LineWidth',W);
% set(hBS2,'LineWidth',W);
% set(hBS3,'LineWidth',W);
