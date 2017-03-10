function status = draw_Field_v002(Field);

% This data structure will draw the robot field and all the objects on it
% Objects:
%   Basic field rectangle
%   Airships
%   Loading Stations
%   Boilers

status = 1;

% f1 = figure;
% hold on
% axis('equal')
% set(f1, 'DefaultLineLineWidth', 3);

text(Field.L/2, Field.W/2, ['t = ' num2str(Field.t) ' s']);
xlabel('x [m]')
ylabel('y [m]')
title_text = title('FRC Team 2170 Robot Simulator');

% Draw field bounding rectangle
% plot([ 0 Field.L Field.L 0 0 ], [ 0 0 Field.W Field.W 0 ], [0.5 0.5 0.5])
hbr = plot([ 0 Field.L Field.L 0 0 ], [ 0 0 Field.W Field.W 0 ], 'm');
set(hbr, 'color', [1 1 1]*0.7);
set(hbr, 'LineWidth', 2);


for a=1:2
    if a == 1
        AirShip = Field.AirShipRed;
        AirShip.color = 'r';
    else
        AirShip = Field.AirShipBlue;
        AirShip.color = 'b';
    end
    
    plot(AirShip.x, AirShip.y, [ AirShip.color 'x' ]);
    
    for i = 1:6
        plot(AirShip.C_x(i), AirShip.C_y(i), [ AirShip.color 'o' ]);
    end
    
    plot([ AirShip.C_x AirShip.C_x(1) ], [ AirShip.C_y AirShip.C_y(1) ], AirShip.color);
end

for bo=1:2
    if bo == 1
        Boiler = Field.BoilerRed;
        Boiler.color = 'r';
    else
        Boiler = Field.BoilerBlue;
        Boiler.color = 'b';
    end
    
    plot([ Boiler.C1_x Boiler.C2_x ], [ Boiler.C1_y Boiler.C2_y ], Boiler.color);
    
    h34 = plot([ Boiler.C3_x Boiler.C4_x ], [ Boiler.C3_y Boiler.C4_y ], 'k');
    set(h34, 'LineWidth', 6);
end

for bl=1:2
    if bl == 1
        BaseLine = Field.BaseLineBlue;
        BaseLine.color = 'b';
    else
        BaseLine = Field.BaseLineRed;
        BaseLine.color = 'r';
    end
    
    BaseLinePlot = plot([ BaseLine.C1_x BaseLine.C2_x ], [ BaseLine.C1_y BaseLine.C2_y ], BaseLine.color);
    set(BaseLinePlot, 'LineStyle', ':');
    
    for p=1:2
        if p == 1
            Peg1 = Field.RP1;
            Peg2 = Field.RP2;
            Peg3 = Field.RP3;
            Peg.color = 'r';
        else
            Peg1 = Field.BP1;
            Peg2 = Field.BP2;
            Peg3 = Field.BP3;
            Peg.color = 'b';
        end
        
        plot([ Peg1.C1_x Peg1.C2_x ], [ Peg1.C1_y Peg1.C2_y ], Peg.color);
        plot([ Peg2.C1_x Peg2.C2_x ], [ Peg2.C1_y Peg2.C2_y ], Peg.color);
        plot([ Peg3.C1_x Peg3.C2_x ], [ Peg3.C1_y Peg3.C2_y ], Peg.color);
        
    end
    
    for h=1:5
        
        %    2*(2*ft + 2.5*in) - (2*ft + 6*in)
        plot([ Field.Hopper(h).x , Field.Hopper(h).x + Field.Hopper(h).W1], [ Field.Hopper(h).y Field.Hopper(h).y ], 'k');
        hh = plot([ Field.Hopper(h).x + Field.Hopper(h).W1, Field.Hopper(h).x + Field.Hopper(h).W1 + Field.Hopper(h).W2], [ Field.Hopper(h).y Field.Hopper(h).y ], 'g');
        set(hh, 'LineWidth', 6);
        plot([ Field.Hopper(h).x + Field.Hopper(h).W1 + Field.Hopper(h).W2, Field.Hopper(h).x + Field.Hopper(h).W1 + Field.Hopper(h).W2 + Field.Hopper(h).W3], [ Field.Hopper(h).y Field.Hopper(h).y ], 'k');
        
        %     if h == 1
        %         Hopper = Field.Hopper1;
        %     else
        %         Hopper = Field.Hopper2;
        %     end
        %
        %     Hopper.color = 'k';
        %
        %     for i=2:3
        %         plot([ Hopper.C_x(i) Hopper.C_x(i+1) ], [ Hopper.C_y(i) Hopper.C_y(i+1) ], Hopper.color)
        %     end
    end
    
end

%       Plot starting positions

hRS1 = plot(Field.RS1.x, Field.RS1.y, 'rx');
hRS2 = plot(Field.RS2.x, Field.RS2.y, 'ro');
hRS3 = plot(Field.RS3.x, Field.RS3.y, 'rd');
W    = 1.5;
set(hRS1,'LineWidth',W);
set(hRS2,'LineWidth',W);
set(hRS3,'LineWidth',W);

hBS1 = plot(Field.BS1.x, Field.BS1.y, 'bx');
hBS2 = plot(Field.BS2.x, Field.BS2.y, 'bo');
hBS3 = plot(Field.BS3.x, Field.BS3.y, 'bd');
set(hBS1,'LineWidth',W);
set(hBS2,'LineWidth',W);
set(hBS3,'LineWidth',W);

    xlabel('x [m]     (Starting Positions: x - cS1, o - cS2, d - cS3)')

% plot([ Boiler2.C1_x Boiler2.C2_x ], [ Boiler2.C1_y Boiler2.C2_y ], Boiler2.color)
% % plot([ 16.7750 15.5709 ], [ 0.6020 0 ], 'b') % With raw numbers instead of variables
%
% h34_2 = plot([ Boiler2.C3_x Boiler2.C4_x ], [ Boiler2.C3_y Boiler2.C4_y ], 'k');
% set(h34_2, 'LineWidth', 6);

