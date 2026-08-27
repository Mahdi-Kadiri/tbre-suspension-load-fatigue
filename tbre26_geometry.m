function GEO = tbre26_geometry()
% TBRE26_GEOMETRY  TBRe26 hardpoints for FatiguePipeline_V2.
% ------------------------------------------------------------------------
% PUBLICATION VERSION - REPRESENTATIVE PLACEHOLDER COORDINATES.
% The team's actual hardpoints are competition-confidential and are not
% published. Offline, the real set (points_2_optimumkinematics) reproduces
% the team's published member loads to 0.5% median front / 0.6% rear across
% all ten design load cases; every result quoted in the README was produced
% with the real geometry, supplied to this same code path.
%
% Member order: Up-Fore, Up-Aft, Low-Fore, Low-Aft, Push, Tie.
% Coordinates mm, converted to m. Left side given, right mirrored in y.

Fin  = [ 520 -250   25;  750 -250    0;  500 -170 -120;
         770 -170 -120;  640 -360   50;  530 -220  -40];
Fout = [ 650 -520   55;  650 -520   55;  640 -555 -120;
         640 -555 -120;  640 -520  -85;  575 -545  -15];
Rin  = [2030 -240    5; 2310 -240   10; 2025 -230 -110;
        2305 -225 -120; 2195 -185  250; 2340 -230  -95];
Rout = [2195 -520   55; 2195 -520   55; 2160 -540 -110;
        2160 -540 -110; 2195 -485   75; 2245 -570  -75];

GEO = pack(Fin,Fout,Rin,Rout);
GEO.source = 'PLACEHOLDER (real TBRe26 set validated offline, 0.5% vs published loads)';
end

function GEO = pack(Fin,Fout,Rin,Rout)
GEO.FL.in  = Fin*1e-3;   GEO.FL.out = Fout*1e-3;
GEO.RL.in  = Rin*1e-3;   GEO.RL.out = Rout*1e-3;
GEO.FR.in  = GEO.FL.in;  GEO.FR.in(:,2)  = -GEO.FR.in(:,2);
GEO.FR.out = GEO.FL.out; GEO.FR.out(:,2) = -GEO.FR.out(:,2);
GEO.RR.in  = GEO.RL.in;  GEO.RR.in(:,2)  = -GEO.RR.in(:,2);
GEO.RR.out = GEO.RL.out; GEO.RR.out(:,2) = -GEO.RR.out(:,2);
end
