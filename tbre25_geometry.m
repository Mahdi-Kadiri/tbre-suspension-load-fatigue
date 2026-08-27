function GEO = tbre25_geometry(setName)
% TBRE25_GEOMETRY  TBRe25 hardpoints for FatiguePipeline_V2.
% ------------------------------------------------------------------------
% PUBLICATION VERSION - REPRESENTATIVE PLACEHOLDER COORDINATES.
% The team's actual hardpoints are competition-confidential. Offline, the
% real geometry exists as multiple candidate sets whose adjudication is
% itself documented (source conflicts up to 22-47 mm on inboard points, a
% degenerate rear toe-link entry caught by a conditioning check, and a
% steering-sheet cross-check on kingpin inclination). The setName argument
% ('hybrid'|'datasheet'|'ipg') selects between those sets in the private
% copy; here it is accepted for interface compatibility and ignored.

if nargin<1, setName = 'hybrid'; end %#ok<INUSD>

Fin  = [ 505 -260   25;  755 -260    0;  505 -165 -128;
         755 -165 -128;  645 -360   90;  575 -220  -48];
Fout = [ 655 -520   53;  655 -520   53;  645 -555 -120;
         645 -555 -120;  645 -510  -90;  575 -545  -16];
Rin  = [2030 -237    5; 2315 -237    8; 2025 -230 -110;
        2305 -227 -125; 2130 -278 -125; 2340 -232  -95];
Rout = [2195 -523   52; 2195 -523   52; 2160 -540 -113;
        2160 -540 -113; 2175 -497   28; 2245 -580  -75];

GEO.FL.in  = Fin*1e-3;   GEO.FL.out = Fout*1e-3;
GEO.RL.in  = Rin*1e-3;   GEO.RL.out = Rout*1e-3;
GEO.FR.in  = GEO.FL.in;  GEO.FR.in(:,2)  = -GEO.FR.in(:,2);
GEO.FR.out = GEO.FL.out; GEO.FR.out(:,2) = -GEO.FR.out(:,2);
GEO.RR.in  = GEO.RL.in;  GEO.RR.in(:,2)  = -GEO.RR.in(:,2);
GEO.RR.out = GEO.RL.out; GEO.RR.out(:,2) = -GEO.RR.out(:,2);
GEO.source = 'PLACEHOLDER (real TBRe25 sets adjudicated offline)';
end
