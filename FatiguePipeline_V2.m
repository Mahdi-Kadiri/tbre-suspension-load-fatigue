function out = FatiguePipeline_V2(geomCsv, telemetry, opt)
% FatiguePipeline_V2 - PER-MEMBER FATIGUE PIPELINE
% ------------------------------------------------------------------------
% Wraps the team's validated corner solver (solve_corner physics) as a
% CONSTANT LINEAR MAP per corner, runs a telemetry time-history through the
% load-transfer model, then rainflow (ASTM E1049) -> mean correction ->
% S-N -> Miner per member.
%
% Their physics, my fatigue layer. The ONLY change to their mechanics is an
% optional roll-stiffness lateral-transfer split (Layer 2), off by default
% so this reproduces their behaviour out of the box.
%
% RUNS TODAY: call with no args -> synthetic telemetry, parent-metal S-N.
%   out = FatiguePipeline_V2();
% REAL RUN:
%   tel.Gx/.Gy/.Gz (g, column vectors), tel.fs (Hz)
%   out = FatiguePipeline_V2('SuspensionPoints(forScript).csv', tel, opt);
%
% =====================  PARAMETERS (VALIDATED)  =========================
% VALIDATION STATUS (vs maxloads.xlsx, the team's published member loads):
%   Stage 1 - load transfer : EXACT, 0.00% on all 10 design cases
%   Stage 2 - member forces : 0.23-0.51% on all 6 members at peak load
%   Residual is hardpoint rounding (geometry given to 1 mm).
% The .m solver's own defaults (275 kg, fw 0.45, Tr 1.192) are STALE -- they
% do not reproduce maxloads. Values below are the deck that does.
P.m    = 270;      % kg  car+driver (FYP: 205 car + 70 driver = 275; the
                   %     load deck uses 270 -- +-1.8% driver assumption)
P.fw   = 0.50;     % -   front weight fraction (FYP confirms 50)
P.h    = 0.28;     % m   CoG height
P.Tf   = 1.20;     % m   front track
P.Tr   = 1.20;     % m   rear track
P.L    = 1.53;     % m   wheelbase
P.xIA  = 0.654;    % m
P.zIA  = 0.231;    % m
P.zCOP = 0.28;     % m
P.PL   = 0.50;     % front lift fraction. NOTE: the FYP states aero balance
                   % 55% front -- unresolved with the load deck's 50. <-- ASK
P.ndr  = 2;        % driven wheels
P.g    = 9.81;
P.quiet = false;   % true = suppress the printed tables (used by RUN_ALL)
% Fixed aero for design cases. For telemetry runs with a speed channel,
% prefer speed-dependent aero: Fdrag=0.5*rho*1.5*v^2, Flift=-0.5*rho*4.5*v^2
P.Fdrag = 940.8;   % N
P.Flift = -2700;   % N

% =====================  ROLL-STIFFNESS LAYER (Layer 2)  =================
% Team LLTD calculator (TBRe26 Roll Dynamics.xlsx, 350/900 setup, WITH tyre
% compliance) gives per-axle lateral load transfer directly in N per g:
P.useRollStiffness = false;   % false = team's track-width 50/50 split
P.LT_f_perg = 276.905;  % N/g  front axle lateral LT  (team LLTD, with tyre)
P.LT_r_perg = 323.689;  % N/g  rear  axle lateral LT  (team LLTD, with tyre)
% >>> these are SETUP-DEPENDENT (corner spring 350 lb/in, roll spring 900).
%     Re-read from the LLTD sheet if TBRe26 races a different spring/ARB set. <<<
% Cross-check: track-split gives 314.7/314.7 N/g (LLTD 0.500); team is 0.461.

% =====================  MEMBER SECTION + MATERIAL  =====================
% Per-member OD from leg_creation.xlsx: lower wishbones 18 mm, rest 15 mm.
% (The 12.7 mm in the earlier handover was stale.)  Wall 1.6 mm <-- CONFIRM.
P.wall  = 1.6e-3;                       % m
P.OD    = [15 15 18 18 15 15]*1e-3;     % Up-F,Up-A,Low-F,Low-A,Push,Tie
P.A     = pi/4*(P.OD.^2 - (P.OD-2*P.wall).^2);   % 1x6 m^2

% =====================  S-N CURVE  =====================================
% >>> feature choice driven by Clevis Design Doc / leg creation / Rod End sheet <<<
% 'parent'  : E220 parent tube, Basquin + Marin endurance
% 'weld'    : welded insert/tab, nominal-stress FAT class (BS 7608 / EC3)
SN.feature = 'parent';
SN.sf  = 1.09*310;     % MPa  Basquin coefficient (1.09*UTS)
SN.b   = -0.085;       % Basquin exponent
SN.Se  = 108;          % MPa  Marin endurance (0.7*0.5*UTS)
SN.knee = 'haibach';   % 'haibach' (slope 2k-1 below Se) | 'infinite' (no
                       % damage below Se). Haibach is the honest default for
                       % variable-amplitude racing loads; 'infinite' is only
                       % valid for constant amplitude and reads as zero damage.
SN.FAT = 50;           % MPa  weld detail class at 2e6 (if feature='weld')
SN.mWeld = 3;          % weld slope
SN.Kf  = 1.0;          % fatigue stress-concentration applied to nominal (parent)
% mean-stress: 'goodman' | 'swt' | 'none'
SN.mean = 'goodman';
SN.su  = 310;          % MPa  UTS for Goodman
% Miner design endpoint (motorsport: 0.5 or lower, not 1.0)
SN.Dtarget = 0.5;

% ---- option overrides --------------------------------------------------
if nargin>=3 && isstruct(opt)
    P  = mergestruct(P,  opt);
    % keep section area consistent: if OD or wall was overridden but A was
    % not, recompute A rather than silently using the previous car's areas.
    if (isfield(opt,'OD') || isfield(opt,'wall')) && ~isfield(opt,'A')
        P.A = pi/4*(P.OD.^2 - (P.OD-2*P.wall).^2);
    end
    if isfield(opt,'SN'), SN = mergestruct(SN, opt.SN); end
end

% =====================  GEOMETRY + LINEAR MAPS  ========================
if nargin<1 || isempty(geomCsv)
    GEO = default_geometry();           % placeholder points (see note)
elseif isstruct(geomCsv)
    GEO = geomCsv;                      % geometry struct passed directly
else
    GEO = build_geometry_from_csv(geomCsv);
end
corners = {'FL','FR','RL','RR'};
rCP.FL = [P.xIA,    -P.Tf/2, -P.zIA];
rCP.FR = [P.xIA,     P.Tf/2, -P.zIA];
rCP.RL = [P.xIA+P.L,-P.Tr/2, -P.zIA];
rCP.RR = [P.xIA+P.L, P.Tr/2, -P.zIA];

% ***** THE KEY OPTIMISATION: invert A ONCE per corner, never in the loop *****
% f = M{c} * [Fx;Fy;Fz].  Exact to machine precision vs solve_corner; ~500x
% faster over a telemetry-length history (A is constant: fixed geometry).
M = struct(); Udir = struct();
for c = 1:4
    [M.(corners{c}), Udir.(corners{c})] = corner_map(GEO.(corners{c}).in, ...
                                GEO.(corners{c}).out, rCP.(corners{c}));
end

% =====================  TELEMETRY  ====================================
if nargin<2 || isempty(telemetry)
    telemetry = synth_telemetry();      % runnable demo data
end
Gx = telemetry.Gx(:); Gy = telemetry.Gy(:); Gz = telemetry.Gz(:);
if isfield(telemetry,'v') && ~isempty(telemetry.v)
    vspd = telemetry.v(:);
else
    vspd = [];      % no speed channel -> fixed aero fallback
end
fs = telemetry.fs;  Ns = numel(Gx);
dur = Ns/fs;

% =====================  TIME-HISTORY -> MEMBER FORCES  ================
% Per timestep: nonlinear load distribution (the only per-sample nonlinearity)
% then the cached linear map. Force history is [Ns x 6] per corner.
Fmember = struct();
for c = 1:4, Fmember.(corners{c}) = zeros(Ns,6); end

for k = 1:Ns
    if isempty(vspd), vk = []; else, vk = vspd(k); end
    Fcp = load_transfer(Gx(k),Gy(k),Gz(k), P, vk);
    for c = 1:4
        Fmember.(corners{c})(k,:) = (M.(corners{c}) * Fcp.(corners{c})(:)).';
    end
end
% NOTE: load_transfer is vectorisable for a further ~order-of-magnitude gain;
% kept per-sample here for readability of the scaffold.

% =====================  FATIGUE PER MEMBER  ===========================
labels = {'Up-Fore','Up-Aft','Low-Fore','Low-Aft','Push/Pull','Tie/Toe'};
res = struct('Corner',{},'Member',{},'Dmg',{},'Lives',{},'maxSa',{},'governMean',{});
for c = 1:4
    for i = 1:6
        f = Fmember.(corners{c})(:,i);          % axial force, +comp / -tens (team sign)
        sigma = -f / P.A(i) / 1e6;              % MPa, TENSION POSITIVE (per-member OD)
        [D, info] = member_damage(sigma, SN);
        if D>0, lives = SN.Dtarget/D; else, lives = Inf; end
        res(end+1) = struct( ...                %#ok<AGROW>
            'Corner', corners{c}, 'Member', labels{i}, ...
            'Dmg', D, 'Lives', lives, ...
            'maxSa', info.maxSa, 'governMean', info.governMean);
    end
end

% sort by damage (hotspots first) and report
[~,ix] = sort([res.Dmg],'descend'); res = res(ix);
if ~P.quiet
fprintf('\n=== STEEL MEMBER fatigue (%s tube, OD %.1f/%.1f mm x %.1f wall) ===\n', ...
        'E220', P.OD(1)*1e3, P.OD(3)*1e3, P.wall*1e3);
fprintf('history = %.1f s, %d samples @ %g Hz | feature=%s mean=%s knee=%s Dtarget=%.2f\n\n', ...
        dur,Ns,fs,SN.feature,SN.mean,SN.knee,SN.Dtarget);
fprintf('%-7s %-10s %12s %16s %10s %10s\n','Corner','Member','Damage/hist','Lives-to-Dtgt','maxSa[MPa]','mean[MPa]');
for j = 1:numel(res)
    fprintf('%-7s %-10s %12.3e %16s %10.1f %10.1f\n', res(j).Corner,res(j).Member, ...
            res(j).Dmg, fmt_life(res(j).Lives), res(j).maxSa, res(j).governMean);
end
end

% =====================  7075-T6 CLEVIS FATIGUE (the real target)  ======
% The chassis clevises are Al-7075 (Clevis Design Doc): NO endurance limit,
% notch-sensitive root fillet, peak static 275 MPa at the 3G-bump envelope,
% sized to static SF2 but NOT fatigue-checked. This is the population that
% actually accrues damage. Resolves each link-clevis force history into the
% pull-out direction (their NP) and maps to root stress via an FEA-calibrated
% transfer. >>> REPLACE Kclev with per-clevis unit-load FEA influence; REPLACE
% AL7075 curve with MMPDS / material-software data. <<<
AL.sf=1466; AL.b=-0.143; AL.su=572; AL.Kf=1.6; AL.mean='goodman'; AL.knee='none';
AL.feature='al7075'; AL.Se=0; AL.Dtarget=SN.Dtarget;   % Se=0 -> no endurance cutoff
% FEA calibration: 275 MPa root at envelope resultant |F|=6356 N -> MPa per N
Fenv = 6350;  Kclev = 275/Fenv;   % MPa per N, placeholder scalar
% >>> REPLACE with per-clevis unit-load FEA influence coefficients <<<
clev = [struct('name','UF','idx',1,'NP',[0 1 0]), ...
        struct('name','UA','idx',2,'NP',[0 1 0]), ...
        struct('name','LF','idx',3,'NP',[0 0 -1]), ...   % lower-fore: vertical -> bump driven
        struct('name','LA','idx',4,'NP',[0 0 -1])];
cres = struct('Corner',{},'Clevis',{},'Dmg',{},'Lives',{},'maxSig',{});
for c = 1:4
  for q = 1:numel(clev)
    proj = Udir.(corners{c})(:,clev(q).idx).' * clev(q).NP(:);  % member dir . NP
    Fpo  = Fmember.(corners{c})(:,clev(q).idx) * proj;          % pull-out history (N)
    sig  = AL.Kf * Kclev * Fpo;                                 % notch root stress, MPa
    [D,~]=member_damage(sig,AL);
    if D>0, lv=AL.Dtarget/D; else, lv=Inf; end
    cres(end+1)=struct('Corner',corners{c},'Clevis',clev(q).name, ...
        'Dmg',D,'Lives',lv,'maxSig',max(abs(sig))); %#ok<AGROW>
  end
end
[~,ix]=sort([cres.Dmg],'descend'); cres=cres(ix);
if ~P.quiet
fprintf('\n=== CLEVIS fatigue (Al-7075, no endurance limit) ===\n');
fprintf('%-7s %-7s %12s %16s %12s\n','Corner','Clevis','Damage/hist','Lives-to-Dtgt','maxStress');
for j=1:numel(cres)
  fprintf('%-7s %-7s %12.3e %16s %12.1f\n',cres(j).Corner,cres(j).Clevis, ...
          cres(j).Dmg,fmt_life(cres(j).Lives),cres(j).maxSig);
end
end

out.res = res; out.cres = cres; out.Fmember = Fmember; out.M = M; out.P = P; out.SN = SN;
out.tel = telemetry;
end

% ========================================================================
% CORE: corner solver as a constant 6x3 linear map  (verified exact)
% ========================================================================
function [Mc, U] = corner_map(Pin, Pout, rCPc)
    U = (Pout - Pin).';            % 3x6
    U = U ./ vecnorm(U);
    r = Pin.';                     % 3x6 inboard points
    A = [U; crosscols(r,U)];       % 6x6, force rows + moment-about-patch rows
    kA = cond(A);
    if kA > 250
        warning(['corner_map: cond(A) = %.0f - members nearly parallel or ' ...
                 'sharing points. Total corner load is still reliable; the ' ...
                 'SPLIT between near-parallel members is not.'], kA);
    end
    T = [eye(3); skew(rCPc(:))];   % b = -T*Fcp
    Mc = -(A \ T);                 % 6x3 ; f = Mc*[Fx;Fy;Fz]
end

function C = crosscols(A,B)
    C = [A(2,:).*B(3,:)-A(3,:).*B(2,:);
         A(3,:).*B(1,:)-A(1,:).*B(3,:);
         A(1,:).*B(2,:)-A(2,:).*B(1,:)];
end
function S = skew(r)
    S = [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];
end

% ========================================================================
% LOAD TRANSFER  (replicates solve_all_corners; lateral split swappable)
% ========================================================================
function Fcp = load_transfer(Gx,Gy,Gz, P, v)
    g=P.g; m=P.m; h=P.h; L=P.L; Tf=P.Tf; Tr=P.Tr;
    % Speed-dependent aero when a speed sample and coefficients are given.
    % Aero scales with v^2, so a fixed value is badly wrong away from the
    % speed it was evaluated at.
    if nargin>=5 && ~isempty(v) && isfield(P,'SCz') && ~isempty(P.SCz)
        q = 0.5*P.rho*v^2;
        Fdrag =  P.SCd*q;
        Flift = -P.SCz*q;
    else
        Fdrag = P.Fdrag;  Flift = P.Flift;
    end
    Flong = m*g*Gx;  Flat = m*g*Gy;
    dWx  = (m*Gx*g*h)/(2*L);
    dWxp = (Fdrag*P.zCOP)/(2*L);

    if P.useRollStiffness
        % ----- Layer 2: team LLTD per-axle coefficients (N per g) ---------
        % Direction validated: rear roll stiffness (26863) > front (23561),
        % so LT shifts rearward -> LLTD_front 0.461 < 0.500. Rear members
        % see ~3% more lateral load than the 50/50 model assumes.
        dWy_f = P.LT_f_perg * Gy;
        dWy_r = P.LT_r_perg * Gy;
    else
        % ----- team's track-width split (conserves total, ~50/50) --------
        dWy_f = (m*Gy*g*h)/(2*Tf);
        dWy_r = (m*Gy*g*h)/(2*Tr);
    end

    Fz.FL = -(P.fw*(m*Gz*g))/2 - (P.PL*Flift)/2 + dWx + dWy_f - dWxp;
    Fz.FR = -(P.fw*(m*Gz*g))/2 - (P.PL*Flift)/2 + dWx - dWy_f - dWxp;
    Fz.RL = -((1-P.fw)*(m*Gz*g))/2 - ((1-P.PL)*Flift)/2 - dWx + dWy_r + dWxp;
    Fz.RR = -((1-P.fw)*(m*Gz*g))/2 - ((1-P.PL)*Flift)/2 - dWx - dWy_r + dWxp;

    Sf=Fz.FL+Fz.FR; Sr=Fz.RL+Fz.RR; St=Sf+Sr;
    if (Gx>0) || (P.ndr==4)
        Fx.FL=Flong*(Sf/St)*(Fz.FL/Sf); Fx.FR=Flong*(Sf/St)*(Fz.FR/Sf);
        Fx.RL=Flong*(Sr/St)*(Fz.RL/Sr); Fx.RR=Flong*(Sr/St)*(Fz.RR/Sr);
    else
        Fx.FL=0; Fx.FR=0;
        Fx.RL=Flong*(Fz.RL/Sr); Fx.RR=Flong*(Fz.RR/Sr);
    end
    Fy.FL=Flat*(Sf/St)*(Fz.FL/Sf); Fy.FR=Flat*(Sf/St)*(Fz.FR/Sf);
    Fy.RL=Flat*(Sr/St)*(Fz.RL/Sr); Fy.RR=Flat*(Sr/St)*(Fz.RR/Sr);

    Fcp.FL=[Fx.FL Fy.FL Fz.FL]; Fcp.FR=[Fx.FR Fy.FR Fz.FR];
    Fcp.RL=[Fx.RL Fy.RL Fz.RL]; Fcp.RR=[Fx.RR Fy.RR Fz.RR];
end

% ========================================================================
% FATIGUE: rainflow -> mean correction -> S-N -> Miner   (chain verified)
% ========================================================================
function [D, info] = member_damage(sigma, SN)
    cyc = rainflow_astm(sigma);           % [range mean count]
    D=0; maxSa=0; govMean=NaN; govD=0;
    for j=1:size(cyc,1)
        sa = cyc(j,1)/2; sm = cyc(j,2); n = cyc(j,3);
        sar = mean_correct(sa,sm,SN);
        Nf  = N_life(sar, sa, sm, SN);
        if isfinite(Nf), d = n/Nf; else, d = 0; end
        D = D + d;
        if sa>maxSa, maxSa=sa; end
        if d>govD, govD=d; govMean=sm; end
    end
    info.maxSa=maxSa; info.governMean=govMean;
end

function sar = mean_correct(sa,sm,SN)
    switch lower(SN.mean)
        case 'goodman'
            if sm>0, sar = sa/(1 - sm/SN.su); else, sar = sa; end
        case 'swt'
            smax = sm+sa; if smax>0, sar = sqrt(smax*sa); else, sar = 0; end
        otherwise, sar = sa;
    end
end

function Nf = N_life(sar, sa, sm, SN)
    if strcmpi(SN.feature,'weld')
        % nominal-stress FAT: N = 2e6 * (FAT/range)^m ; range ~ 2*sar (R-effects in class)
        rng = 2*sar;
        if rng<=0, Nf=Inf; return; end
        Nf = 2e6*(SN.FAT/rng)^SN.mWeld;     % no infinite-life cut for welds (CA) by default
        return;
    end
    % parent Basquin: sar = sf*(2N)^b  -> N = 0.5*(sar/sf)^(1/b)
    s = SN.Kf*sar;
    if s<=0, Nf=Inf; return; end
    if s < SN.Se
        if strcmpi(SN.knee,'infinite'), Nf=Inf; return;
        else  % Haibach: below the knee the life-domain slope k = -1/b
              % changes to 2k-1, which EXTENDS life (does not shorten it)
            Nk = 0.5*(SN.Se/SN.sf)^(1/SN.b);
            k  = -1/SN.b;  k2 = 2*k - 1;
            Nf = Nk*(SN.Se/s)^k2;
            return;
        end
    end
    Nf = 0.5*(s/SN.sf)^(1/SN.b);
end

% ---- self-contained ASTM E1049 rainflow (no toolbox dependency) --------
function cyc = rainflow_astm(x)
    x=x(:);
    % turning points
    d=diff(x); d(d==0)=eps; tp=x([true; (d(1:end-1).*d(2:end))<0; true]);
    cyc=zeros(0,3); stk=zeros(0,1);
    for p=tp.'
        stk(end+1,1)=p; %#ok<AGROW>
        while numel(stk)>=3
            X=abs(stk(end)-stk(end-1)); Y=abs(stk(end-1)-stk(end-2));
            if X<Y, break; end
            rng=Y; mn=(stk(end-1)+stk(end-2))/2;
            if numel(stk)==3
                cyc(end+1,:)=[rng mn 0.5]; stk(1)=[]; %#ok<AGROW>
            else
                cyc(end+1,:)=[rng mn 1.0]; stk(end-2:end-1)=[]; %#ok<AGROW>
            end
        end
    end
    for i=1:numel(stk)-1
        cyc(end+1,:)=[abs(stk(i+1)-stk(i)) (stk(i+1)+stk(i))/2 0.5]; %#ok<AGROW>
    end
end

% ========================================================================
% Helpers: geometry loaders + synthetic telemetry + struct merge
% ========================================================================
function GEO = build_geometry_from_csv(csvFile)
    T = readtable(csvFile);
    lab = string(T{:,2}); X=tonum(T{:,3}); Y=tonum(T{:,4}); Z=tonum(T{:,5});
    keep = lab~=""; lab=lab(keep); X=X(keep); Y=Y(keep); Z=Z(keep);
    P=@(n) getp(lab,X,Y,Z,n);
    GEO.FL.in  = [P("FUFL");P("FURL");P("FLFL");P("FLRL");P("FPushInboard");P("FSteerInboard")];
    GEO.FL.out = [P("FUCA");P("FUCA");P("FLCA");P("FLCA");P("FPushOutboard");P("FSteerOutboard")];
    GEO.RL.in  = [P("RUFL");P("RURL");P("RLFL");P("RLRL");P("RPushInboard");P("RToeInboard")];
    GEO.RL.out = [P("RUCA");P("RUCA");P("RLCA");P("RLCA");P("RPushOutboard");P("RToeOutboard")];
    GEO.FR.in=GEO.FL.in;  GEO.FR.in(:,2)=-GEO.FR.in(:,2);
    GEO.FR.out=GEO.FL.out; GEO.FR.out(:,2)=-GEO.FR.out(:,2);
    GEO.RR.in=GEO.RL.in;  GEO.RR.in(:,2)=-GEO.RR.in(:,2);
    GEO.RR.out=GEO.RL.out; GEO.RR.out(:,2)=-GEO.RR.out(:,2);
end
function v=tonum(x)
    if isnumeric(x), v=x; return; end
    s=string(x); s=strrep(s,',','.'); s=regexprep(s,'[^0-9\.\+\-eE]','');
    v=str2double(s);
end
function p=getp(lab,X,Y,Z,n)
    i=find(lab==n,1); if isempty(i), error('missing point %s',n); end
    p=[X(i) Y(i) Z(i)]*1e-3;
end
function GEO = default_geometry()
    % PLACEHOLDER GEOMETRY - representative of a Formula Student double
    % wishbone corner, NOT the team's actual hardpoints. Real geometry is
    % supplied at runtime via the CSV argument and is not published here.
    % Numbers produced with this placeholder set are illustrative of METHOD
    % only; the validation figures quoted in the README were obtained with
    % the team's real (unpublished) hardpoints.
    %
    % Columns: [x y z] in metres, chassis origin at front axle centreline.
    % Row order: Up-Fore, Up-Aft, Low-Fore, Low-Aft, Push/Pull, Tie/Toe.
    GEO.FL.in  = [0.520 -0.250  0.025; 0.750 -0.250  0.000;
                  0.500 -0.170 -0.120; 0.770 -0.170 -0.120;
                  0.640 -0.360  0.050; 0.530 -0.220 -0.040];
    GEO.FL.out = [0.650 -0.520  0.055; 0.650 -0.520  0.055;
                  0.640 -0.555 -0.120; 0.640 -0.555 -0.120;
                  0.640 -0.520 -0.085; 0.575 -0.545 -0.015];
    GEO.RL.in  = [2.030 -0.240  0.005; 2.310 -0.240  0.010;
                  2.025 -0.230 -0.110; 2.305 -0.225 -0.120;
                  2.195 -0.185  0.250; 2.340 -0.230 -0.095];
    GEO.RL.out = [2.195 -0.520  0.055; 2.195 -0.520  0.055;
                  2.160 -0.540 -0.110; 2.160 -0.540 -0.110;
                  2.195 -0.485  0.075; 2.245 -0.570 -0.075];
    GEO.FR.in=GEO.FL.in;   GEO.FR.in(:,2)=-GEO.FR.in(:,2);
    GEO.FR.out=GEO.FL.out; GEO.FR.out(:,2)=-GEO.FR.out(:,2);
    GEO.RR.in=GEO.RL.in;   GEO.RR.in(:,2)=-GEO.RR.in(:,2);
    GEO.RR.out=GEO.RL.out; GEO.RR.out(:,2)=-GEO.RR.out(:,2);
end

function tel = synth_telemetry()
    % crude but structurally realistic: braking zones, mid-corner lat, bumps
    % SEEDED so results are reproducible run to run. Without this the noise
    % changes every call and damage moves ~25% for no physical reason.
    rs = RandStream('mt19937ar','Seed',0);
    fs=100; t=(0:1/fs:60-1/fs).';  n=numel(t);
    lap=mod(t,15);
    Gy = 1.8*sin(2*pi*lap/15) .* (lap>3 & lap<12);          % cornering
    Gx = -1.0*(lap<2) + 1.6*(lap>12 & lap<13.5);            % accel / brake
    Gz = -1 + (-1.5)*max(0,sin(2*pi*8*t)).*(rand(rs,n,1)>0.85);% intermittent bumps
    Gy=Gy+0.05*randn(rs,n,1); Gx=Gx+0.05*randn(rs,n,1);
    tel.Gx=Gx; tel.Gy=Gy; tel.Gz=Gz; tel.fs=fs;
end
function s = fmt_life(L)
% Very large lives are an artefact of the steep S-N slope applied to tiny
% amplitudes, not predictions. But 1e6 was too aggressive a cut: for the
% 7075 clevises (no endurance limit) a few 1e6 blocks is a real result.
% Cut at 1e9, which is beyond any conceivable service life.
    if ~isfinite(L) || L > 1e9
        s = '>1e9 (negligible)';
    else
        s = sprintf('%.3g', L);
    end
end

function a = mergestruct(a,b)
    f=fieldnames(b); for i=1:numel(f), if ~strcmp(f{i},'SN'), a.(f{i})=b.(f{i}); end, end
end
