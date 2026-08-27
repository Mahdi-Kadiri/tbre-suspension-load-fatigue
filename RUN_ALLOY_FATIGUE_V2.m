%% RUN_ALLOY_FATIGUE_V2 - aluminium clevis and wishbone-insert fatigue
% Press Run. Add-on entry point - does not modify the main toolchain.
%
% Needs in the folder: FatiguePipeline_V2.m, tbre26_params_V2.m,
%                      tbre26_geometry.m, alloy_fatigue.m, alloy_lib.m
%
% WHY THIS EXISTS
%   The steel links come out effectively infinite-life. The parts that
%   actually accrue fatigue damage are aluminium: the chassis clevises and
%   the metallic inserts in the wishbones. Aluminium has no endurance
%   limit, so every cycle counts. Neither population was fatigue-checked at
%   design; both were sized to a static safety factor.

clear functions; clear; clc; close all
rng(0);

% ── CONFIGURATION: TBRe26 hardware (the clevises/inserts being designed) ──
% Driven by TBRe25 telemetry as a PROVISIONAL duty cycle, same as
% RUNALL_TBRE26. Telemetry file is auto-detected from this folder.
DATA_FILE = 'auto';

% ---- stress per unit load [MPa/N] at the critical feature ---------------
% BOTH ARE PLACEHOLDERS. Each is derived from the best available source and
% flagged, so the RANKING can be used while the absolute lives cannot.
%
% K_CLEVIS - single-point scaling from the Clevis Design Doc: 275 MPa peak
%   at the 3G-bump envelope, resultant |F| = 6350 N -> 275/6350.
%   Assumes stress scales with load MAGNITUDE regardless of DIRECTION,
%   which is wrong for a clevis. Replace with unit-load FEA influence
%   coefficients (1 N in X, then Y, then Z; peak root stress each time).
%   >>> PENDING: Jonny Masters (corner FEA) / Michael Palmer (clevis).
%
% K_INSERT - computed from the TBRe25 rear rocker insert drawing
%   (TBRe25-CH-MST-P-0101): flange OD 25.00, bore 8.20, fillet R2.00.
%   K = 1/A_net at the waist; waist NOT dimensioned on the drawing
%   ("MANUFACTURE TO CAD DATA"), assumed 14 mm mid-range.
%   Sensitivity: waist 12->18 mm swings A_net 3.3x, and stress with it.
%   THE WAIST DIAMETER IS THE SINGLE MISSING NUMBER for this population.
%   >>> PENDING: waist from CAD; and confirm the wishbone insert resembles
%       this rocker insert (Colin describes failure at a gun-drill
%       termination, which this part does not have).
K_CLEVIS  = 0.04331;   % MPa/N   PLACEHOLDER - see above
K_INSERT  = 0.00989;   % MPa/N   PLACEHOLDER - waist assumed 14 mm
KF_CLEVIS = 1.6;       % notch factor, generic
KF_INSERT = 1.72;      % fillet R2 on 14 mm waist in 25 mm flange
                       % (weakly sensitive: 1.48-1.81 over waist 12-18 mm)

CLEVIS_ALLOY = '7075-T6';
INSERT_ALLOY = '6082-T6';    % CONFIRMED by Huw. Use '6082-T6-WELDED' if the
                             % inserts are welded rather than bonded - that
                             % HALVES the fatigue strength and is the single
                             % largest open question on this population.

%% ---- find telemetry -----------------------------------------------------
if strcmpi(DATA_FILE,'auto')
    cand=[dir(fullfile(fileparts(mfilename('fullpath')),'*.csv'));
          dir(fullfile(fileparts(mfilename('fullpath')),'*.xlsx'))];
    DATA_FILE='';
    for q=1:numel(cand)
        f=fullfile(cand(q).folder,cand(q).name);
        try
            hdr=readtable(f); vnq=lower(string(hdr.Properties.VariableNames));
            if any(contains(vnq,'speed')) && any(contains(vnq,'time'))
                DATA_FILE=f; fprintf('Auto-detected telemetry: %s\n',cand(q).name); break
            end
        catch
        end
    end
    if isempty(DATA_FILE)
        fprintf('No telemetry found - using synthetic data.\n');
    end
end
if isempty(DATA_FILE), tel = []; else, tel = load_csv(DATA_FILE); end

%% ---- run the load pipeline ---------------------------------------------
VEH = tbre26_params_V2();  VEH.quiet = true;
fprintf(['CONFIG: TBRe26 hardware, LLTD ON.\n' ...
         'Duty cycle: TBRe25 telemetry (PROVISIONAL until 26 runs).\n']);
out = FatiguePipeline_V2(tbre26_geometry(), tel, VEH);

corners = {'FL','FR','RL','RR'};
labels  = {'Up-Fore','Up-Aft','Low-Fore','Low-Aft','Push/Pull','Tie/Toe'};

%% ---- 1. chassis clevises (wishbone mounts) -----------------------------
fprintf('\n================= CHASSIS CLEVISES (%s) =================\n', CLEVIS_ALLOY);
matC = alloy_lib(CLEVIS_ALLOY);
resC = struct('part',{},'D',{},'blocks',{},'stress',{});
for c = 1:4
    for i = 1:4                                    % four wishbone links
        F = out.Fmember.(corners{c})(:,i);
        A = alloy_fatigue(F, matC, struct('K',K_CLEVIS,'Kf',KF_CLEVIS, ...
              'name',sprintf('%s %s clevis',corners{c},labels{i}), ...
              'compareCurves', c==1&&i==3));       % show curve comparison once
        resC(end+1) = struct('part',sprintf('%s %s',corners{c},labels{i}), ...
              'D',A.D,'blocks',A.blocks,'stress',A.maxStress); %#ok<AGROW>
    end
end

%% ---- 2. wishbone inserts -----------------------------------------------
fprintf('\n================= WISHBONE INSERTS (%s) =================\n', INSERT_ALLOY);
matI = alloy_lib(INSERT_ALLOY);
resI = struct('part',{},'D',{},'blocks',{},'stress',{});
for c = 1:4
    for i = 1:6                                    % every link has inserts
        F = out.Fmember.(corners{c})(:,i);
        A = alloy_fatigue(F, matI, struct('K',K_INSERT,'Kf',KF_INSERT, ...
              'name',sprintf('%s %s insert',corners{c},labels{i}), ...
              'compareCurves', false));
        resI(end+1) = struct('part',sprintf('%s %s',corners{c},labels{i}), ...
              'D',A.D,'blocks',A.blocks,'stress',A.maxStress); %#ok<AGROW>
    end
end

%% ---- 3. ranked summary -------------------------------------------------
rank_and_print('CLEVISES', resC);
rank_and_print('INSERTS',  resI);

fprintf('\nCAVEATS\n');
fprintf('  - Duty cycle is TBRe25 telemetry: PROVISIONAL for TBRe26 hardware.\n');
fprintf('  - Stress-per-unit-load factors are PLACEHOLDERS:\n');
fprintf('      clevis: single-point scaling ignoring load direction (needs FEA)\n');
fprintf('      insert: 1/A_net from the rocker drawing, waist assumed 14 mm\n');
fprintf('    The RANKING is meaningful; absolute lives are not.\n');
fprintf('  - 6082 published fatigue data spans 2.2x at 1e7; the conservative\n');
fprintf('    anchor is used. If inserts are WELDED, strength roughly halves -\n');
fprintf('    switch INSERT_ALLOY to ''6082-T6-WELDED''.\n');

%% ---- helpers -----------------------------------------------------------
function rank_and_print(hdr, res)
    [~,ix] = sort([res.D],'descend'); res = res(ix);
    fprintf('\n--- %s ranked by damage ---\n', hdr);
    fprintf('%-18s %12s %18s %12s\n','part','damage','blocks to D=0.5','peak[MPa]');
    for j = 1:min(numel(res),10)
        b = res(j).blocks;
        if ~isfinite(b) || b>1e9, bs='>1e9 (negligible)'; else, bs=sprintf('%.3g',b); end
        fprintf('%-18s %12.3e %18s %12.1f\n', res(j).part, res(j).D, bs, res(j).stress);
    end
end

function tel = load_csv(f)
    T=readtable(f); vn=string(T.Properties.VariableNames);
    col=@(n) T{:,find(strcmpi(vn,n),1)};
    spd=col('speed'); mv=spd>3;
    t=col('Time'); t=t(mv); v=spd(mv); r=deg2rad(col('YawRate')); r=r(mv);
    aLat=v.*r/9.81; aLong=movmean(gradient(v,t)/9.81,21);
    cand=vn(contains(vn,["Acc","GXg","GYg","GZg"],'IgnoreCase',true));
    cand=cand(~contains(cand,["Accy","Accuracy"],'IgnoreCase',true));
    nm={};rl=[];rg=[];mu=[];sd=[];nz=[];
    for q=1:numel(cand)
        x=T{mv,find(strcmpi(vn,cand(q)),1)};
        if ~isnumeric(x)||all(isnan(x))||std(x,'omitnan')==0, continue; end
        nm{end+1}=cand(q); %#ok<AGROW>
        a=x-mean(x,'omitnan');
        rl(end+1)=cc(a,aLat); rg(end+1)=cc(a,aLong); %#ok<AGROW>
        mu(end+1)=mean(x,'omitnan'); sd(end+1)=std(x,'omitnan'); nz(end+1)=std(diff(x),'omitnan'); %#ok<AGROW>
    end
    iL=pickq(abs(rl),nz,0.90); iG=pickq(abs(rg),nz,0.60);
    okV=sd<0.5 & abs(abs(mu)-1)<0.2;
    if any(okV), iv=find(okV); [~,k]=min(nz(iv)); iV=iv(k);
    else, [~,iV]=min(abs(abs(mu)-1)); end
    pull=@(n) T{mv,find(strcmpi(vn,n),1)};
    tel.Gy=sign(rl(iL))*pull(nm{iL});
    tel.Gx=sign(rg(iG))*pull(nm{iG});
    tel.Gz=pull(nm{iV}); if mean(tel.Gz,'omitnan')>0, tel.Gz=-tel.Gz; end
    tel.fs=1/median(diff(t));
    tel.v=v;
end

function r=cc(a,b)
    a=a(:); b=b(:); k=~isnan(a)&~isnan(b);
    a=a(k)-mean(a(k)); b=b(k)-mean(b(k));
    d=norm(a)*norm(b); if d==0, r=0; else, r=(a.'*b)/d; end
end

function i=pickq(score,noise,th)
    ok=score>th;
    if ~any(ok), [~,i]=max(score);
    else, idx=find(ok); [~,k]=min(noise(idx)); i=idx(k); end
end
