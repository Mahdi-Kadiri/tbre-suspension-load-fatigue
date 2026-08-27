%% RUNALL_V2 - SUSPENSION LOAD & FATIGUE TOOLCHAIN
% Press Run. Set DATA_FILE below to a logged CSV, or leave it empty ('')
% to use the built-in synthetic telemetry.
%
% Needs in the same folder:
%   FatiguePipeline_V2.m   vd_load_spectrum.m   rainflow_matrix.m
%
% Channel axes are resolved from physics (V*yawrate and dV/dt) rather than
% from the header names, which do not match the axes they suggest and change
% between events. This runs silently.

clear functions; clear; clc; close all
rng(0);   % reproducible run-to-run results

% PARAMETER SET. The pipeline defaults are TBRe26. FSG25/FSUK25 telemetry
% was logged on TBRe25, which has DIFFERENT TUBE SECTIONS (12.7/15.875 mm
% OD x 1.2 mm wall vs 15/18 x 1.6). Stress is ~1.5x higher on the real
% sections, which is ~2 orders of magnitude in damage. The defaults below
% are set for TBRe25, which is the car the FSG25/FSUK25 logs came from.
%   VEH  = []                    -> pipeline defaults (TBRe26)
%   VEH  = tbre25_params_V2()    -> TBRe25 mass, tubes, aero
%   GEOM = []                    -> placeholder geometry
%   GEOM = tbre25_geometry()     -> TBRe25 hardpoints, IPG set (default)
%   GEOM = tbre25_geometry('datasheet') -> the alternative set, for comparison
%
% Running TBRe25 telemetry with TBRe26 geometry or tube sections is the
% single easiest way to get a wrong answer that looks entirely plausible.
VEH  = tbre25_params_V2();
GEOM = tbre25_geometry();

DATA_FILE = 'auto';  % 'auto' = find the telemetry file in this folder by
                     % itself. Put the CSV/XLSX next to these scripts and
                     % press Run - nothing to type. Set a filename here to
                     % force a specific file, or '' for synthetic telemetry.
SPEED_MIN = 3;      % m/s - drop stationary samples
GATE_G    = 0.30;   % g - acceleration cycle gate. Set from the MEASURED
                    % sensor noise floor (3x the sample-to-sample std of the
                    % accelerometer channels), not chosen by eye. Below this
                    % the gate sits inside the noise and counts sensor jitter
                    % as load cycles. Section 2 prints the sensitivity.
GATE_N    = 0;      % N - member force gate. 0 = derive ONE GLOBAL gate at
                    % 3x the mean member-force noise.
                    % A common gate is required for the counts to be
                    % comparable across members. Per-member gates were tried
                    % and rejected: scaling each gate to its own member's
                    % noise flattens every member to ~30 cycles/km and
                    % destroys the real ranking (pushrods take far more
                    % reversals than tie rods).

%% ---------------- find the data file ----------------
if strcmpi(DATA_FILE,'auto')
    cand = [dir(fullfile(fileparts(mfilename('fullpath')),'*.csv'));
            dir(fullfile(fileparts(mfilename('fullpath')),'*.xlsx'))];
    DATA_FILE = '';
    for q = 1:numel(cand)
        f = fullfile(cand(q).folder, cand(q).name);
        try
            hdr = readtable(f,'ReadVariableNames',true);      % header only check
            vnq = lower(string(hdr.Properties.VariableNames));
            if any(contains(vnq,'speed')) && any(contains(vnq,'time'))
                DATA_FILE = f;
                fprintf('Auto-detected telemetry: %s\n', cand(q).name);
                break
            end
        catch
            % not a readable table - skip
        end
    end
    if isempty(DATA_FILE)
        fprintf(['No telemetry file found in this folder ' ...
                 '(need Time and speed columns). Using synthetic data.\n']);
    end
end

%% ---------------- load + verify axes ----------------
if isempty(DATA_FILE)
    tel = [];
    fprintf('Using built-in synthetic telemetry.\n');
else
    T  = readtable(DATA_FILE);
    vn = string(T.Properties.VariableNames);
    col = @(nm) T{:, find(strcmpi(vn,nm),1)};

    tRaw = col('Time');  spd = col('speed');  yaw = col('YawRate');
    if isempty(spd), error('No ''speed'' channel found in %s', DATA_FILE); end

    mv = spd > SPEED_MIN;                          % moving only
    t  = tRaw(mv);  v = spd(mv);  r = deg2rad(yaw(mv));
    aLat  = v.*r/9.81;                              % kinematic lateral, g
    aLong = movmean(gradient(v,t)/9.81, 21);        % smoothed dV/dt, g

    cand = vn(contains(vn,["Acc","GXg","GYg","GZg"],'IgnoreCase',true));
    cand = cand(~contains(cand,["Accy","Accuracy"],'IgnoreCase',true));

    % Score every candidate, then choose. Where several channels clearly
    % measure the same axis (redundant IMUs), correlation alone is a coin
    % toss - a 0.006 difference in r is meaningless. Pick the QUIETEST valid
    % channel instead, because cycle counting is sensitive to noise near the
    % gate: on this log the two lateral channels differ by 37% in cycle count.
    nm_={}; rl_=[]; rg_=[]; mu_=[]; sd_=[]; nz_=[];
    for q = 1:numel(cand)
        x = T{mv, find(strcmpi(vn,cand(q)),1)};
        if ~isnumeric(x) || all(isnan(x)) || std(x,'omitnan')==0, continue; end
        nm_{end+1}=cand(q); %#ok<AGROW>
        rl_(end+1)=corr_(x,aLat); rg_(end+1)=corr_(x,aLong); %#ok<AGROW>
        mu_(end+1)=mean(x,'omitnan'); sd_(end+1)=std(x,'omitnan'); %#ok<AGROW>
        nz_(end+1)=std(diff(x),'omitnan'); %#ok<AGROW>
    end
    bLat = pickAxis(nm_, abs(rl_), rl_, nz_, 0.90);
    bLon = pickAxis(nm_, abs(rg_), rg_, nz_, 0.60);
    okV  = sd_<0.5 & abs(abs(mu_)-1)<0.2;
    if any(okV), iv=find(okV); [~,k]=min(nz_(iv)); bVer={nm_{iv(k)}, mu_(iv(k))};
    else, [~,k]=min(abs(abs(mu_)-1)); bVer={nm_{k}, mu_(k)}; end

    pull = @(nm) T{mv, find(strcmpi(vn,nm),1)};
    tel.Gy = sign(bLat{2}) * pull(bLat{1});          % lateral
    tel.Gx = sign(bLon{2}) * pull(bLon{1});          % longitudinal
    tel.Gz = pull(bVer{1});
    if mean(tel.Gz,'omitnan') > 0, tel.Gz = -tel.Gz; end   % static should be -1 g
    tel.fs = 1/median(diff(t));
    tel.v  = v;      % speed [m/s] -> enables speed-dependent aero

    if any(strcmpi(vn,'sRun'))
        s = pull('sRun'); tel.dist_km = (max(s)-min(s))/1000;
    else
        tel.dist_km = NaN;
    end
    fprintf('Loaded %s\n', DATA_FILE);
    fprintf('  %d samples @ %.1f Hz | %.1f min moving | %.1f km\n', ...
            numel(tel.Gx), tel.fs, (t(end)-t(1))/60, tel.dist_km);
    fprintf('  Gx %+.2f..%+.2f | Gy %+.2f..%+.2f | Gz %+.2f..%+.2f  [g]\n', ...
            min(tel.Gx),max(tel.Gx),min(tel.Gy),max(tel.Gy),min(tel.Gz),max(tel.Gz));
end

%% ---------------- 1. fatigue pipeline ----------------
fprintf('\n\n==================== 1. FATIGUE PIPELINE ====================\n');
out = FatiguePipeline_V2(GEOM, tel, VEH);
if isempty(tel), tel = out.tel; tel.dist_km = NaN; end

%% ---------------- 2. load spectrum ----------------
fprintf('\n\n==================== 2. LOAD SPECTRUM =======================\n');
% One global force gate at 3x the mean member-force noise.
if GATE_N <= 0
    nzF = [];
    for cc = {'FL','FR','RL','RR'}
        for ii = 1:6
            nzF(end+1) = std(diff(out.Fmember.(cc{1})(:,ii)),'omitnan'); %#ok<AGROW>
        end
    end
    GATE_N = 3*mean(nzF);
    fprintf('Global force gate: 3 x mean member-force noise (%.0f N) = %.0f N\n\n', ...
            mean(nzF), GATE_N);
end
S = vd_load_spectrum(out.Fmember, tel, struct('gate_g',GATE_G,'gate_N',GATE_N));

% ---- gate sensitivity: the cycle count is NOT a single number ----
fprintf('\n--- GATE SENSITIVITY (cycle count vs noise gate) ---\n');
nzG = [std(diff(tel.Gx),'omitnan') std(diff(tel.Gy),'omitnan') std(diff(tel.Gz),'omitnan')];
fprintf('sample-to-sample noise: Gx %.3f  Gy %.3f  Gz %.3f g\n', nzG);
fprintf('suggested gate (3x noise): %.2f g\n\n', 3*mean(nzG));
gates = [0.10 0.15 0.25 0.40 3*mean(nzG)];
fprintf('%-12s', 'gate [g]'); fprintf('%9.2f', gates); fprintf('\n');
axn = {'Long','Lat','Vert'}; axd = {tel.Gx, tel.Gy, tel.Gz};
for a = 1:3
    fprintf('%-12s', axn{a});
    for gq = gates, fprintf('%9.0f', countCycles(axd{a}, gq)); end
    fprintf('\n');
end
fprintf('\nReport cycle counts WITH the gate. Counts vary several-fold across\n');
fprintf('this range; mean loads and amplitudes are far more robust.\n');
if isfield(tel,'dist_km') && ~isnan(tel.dist_km)
    fprintf('\nCycles per km (scale to any distance):\n');
    fprintf('%-7s %-10s %10s %12s\n','Corner','Member','cycles','cycles/km');
    for j = 1:numel(S)
        fprintf('%-7s %-10s %10.0f %12.1f\n', S(j).Corner, S(j).Member, ...
                S(j).Cycles, S(j).Cycles/tel.dist_km);
    end
end

%% ---------------- 3. rainflow matrices ----------------
fprintf('\n\n==================== 3. RAINFLOW MATRICES ===================\n');
corners = {'FL','FR','RL','RR'};
labels  = {'Up-Fore','Up-Aft','Low-Fore','Low-Aft','Push/Pull','Tie/Toe'};
[~,ord] = sort([out.res.Dmg],'descend');
shown = {}; nShown = 0;
for q = 1:numel(ord)
    rr = out.res(ord(q));
    if any(strcmp(shown, rr.Member)), continue; end
    c = find(strcmp(corners, rr.Corner)); i = find(strcmp(labels, rr.Member));
    sig = -out.Fmember.(corners{c})(:,i)/out.P.A(i)/1e6;
    rainflow_matrix(sig, [], struct('name',sprintf('%s %s',rr.Corner,rr.Member)));
    shown{end+1} = rr.Member; nShown = nShown+1; %#ok<AGROW>
    if nShown>=3, break; end
end

%% ---------------- 4. load transfer comparison ----------------
hasLLTD = ~isempty(VEH) && isfield(VEH,'LT_f_perg') && all(isfinite([VEH.LT_f_perg VEH.LT_r_perg]));
if isempty(VEH) || hasLLTD
    fprintf('\n\n============ 4. LOAD TRANSFER: 50/50 vs TEAM LLTD ===========\n');
    oA = FatiguePipeline_V2(GEOM, tel, setopt(VEH,'useRollStiffness',false));
    oB = FatiguePipeline_V2(GEOM, tel, setopt(VEH,'useRollStiffness',true));
    fprintf('%-7s %-10s %12s %12s %9s\n','Corner','Member','50/50 peak','LLTD peak','change');
    for c = 1:4
        for i = 1:6
            a = max(abs(oA.Fmember.(corners{c})(:,i)));
            b = max(abs(oB.Fmember.(corners{c})(:,i)));
            fprintf('%-7s %-10s %12.0f %12.0f %8.1f%%\n', corners{c}, labels{i}, a, b, 100*(b/a-1));
        end
    end
else
    fprintf('\n\n============ 4. LOAD TRANSFER COMPARISON: SKIPPED ============\n');
    fprintf('No LLTD coefficients for this car.\n\n');
    fprintf('The TBRe26 values (276.9 / 323.7 N per g) do NOT transfer to TBRe25:\n');
    fprintf('  - TBRe25 ran NO FRONT ARB, so the roll-stiffness split is different\n');
    fprintf('  - motion ratios differ (0.90/0.69 vs 0.929/0.823)\n');
    fprintf('Applying them anyway would be a silent error. Recompute LLTD for\n');
    fprintf('TBRe25 and set LT_f_perg / LT_r_perg in tbre25_params_V2 to enable.\n');
    fprintf('\nPeak member loads from the run above (track-width split):\n');
    fprintf('%-7s %-10s %12s\n','Corner','Member','peak [N]');
    for c = 1:4
        for i = 1:6
            fprintf('%-7s %-10s %12.0f\n', corners{c}, labels{i}, ...
                    max(abs(out.Fmember.(corners{c})(:,i))));
        end
    end
end

fprintf('\n\nDONE.\n');
fprintf('Reminders:\n');
fprintf('  - Vertical content is BANDWIDTH-limited, not calibration-limited. The logger\n');
fprintf('    samples at 20 Hz (Nyquist 10 Hz) while wheel hop was rig-MEASURED at\n');
fprintf('    20-27 Hz, so\n');
fprintf('    kerb-strike content is ALIASED and unrecoverable by any post-processing\n');
fprintf('    route. Damper-derived Fz improves ride/roll-frequency load estimation but\n');
fprintf('    does not recover it. Fix is a faster logging rate on the damper and\n');
fprintf('    accelerometer channels.\n');
fprintf('  - Basquin/Miner/Goodman are METALS methods - not valid for the CFRP suspension.\n');
fprintf('  - Life figures >1e9 blocks are reported as negligible, not as predictions.\n');
fprintf('  - Members whose average amplitude is near the gate (Up-Fore, Tie/Toe) have\n');
fprintf('    UNSTABLE cycle counts - small changes flip cycles in or out. Their counts\n');
fprintf('    are indicative only; their mean loads and amplitudes remain reliable.\n');

%% ---------------- helpers ----------------
function o = setopt(base, field, val)
% merge an override onto the chosen parameter set, keeping it quiet
    if isempty(base), o = struct(); else, o = base; end
    o.(field) = val;  o.quiet = true;
end

function b = pickAxis(nm, score, signed, noise, thresh)
% Among channels that clearly measure this axis, take the quietest.
    ok = score > thresh;
    if ~any(ok), [~,k] = max(score); b = {nm{k}, signed(k)}; return; end
    idx = find(ok); [~,k] = min(noise(idx)); k = idx(k);
    b = {nm{k}, signed(k)};
end

function n = countCycles(x, gate)
    x = x(:); tp = x(1);
    for q = 2:numel(x)
        v = x(q);
        if abs(v-tp(end)) >= gate
            if numel(tp)>=2 && sign(v-tp(end))==sign(tp(end)-tp(end-1)), tp(end)=v;
            else, tp(end+1,1)=v; end %#ok<AGROW>
        end
    end
    n = (numel(tp)-1)/2;
end

function r = corr_(a,b)
    a = a(:); b = b(:); k = ~isnan(a) & ~isnan(b);
    if nnz(k) < 10, r = 0; return; end
    a = a(k) - mean(a(k));  b = b(k) - mean(b(k));
    den = norm(a)*norm(b);
    if den == 0, r = 0; else, r = (a.'*b)/den; end
end