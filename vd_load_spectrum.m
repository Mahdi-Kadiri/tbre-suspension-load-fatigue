function S = vd_load_spectrum(Fmember, telemetry, opt)
% VD MathMod load spectrum.
% For each wishbone member, from a historic drive, returns:
%   - mean loading  (average axial force, tension +, N)
%   - number of load cycles
%   - average cycle amplitude (N)
% A "cycle" = a change of direction, found from the gradient (turning points).
%
% USAGE:
%   out = FatiguePipeline('SuspensionPoints(forScript).csv', tel);
%   S   = vd_load_spectrum(out.Fmember, tel);
%
% Fmember   : struct with .FL/.FR/.RL/.RR  = [Ns x 6] axial force histories
% telemetry : struct with .Gx/.Gy/.Gz (g) and .fs (Hz)
% opt.gate_g: ignore acceleration reversals smaller than this (g)  [def 0.15]
% opt.gate_N: ignore force reversals smaller than this (N)         [def 150]

if nargin<3, opt=struct; end
if ~isfield(opt,'gate_g'), opt.gate_g=0.15; end   % noise gate on accel
if ~isfield(opt,'gate_N'), opt.gate_N=0;    end   % 0 = per-member from noise
if ~isfield(opt,'gateK'),  opt.gateK =3;    end   % gate = gateK x member noise
corners={'FL','FR','RL','RR'};
labels ={'Up-Fore','Up-Aft','Low-Fore','Low-Aft','Push/Pull','Tie/Toe'};

% ---- (1) cycle count on the accelerations (the literal definition) --------
fprintf('\nCYCLE COUNT from accelerations (gate = %.2f g):\n',opt.gate_g);
axn={'Long (X)','Lat (Y)','Vert (Z)'}; axd={telemetry.Gx,telemetry.Gy,telemetry.Gz};
for a=1:3
    [nc,~]=spectrum(axd{a}(:),opt.gate_g);
    fprintf('   %-9s : %6.0f cycles\n',axn{a},nc);
end

% ---- (2) three deliverables per wishbone ---------------------------------
if opt.gate_N>0
    fprintf('\nPER-WISHBONE (gate = %.0f N, global)   [tension +, compression -]:\n',opt.gate_N);
else
    fprintf(['\nPER-WISHBONE (gate = %gx each member''s OWN noise)   ' ...
             '[tension +, compression -]:\n'],opt.gateK);
    fprintf('  A single global gate distorts lightly-loaded members: it can exceed\n');
    fprintf('  their amplitude entirely, making counts unstable and left/right asymmetric.\n');
end
fprintf('%-7s %-10s %12s %8s %10s %9s\n','Corner','Member','MeanLoad[N]','Cycles','AvgAmp[N]','gate[N]');
S=struct('Corner',{},'Member',{},'MeanLoad',{},'Cycles',{},'AvgAmp',{},'Gate',{});
for c=1:4
    F=Fmember.(corners{c});                 % [Ns x 6], +comp / -tens (team sign)
    for i=1:6
        f = -F(:,i);                        % tension-positive axial force
        if opt.gate_N>0
            gN = opt.gate_N;
        else
            gN = opt.gateK*std(diff(f),'omitnan');   % this member's own noise
        end
        [nc,amp]=spectrum(f,gN);
        mean_load=mean(f);
        S(end+1)=struct('Corner',corners{c},'Member',labels{i}, ...  %#ok<AGROW>
            'MeanLoad',mean_load,'Cycles',nc,'AvgAmp',amp,'Gate',gN);
        if amp < gN, flag = '  <- amp below gate, count unstable'; else, flag = ''; end
        fprintf('%-7s %-10s %12.0f %8.0f %10.0f %9.0f%s\n', ...
            corners{c},labels{i},mean_load,nc,amp,gN,flag);
    end
end
end

% ==== a cycle = a change of direction, with a noise gate ==================
function tp = turning_points(x, gate)
    x=x(:); tp=x(1);
    for k=2:numel(x)
        v=x(k);
        if abs(v-tp(end))>=gate
            if numel(tp)>=2 && sign(v-tp(end))==sign(tp(end)-tp(end-1))
                tp(end)=v;          % same direction: extend the move
            else
                tp(end+1,1)=v;      % genuine reversal
            end
        end
    end
end

function [ncyc, amp] = spectrum(sig, gate)
    tp=turning_points(sig,gate);
    sw=abs(diff(tp));               % size of each up/down move
    ncyc=numel(sw)/2;               % 2 moves = 1 full cycle
    if isempty(sw), amp=0; else, amp=mean(sw)/2; end
end