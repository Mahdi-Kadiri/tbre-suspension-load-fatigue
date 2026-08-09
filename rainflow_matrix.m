function R = rainflow_matrix(sig, SN, opt)
% RAINFLOW_MATRIX  2D range-mean rainflow matrix + damage matrix.
% ------------------------------------------------------------------------
% The industry-standard way a load spectrum is stored and exchanged: cycles
% binned by BOTH amplitude and mean, so mean-stress content is preserved
% instead of being averaged away. Adds the damage matrix (which bins
% actually consume life) and the damage-equivalent load.
%
% USAGE
%   R = rainflow_matrix(sigma_MPa);                  % defaults, steel
%   R = rainflow_matrix(sigma_MPa, SN, opt);
%
% INPUT
%   sig : stress (or force) history, vector. TENSION POSITIVE.
%   SN  : .sf .b .su .Se .mean('goodman'|'swt'|'none') .knee('haibach'|'infinite')
%   opt : .nAmp .nMean .gate .plot .name
%
% OUTPUT  R.countMatrix  R.damageMatrix  R.ampEdges  R.meanEdges
%         R.cycles [range mean count]  R.D  R.Seq  R.Neq  R.blocksToTarget
%
% NOTE ON THE KNEE
%   'infinite' : cycles below Se do no damage. Valid for constant amplitude,
%                OPTIMISTIC for variable amplitude - large cycles degrade the
%                endurance limit so small ones start counting.
%   'haibach'  : below the knee the S-N slope changes k -> 2k-1 (k = -1/b).
%                The honest default for racing spectra. THIS IS THE DEFAULT.
%
% NOT VALID FOR COMPOSITES. Basquin/Miner/Goodman are metals methods. CFRP
% needs R-ratio-dependent S-N, is compression-governed, and usually fails at
% the bonded insert, not the tube.

if nargin<2 || isempty(SN), SN=struct; end
d=@(f,v) SN.(f);
if ~isfield(SN,'sf'),   SN.sf   = 1.09*310;  end   % MPa, Basquin coefficient
if ~isfield(SN,'b'),    SN.b    = -0.085;    end
if ~isfield(SN,'su'),   SN.su   = 310;       end   % MPa, UTS
if ~isfield(SN,'Se'),   SN.Se   = 108;       end   % MPa, Marin endurance
if ~isfield(SN,'mean'), SN.mean = 'goodman'; end
if ~isfield(SN,'knee'), SN.knee = 'haibach'; end
if ~isfield(SN,'Dtarget'), SN.Dtarget = 0.5; end
if nargin<3 || isempty(opt), opt=struct; end
if ~isfield(opt,'nAmp'),  opt.nAmp  = 12;  end
if ~isfield(opt,'nMean'), opt.nMean = 12;  end
if ~isfield(opt,'gate'),  opt.gate  = 0;   end   % ignore reversals < gate
if ~isfield(opt,'plot'),  opt.plot  = true; end
if ~isfield(opt,'name'),  opt.name  = 'member'; end

% ---- rainflow (ASTM E1049) --------------------------------------------
cyc = rainflow_astm(sig(:), opt.gate);       % [range mean count]
if isempty(cyc), R=struct('cycles',cyc,'D',0); return; end
amp = cyc(:,1)/2;  mn = cyc(:,2);  cnt = cyc(:,3);

% ---- bin edges ---------------------------------------------------------
aMax = max(amp)*(1+1e-9);
ae = linspace(0, aMax, opt.nAmp+1);
mLo = min(mn); mHi = max(mn);
if mHi<=mLo, mHi = mLo+1; end
me = linspace(mLo, mHi*(1+1e-9)+eps, opt.nMean+1);

% ---- S-N slopes --------------------------------------------------------
k  = -1/SN.b;                      % life-domain slope
Nk = 0.5*(SN.Se/SN.sf)^(1/SN.b);   % cycles at the knee
k2 = 2*k - 1;                      % Haibach sub-knee slope

% ---- fill count + damage matrices -------------------------------------
C = zeros(opt.nAmp, opt.nMean);
D = zeros(opt.nAmp, opt.nMean);
for q = 1:numel(amp)
    i = min(discretize_(amp(q), ae), opt.nAmp);
    j = min(discretize_(mn(q),  me), opt.nMean);
    C(i,j) = C(i,j) + cnt(q);
    sar = mean_correct(amp(q), mn(q), SN);
    Nf  = life(sar, SN, Nk, k2);
    if isfinite(Nf) && Nf>0, D(i,j) = D(i,j) + cnt(q)/Nf; end
end

% ---- damage-equivalent load -------------------------------------------
% single amplitude that, applied Neq times, causes the same total damage
Dtot = sum(D(:));  Neq = sum(C(:));
if Dtot>0, Seq = SN.sf*(2*(Neq/Dtot))^SN.b; else, Seq = NaN; end

R.countMatrix=C; R.damageMatrix=D; R.ampEdges=ae; R.meanEdges=me;
R.cycles=cyc; R.D=Dtot; R.Neq=Neq; R.Seq=Seq;
R.blocksToTarget = SN.Dtarget/max(Dtot,realmin);
R.maxAmp=max(amp); R.SN=SN;

fprintf('\n%s: %.1f cycles | maxAmp %.1f MPa | Se %.0f MPa | knee=%s\n', ...
        opt.name, Neq, max(amp), SN.Se, SN.knee);
fprintf('  damage/block %.3e | damage-equiv amp %.1f MPa | blocks to D=%.2f: %.4g\n', ...
        Dtot, Seq, SN.Dtarget, R.blocksToTarget);

if opt.plot, plot_matrices(R, opt.name); end
end

% ========================================================================
function i = discretize_(x, edges)
    i = find(x >= edges, 1, 'last');
    if isempty(i), i=1; end
    i = max(1, min(i, numel(edges)-1));
end

function sar = mean_correct(sa, sm, SN)
    switch lower(SN.mean)
        case 'goodman'
            if sm>0, sar = sa/(1 - sm/SN.su); else, sar = sa; end
        case 'swt'
            smax = sm+sa;
            if smax>0, sar = sqrt(smax*sa); else, sar = 0; end
        otherwise, sar = sa;
    end
end

function Nf = life(sar, SN, Nk, k2)
    if sar<=0, Nf=Inf; return; end
    if sar >= SN.Se
        Nf = 0.5*(sar/SN.sf)^(1/SN.b);
    elseif strcmpi(SN.knee,'haibach')
        Nf = Nk*(SN.Se/sar)^k2;      % extends life below knee (slope 2k-1)
    else
        Nf = Inf;                    % infinite-life cutoff
    end
end

% ---- self-contained ASTM E1049 rainflow --------------------------------
function cyc = rainflow_astm(x, gate)
    if nargin<2, gate=0; end
    tp = x(1);
    for q = 2:numel(x)
        v = x(q);
        if abs(v-tp(end)) >= gate
            if numel(tp)>=2 && sign(v-tp(end))==sign(tp(end)-tp(end-1))
                tp(end)=v;
            else
                tp(end+1,1)=v; %#ok<AGROW>
            end
        end
    end
    cyc=zeros(0,3); s=zeros(0,1);
    for p = tp.'
        s(end+1,1)=p; %#ok<AGROW>
        while numel(s)>=3
            X=abs(s(end)-s(end-1)); Y=abs(s(end-1)-s(end-2));
            if X<Y, break; end
            rng=Y; mn=(s(end-1)+s(end-2))/2;
            if numel(s)==3
                cyc(end+1,:)=[rng mn 0.5]; s(1)=[]; %#ok<AGROW>
            else
                cyc(end+1,:)=[rng mn 1.0]; s(end-2:end-1)=[]; %#ok<AGROW>
            end
        end
    end
    for q=1:numel(s)-1
        cyc(end+1,:)=[abs(s(q+1)-s(q)) (s(q+1)+s(q))/2 0.5]; %#ok<AGROW>
    end
end

% ---- dark-background plots ---------------------------------------------
function plot_matrices(R, name)
    ac=(R.ampEdges(1:end-1)+R.ampEdges(2:end))/2;
    mc=(R.meanEdges(1:end-1)+R.meanEdges(2:end))/2;
    f=figure('Color','k','Position',[100 100 1150 460]);

    subplot(1,2,1);
    Cp=R.countMatrix; Cp(Cp==0)=NaN;
    imagesc(mc,ac,Cp); set(gca,'YDir','normal');
    title(sprintf('%s - cycle count',name),'Color','w','Interpreter','none');
    xlabel('mean stress [MPa]','Color','w'); ylabel('amplitude [MPa]','Color','w');
    cb=colorbar; cb.Color='w'; cb.Label.String='cycles'; cb.Label.Color='w';
    set(gca,'Color','k','XColor','w','YColor','w','FontSize',11);
    hold on; yline(R.SN.Se,'--','Se','Color',[1 .4 .4],'LineWidth',1.5,'LabelHorizontalAlignment','left');

    subplot(1,2,2);
    Dp=R.damageMatrix; Dp(Dp<=0)=NaN;
    imagesc(mc,ac,log10(Dp)); set(gca,'YDir','normal');
    title(sprintf('%s - log_{10} damage',name),'Color','w','Interpreter','none');
    xlabel('mean stress [MPa]','Color','w'); ylabel('amplitude [MPa]','Color','w');
    cb=colorbar; cb.Color='w'; cb.Label.String='log_{10} damage'; cb.Label.Color='w';
    set(gca,'Color','k','XColor','w','YColor','w','FontSize',11);
    hold on; yline(R.SN.Se,'--','Se','Color',[1 .4 .4],'LineWidth',1.5,'LabelHorizontalAlignment','left');
    colormap(f,'turbo');
end
