function m = alloy_lib(name)
% ALLOY_LIB  Material properties for the aluminium fatigue module.
% ------------------------------------------------------------------------
% Values below are PUBLISHED LITERATURE figures, used so the pipeline runs
% before Granta access. Replace .UTS / .Sf_ref / .N_ref with Ansys Granta
% EduPack values and note in the write-up which source was used.
%
% From Granta, record per alloy:
%   UTS [MPa], yield [MPa], E [GPa], and FATIGUE STRENGTH at its stated
%   cycle count (usually 1e7). Granta reports RANGES - take the LOWER bound
%   for fatigue strength; that is the conservative choice.
%
% .sf_alt / .b_alt are an independent strain-life Basquin pair for the same
% alloy, carried so alloy_fatigue can report how far the two constructions
% disagree. They are not interchangeable.

switch upper(strrep(name,' ',''))

    case '7075-T6'                 % chassis clevises
        m.name   = '7075-T6';
        m.UTS    = 572;            % MPa   <-- replace with Granta
        m.yield  = 503;            % MPa
        m.E      = 71.7;           % GPa
        m.Sf_ref = 159;            % MPa at 5e8 (rotating beam, literature).
        m.N_ref  = 5e8;            % Anodising drops the endurance limit
                                   % below 200 MPa; shot peening raises it
                                   % to ~300. Surface finish matters here.
        m.sf_alt = 1466;           % strain-life pair, literature
        m.b_alt  = -0.143;

    case '6082-T6'                 % wishbone inserts (confirmed by team)
        % PUBLISHED FATIGUE DATA SPANS A FACTOR OF 2.2 AT 1e7 CYCLES:
        %   77.3 MPa  Zhang et al., Mater. Res. Express 8 (2021), base metal
        %  109.3 MPa  same study, second specimen set
        %  166.4 MPa  same study, third condition
        % Gigacycle work (Bezier sonotrode, 1e9) gives a fatigue limit of
        % 104 MPa and an "endurance strength" of 84 MPa - consistent with
        % the LOW end of the 1e7 range, which is why the low anchor is used.
        % The 77.3 value is taken as the DESIGN value: conservative, and
        % supported by the independent gigacycle result.
        % Picking the high anchor instead changes predicted life by 5-6
        % ORDERS OF MAGNITUDE at service amplitudes. Report which was used.
        m.name   = '6082-T6';
        m.UTS    = 310;            % MPa (team FEA report uses 250 MPa YIELD)
        m.yield  = 250;            % MPa - TBRe25 firewall FEA report value
        m.E      = 70;             % GPa - team value
        m.Sf_ref = 77.3;           % MPa at N_ref - CONSERVATIVE anchor
        m.N_ref  = 1e7;
        m.sf_alt = 605;            % from the 109.3 MPa anchor, for comparison
        m.b_alt  = -0.1017;

    case '6082-T6-WELDED'          % if inserts are welded, not bonded
        % Welding halves the fatigue strength: MIG-welded 6082 gives
        % 48.7 MPa at 1e7 (base metal 77.3 in the same study), and an
        % S-N based estimate of 37.6 MPa. HAZ hardness drops to 65.5 HV.
        % USE THIS if the inserts are welded into the tubes.
        m.name   = '6082-T6 (welded)';
        m.UTS    = 249;            % MPa, welded condition
        m.yield  = 180;
        m.E      = 70;
        m.Sf_ref = 37.6;           % MPa at 1e7 - lower of the two published
        m.N_ref  = 1e7;
        m.sf_alt = 790;            % from the 48.7 MPa anchor
        m.b_alt  = -0.1657;

    case '2024-T3'
        m.name   = '2024-T3';
        m.UTS    = 483;
        m.yield  = 345;
        m.E      = 73.1;
        m.Sf_ref = 138;
        m.N_ref  = 5e8;
        m.sf_alt = 1100;
        m.b_alt  = -0.124;

    case '7075-T651'
        m.name   = '7075-T651';
        m.UTS    = 560;
        m.yield  = 480;
        m.E      = 71.7;
        m.Sf_ref = 152;
        m.N_ref  = 5e8;
        m.sf_alt = 1466;
        m.b_alt  = -0.143;

    otherwise
        error(['Unknown alloy ''%s''. Add it to alloy_lib with UTS, ' ...
               'Sf_ref and N_ref from Granta.'], name);
end
end
