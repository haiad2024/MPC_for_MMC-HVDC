function [S_opt, Sopt, Vi_opt, g_opt,i]  = MPC( iah, ibh, ich, ro, R, Ts, C, Lo, vah, vbh, vch, vag, vbg, vcg, ipa, ipb, ipc, ina, inb, inc, vdc, iaref, ibref, icref, vdcref, idc, idcref, vdc2,vdc3, vsa, vsb, vsc,vcan,vcbn,vccn,vcap,vcbp,vccp,izaref,izbref,izcref)

     % In order to call "cont2dis" function
    coder.extrinsic('cont2dis_LC');
    persistent ia ib ic x_opt g vcm van vbn vcn vap vbp vcp iza izb izc vsum idc1 vga vgb vgc

    % Initialize variables and get the parameters of the Discrete model based on Ts
    if isempty(x_opt), x_opt = 1; end
    if isempty(ia), ia = zeros(1, 4); end
    if isempty(ib), ib = zeros(1, 4); end
    if isempty(ic), ic = zeros(1, 4); end
    if isempty(van), van = zeros(1, 4); end
    if isempty(vbn), vbn = zeros(1, 4); end
    if isempty(vcn), vcn = zeros(1, 4); end
    if isempty(vap), vap = zeros(1, 4); end
    if isempty(vbp), vbp = zeros(1, 4); end
    if isempty(vcp), vcp = zeros(1, 4); end
    if isempty(vcm), vcm = zeros(1, 4); end
    if isempty(iza), iza = zeros(1, 4); end
    if isempty(izb), izb = zeros(1, 4); end
    if isempty(izc), izc = zeros(1, 4); end
    if isempty(vga), vga = zeros(1, 4); end
    if isempty(vgb), vgb = zeros(1, 4); end
    if isempty(vgc), vgc = zeros(1, 4); end
    if isempty(vsum), vsum = zeros(1, 4); end
    if isempty(idc1), idc1 = zeros(1, 4); end

    % Intialize the cost function
    g_opt = inf;
    % To generate the switching states matrix
    states = permute_states2(3, 36);

    % Read current measurements at sampling instant k

       ia(1) = iah;
       ib(1) = ibh;
       ic(1) = ich;
       van(1) = vah;
       vbn(1) = vbh;
       vcn(1) = vch;
       vap(1) = vag;
       vbp(1) = vbg;
       vcp(1) = vcg;

       vga(1) = vsa;
       vgb(1) = vsb;
       vgc(1) = vsc;

       iza(1) = ((ipa+ina)/2)-(idc/3);
       izb(1) = ((ipb+inb)/2)-(idc/3);
       izc(1) = ((ipc+inc)/2)-(idc/3);
       vcm(1) = (1/6)*(van(1)-vap(1)+vbn(1)-vbp(1)+vcn(1)-vcp(1));
       vsum(1) = (1/3)*(van(1)+vap(1)+vbn(1)+vbp(1)+vcn(1)+vcp(1));
       idc1(1) = idc;

    % FOR loop to find the best switch in the next instant and its corresponding V(i)
    for a1 = 1 : 50653
        %constraint check

        V = abs(states(a1,:)-states(x_opt,:)) ;
        H = states (a1,:);

        if ( V(1)>36 || V(2)>36 || V(3)>36 )
             continue
        end

        %state calculation
        
        % Output Prediction at instant k+1

       A = 1-((((2*1)*Ts))/(2*7+Lo));
       B = Ts/(7+2*Lo);
       C = Ts/(2*Lo);

%         van(2) = (H(1)/36)*vcan;
%         vbn(2) = (H(2)/36)*vcbn;
%         vcn(2) = (H(3)/36)*vccn;
%         vap(2) = (1-(H(1)/36))*vcap;
%         vbp(2) = (1-(H(2)/36))*vcbp;
%         vcp(2) = (1-(H(3)/36))*vccp;
%         vcm(2) = (1/6)*(van(2)-vap(2)+vbn(2)-vbp(2)+vcn(2)-vcp(2));
%        vsum(2) = (1/3)*(van(2)+vap(2)+vbn(2)+vbp(2)+vcn(2)+vcp(2));

        van(2) = (H(1)/36)*vcan;
        vbn(2) = (H(2)/36)*vcbn;
        vcn(2) = (H(3)/36)*vccn;
        vap(2) = (1-(H(1)/36))*vcap;
        vbp(2) = (1-(H(2)/36))*vcbp;
        vcp(2) = (1-(H(3)/36))*vccp;
        vcm(2) = (1/6)*(van(2)-vap(2)+vbn(2)-vbp(2)+vcn(2)-vcp(2));
       vsum(2) = (1/3)*(van(2)+vap(2)+vbn(2)+vbp(2)+vcn(2)+vcp(2));

       ia(2) = A*ia(1) + B*(van(2)-vap(2)-2*vcm(2)) ;
       ib(2) = A*ib(1) + B*(vbn(2)-vbp(2)-2*vcm(2)) ;
       ic(2) = A*ic(1) + B*(vcn(2)-vcp(2)-2*vcm(2)) ;

       iza(2) = iza(1) + C*(vdc-vap(2)-van(2)) ;
       izb(2) = izb(1) + C*(vdc-vbp(2)-vbn(2)) ;
       izc(2) = izc(1) + C*(vdc-vcp(2)-vcn(2)) ;

       idc1(2) = idc1(1) + 3*C*(vdc-vsum(2));

%        vga(2) = -(H(1)/36)*vdcref/2+((36-(H(1)))/36)*vdcref/2;
%        vgb(2) = -(H(2)/36)*vdcref/2+((36-(H(2)))/36)*vdcref/2;
%        vgc(2) = -(H(3)/36)*vdcref/2+((36-(H(3)))/36)*vdcref/2;

       vga(2) = -(van(2)+vap(2))/2+vdc/2;
       vgb(2) = -(vbn(2)+vbp(2))/2+vdc/2;
       vgc(2) = -(vcn(2)+vcp(2))/2+vdc/2;

%         vga(2) = (vdcref/6)-((vcan+vcap)/2);
%         vgb(2) = (vdcref/6)-((vcbn+vcbp)/2);
%         vgc(2) = (vdcref/6)-((vccn+vccp)/2);

    g1 = (abs(iaref-ia(2))+abs(ibref-ib(2))+abs(icref-ic(2)));
    g2 = (abs(iza(2)-izaref)+abs(izb(2)-izbref)+abs(izc(2)-izcref));
    g3 = (abs(idcref-idc1(2)));
    g4 = abs(van(2)-van(1))+abs(vbn(2)-vbn(1))+abs(vcn(2)-vcn(1))+abs(vap(2)-vap(1))+abs(vbp(2)-vbp(1))+abs(vcp(2)-vcp(1));
%     g4 = abs(van(2)-vdcref/36)+abs(vbn(2)-vdcref/36)+abs(vcn(2)-vdcref/36)+abs(vap(2)-vdcref/36)+abs(vbp(2)-vdcref/36)+abs(vcp(2)-vdcref/36);
    g5 = (abs(vga(2)-vga(1))+abs(vgb(2)-vgb(1))+abs(vgc(2)-vgc(1)));
    g6 = abs(vdc-vdcref);

      g = 1*g1 + 1*g2 + 1*g3 + 0*g4 + 0*(abs(vcm(1))) + 0*g5 + 0*g6 ;


        if (g<g_opt)
            x_opt = a1;
            g_opt = g; 
        end
    end

    % Output switching states and its voltage vector. 

    i = van(2);

    Vi_opt =  x_opt; 
    S_opt = states(x_opt,:);
    d_opt = [36 36 36]-(S_opt);
    Sopt = [S_opt,d_opt];     

end