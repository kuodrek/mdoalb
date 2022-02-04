import numpy as np

# Trim da aeronave para voo de cruzeiro
# Adaptado de ROSKAM, Airplane Flight Dynamics and Automatic Flight part 1

def trim(asa, eh, aviao_geometria, aviao_desempenho, cond_voo, superficie):
    # Dados asa
    c_w = asa.chords
    cr_w = c_w[0]
    sw = asa.surface
    mac_w = asa.macw
    ar_w = asa.aspect_ratio
    iw = asa.iw
    xac_w = asa.xac
    CLalpha_w = asa.clalpha
    CL0_w = asa.cl0
    Cmac_w = asa.cmac

    # Dados EH
    sh = eh.surface
    CLalpha_h = eh.clalpha
    CL0_h = eh.cl0
    taue = eh.taue
    ih = eh.ih

    # Dados avião
    xcg = aviao_geometria.xcg
    lt = aviao_geometria.lt

    # Condições de voo
    v = aviao_desempenho.v_stall
    m = aviao_desempenho.mtow
    rho = cond_voo.rho
    g = cond_voo.g

    # Cálculos gerais
    epsilon0 = 360 * CL0_w / (np.pi ** 2 * ar_w) # Downwash quando alfa = 0
    depsilondalpha = 360 * CLalpha_w / (np.pi ** 2 * ar_w) # Variação do downwash pelo alfa
    n = 0.8 # Eficiência da EH
    Vh = lt * sh / (mac_w * sw) # Volume de cauda da EH

    # Coeficientes de sustentação:
    CLdeltae = CLalpha_h * n * sh / sw * taue
    CLih = CLalpha_h * n * sh / sw
    # Global
    CL0 = CL0_w + n * sh / sw * (-CL0_h + CLalpha_h * (-iw - epsilon0))
    CLalpha = CLalpha_w + n * sh / sw * CLalpha_h * (1 - depsilondalpha)
    # CL para trimagem
    CL1 = m * g / (0.5 * rho * v ** 2 * sw)

    # Coeficientes de momento:
    CM0w = Cmac_w + CL0_w * (xcg - xac_w)    # xCG e xACw estão em % da corda média aerodinâmica (MACw)
    CMalphaw = CLalpha_w * (xcg - xac_w)
    CM0t = -n * Vh * (-CL0_h + CLalpha_h * (-epsilon0 - iw))
    CMalphat = -n * Vh * CLalpha_h * (1 - depsilondalpha)
    CMdeltae = -CLalpha_h * n * Vh * taue
    CMih = -CLalpha_h * n * Vh

    # Global:
    CM0 = CM0w + CM0t
    CMalpha = CMalphaw + CMalphat

    if superficie == 'eh': # EH + profundor
        # Alpha trim
        alpha_trim = ((CL1 - CL0 - CLih * ih) * CMdeltae + (CM0 + CMih * ih) * CLdeltae ) / (CLalpha * CMdeltae - CMalpha * CLdeltae)
        
        # Elevator trim
        elevator_trim =  (-CLalpha * (CM0 + CMih * ih) - CMalpha * (CL1 - CL0 - CLih * ih)) / (CLalpha * CMdeltae - CMalpha * CLdeltae)
    elif superficie == 'tm': # EH Totalmente móvel
        A = [[CLalpha, CLih], [CMalpha, CMih]]
        B = [CL1 - CL0, -CM0]
        X = np.inverse(A).B
        alpha_trim = X[0]
        elevator_trim = X[1]
    trim_angles = [alpha_trim, elevator_trim]
    return trim_angles

"""
function [alpha_trim,elevator_trim] = trimagem(asa_geometria,EH_geometria,iw,ih,xCG,lt,taue,flow_settings,Vdesempenho,WingCoef,EHCoef,m,superficie)
%Atribuição das variáveis
cw = asa_geometria{2,1};
crw = cw(1);
Sw = asa_geometria{1,3};
MACw = asa_geometria{2,3};
ARw = asa_geometria{4,3};
xACw = asa_geometria{6,3};
xCG = xCG*crw/MACw;
CLalphaw = WingCoef(1);
CL0w = WingCoef(2);
Cmacw = asa_geometria{5,3};

CLalphah = EHCoef(1);
CL0h = EHCoef(2);
Sh = EH_geometria{1,3};

% -> Cálculos gerais <-
epsilon0 = 360*CL0w/(pi^2*ARw);
depsilondalpha = 360*CLalphaw/(pi^2*ARw);
n = 0.8;
Vh = lt*Sh/(MACw*Sw);
g = 9.80665;
rho = flow_settings(2);
V = Vdesempenho(5)*1.1; % Velocidade de estol da aeronave

% Coeficientes de sustentação:
CLdeltae = CLalphah*n*Sh/Sw*taue;
CLih = CLalphah*n*Sh/Sw;
% Global
CL0 = CL0w + n*Sh/Sw*(-CL0h+CLalphah*(-iw-epsilon0));
CLalpha = CLalphaw + n*Sh/Sw*CLalphah*(1-depsilondalpha);
% CL para trimagem
CL1 = m*g/(0.5*rho*V^2*Sw);

% Coeficientes de momento:
CM0w = Cmacw+CL0w*(xCG-xACw);    % Lembrando que xCG e xACw estão em % da corda média aerodinâmica (MACw)
CMalphaw = CLalphaw*(xCG-xACw);
CM0t = -n*Vh*(-CL0h+CLalphah*(-epsilon0-iw));
CMalphat = -n*Vh*CLalphah*(1-depsilondalpha);
CMdeltae = -CLalphah*n*Vh*taue;
CMih = -CLalphah*n*Vh;
% Global:
CM0 = CM0w + CM0t;
CMalpha = CMalphaw + CMalphat;

if strcmp(superficie,'profundor') == 1 % EH + profundor
    % Alpha trim
    alpha_trim = ( (CL1-CL0-CLih*ih)*CMdeltae + (CM0 + CMih*ih)*CLdeltae )/(CLalpha*CMdeltae - CMalpha*CLdeltae);
    
    % Elevator trim
    elevator_trim =  (-CLalpha*(CM0 + CMih*ih) - CMalpha*(CL1 - CL0 - CLih*ih))/(CLalpha*CMdeltae - CMalpha*CLdeltae);
elseif strcmp(superficie,'tm') == 1 % EH Totalmente móvel
    A = [CLalpha, CLih; CMalpha CMih];
    B = [(CL1 - CL0); -CM0];
    x = linsolve(A,B);
    alpha_trim = x(1);
    elevator_trim = x(2);
end
end
"""