import numpy as np

def tensaoAsa(De, Di, T, M, Xt, k):
    # 1. Propriedades dos tubos de carbono
    I = np.pi * (De ** 4-Di ** 4)/64
    J = np.pi * (De ** 4-Di ** 4)/32
    
    # 2. Calculo das tensoes no ponto
    stressP1 = [M * (De / 2) * (1 / I), T * (De / 2) * (1 / J)]
    
    # 3. Aplicar fator de concentracao k
    stressP1 *= k; 
    
    # 4. Aplicacao do Criterio de falha de Rankine
    sigmax = stressP1[0]
    sigmay = 0
    Tau = stressP1[1]
    
    # Tensoes principais
    Sm =  (sigmax + sigmay) / 2             
    R = (((sigmax - sigmay) / 2) ** 2 + Tau ** 2) ** 0.5
    P_Stress = [abs(Sm + R), abs(Sm - R)]
    
    # Calculo do fator de seguranca
    FS = Xt / max(P_Stress)
    
    return FS

"""
Funcao  matlab
function [FS] = tensaoAsa(De,Di,T,M,Xt,k)
%% 1.Propriedades dos tubos de carbono
I= pi*(De^4-Di^4)/64;                                                       % Momento de inércia 
J= pi*(De^4-Di^4)/32;                                                       % Momento polar de inércia 

%% 2. Cálculo das tensoes no ponto
stressP1(1) = M*(De/2)*(1/I);                                               % Sigma xx no ponto crítico
stressP1(2) = T*(De/2)*(1/J);                                               % Sigma xy no ponto crítico

%% 3. Aplica fator de concentracao k 
stressP1=stressP1*k;                                                        % k = fator de concentração de tensão

%% 4.Aplicação do Critério de falha de Rankine 
sigmax = stressP1(1);
sigmay = 0;
Tau = stressP1(2);

%tensões principais
Sm =  (sigmax + sigmay)/2;                                                  % Sigma médio
R = (((sigmax - sigmay)/2)^2+Tau^2)^0.5;                                    % Raio do círculo de mohr
P_Stress(1) = abs(Sm + R);                                                  % Sigma1
P_Stress(2) = abs(Sm - R);                                                  % Sigma2

%Fator de segurança
FS = Xt/max(P_Stress);

end
"""



