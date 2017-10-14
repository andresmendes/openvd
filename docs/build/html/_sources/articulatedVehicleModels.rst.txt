Articulated vehicle models
********************************************************************************

The linear version of this model is described by equations ?? and the nonlinear version of this model is described by equations ??.

Description
Bicycle model

.. figure:: https://andresmendes.github.io/openvd/illustrations/modelArticulatedBicycleApprox.svg

Free body diagram

.. figure:: https://andresmendes.github.io/openvd/illustrations/modelArticulatedFreeBodyDiagram.svg

The center of gravity of the tractor and semitrailer are located at the point :math:`T` and :math:`S`, respectively. The front and rear axles are located at the points :math:`F` and :math:`R`, respectively. :math:`A` is the articulation point and :math:`M` is the axle of the semitrailer. The constant :math:`a` measures the distance of point :math:`F` to :math:`T` and :math:`b` the distance of point :math:`T` to :math:`R`. The distance of the articulation from the rear axle of the tractor is given by :math:`c`. :math:`d` and :math:`e` are the distances from the semitrailer. The angles :math:`\alpha_F` e :math:`\alpha_R` are the front and rear slip angles, respectively. :math:`\alpha_T` is the vehicle side slip angle and :math:`\psi` is the vehicle yaw angle. :math:`\delta` is the steering angle.

.. figure:: https://andresmendes.github.io/openvd/illustrations/modelArticulated.svg

Este modelo  escrito na forma:

.. math:: {\bf M}({\bf x}) \dot{{\bf x}} = {\bf f}({\bf x})


Where :math:`{\bf x}` is the state vector, :math:`{\bf M}({\bf x})` the mass matrix and :math:`{\bf f}({\bf x})` is the vector function. Therefore, a function that allows the integration of the system with an explicit mass matrix is necessary. In this package the _ode45_ function is used. Details: `ode45 (Mass matrix) <http://www.mathworks.com/help/matlab/ref/ode45.html?searchHighlight=%22mass%20matrix%22>`_.

Articulated vehicle model
================================================================================

O modelo físico do conjunto é ilustrado na figura \ref{modelSimple}. Para caracterizar a dinâmica deste sistema é utilizada a base :math:`\Omega_{\rm O} = \{ {\rm O} {\bf i} {\bf j} {\bf k} \}` fixa no referencial inercial. A base :math:`\Omega_{\rm T} = \{ {\rm T} {\bf t}_x {\bf t}_y {\bf t}_z \}` é solidária ao caminhão-trator e a base :math:`\Omega_{\rm S} = \{ {\rm S} {\bf s}_x {\bf s}_y {\bf s}_z \}` é solidária ao semirreboque. Os versores :math:`{\bf t}_x` e :math:`{\bf s}_x` apontam para frente na direção longitudinal de ambos os módulos e os versores :math:`{\bf t}_y` e :math:`{\bf s}_y` apontam para a esquerda. Para auxiliar a descrição das grandezas no eixo dianteiro é definida a base :math:`\Omega_{\rm F} = \{ {\rm F} {\bf e}_x {\bf e}_y {\bf e}_z \}` solidária ao eixo dianteiro com o versor :math:`{\bf e}_x` apontando para frente na direção longitudinal do pneu e :math:`{\bf e}_y` apontando para a esquerda. Os pontos :math:`{\rm T}` e :math:`{\rm S}` localizam o centro da massa do caminhão-trator e do semirreboque, respectivamente. :math:`{\rm F}` e :math:`{\rm R}` localizam os eixos dianteiro e traseiro, respectivamente. :math:`{\rm A}` é o ponto de articulação e :math:`{\rm M}` é o eixo do semirreboque. O ponto :math:`{\rm O}` é a origem do sistema e se encontra fixo no referencial inercial. A distância :math:`a` separa os pontos :math:`{\rm F}` e :math:`{\rm T}` e a distância :math:`b` separa os pontos :math:`{\rm T}` e :math:`{\rm R}`. :math:`c` separa os pontos :math:`{\rm R}` e :math:`{\rm A}`, :math:`d` separa os pontos :math:`{\rm A}` e :math:`{\rm S}` e :math:`e` separa os pontos :math:`{\rm S}` e :math:`{\rm M}`. Os vetores velocidade :math:`{\bf v}` e os ângulos de deriva :math:`\alpha` recebem os subscritos referentes aos pontos aos quais eles estão associados.

A modelagem do caminhão-trator e semirreboque consiste na utilização de dois corpos rígidos que se movimentam sobre um plano horizontal e são unidos por um ponto de articulação. Desta forma, o modelo apresenta quatro graus de liberdade. Portanto, as coordenadas generalizadas podem ser dadas por

.. math::

    q_1 &= x \\
    q_2 &= y \\
    q_3 &= \psi \\
    q_4 &= \phi,

onde :math:`x` e :math:`y` são as coordenadas longitudinal e transversal do centro de massa do caminhão-trator, respectivamente. :math:`\psi` é o ângulo de orientação absoluta do caminhão-trator e :math:`\phi` é o ângulo de orientação relativa do semirreboque.

*Modelo não linear*

O vetor posição do centro de massa do caminhão-trator em relação ao ponto :math:`O` é

.. math:: {\bf p}_{{\rm T}/{\rm O}} = x {\bf i} + y {\bf j}.
    :label: positionTractor

O vetor posição do centro de massa do semirreboque é

.. math:: {\bf p}_{{\rm S}/{\rm O}} = \left[ x - \left( b + c \right) \cos \psi - d \cos \left( \psi - \phi \right) \right] {\bf i} + \left[ y - \left( b + c \right) \sin \psi - d \sin \left( \psi - \phi \right) \right] {\bf j}.
    :label: positionSemitrailer

Derivando a equação :eq:`positionTractor` em relação ao tempo temos

.. math:: {\bf v}_{\rm T} = \dot{x} {\bf i} + \dot{y} {\bf j}.
    :label: velocityTractor

Derivando a equação :eq:`positionSemitrailer` em relação ao tempo temos

.. math::
    :label: velocitySemitrailer

    {\bf v}_{\rm S} &= \left[ \dot{x} + \left( b + c \right) \dot{\psi} \sin \psi + d \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \right] {\bf i} + ... \\
     ... &+ \left[ \dot{y} - \left( b + c \right) \dot{\psi} \cos \psi - d \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) \right] {\bf j}. \\

O vetor velocidade angular do caminhão-trator é

.. math:: {\bf w}_T = \dot{\psi} {\bf k}.
    :label: angulaVelTrailer

O vetor velocidade angular do semirreboque é

.. math:: {\bf w}_S = \left( \dot{\psi} - \dot{\phi} \right) {\bf k}
    :label: angulaVelSemir

A energia cinética do sistema é

.. math:: T = \frac{1}{2} m_{T} {\bf v}_{\rm T} \cdot {\bf v}_{\rm T} + \frac{1}{2} m_{S} {\bf v}_{\rm S} \cdot {\bf v}_{\rm S} + \frac{1}{2} \left\{ {\bf w}_T \right\}^T \left[ {\bf J}_T \right] \left\{ {\bf w}_T \right\} + \frac{1}{2} \left\{ {\bf w}_S \right\}^T \left[ {\bf J}_S \right] \left\{ {\bf w}_S \right\}.
    :label: kinEnergyGeneral

Substituindo as equações :eq:`velocityTractor`, :eq:`velocitySemitrailer`, :eq:`angulaVelTrailer`, :eq:`angulaVelSemir` em :eq:`kinEnergyGeneral` temos

.. math:: T = \frac{1}{2} m_{T} \left( \dot{x}^2 + \dot{y}^2 \right) + \frac{1}{2} m_{S} \left( C_1^2 + C_2^2 \right) + \frac{1}{2} I_{T} \dot{\psi}^2 + \frac{1}{2} I_{S} \left( \dot{\psi} - \dot{\psi} \right)^2,
    :label: kinEnergyCoord

onde

.. math::
    :label: constants

    C_1 &= \dot{x} + \left( b + c \right) \dot{\psi} \sin \psi + d \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \\
    C_2 &= \dot{y} - \left( b + c \right) \dot{\psi} \cos \psi - d \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right).

Derivando a equação :eq:`constants` temos

.. math::
    :label: constantsTimeDiff

    \dot{C}_1 &= \ddot{x} + \left( b + c \right) \ddot{\psi} \sin \psi + \left( b + c \right) \dot{\psi}^2 \cos \psi + d \left( \ddot{\psi} - \ddot{\phi} \right) \sin \left( \psi - \phi \right) + d \left( \dot{\psi} - \dot{\phi} \right)^2 \cos \left( \psi - \phi \right) \\
    \dot{C}_2 &= \ddot{y} - \left( b + c \right) \ddot{\psi} \cos \psi + \left( b + c \right) \dot{\psi}^2 \sin \psi - d \left( \ddot{\psi} - \ddot{\phi} \right) \cos \left( \psi - \phi \right) + d \left( \dot{\psi} - \dot{\phi} \right)^2 \sin \left( \psi - \phi \right)

As derivadas parciais da energia cinética do sistema (equação :eq:`kinEnergyCoord`) em relação às coordenadas generalizadas são

.. math::
    :label: lagrangePartialTerm

    \frac{\partial T}{\partial q_1} &= \frac{\partial T}{\partial x} = 0 \\
    \frac{\partial T}{\partial q_2} &= \frac{\partial T}{\partial y} = 0 \\
    \frac{\partial T}{\partial q_3} &= \frac{\partial T}{\partial \psi} &=& m_S C_1 \left[ \left( b + c \right) \dot{\psi} \cos \psi + d \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) \right] + ... \\
    ... &+ m_S C_2 \left[ \left( b + c \right) \dot{\psi} \sin \psi + d \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \right] \\
    \frac{\partial T}{\partial q_4} &= \frac{\partial T}{\partial \phi} = m_S C_1 \left[ - d \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) \right] + m_S C_2 \left[ - d \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \right].

As derivadas parciais da energia cinética do sistema em relação às derivadas temporais das coordenadas generalizadas são

.. math::
    :label: partialDiff

    \frac{\partial T}{\partial \dot{q}_1} &= \frac{\partial T}{\partial \dot{x}} = m_{T} \dot{x} + m_S C_1 \\
    \frac{\partial T}{\partial \dot{q}_2} &= \frac{\partial T}{\partial \dot{y}} = m_{T} \dot{y} + m_S C_2 \\
    \frac{\partial T}{\partial q_3} &= \frac{\partial T}{\partial \dot{\psi}} &=& m_S C_1 \left[ \left( b + c \right) \sin \psi + d \sin \left( \psi - \phi \right) \right] + ... \\
    ... &+ m_S C_2 \left[ - \left( b + c \right) \cos \psi - d \cos \left( \psi - \phi \right) \right] + I_T \dot{\psi} + I_S \left( \dot{\psi} - \dot{\phi} \right) \\
    \frac{\partial T}{\partial q_4} &= \frac{\partial T}{\partial \dot{\phi}} = m_S C_1 \left[ - d \sin \left( \psi - \phi \right) \right] + m_S C_2 \left[ d \cos \left( \psi - \phi \right) \right] - I_S \left( \dot{\psi} - \dot{\phi} \right).

Derivando as equações :eq:`partialDiff` em relação ao tempo temos

.. math::
    :label: lagrangeTimeTerm

    \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_1} \right) &= \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{x}} \right) = m_{T} \ddot{x} + m_S \dot{C}_1 \\
    \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_2} \right) &= \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{y}} \right) = m_{T} \ddot{y} + m_S \dot{C}_2 \\
    \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_3} \right) &= \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{\psi}} \right) &=& m_S \dot{C}_1 \left[ \left( b + c \right) \sin \psi + d \sin \left( \psi - \phi \right) \right] + ... \\
    ... &+ m_S C_1 \left[ \left( b + c \right) \dot{\psi} \cos \psi + d \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) \right] + ... \\
    ... &+ m_S \dot{C}_2 \left[ - \left( b + c \right) \cos \psi - d \cos \left( \psi - \phi \right) \right] + ... \\
    ... &+ m_S C_2 \left[ \left( b + c \right) \dot{\psi} \sin \psi + d \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \right] + ... \\
    ... &+ I_T \ddot{\psi} + I_S \left( \ddot{\psi} - \ddot{\phi} \right) \\
    \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_4} \right) = \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{\phi}} \right) &=& m_S \dot{C}_1 \left[ - d \sin \left( \psi - \phi \right) \right] + m_S C_1 \left[ - d \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) \right] + ... \\
    ... &+& m_S \dot{C}_2 \left[ d \cos \left( \psi - \phi \right) \right] + m_S C_2 \left[ - d \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \right] - ... \\
    ... &+ - I_S \left( \ddot{\psi} - \ddot{\phi} \right)

A força no eixo dianteiro é dadas por

.. math::
    :nowrap:

    \begin{equation}
        {\bf F}_{\rm F} = F_{x,{\rm F}} {\bf e}_x + F_{y,{\rm F}} {\bf e}_x,
    \end{equation}

que pode ser escrita como

.. math:: {\bf F}_{\rm F} = \left[ F_{x,{\rm F}} \cos \left( \psi + \delta \right) - F_{y,{\rm F}} \sin \left( \psi + \delta \right) \right] {\bf i} + \left[ F_{x,{\rm F}} \sin \left( \psi + \delta \right) + F_{y,{\rm F}} \cos \left( \psi + \delta \right) \right] {\bf j}.
    :label: ForceFront

A força no eixo traseiro é

.. math::
    :nowrap:

    \begin{equation}
        {\bf F}_{\rm R} = F_{x,{\rm R}} {\bf t}_x + F_{y,{\rm R}} {\bf t}_y
    \end{equation}

ou

.. math:: {\bf F}_{\rm R} = \left[ F_{x,{\rm R}} \cos \psi - F_{y,{\rm R}} \sin \psi \right] {\bf i} + \left[ F_{x,{\rm R}} \sin \psi + F_{y,{\rm R}} \cos \psi \right] {\bf j}.
    :label: ForceRear

A força no eixo do semirreboque é

.. math::
    :nowrap:

    \begin{equation}
        {\bf F}_{\rm M} = F_{x,{\rm M}} {\bf s}_x + F_{y,{\rm M}} {\bf s}_y
    \end{equation}

ou

.. math:: {\bf F}_{\rm M} = \left[ F_{x,{\rm M}} \cos \left( \psi - \phi \right) - F_{y,{\rm M}} \sin \left( \psi - \phi \right) \right] {\bf i} + \left[ F_{x,{\rm M}} \sin \left( \psi - \phi \right) + F_{y,{\rm M}} \cos \left( \psi - \phi \right) \right] {\bf j}.
    :label: ForceSemi

As forças generalizadas são

.. math::
    :nowrap:

    \begin{equation}
        Q_k = \sum_{j = 1} ^p {\bf F}_j \cdot \frac{\partial {\bf p}_j}{\partial q_k} \qquad \qquad \begin{array}{c} k = 1, 2, 3, 4 \\ j = {\rm F}, {\rm R}, {\rm M} \end{array}.
    \end{equation}

Ou seja,

.. math::
    :label: generalizedForces

    Q_1 &= {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_1} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_1} + {\bf F}_{\rm M} \cdot \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_1} \\
    Q_2 &= {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_2} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_2} + {\bf F}_{\rm M} \cdot \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_2} \\
    Q_3 &= {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_3} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_3} + {\bf F}_{\rm M} \cdot \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_3}, \\
    Q_4 &= {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_4} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_4} + {\bf F}_{\rm M} \cdot \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_4}.

Os pontos de aplicação das forças são localizados por


.. math::
    :label: positionForce

    {\bf p}_{{\rm F}/{\rm O}} &= \left( x + a \cos \psi \right) {\bf i} + \left( y + a \sin \psi \right) {\bf j}. \\
    {\bf p}_{{\rm R}/{\rm O}} &= \left( x - b \cos \psi \right) {\bf i} + \left( y - b \sin \psi \right) {\bf j}. \\
    {\bf p}_{{\rm M}/{\rm O}} &= \left[ x - \left( b + c \right) \cos \psi - \left( d + e \right) \cos \left( \psi - \phi \right) \right] {\bf i} + ... \\
    ... &+ \left[ y - \left( b + c \right) \sin \psi - \left( d + e \right) \sin \left( \psi - \phi \right) \right] {\bf j}.

Logo, as derivadas parciais

.. math::
    :label: termGenFor1

    \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_1} &= \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial x} = {\bf i} \\
    \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_2} &= \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial y} = {\bf j} \\
    \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_3} &= \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial \psi} = - a \sin \psi {\bf i} + a \cos \psi {\bf j} \\
    \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_4} &= \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial \phi} = 0,

.. math::
    :label: termGenFor2

    \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_1} &= \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial x} = {\bf i} \\
    \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_2} &= \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial y} = {\bf j} \\
    \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_3} &= \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial \psi} = b \sin \psi {\bf i} - b \cos \psi {\bf j} \\
    \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_4} &= \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial \phi} = 0

e

.. math::
    :label: termGenFor3

    \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_1} &= \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial x} = {\bf i} \\
    \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_2} &= \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial y} = {\bf j} \\
    \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_3} &= \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial \psi} &=& \left[ \left( b + c \right) \sin \psi + \left( d + e \right) \sin \left( \psi - \phi \right) \right] {\bf i} + ...\\
    ... &+ \left[ - \left( b + c \right) \cos \psi - \left( d + e \right) \cos \left( \psi - \phi \right) \right] {\bf j} \\
    \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_4} &= \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial \phi} = \left[ - \left( d + e \right) \sin \left( \psi - \phi \right) \right] {\bf i} + \left[ \left( d + e \right) \cos \left( \psi - \phi \right) \right] {\bf j}

Substituindo as equações :eq:`ForceFront`, :eq:`ForceRear`, :eq:`ForceSemi`, :eq:`termGenFor1`, :eq:`termGenFor2` e  :eq:`termGenFor3` nas equações :eq:`generalizedForces` temos

.. math::
    :label: generForces

    Q_1 &= F_{x,{\rm F}} \cos \left( \psi + \delta \right) + F_{x,{\rm R}} \cos \psi + F_{x,{\rm M}} \cos \left( \psi - \phi \right) -...\\
    ... &- F_{y,{\rm F}} \sin \left( \psi + \delta \right) - F_{y,{\rm R}} \sin \psi - F_{y,{\rm M}} \sin \left( \psi - \phi \right) \\
    Q_2 &= F_{x,{\rm F}} \sin \left( \psi + \delta \right) + F_{x,{\rm R}} \sin \psi + F_{x,{\rm M}} \sin \left( \psi - \phi \right) + ... \\
    ... &- F_{y,{\rm F}} \cos \left( \psi + \delta \right) + F_{y,{\rm R}} \cos \psi + F_{y,{\rm M}} \cos \left( \psi - \phi \right) \\
    Q_3 &=  F_{x,{\rm F}} a \sin \delta + F_{x,{\rm M}} \left( b + c \right) \sin \phi + ... \\
    ... &+ F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b - F_{y,{\rm M}} \left[ \left( b + c \right) \cos \phi + \left( d + e \right) \right] \\
    Q_4 &=  F_{y,{\rm M}} \left( d + e \right)

A formulação de Euler-Lagrange para este sistema é dada por

.. math:: \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_k} \right) - \frac{\partial T}{\partial q_k} = Q_k \qquad \qquad k = 1, 2, 3, 4,
    :label: lagrange

Substituindo as equações :eq:`lagrangePartialTerm`, :eq:`lagrangeTimeTerm` e :eq:`generForces` em :eq:`lagrange` temos

.. math::

    m_{T} \ddot{x} + m_S \dot{C}_1 = Q_1 \\
    m_{T} \ddot{y} + m_S \dot{C}_2 = Q_2 \\
    m_S \dot{C}_1 \left[ \left( b + c \right) \sin \psi + d \sin \left( \psi - \phi \right) \right] + m_S \dot{C}_2 \left[ - \left( b + c \right) \cos \psi - d \cos \left( \psi - \phi \right) \right] + &...& \\
    ... + I_T \ddot{\psi} + I_S \left( \ddot{\psi} - \ddot{\phi} \right) &=& Q_3 \\
    m_S \dot{C}_1 \left[ - d \sin \left( \psi - \phi \right) \right] + m_S \dot{C}_2 \left[ d \cos \left( \psi - \phi \right) \right] - I_S \left( \ddot{\psi} - \ddot{\phi} \right) = Q_4

.. math::
    :label: equationOfMotionXY

    m_{T} \ddot{x} + m_S \dot{C}_1 &=& F_{x,{\rm F}} \cos \left( \psi + \delta \right) + F_{x,{\rm R}} \cos \psi + F_{x,{\rm M}} \cos \left( \psi - \phi \right) - ...\\
    ... &-&  F_{y,{\rm F}} \sin \left( \psi + \delta \right) - F_{y,{\rm R}} \sin \psi - F_{y,{\rm M}} \sin \left( \psi - \phi \right) \\
    m_{T} \ddot{y} + m_S \dot{C}_2 &=& F_{x,{\rm F}} \sin \left( \psi + \delta \right) + F_{x,{\rm R}} \sin \psi + F_{x,{\rm M}} \sin \left( \psi - \phi \right) + ...\\
     ... &-& F_{y,{\rm F}} \cos \left( \psi + \delta \right) + F_{y,{\rm R}} \cos \psi + F_{y,{\rm M}} \cos \left( \psi - \phi \right) \\
    m_S \dot{C}_1 \left[ \left( b + c \right) \sin \psi + d \sin \left( \psi - \phi \right) \right] + m_S \dot{C}_2 \left[ - \left( b + c \right) \cos \psi - d \cos \left( \psi - \phi \right) \right] + ... \\
    ... + I_T \ddot{\psi} + I_S \left( \ddot{\psi} - \ddot{\phi} \right) = \\
    F_{x,{\rm F}} a \sin \delta + F_{x,{\rm M}} \left( b + c \right) \sin \phi + F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b - F_{y,{\rm M}} \left[ \left( b + c \right) \cos \phi + \left( d + e \right) \right] \\
    m_S \dot{C}_1 \left[ - d \sin \left( \psi - \phi \right) \right] + m_S \dot{C}_2 \left[ d \cos \left( \psi - \phi \right) \right] - I_S \left( \ddot{\psi} - \ddot{\phi} \right) = F_{y,{\rm M}} \left( d + e \right)

Substituindo as equações :eq:`constantsTimeDiff` em :eq:`equationOfMotionXY`

.. math::
    :label: eqMotSem

    \left( m_{T} + m_S \right) \ddot{x} + m_S \left[ \left( b + c \right) \sin \psi + d \sin \left( \psi - \phi \right) \right] \ddot{\psi} - m_S d \sin \left( \psi - \phi \right) \ddot{\phi} =  \\
    F_{x,{\rm F}} \cos \left( \psi + \delta \right) + F_{x,{\rm R}} \cos \psi + F_{x,{\rm M}} \cos \left( \psi - \phi \right) - ... \\
    ... - F_{y,{\rm F}} \sin \left( \psi + \delta \right) - F_{y,{\rm R}} \sin \psi - F_{y,{\rm M}} \sin \left( \psi - \phi \right) \\
    - m_S \left( b + c \right) \dot{\psi}^2 \cos \psi - m_S d \left( \dot{\psi} - \dot{\phi} \right)^2 \cos \left( \psi - \phi \right) \\
    \left( m_{T} + m_S \right) \ddot{y} - m_S \left[ \left( b + c \right) \cos \psi + d \cos \left( \psi - \phi \right) \right] \ddot{\psi} + m_S d \cos \left( \psi - \phi \right) \ddot{\phi} =  \\
    F_{x,{\rm F}} \sin \left( \psi + \delta \right) + F_{x,{\rm R}} \sin \psi + F_{x,{\rm M}} \sin \left( \psi - \phi \right) + ... \\
    ... + F_{y,{\rm F}} \cos \left( \psi + \delta \right) + F_{y,{\rm R}} \cos \psi + F_{y,{\rm M}} \cos \left( \psi - \phi \right) \\
    - m_S \left( b + c \right) \dot{\psi}^2 \sin \psi - m_S d \left( \dot{\psi} - \dot{\phi} \right)^2 \sin \left( \psi - \phi \right) \\
    m_S \left[ \left( b + c \right) \sin \psi + d \sin \left( \psi - \phi \right) \right] \ddot{x} - m_S \left[ \left( b + c \right) \cos \psi + d \cos \left( \psi - \phi \right) \right] \ddot{y} + ...\\
    ... + \left\{ m_S \left[ \left( b + c \right)^2 + 2 \left( b + c \right) d \cos \phi + d^2 \right] + I_T + I_S \right\} \ddot{\psi} - ... \\
    ... - \left\{ m_S \left[ \left( b + c \right) d \cos \phi + d^2 \right] + I_S \right\} \ddot{\phi} = ... \\
    ... F_{x,{\rm F}} a \sin \delta + F_{x,{\rm M}} \left( b + c \right) \sin \phi + F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b - F_{y,{\rm M}} \left[ \left( b + c \right) \cos \phi + \left( d + e \right) \right] - ... \\
    - m_S \left( b + c \right) d \left( \dot{\psi} - \dot{\phi} \right)^2 \sin \phi + m_S \left( b + c \right) d \dot{\psi}^2 \sin \phi\\
    - m_S d \sin \left( \psi - \phi \right) \ddot{x} + m_S d \cos \left( \psi - \phi \right) \ddot{y} - \left\{ m_S \left[ d^2 + \left( b + c \right) d \cos \phi \right] + I_S \right\} \ddot{\psi} + ... \\
    \left( m_S d^2 + I_S \right) \ddot{\phi} = ... \\
    ... F_{y,{\rm M}} \left( d + e \right) - m_S \left( b + c \right) d \dot{\psi}^2 \sin \phi \\

Os estado podem ser escolhidos como

.. math::

    {\rm z}_1 &= x \\
    {\rm z}_2 &= y \\
    {\rm z}_3 &= \psi \\
    {\rm z}_4 &= \phi \\
    {\rm z}_5 &= \dot{x} \\
    {\rm z}_6 &= \dot{y} \\
    {\rm z}_7 &= \dot{\psi} \\
    {\rm z}_8 &= \dot{\phi}

Logo, as equações de estado são

    \dot{{\rm z}}_1 &= {\rm z}_5 \\
    \dot{{\rm z}}_2 &= {\rm z}_6 \\
    \dot{{\rm z}}_3 &= {\rm z}_7 \\
    \dot{{\rm z}}_4 &= {\rm z}_8 \\
    \left( m_{T} + m_S \right) \dot{{\rm z}}_5 + m_S \left[ \left( b + c \right) \sin {\rm z}_3 + d \sin \left( {\rm z}_3 - {\rm z}_4 \right) \right] \dot{{\rm z}}_7 - m_S d \sin \left( {\rm z}_3 - {\rm z}_4 \right) \dot{{\rm z}}_8 =  \\
    F_{x,{\rm F}} \cos \left( {\rm z}_3 + \delta \right) + F_{x,{\rm R}} \cos {\rm z}_3 + F_{x,{\rm M}} \cos \left( {\rm z}_3 - {\rm z}_4 \right) - ... \\
    ... - F_{y,{\rm F}} \sin \left( {\rm z}_3 + \delta \right) - F_{y,{\rm R}} \sin {\rm z}_3 - F_{y,{\rm M}} \sin \left( {\rm z}_3 - {\rm z}_4 \right) \\
    - m_S \left( b + c \right) {\rm z}_7^2 \cos {\rm z}_3 - m_S d \left( {\rm z}_7 - {\rm z}_8 \right)^2 \cos \left( {\rm z}_3 - {\rm z}_4 \right) \\
    \left( m_{T} + m_S \right) \dot{{\rm z}}_6 - m_S \left[ \left( b + c \right) \cos {\rm z}_3 + d \cos \left( {\rm z}_3 - {\rm z}_4 \right) \right] \dot{{\rm z}}_7 + m_S d \cos \left( {\rm z}_3 - {\rm z}_4 \right) \dot{{\rm z}}_8 =  \\
    F_{x,{\rm F}} \sin \left( {\rm z}_3 + \delta \right) + F_{x,{\rm R}} \sin {\rm z}_3 + F_{x,{\rm M}} \sin \left( {\rm z}_3 - {\rm z}_4 \right) + ... \\
    ... + F_{y,{\rm F}} \cos \left( {\rm z}_3 + \delta \right) + F_{y,{\rm R}} \cos {\rm z}_3 + F_{y,{\rm M}} \cos \left( {\rm z}_3 - {\rm z}_4 \right) \\
    - m_S \left( b + c \right) {\rm z}_7^2 \sin {\rm z}_3 - m_S d \left( {\rm z}_7 - {\rm z}_8 \right)^2 \sin \left( {\rm z}_3 - {\rm z}_4 \right) \\
    m_S \left[ \left( b + c \right) \sin {\rm z}_3 + d \sin \left( {\rm z}_3 - {\rm z}_4 \right) \right] \dot{{\rm z}}_5 - m_S \left[ \left( b + c \right) \cos {\rm z}_3 + d \cos \left( {\rm z}_3 - {\rm z}_4 \right) \right] \dot{{\rm z}}_6 + ...\\
    ... + \left\{ m_S \left[ \left( b + c \right)^2 + 2 \left( b + c \right) d \cos {\rm z}_4 + d^2 \right] + I_T + I_S \right\} \dot{{\rm z}}_7 - ... \\
    ... - \left\{ m_S \left[ \left( b + c \right) d \cos {\rm z}_4 + d^2 \right] + I_S \right\} \dot{{\rm z}}_8 = ... \\
    ... F_{x,{\rm F}} a \sin \delta + F_{x,{\rm M}} \left( b + c \right) \sin {\rm z}_4 + F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b - F_{y,{\rm M}} \left[ \left( b + c \right) \cos {\rm z}_4 + \left( d + e \right) \right] - ... \\
    - m_S \left( b + c \right) d \left( {\rm z}_7 - {\rm z}_8 \right)^2 \sin {\rm z}_4 + m_S \left( b + c \right) d {\rm z}_7^2 \sin {\rm z}_4\\
    - m_S d \sin \left( \psi - \phi \right) \dot{{\rm z}}_5 + m_S d \cos \left( \psi - \phi \right) \dot{{\rm z}}_6 - \left\{ m_S \left[ d^2 + \left( b + c \right) d \cos \phi \right] + I_S \right\} \dot{{\rm z}}_7 + ... \\
    \left( m_S d^2 + I_S \right) \dot{{\rm z}}_8 = ... \\
    ... F_{y,{\rm M}} \left( d + e \right) - m_S \left( b + c \right) d \dot{\psi}^2 \sin \phi \\

Em muitas ocasiões é conveniente fazer a substituição dos estados :math:`\dot{x}` e :math:`\dot{y}` por :math:`v_{\rm T}` e :math:`\alpha_{\rm T}`. A relação entre estes pares de estados é

.. math::
    :label: stateSubstitution

    \dot{x} &= v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) \\
    \dot{y} &= v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right).

Derivando em relação ao tempo a equação :eq:`stateSubstitution` temos

.. math::
    :label: stateDiffSubstitution

    \ddot{x} &= \dot{v}_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) - v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \sin \left( \psi + \alpha_{\rm T} \right) \\
    \ddot{y} &= \dot{v}_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) + v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \cos \left( \psi + \alpha_{\rm T} \right).

Desta forma, substituindo as equações :eq:`stateDiffSubstitution` nas equações :eq:`eqMotSem` temos

.. math::
    :label: eqMovSubs

    \left( m_{T} + m_S \right) \cos \left( \psi + \alpha_{\rm T} \right) \dot{v}_{\rm T} - \left( m_{T} + m_S \right) v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) \dot{\alpha}_{\rm T} + ... \\
    + m_S \left[ \left( b + c \right) \sin \psi + d \sin \left( \psi - \phi \right) \right] \ddot{\psi} - m_S d \sin \left( \psi - \phi \right) \ddot{\phi} =  \\
    F_{x,{\rm F}} \cos \left( \psi + \delta \right) + F_{x,{\rm R}} \cos \psi + F_{x,{\rm M}} \cos \left( \psi - \phi \right) - ... \\
    ... - F_{y,{\rm F}} \sin \left( \psi + \delta \right) - F_{y,{\rm R}} \sin \psi - F_{y,{\rm M}} \sin \left( \psi - \phi \right) \\
    - m_S \left( b + c \right) \dot{\psi}^2 \cos \psi - m_S d \left( \dot{\psi} - \dot{\phi} \right)^2 \cos \left( \psi - \phi \right) + \left( m_{T} + m_S \right) v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) \dot{\psi} \\
    \left( m_{T} + m_S \right) \sin \left( \psi + \alpha_{\rm T} \right) \dot{v}_{\rm T} + \left( m_{T} + m_S \right) v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) \dot{\alpha}_{\rm T} + ... \\
    - m_S \left[ \left( b + c \right) \cos \psi + d \cos \left( \psi - \phi \right) \right] \ddot{\psi} + m_S d \cos \left( \psi - \phi \right) \ddot{\phi} =  \\
    F_{x,{\rm F}} \sin \left( \psi + \delta \right) + F_{x,{\rm R}} \sin \psi + F_{x,{\rm M}} \sin \left( \psi - \phi \right) + ...\\
    ... + F_{y,{\rm F}} \cos \left( \psi + \delta \right) + F_{y,{\rm R}} \cos \psi + F_{y,{\rm M}} \cos \left( \psi - \phi \right) \\
    - m_S \left( b + c \right) \dot{\psi}^2 \sin \psi - m_S d \left( \dot{\psi} - \dot{\phi} \right)^2 \sin \left( \psi - \phi \right) - \left( m_{T} + m_S \right) v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) \dot{\psi}\\
    - m_S \left[ \left( b + c \right) \sin \alpha_{\rm T} + d \sin \left( \alpha_{\rm T} + \phi \right) \right] \dot{v}_{\rm T} - m_S \left[ \left( b + c \right) v_{\rm T} \cos \alpha_{\rm T} + d v_{\rm T} \cos \left( \alpha_{\rm T} + \phi \right) \right] \dot{\alpha}_{\rm T} \\
    ... + \left\{ m_S \left[ \left( b + c \right)^2 + 2 \left( b + c \right) d \cos \phi + d^2 \right] + I_T + I_S \right\} \ddot{\psi} - \left\{ m_S \left[ \left( b + c \right) d \cos \phi + d^2 \right] + I_S \right\} \ddot{\phi} = ... \\
    ... F_{x,{\rm F}} a \sin \delta + F_{x,{\rm M}} \left( b + c \right) \sin \phi + F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b - F_{y,{\rm M}} \left[ \left( b + c \right) \cos \phi + \left( d + e \right) \right] - ... \\
    - m_S \left( b + c \right) d \left( \dot{\psi} - \dot{\phi} \right)^2 \sin \phi + m_S \left( b + c \right) d \dot{\psi}^2 \sin \phi + m_S \left[ \left( b + c \right) v_{\rm T} \cos \alpha_{\rm T} + d v_{\rm T} \cos \left( \alpha_{\rm T} + \phi \right) \right] \dot{\psi}\\
    m_S d \sin \left( \alpha_{\rm T} + \phi \right) \dot{v}_{\rm T} + m_S d v_{\rm T} \cos \left( \alpha_{\rm T} + \phi \right) \dot{\alpha}_{\rm T} - \left\{ m_S \left[ d^2 + \left( b + c \right) d \cos \phi \right] + I_S \right\} \ddot{\psi} + ... \\
    ... + \left( m_S d^2 + I_S \right) \ddot{\phi} = ... \\
    ... F_{y,{\rm M}} \left( d + e \right) - m_S \left( b + c \right) d \dot{\psi}^2 \sin \phi - m_S d v_{\rm T} \cos \left( \alpha_{\rm T} + \phi \right) \dot{\psi}

Os estados podem ser escolhidos como

.. math::

    {\rm x}_1 &= x \\
    {\rm x}_2 &= y \\
    {\rm x}_3 &= \psi \\
    {\rm x}_4 &= \phi \\
    {\rm x}_5 &= \dot{v}_{\rm T} \\
    {\rm x}_6 &= \dot{\alpha}_{\rm T} \\
    {\rm x}_7 &= \dot{\psi} \\
    {\rm x}_8 &= \dot{\phi}

Na forma matricial o sistema da equação :eq:`eqMovSubs` pode ser escrito como

.. math:: {\bf M} \left( {\bf x} \right) \dot{{\bf x}} = {\bf f} \left( {\bf x}, {\bf u} \right),
    :label: eqMovMatrix

onde o vetor de estados é

.. math:: {\bf x} = \left[ \begin{array}{c} {\rm x}_{1} \\ {\rm x}_{2} \\ {\rm x}_{3} \\ {\rm x}_{4} \\ {\rm x}_{5} \\ {\rm x}_{6} \\ {\rm x}_{7} \\ {\rm x}_{8} \end{array} \right]
    :label: stateVector

e o vetor de entradas é

.. math:: {\bf u} = \left[ \begin{array}{c} \delta \\ F_{x,{\rm F}} \\ F_{x,{\rm R}} \\ F_{x,{\rm M}} \\ F_{y,{\rm F}} \\ F_{y,{\rm R}} \\ F_{y,{\rm M}} \end{array} \right].
    :label: inputVector

A matriz :math:`{\bf M}` é dada por

.. math:: {\bf M} = \left[ \begin{array}{ccccccccc} 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & M_{55} & M_{56} & M_{57} & M_{58} \\ 0 & 0 & 0 & 0 & M_{65} & M_{66} & M_{67} & M_{68} \\ 0 & 0 & 0 & 0 & M_{75} & M_{76} & M_{77} & M_{78} \\ 0 & 0 & 0 & 0 & M_{85} & M_{86} & M_{87} & M_{88} \end{array} \right],
    :label: leftMatrix

onde os elementros são

.. math::
    :label: leftMatrixElements

    M_{55} &= \left( m_{T} + m_S \right) \cos \left( \psi + \alpha_{\rm T} \right) \\
    M_{56} &= - \left( m_{T} + m_S \right) v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) \\
    M_{57} &= m_S \left[ \left( b + c \right) \sin \psi + d \sin \left( \psi - \phi \right) \right] \\
    M_{58} &= - m_S d \sin \left( \psi - \phi \right) \\
    M_{65} &= \left( m_{T} + m_S \right) \sin \left( \psi + \alpha_{\rm T} \right) \\
    M_{66} &= \left( m_{T} + m_S \right) v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) \\
    M_{67} &= - m_S \left[ \left( b + c \right) \cos \psi + d \cos \left( \psi - \phi \right) \right] \\
    M_{68} &= m_S d \cos \left( \psi - \phi \right) \\
    M_{75} &= - m_S \left[ \left( b + c \right) \sin \alpha_{\rm T} + d \sin \left( \alpha_{\rm T} + \phi \right) \right] \\
    M_{76} &= - m_S \left[ \left( b + c \right) v_{\rm T} \cos \alpha_{\rm T} + d v_{\rm T} \cos \left( \alpha_{\rm T} + \phi \right) \right] \\
    M_{77} &= m_S \left[ \left( b + c \right)^2 + 2 \left( b + c \right) d \cos \phi + d^2 \right] + I_T + I_S \\
    M_{78} &= - m_S \left[ \left( b + c \right) d \cos \phi + d^2 \right] + I_S \\
    M_{85} &= m_S d \sin \left( \alpha_{\rm T} + \phi \right) \\
    M_{86} &= m_S d v_{\rm T} \cos \left( \alpha_{\rm T} + \phi \right) \\
    M_{87} &= - m_S \left[ d^2 + \left( b + c \right) d \cos \phi \right] + I_S \\
    M_{88} &= \left( m_S d^2 + I_S \right)

As funções são dadas por

.. math:: {\bf f} = \left[ \begin{array}{c} v_{\rm T} \cos \left(\psi + \alpha_{\rm T} \right) \\ v_{\rm T} \sin \left(\psi + \alpha_{\rm T} \right) \\ \dot{\psi} \\ \dot{\phi} \\ f_5 \\ f_6 \\ f_7 \\ f_8 \\  \end{array} \right],
    :label: functionNonlinear

onde

.. math::
    :label: functionNonlinearElements

    f_{5} &=& F_{x,{\rm F}} \cos \left( \psi + \delta \right) + F_{x,{\rm R}} \cos \psi + F_{x,{\rm M}} \cos \left( \psi - \phi \right) - ... \\
    ... &-& F_{y,{\rm F}} \sin \left( \psi + \delta \right) - F_{y,{\rm R}} \sin \psi - F_{y,{\rm M}} \sin \left( \psi - \phi \right) - ...\\
    ... &-& m_S \left( b + c \right) \dot{\psi}^2 \cos \psi - m_S d \left( \dot{\psi} - \dot{\phi} \right)^2 \cos \left( \psi - \phi \right) + \left( m_{T} + m_S \right) v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) \dot{\psi} \\
    f_{6} &=& F_{x,{\rm F}} \sin \left( \psi + \delta \right) + F_{x,{\rm R}} \sin \psi + F_{x,{\rm M}} \sin \left( \psi - \phi \right) + ... \\
    ... &+& F_{y,{\rm F}} \cos \left( \psi + \delta \right) + F_{y,{\rm R}} \cos \psi + F_{y,{\rm M}} \cos \left( \psi - \phi \right) \\
    ... &-& m_S \left( b + c \right) \dot{\psi}^2 \sin \psi - m_S d \left( \dot{\psi} - \dot{\phi} \right)^2 \sin \left( \psi - \phi \right) - \left( m_{T} + m_S \right) v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) \dot{\psi}\\
    f_{7} &=& F_{x,{\rm F}} a \sin \delta + F_{x,{\rm M}} \left( b + c \right) \sin \phi + F_{y,{\rm F}} a \cos \delta - ... \\
    ... &-& F_{y,{\rm R}} b - F_{y,{\rm M}} \left[ \left( b + c \right) \cos \phi + \left( d + e \right) \right] - ... \\
    ... &-& m_S \left( b + c \right) d \left( \dot{\psi} - \dot{\phi} \right)^2 \sin \phi + m_S \left( b + c \right) d \dot{\psi}^2 \sin \phi + ... \\
    ... &+& m_S \left[ \left( b + c \right) v_{\rm T} \cos \alpha_{\rm T} + d v_{\rm T} \cos \left( \alpha_{\rm T} + \phi \right) \right] \dot{\psi}\\
    f_{8} = F_{y,{\rm M}} \left( d + e \right) - m_S \left( b + c \right) d \dot{\psi}^2 \sin \phi - m_S d v_{\rm T} \cos \left( \alpha_{\rm T} + \phi \right) \dot{\psi}.

Portanto, o modelo não linear de veículo articulado é dado pelas equações :eq:`eqMovMatrix`, :eq:`stateVector`, :eq:`inputVector`, :eq:`leftMatrix`, :eq:`leftMatrixElements`, :eq:`functionNonlinear` e :eq:`functionNonlinearElements`.

*Linearization*

A equação não linear :eq:`eqMovMatrix` pode ser linearizada e escrita na forma matricial

.. math:: {\bf E}\dot{{\bf x}} = {\bf A} {\bf x} + {\bf B} {\bf u}.
    :label: linearEq

A linearização deste sistema pode ser feita para um veículo se movimentando em linha reta com uma determinada velocidade :math:`v_{\rm T} > 0`. Neste caso, os estados no ponto de operação são dados por

.. math::
    :label: opStates

    {\rm x}_{1,op} &= x_{op} = ? \\
    {\rm x}_{2,op} &= y_{op} = ? \\
    {\rm x}_{3,op} &= \psi_{op} = 0 \\
    {\rm x}_{4,op} &= \phi_{op} = 0 \\
    {\rm x}_{5,op} &= v_{{\rm T},op} = v_{{\rm T},0} \\
    {\rm x}_{6,op} &= \alpha_{{\rm T},op} = 0 \\
    {\rm x}_{7,op} &= \dot{\psi}_{op} = 0, \\
    {\rm x}_{8,op} &= \dot{\phi}_{op} = 0.

OBS: Os estados :math:`x` e :math:`y` não influenciam a dinâmica do sistema.

O vetor do ponto de operação dos estados é

.. math:: {\bf x}_{op} = \left[ \begin{array}{c} {\rm x}_{1,op} \\ {\rm x}_{2,op} \\ {\rm x}_{3,op} \\ {\rm x}_{4,op} \\ {\rm x}_{5,op} \\ {\rm x}_{6,op} \\ {\rm x}_{7,op} \\ {\rm x}_{8,op} \end{array} \right].

Neste ponto de operação dos estados, o ponto de operação da derivada dos estados é dada por

.. math::
    :label: opDiffStates

    \dot{{\rm x}}_{1,op} &= \dot{x}_{op} = v_{{\rm T},0} \\
    \dot{{\rm x}}_{2,op} &= \dot{y}_{op} = 0 \\
    \dot{{\rm x}}_{3,op} &= \dot{\psi}_{op} = 0 \\
    \dot{{\rm x}}_{4,op} &= \dot{\phi}_{op} = 0 \\
    \dot{{\rm x}}_{5,op} &= \dot{v}_{{\rm T},op} = 0 \\
    \dot{{\rm x}}_{6,op} &= \dot{\alpha}_{{\rm T},op} = 0 \\
    \dot{{\rm x}}_{7,op} &= \ddot{\psi}_{op} = 0, \\
    \dot{{\rm x}}_{8,op} &= \ddot{\phi}_{op} = 0.

O vetor do ponto de operação da derivada dos estados é

.. math:: \dot{{\bf x}}_{op} = \left[ \begin{array}{c} \dot{{\rm x}}_{1,op} \\ \dot{{\rm x}}_{2,op} \\ \dot{{\rm x}}_{3,op} \\ \dot{{\rm x}}_{4,op} \\ \dot{{\rm x}}_{5,op} \\ \dot{{\rm x}}_{6,op} \\ \dot{{\rm x}}_{7,op} \\ \dot{{\rm x}}_{8,op} \end{array} \right].

O ponto de operação das entradas é

.. math::
    :label: opInput

    \delta_{op} &= 0 \\
    F_{x,{\rm F},op} &= 0 \\
    F_{x,{\rm R},op} &= 0 \\
    F_{x,{\rm M},op} &= 0 \\
    F_{y,{\rm F},op} &= 0 \\
    F_{y,{\rm R},op} &= 0 \\
    F_{y,{\rm M},op} &= 0.

O vetor do ponto de operação das entradas é

.. math:: {\bf u}_{op} = \left[ \begin{array}{c} \delta_{op} \\ F_{x,{\rm F},op} \\ F_{x,{\rm R},op} \\ F_{y,{\rm F},op} \\ F_{y,{\rm R},op} \end{array} \right].

Expandindo em série de Taylor a equação :eq:`eqMovMatrix` e truncando nos termos de primeira ordem temos

.. math:: \nabla{\bf g}\left( {\bf x}_{op}, \dot{{\bf x}}_{op},  {\bf u}_{op} \right) \left[ \begin{array}{c} {\bf x} - {\bf x}_{op} \\ \dot{{\bf x}} - \dot{{\bf x}}_{op} \\ {\bf u} - {\bf u}_{op} \end{array} \right] = \nabla{\bf f}\left( {\bf x}_{op}, \dot{{\bf x}}_{op},  {\bf u}_{op} \right) \left[ \begin{array}{c} {\bf x} - {\bf x}_{op} \\ \dot{{\bf x}} - \dot{{\bf x}}_{op} \\ {\bf u} - {\bf u}_{op} \end{array} \right],
    :label: TaylorSeries

onde

.. math:: {\bf g} = \left[ \begin{array}{c} {\rm g}_1 \\ {\rm g}_2 \\ {\rm g}_3 \\ {\rm g}_4 \\ {\rm g}_5 \\ {\rm g}_6 \\ {\rm g}_7 \\ {\rm g}_8 \end{array} \right].
    :label: functiong

é o lado esquerdo da equação :eq:`eqMovMatrix`, enquanto que o lado direito é dado por

.. math:: {\bf f} = \left[ \begin{array}{c} {\rm f}_1 \\ {\rm f}_2 \\ {\rm f}_3 \\ {\rm f}_4 \\ {\rm f}_5 \\ {\rm f}_6 \\ {\rm f}_7 \\ {\rm f}_8 \end{array} \right].
    :label: functionf

O jacobiano das funções :eq:`functiong` e :eq:`functionf` é

.. math::

    \nabla {\bf g} &= \left[ \begin{array}{ccccccc} \frac{ \partial g_1 }{ \partial x } & \hdots & \frac{ \partial g_1 }{ \partial \dot{x} } & \hdots &\frac{ \partial g_1 }{ \partial \delta } & \hdots & \frac{\partial g_1}{\partial F_{y,{\rm R}}} \\ \vdots &  & \vdots &  & \vdots &  & \vdots  \\ \frac{ \partial g_8 }{ \partial x } & \hdots & \frac{ \partial g_8 }{ \partial \dot{x} } & \hdots &\frac{ \partial g_8 }{ \partial \delta } & \hdots & \frac{\partial g_8}{\partial F_{y,{\rm R}}} \end{array} \right] \\
    \nabla {\bf f} &= \left[ \begin{array}{ccccccc} \frac{ \partial f_1 }{ \partial x } & \hdots & \frac{ \partial f_1 }{ \partial \dot{x} } & \hdots &\frac{ \partial f_1 }{ \partial \delta } & \hdots & \frac{\partial f_1}{\partial F_{y,{\rm R}}} \\ \vdots &  & \vdots &  & \vdots &  & \vdots  \\ \frac{ \partial f_8 }{ \partial x } & \hdots & \frac{ \partial f_8 }{ \partial \dot{x} } & \hdots &\frac{ \partial f_8 }{ \partial \delta } & \hdots & \frac{\partial f_8}{\partial F_{y,{\rm R}}} \end{array} \right].

Logo, as equações de movimento linearizadas são dadas por

.. math::
    (m_T + m_S) \dot{v}_{\rm T} &= F_{x,{\rm F}} + F_{x,{\rm R}} + F_{x,{\rm M}} \\
    (m_T + m_S) v_{{\rm T},0} \dot{\alpha}_{\rm T} - m_S (b + c + d) \ddot{\psi} + m_S d \ddot{\phi} &= F_{y,{\rm F}} + F_{y,{\rm R}} + F_{y,{\rm M}} -  (m_S + m_T) v_{{\rm T},0} \dot{\psi} \\
    - m_S (b + c + c) v_{{\rm T},0} \dot{\alpha}_{\rm T} + \left[ I_T + I_S + m_S  (b + c + d)^2 \right] \ddot{\psi} - \left[ I_S + m_S (d^2 + (b + c) d) \right] \ddot{\phi} = \\
    F_{y,{\rm F}} a - F_{y,{\rm R}} b - F_{y,{\rm M}} (b + c + d + e) + m_S (b + c + d) v_{{\rm T},0} \dot{\psi} \\
    m_S d v_{{\rm T},0} \dot{\alpha}_{\rm T} - (I_S + m_S (d^2 + (b + c) d)) \ddot{\psi} + (m_S d^2 + I_S) \ddot{\phi} = F_{y,{\rm M}} (d + e) - m_S d v_{{\rm T},0} \dot{\psi}

É possível notar que quando o somatório das forças longitudinais é zero a velocidade :math:`v_{\rm T}` se mantém constante.

As equações de estado são dadas por

.. math::
    :label: linearEqMot

    \dot{\rm x}_1 &= {\rm x}_5 \\
    \dot{\rm x}_2 &= ({\rm x}_3 + {\rm x}_6) v_{{\rm T},0} \\
    \dot{\rm x}_3 &= {\rm x}_7 \\
    \dot{\rm x}_4 &= {\rm x}_8 \\
    (mS + mT) \dot{\rm x}_5 &= FxF + FxM + FxR
    (m_T + m_S) v_{{\rm T},0} \dot{\rm x}_6 - m_S (b + c + d) \dot{\rm x}_7 + m_S d \dot{\rm x}_8 = F_{y,{\rm F}} + F_{y,{\rm R}} + F_{y,{\rm M}} -  (m_S + m_T) v_{{\rm T},0} {\rm x}_7
    - m_S (b + c + c) v_{{\rm T},0} \dot{\rm x}_6 + \left[ I_T + I_S + m_S  (b + c + d)^2 \right] \dot{\rm x}_7 - \left[ I_S + m_S (d^2 + (b + c) d) \right] \dot{\rm x}_8 = \\
    F_{y,{\rm F}} a - F_{y,{\rm R}} b - F_{y,{\rm M}} (b + c + d + e) + m_S (b + c + d) v_{{\rm T},0} {\rm x} \\
    m_S d v_{{\rm T},0} \dot{\rm x}_6 - (I_S + m_S (d^2 + (b + c) d)) \dot{\rm x}_7 + (m_S d^2 + I_S) \dot{\rm x}_8 = F_{y,{\rm M}} (d + e) - m_S d v_{{\rm T},0} {\rm x}_7

Escrevendo a equação :eq:`linearEqMot` na forma matricial da pela equação :eq:`linearEq` a matriz :math:`{\bf E}` é dada por

.. math:: {\bf E} = \left[ \begin{array}{ccccccccc} 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & E_{55} & E_{56} & E_{57} & E_{58} \\ 0 & 0 & 0 & 0 & E_{65} & E_{66} & E_{67} & E_{68} \\ 0 & 0 & 0 & 0 & E_{75} & E_{76} & E_{77} & E_{78} \\ 0 & 0 & 0 & 0 & E_{85} & E_{86} & E_{87} & E_{88} \end{array} \right],
    :label: matrixE

onde os elementros são

.. math::

    E_{55} &= \left( m_{T} + m_S \right) \\
    E_{56} &= 0 \\
    E_{57} &= 0 \\
    E_{58} &= 0 \\
    E_{65} &= 0 \\
    E_{66} &=  \left( m_S + m_T \right) v_{{\rm T},0} \\
    E_{67} &= -m_S \left(b + c + d \right) \\
    E_{68} &= d m_S \\
    E_{75} &= 0 \\
    E_{76} &= -m_S v_{{\rm T},0} \left( b + c + d \right) \\
    E_{77} &= I_T + I_S + m_S (b + c + d)^2 \\
    E_{78} &= - I_S - m_S \left[ d^2 + (b + c) d \right] \\
    E_{85} &= 0 \\
    E_{86} &= d m_S v_{{\rm T},0} \\
    E_{87} &= - I_S - m_S \left[ d^2 + (b + c) d \right] \\
    E_{88} &= m_S d^2 + I_S

A matriz dinâmica do sistema é dada por

.. math::
    :label: matrixA

    {\bf A} = \left[ \begin{array}{cccccccc} 0 & 0 &  0 & 0 & 1 &  0 &                 0 & 0 \\
                                0 & 0 & v_{{\rm T},0} & 0 & 0 & v_{{\rm T},0} &                 0 & 0 \\
                                0 & 0 &  0 & 0 & 0 &  0 &                 1 & 0 \\
                                0 & 0 &  0 & 0 & 0 &  0 &                 0 & 1 \\
                                0 & 0 &  0 & 0 & 0 &  0 &                 0 & 0 \\
                                0 & 0 &  0 & 0 & 0 &  0 &     -(m_S + m_T) v_{{\rm T},0} & 0 \\
                                0 & 0 &  0 & 0 & 0 &  0 & m_S (b + c + d) v_{{\rm T},0} & 0 \\
                                0 & 0 &  0 & 0 & 0 &  0 &          - m_S d v_{{\rm T},0} & 0 \end{array} \right]

.. math::
    :label: matrixB

    {\bf B} = \left[ \begin{array}{cccccccc}
        0 & 0 & 0 & 0 & 0 &  0 &               0 \\
        0 & 0 & 0 & 0 & 0 &  0 &               0 \\
        0 & 0 & 0 & 0 & 0 &  0 &               0 \\
        0 & 0 & 0 & 0 & 0 &  0 &               0 \\
        0 & 1 & 1 & 1 & 0 &  0 &               0 \\
        0 & 0 & 0 & 0 & 1 &  1 &               1 \\
        0 & 0 & 0 & 0 & a & - b & - (b + c + d + e) \\
        0 & 0 & 0 & 0 & 0 &  0 &           d + e \end{array} \right]

Portanto, o modelo linear de veículo articulado é dado pela equação :eq:`linearEq` com as matrizes dadas pelas equações :eq:`matrixE`, :eq:`matrixA` e :eq:`matrixB`.


*Slip angles*

A velocidade no eixo dianteiro é dada por

.. math:: {\bf v}_{\rm F} = {\bf v}_{\rm T} + {\bf w}_T \wedge {\bf r}_{{\rm F}/{\rm T}},

onde :math:`{\bf r}_{{\rm F}/{\rm T}}` é o vetor posição do ponto :math:`{\rm F}` em relação ao ponto :math:`{\rm T}`. Logo,

.. math:: {\bf v}_{\rm F} = \left( \dot{x} - a \dot{\psi} \sin \psi \right) {\bf i} + \left( \dot{y} + a \dot{\psi} \cos \psi \right) {\bf j}.
    :label: velocityVectorFront

A velocidade no eixo traseiro é

.. math:: {\bf v}_{\rm R} = {\bf v}_{\rm T} + {\bf w}_T \wedge {\bf r}_{{\rm R}/{\rm T}},

onde :math:`{\bf r}_{{\rm R}/{\rm T}}` é o vetor posição do ponto :math:`{\rm R}` em relação ao ponto :math:`{\rm T}`. Logo,

.. math:: {\bf v}_{\rm R} = \left( \dot{x} + b \dot{\psi} \sin \psi \right) {\bf i} + \left( \dot{y} -b \dot{\psi} \cos \psi \right) {\bf j}
    :label: velocityVectorRear

De maneira análoga, a velocidade do eixo do semirreboque é

.. math::
    :label: velocitySemi

    {\bf v}_{\rm M} &=& \left[ \dot{x} + \left( b + c \right) \dot{\psi} \sin \psi + \left( d + e \right) \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \right] {\bf i} + ... \\
    ... &+& \left[ \dot{y} - \left( b + c \right) \dot{\psi} \cos \psi - \left( d + e \right) \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) \right] {\bf j}.

Utilizando as equações :eq:`velocityVectorFront`, :eq:`velocityVectorRear` e :eq:`velocitySemi`, os ângulos de deriva podem ser escritos como

.. math:: \alpha_{\rm F} = \arctan \left( \frac{\dot{y} + a \dot{\psi} \cos \psi}{ \dot{x} - a \dot{\psi} \sin \psi} \right) - \left( \delta + \psi \right)
    :label: slipAngleFront

.. math:: \alpha_{\rm R} = \arctan \left( \frac{\dot{y} - b \dot{\psi} \cos \psi}{ \dot{x} + b \dot{\psi} \sin \psi} \right) - \psi

.. math:: \alpha_{\rm R} = \arctan \left( \frac{\dot{y} - \left( b + c \right) \dot{\psi} \cos \psi - \left( d + e \right) \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right)}{ \dot{x} + \left( b + c \right) \dot{\psi} \sin \psi + \left( d + e \right) \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right)} \right) - \left(\psi - \phi \right)
    :label: slipAngleRear

Realizando a mudança de varíaveis proposta na equação :eq:`stateSubstitution`, os ângulos de deriva passam a ser

.. math::

    \alpha_{\rm F} &= \arctan \left( \frac{v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) + a \dot{\psi} \cos \psi}{ v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) - a \dot{\psi} \sin \psi} \right) - \left( \delta + \psi \right) \\
    \alpha_{\rm R} &= \arctan \left( \frac{v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) - b \dot{\psi} \cos \psi}{ v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) + b \dot{\psi} \sin \psi} \right) - \psi \\
    \alpha_{\rm M} &=

Simplificando, temos que

.. math::
    :label: slipAngleSimple
    
    \alpha_{\rm F} &= \arctan \left( \frac{v_{\rm T} \sin \alpha_{\rm T} + a \dot{\psi}}{ v_{\rm T} \cos \alpha_{\rm T}} \right) - \delta \\
    \alpha_{\rm R} &= \arctan \left( \frac{v_{\rm T} \sin \alpha_{\rm T} - b \dot{\psi}}{ v_{\rm T} \cos \alpha_{\rm T}} \right) \\
    \alpha_{\rm M} &= ?

Linearizando em torno do ponto de operação dado pelas equações :eq:`opStates`, :eq:`opDiffStates` e :eq:`opInput` temos

.. math::

    \alpha_{{\rm F},lin} &= \alpha_{{\rm T}} + \frac{a}{v_{{\rm T},0}} \dot{\psi} - \delta \\
    \alpha_{{\rm F},lin} &= \alpha_{{\rm T}} - \frac{b}{v_{{\rm T},0}} \dot{\psi}. \\
    \alpha_{{\rm M},lin} &=

*Linear tire*

Linearizando o valor do dos ângulos de deriva em :eq:`slipAngleSimple` no mesmo ponto de operação temos

Supondo uma lei de força linear para os pneu temos

.. math::
    :label: linearTire

    F_{y,{\rm F}} &= - K_{\rm F} \alpha_{\rm F} = - K_{\rm F} \alpha_{{\rm T}} - \frac{a K_{\rm F}}{v_{{\rm T},0}} \dot{\psi} + K_{\rm F} \delta \\
    F_{y,{\rm R}} &= - K_{\rm R} \alpha_{\rm R} =  - K_{\rm R} \alpha_{{\rm T}} + \frac{b K_{\rm R}}{v_{{\rm T},0}} \dot{\psi}

Substituir as equações :eq:`linearTire` nas equações linearizadas em :eq:`linearModel` temos

.. math::

    f_{1,lin} &= \dot{x} = v_{\rm T} \\
    f_{2,lin} &= \dot{y} = v_{{\rm T},0} \left( \psi + \alpha_{{\rm T}}\right) \\
    f_{3,lin} &= \dot{\psi} = \dot{\psi} \\
    f_{4,lin} &= \dot{v}_{\rm T} = \frac{F_{x,{\rm F}} + F_{x,{\rm R}}}{m_{T}} \\
    f_{5,lin} &= \dot{\alpha}_{{\rm T}} = - \frac{K_{\rm F} + K_{\rm R}}{m_{T} v_{{\rm T},0}} \alpha_{{\rm T}} - \frac{m_{T} v_{{\rm T},0} + \frac{a K_{\rm F} - b K_{\rm R}}{v_{{\rm T},0}}}{m_{T} v_{{\rm T},0}} \dot{\psi} + \frac{K_{\rm F}}{m_{T} v_{{\rm T},0}} \delta \\
    f_{6,lin} &= \ddot{\psi} = - \frac{a K_{\rm F} - b K_{\rm R}}{I_{T}} \alpha_{{\rm T}} - \frac{a^2 K_{\rm F} + b^2 K_{\rm R}}{I_{T}  v_{{\rm T},0}} \dot{\psi} + \frac{a K_{\rm F}}{I_{T}} \delta

Na forma matricial

.. math:: \dot{{\bf x}} = {\bf A} {\bf x} + {\bf B} {\bf \hat{u}}

Ou ainda

.. math::

    \left[ \begin{array}{c} \dot{x} \\ \dot{y} \\ \dot{\psi} \\ \dot{v}_{\rm T} \\ \dot{\alpha}_{\rm T} \\ \ddot{\psi} \end{array} \right] = \left[ \begin{array}{cccccc} 0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & v_{{\rm T},0} & 0 & v_{{\rm T},0} & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & - \frac{K_{\rm F} + K_{\rm R}}{m_{T} v_{{\rm T},0}} & - \frac{m_{T} v_{{\rm T},0} + \frac{a K_{\rm F} - b K_{\rm R}}{v_{{\rm T},0}}}{m_{T} v_{{\rm T},0}} \\ 0 & 0 & 0 & 0 & - \frac{a K_{\rm F} - b K_{\rm R}}{I_{T}} & - \frac{a^2 K_{\rm F} + b^2 K_{\rm R}}{I_{T}  v_{{\rm T},0}} \end{array} \right] \left[ \begin{array}{c} x \\ y \\ \psi \\ v_{\rm T} \\ \alpha_{\rm T} \\ \dot{\psi} \end{array} \right] + \left[ \begin{array}{ccccc} 0 & 0 & 0 & \\ 0 & 0 & 0 & \\ 0 & 0 & 0 & \\ 0 & \frac{1}{m_{T}} & \frac{1}{m_{T}} & \\ \frac{K_{\rm F}}{m_{T} v_{{\rm T},0}} & 0 & 0 & \\  \frac{a K_{\rm F}}{I_{T}} & 0 & 0 \end{array} \right] \left[ \begin{array}{c} \delta \\ F_{x,{\rm F}} \\ F_{x,{\rm R}} \end{array} \right]
