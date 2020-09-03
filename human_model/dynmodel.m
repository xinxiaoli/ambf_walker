%% Final Project Dynamical Model

%% Set up Parameters
clear; clc;
syms t;
syms q_1(t) q_2(t) q_3(t) q_4(t) q_5(t) q_6(t);

total_mass = 100;
m_h = 0.5926*total_mass;
m_t = 0.1447*total_mass;
m_c = 0.0457*total_mass;
m_f = 0.0133*total_mass;

i_f = diag([0.174576, 0.173107, 0.0]) * m_f;
i_c = diag([0.05865, 0.0, 0.0]) * m_c;
i_t = diag([0.433321, 0.435854, 0.0]) * m_t;
i_h = diag([0.092407, 0.032753, 0.0]) * m_h;

l_c = norm([0.0, 0.00623, -0.41995]);
l_t = norm([0.05359, 0.00073, 0.40753]);
l_h = 0.066515*2;

c_c = norm([0.000001, -0.201606, -0.025365]);
c_t = norm([0.042817, -0.010513, 0.18499]);
c_h = 0.066515;

g = 9.8;

dq_1 = diff(q_1,t);
dq_2 = diff(q_2,t);
dq_3 = diff(q_3,t);
dq_4 = diff(q_4,t);
dq_5 = diff(q_5,t);
dq_6 = diff(q_6,t);

ddq_1 = diff(dq_1,t);
ddq_2 = diff(dq_2,t);
ddq_3 = diff(dq_3,t);
ddq_4 = diff(dq_4,t);
ddq_5 = diff(dq_5,t);
ddq_6 = diff(dq_6,t);

q = [q_1(t);q_2(t);q_3(t);q_4(t);q_5(t);q_6(t)];
dq = [dq_1(t);dq_2(t);dq_3(t);dq_4(t);dq_5(t);dq_6(t)];
ddq = [ddq_1(t);ddq_2(t);ddq_3(t);ddq_4(t);ddq_5(t);ddq_6(t)];

%% Set up DH Frames

T00  = dh(   0,    0,       0, -sym(pi)/2); %foot to ankle
T01c = dh( q_1,    0, l_c-c_c,          0); %ankle to knee center
T01  = dh( q_1,    0,    -l_c,          0); %ankle to knee
T12c = dh(-q_2,    0, l_t-c_t,          0); %knee to hip center
T12  = dh(-q_2,    0,    -l_t,          0); %knee to hip
T23c = dh( q_3,  c_h,       0,          0); %hip to hip center
T23  = dh( q_3,  l_h,       0,          0); %hip to hip
T34c = dh(-q_4,    0,     c_t,          0); %hip to knee center
T34  = dh(-q_4,    0,     l_t,          0); %hip to knee
T45c = dh( q_5,    0,     c_c,          0); %knee to ankle center
T45  = dh( q_5,    0,     l_c,          0); %knee to ankle
T56c = dh(-q_6,    0,       0,          0); %ankle to foot center
T56  = dh(-q_6,    0,       0,          0); %ankle to foot

T01  = T00*T01;
T02c = T01*T12c;
T02  = T01*T12;
T03c = T02*T23c;
T03  = T02*T23;
T04c = T03*T34c;
T04  = T03*T34;
T05c = T04*T45c;
T05  = T04*T45;
T06c = T05*T56c;
T06  = T05*T56;

%% Calculate D Matrix
j1v = squeeze(FDM(T01c(1:3,4),q,6));
j2v = squeeze(FDM(T02c(1:3,4),q,6));
j3v = squeeze(FDM(T03c(1:3,4),q,6));
j4v = squeeze(FDM(T04c(1:3,4),q,6));
j5v = squeeze(FDM(T05c(1:3,4),q,6));
j6v = squeeze(FDM(T06c(1:3,4),q,6));

kv1 = transpose(j1v)*m_c*j1v;
kv2 = transpose(j2v)*m_t*j2v;
kv3 = transpose(j3v)*m_h*j3v;
kv4 = transpose(j4v)*m_t*j4v;
kv5 = transpose(j5v)*m_c*j5v;
kv6 = transpose(j6v)*m_f*j6v;
Kv = kv1+kv2+kv3+kv4+kv5+kv6;

j1w = [[0;0;1],   [0;0;0],   [0;0;0],   [0;0;0],   [0;0;0],   [0;0;0]];
j2w = [[0;0;1],T01(1:3,3),   [0;0;0],   [0;0;0],   [0;0;0],   [0;0;0]];
j3w = [[0;0;1],T01(1:3,3),T02(1:3,3),   [0;0;0],   [0;0;0],   [0;0;0]];
j4w = [[0;0;1],T01(1:3,3),T02(1:3,3),T03(1:3,3),   [0;0;0],   [0;0;0]];
j5w = [[0;0;1],T01(1:3,3),T02(1:3,3),T03(1:3,3),T04(1:3,3),   [0;0;0]];
j6w = [[0;0;1],T01(1:3,3),T02(1:3,3),T03(1:3,3),T04(1:3,3),T05(1:3,3)];

kw1 = transpose(j1w)*T01(1:3,1:3)*i_c*transpose(T01(1:3,1:3))*j1w;
kw2 = transpose(j2w)*T02(1:3,1:3)*i_t*transpose(T02(1:3,1:3))*j2w;
kw3 = transpose(j3w)*T03(1:3,1:3)*i_h*transpose(T03(1:3,1:3))*j3w;
kw4 = transpose(j4w)*T04(1:3,1:3)*i_t*transpose(T04(1:3,1:3))*j4w;
kw5 = transpose(j5w)*T05(1:3,1:3)*i_c*transpose(T05(1:3,1:3))*j5w;
kw6 = transpose(j6w)*T06(1:3,1:3)*i_f*transpose(T06(1:3,1:3))*j6w;
Kw = kw1+kw2+kw3+kw4+kw5+kw6;

D = simplify(Kv+Kw);
vpa(D,2);

%% Calculate C Matrix
joints = 6;
derivs = FDM(D,q,joints);

chris = sym(zeros(joints,joints,joints));
for i=1:joints
    for j=1:joints
        for k=1:joints
            chris(i,j,k)=0.5*(derivs(k,j,i)+derivs(k,i,j)-derivs(i,j,k));
        end
    end
end

C = sym(zeros(3,3));
for i=1:joints
    for j=1:joints
        temp = sym(zeros(1,6));
        for k=1:joints
            temp(k) = q(k)*chris(k,i,j);
        end
        C(j,i) = sum(temp);
    end
end
C=simplify(C);
vpa(C,2);

%% Calculate G Matrix
P1 = m_c*g*(c_c*cos(q_2));
P2 = m_t*g*(l_c*cos(q_2)+c_t*cos(q_2-q_3));
P3 = m_h*g*(l_c*cos(q_2)+l_t*cos(q_2-q_3));
P4 = m_t*g*(l_c*cos(q_2)+l_t*cos(q_2-q_3)-c_t*cos(q_2-q_3+q_4));
P5 = m_c*g*(l_c*cos(q_2)+l_t*cos(q_2-q_3)-l_t*cos(q_2-q_3+q_4)-c_c*cos(q_2-q_3+q_4-q_5));
P6 = m_f*g*(l_c*cos(q_2)+l_t*cos(q_2-q_3)-l_t*cos(q_2-q_3+q_4)-l_c*cos(q_2-q_3+q_4-q_5));

P = P1+P2+P3+P4+P5+P6;
G = simplify(functionalDerivative(P,q));
vpa(G,2);

%% Calculate Tau
tau = simplify(D*ddq + C*dq + G);
%vpa(tau,2);
% Printing this out causes issues in the MATLAB terminal

%% Helper Functions
function frame = dh(th,d,a,al)
    zerM = zeros(1,size(th,2));
    oneM = ones(1,size(th,2));
    frame = sym('frame',[4,4,size(th,2)]);
    frame(1,:,:) = [cos(th);-sin(th).*cos(al); sin(th).*sin(al);a.*cos(th)];
    frame(2,:,:) = [sin(th); cos(th).*cos(al);-cos(th).*sin(al);a.*sin(th)];
    frame(3,:,:) = [   zerM;          sin(al);          cos(al);         d];
    frame(4,:,:) = [   zerM;             zerM;             zerM;      oneM];
end

function deriv = FDM(mat,vars,numvars)
    deriv = sym(zeros(size(mat,1),size(mat,2),numvars));
    for i = 1:size(mat,1)
        for j = 1:size(mat,2)
            deriv(i,j,:) = functionalDerivative(mat(i,j),vars);
        end
    end
end