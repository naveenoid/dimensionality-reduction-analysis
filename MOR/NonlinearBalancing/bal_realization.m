
function [Trans, invTrans, Wc, Wo, svd_Wc, svd_Wo] = bal_realization(yhat, xhat, n) 

% [Trans, invTrans, Wc, Wo, svd_Wc, svd_Wo] = bal_realization(yhat, xhat, n) 
% bal_realization is to obtain the balanced realization for nonlinear system 
% when the controllability and observability gramian or covariance matrix are known.
% Trans: the transformation matrix
% invTrans: the inverse of the transformation matrix
% Wc: balanced controllability gramian/covariance matrix
% Wo: balanced observability gramian/covariance matrix
% svd_Wc: singular values
% svd_Wo: singular values
% yhat: controllability gramian/covariance matrix
% xhat: observability gramian/covariance matrix
% n: Number of states


P = yhat;
Q = xhat;
n_P = rank(P);
offdiag = zeros(n,n);
for i = 1:n
   offdiag(i,n-i+1) = 1;
end

[U,T] = schur(P);
if T(1,1)<T(n,n)
   U = U*offdiag;
end

dia = diag(T);
if dia(1)<dia(n)
   dia = offdiag*dia;
end
invdia = [sqrt(inv(diag(dia(1:n_P)))) zeros(n_P,n-n_P);zeros(n-n_P,n_P) eye(n-n_P,n-n_P)];
V = U';
T1 = invdia*V;
T1*P*T1';

Qtrans = inv(T1')*Q*inv(T1);
Q11 = Qtrans(1:n_P,1:n_P);
[U1,T] = schur(Q11);
U1 = U1';
sigmasquared = U1*Q11*U1';

n_S = rank(sigmasquared);
T2 = inv([U1 zeros(n_P,n-n_P);zeros(n-n_P,n_P) eye(n-n_P,n-n_P)])';
Qtrans = inv(T2')*inv(T1')*Q*inv(T1)*inv(T2);
Q121 = Qtrans(1:n_S,n_P+1:n);
T3 = inv([eye(n_P) zeros(n_P,n-n_P);-Q121'*inv(sigmasquared(1:n_S,1:n_S)) zeros(n-n_P,n_P-n_S) eye(n-n_P)])';
Qtrans = inv(T3')*inv(T2')*inv(T1')*Q*inv(T1)*inv(T2)*inv(T3);

Qt = Qtrans(n_P+1:n,n_P+1:n);
[U2,T] = schur(Qt);
U2 = U2';
sigma3 = U2*Qt*U2';
T4 = inv([sigmasquared(1:n_S,1:n_S)^-0.25 zeros(n_S,n-n_S);zeros(n_P-n_S,n_S) eye(n_P-n_S) zeros(n_P-n_S,n-n_P);zeros(n-n_P,n_P) U2])';
T = T4*T3*T2*T1;

Trans=T;  % the transformation matrix
invTrans=inv(T);  % the inverse of the transformation matrix


Wc = T*P*T';  % balanced controllability gramian
Wo = inv(T')*Q*inv(T);  % balanced observability gramian

svd_Wc = svd(Wc);
svd_Wo = svd(Wo);



