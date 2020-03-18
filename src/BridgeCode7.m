%Bridge Analysis Code version 7
%Set up parameters
load('TrussDesignN_NicoleKathleenEsther_A5.mat','C','Sx','Sy','X','Y')

L= [0;0;0;0;0;0;0;0;0;0;0;0;0;0;1;0;0;0;0;0];
%Calculate Number of Joints and Members
[Joints, Members] = size(C);

%Create Matrix to find Non-Zero Indices
K = zeros(2, Members);
q = 1;

while q < (Members + 1)
    K(:, q)= find(C(:,q));
    q = q + 1;
end

%Create x-diff matrix and y-diff matrix

XDiff = zeros(Joints, Members);
YDiff = zeros(Joints, Members);
q = 1;
radius =zeros(1, Members);

while q < (Members + 1)
    Xlength = (X(K(1,q)))-(X(K(2,q)));
    Ylength = (Y(K(1,q)))-(Y(K(2,q)));
    radius(1, q) = sqrt((Xlength^2)+(Ylength^2));
    
    XDiff(K(1,q), q) = (X(K(1,q)) - X(K(2,q)))/radius(1,q) ;
    XDiff(K(2,q), q) = (X(K(2,q)) - X(K(1,q)))/radius(1,q) ;
    
    YDiff(K(1,q), q) = (Y(K(1,q)) - Y(K(2,q)))/radius(1,q) ;
    YDiff(K(2,q), q) = (Y(K(2,q)) - Y(K(1,q)))/radius(1,q) ;
    
    q = q+1;
end

%Allocate A matrix
A = zeros((2*Joints), Members + 3);

%Fill in the Matrix
a = 1;
b = Members;

while a < (2*Joints + 1)
    if a < Joints + 1
          A(a,1:b) = XDiff(a,:);
          A(a, (b+1):(b+3)) = Sx(a, 1:3);
    end
        a = a + 1;
end

d = 1;
c = Joints + 1;
while c < 2*Joints + 1
     A(c,1:b) = YDiff(d,:);
     A(c, (b+1):(b+3)) = Sy(d, 1:3);
     d = d + 1;
     c = c + 1;
end

% Calculate Tensions
T = inv(-A)*L;
Class = zeros(1, Members);
n = 1;
while n < (Members + 1)
    if T(n) > 0
        Class(n) = 'T';
    elseif T(n) < 0 
        Class(n) = 'C';
    end
    n = n+1;
end

%Print to CW
fprintf('EK301, Section A5, Nicole Kwon, Kathleen Wong, Esther Ye \n')
fprintf('Load: %d N \n', (L(find(L))))
fprintf('Member forces in Newtons \n')
n = 1;
while n < (Members + 1)
    fprintf('m%d: %.3f (%s)\n', n, T(n), Class(n))
    n = n + 1;
end

%Calculate Buckling
FBuckle = zeros(Members, 1);
SR = zeros(Members,1);

h = 1;
while h < Members + 1
FBuckle(h) = (1334.8/(radius(1, h).^2));
SR(h) = T(h)/(FBuckle(h)-0.05);
h = h + 1;
end
SRMax = min(SR);
Ffail = (L(find(L)))/SRMax;

Cost = (10*Joints) + (sum(radius));
LoadCost = Ffail/Cost;
fprintf('Reaction Forces in Newtons:\n')
fprintf('Sx1: %.2f\n', T(numel(T)-3));
fprintf('Sy1: %.2f\n', T(numel(T)-2));
fprintf('Sy2: %.2f\n', T(numel(T)-1));
fprintf('Cost of Truss: $%.0f\n', Cost);
fprintf('Theoretical max load/cost ratio in N/$: %.3f\n', LoadCost);
fprintf('Max Load: %.2f\n', Ffail);

%Addition for the Final Design Report
fprintf('\nNow Calculating with Actual Straw Length\n');
%Actual Straw Length (shorten each by 0.5cm)
NewRadius = radius - 0.5;

%Calculate Buckling
NewFBuckle = zeros(Members, 1);
NewSR = zeros(Members,1);

h = 1;
while h < Members + 1
NewFBuckle(h) = 1334.8/(NewRadius(1, h).^2);
NewSR(h) = T(h)/(NewFBuckle(h)-0.05);
h = h + 1;
end
NewSRMax = min(NewSR);
NewFfail = (L(find(L)))/NewSRMax;

NewCost = (10*Joints) + (sum(NewRadius));
NewLoadCost = NewFfail/NewCost;
fprintf('Cost of Truss: $%.0f\n', NewCost);
fprintf('Calculated max load/cost ratio for actual straw length in N/$: %.3f\n', NewLoadCost);

fprintf('New Max Load: %.2f\n', NewFfail);

%Calculate Uncertainty
%Since largest uncertainty for each length is 
%Max uncertainty
MaxFBuckle = zeros(Members, 1);
MaxSR = zeros(Members,1);

h = 1;
while h < Members + 1
MaxFBuckle(h) = (1334.8/(NewRadius(1, h).^2)) + 1.45;
MaxSR(h) = T(h)/(MaxFBuckle(h)-0.05);
h = h + 1;
end
MaxSRMax = min(MaxSR);
MaxFfail = (L(find(L)))/MaxSRMax;

%Min uncertainty
MinFBuckle = zeros(Members, 1);
MinSR = zeros(Members,1);

h = 1;
while h < Members + 1
MinFBuckle(h) = 1334.8/(NewRadius(1, h).^2) - 1.45;
MinSR(h) = T(h)/(MinFBuckle(h)-0.05);
h = h + 1;
end
MinSRMax = min(MinSR);
MinFfail = (L(find(L)))/MinSRMax;

fprintf('\nWith Uncertainty Max Load will be between: %.2f and %.2f\n', MaxFfail, MinFfail);



