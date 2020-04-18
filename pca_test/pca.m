#load testPCA.txt
#testPCA = testPCA + 10;
#carCluster = testPCA

load carCluster.txt

x = mean(carCluster);
covariance = zeros(2,2);

for i = 1:length(carCluster)
   covariance = covariance + ((carCluster(i,:)-x)'*(carCluster(i,:)-x));
end

covariance = covariance/length(carCluster);
[v,lamda] = eig(covariance);

%[ee,perm] = sort(diag(lamda), "descend");
%v =v(:,perm);
%v(:,1) = v(:,1)/ee(1); 
%v(:,2) = v(:,2)/ee(2); 

%project into PCA space
transformedPoints = zeros(size(carCluster));

T = -1*v'*x';
R = v';
COB = [R T]; 

for i = 1:length(carCluster)
    
    p = carCluster(i,:)'; %transpose
    p =  R*p + T; 
   transformedPoints(i,:) = p'; 
end



