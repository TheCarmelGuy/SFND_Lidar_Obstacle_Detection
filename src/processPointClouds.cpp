// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <vector>
#include <algorithm>
#include <random>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();


    //serialize pcl
   pcl::PCLPointCloud2::Ptr pointcloudSerialized(new pcl::PCLPointCloud2());
   pcl::PCLPointCloud2::Ptr pointcloudSerializedFiltered(new pcl::PCLPointCloud2());
   pcl::toPCLPointCloud2(*cloud,*pointcloudSerialized);
   

    //Downsample 
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrider;
    voxelGrider.setInputCloud(pointcloudSerialized);
    voxelGrider.setLeafSize(filterRes,filterRes,filterRes);
    voxelGrider.filter(*pointcloudSerializedFiltered);

    //deserialize
   typename pcl::PointCloud<PointT>::Ptr pointcloudOut(new typename pcl::PointCloud<PointT>());
   pcl::fromPCLPointCloud2(*pointcloudSerializedFiltered,*pointcloudOut);


   //filter swath
   pcl::CropBox<PointT> boxFilter(true);
   boxFilter.setMax(maxPoint);
   boxFilter.setMin(minPoint);
   boxFilter.setInputCloud(pointcloudOut);
   boxFilter.filter(*pointcloudOut);
    



    //filter roof
  
   std::vector<int> idx; 
   pcl::CropBox<PointT> roofFilter(true);
   roofFilter.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
   roofFilter.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
   roofFilter.setInputCloud(pointcloudOut);
   roofFilter.filter(idx);
 
 
   pcl::PointIndices::Ptr Idx(new pcl::PointIndices);
   Idx->indices=idx;
   
   pcl::ExtractIndices<PointT> extract;
   extract.setInputCloud(pointcloudOut);
   extract.setIndices(Idx);
   extract.setNegative(true); 
   extract.filter(*pointcloudOut); 


  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return pointcloudOut;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  typename  pcl::PointCloud<PointT>::Ptr obsCloud (new pcl::PointCloud<PointT> ()) ; 
  typename pcl::PointCloud<PointT>::Ptr groundCloud (new pcl::PointCloud<PointT> ()); 


  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  //filter ground points
  extract.setNegative(false);
  extract.filter(*groundCloud);
 
 //filter obstacle points 
  extract.setNegative(true);
  extract.filter(*obsCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(groundCloud, obsCloud);
  return segResult;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlanePCL(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    



    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
 
   auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}





template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    

    std::vector<int> inliers;
  
  
  	// For max iterations 
  
    for(int numIter = 0; numIter < maxIterations; numIter++)
    {
  
      

      //Generate 3 random index values for inliers
       std::vector<int> currentInliers;
       std::vector<int>randomIdx(cloud->points.size());
       std::iota (std::begin(randomIdx), std::end(randomIdx), 0);  


      //generate random numbers
       unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
       auto rng = std::default_random_engine(seed);
       std::shuffle(std::begin(randomIdx), std::end(randomIdx),rng);
  
       int idx_1  = randomIdx[0];
       int idx_2 = randomIdx[1];
       int idx_3  = randomIdx[2];
      
 
        // Randomly sample subset and fit line
       PointT p_1 = cloud->points[idx_1];
       PointT p_2= cloud->points[idx_2];
       PointT p_3= cloud->points[idx_3];
  
       currentInliers.push_back(idx_1);
    
       currentInliers.push_back(idx_2);
       currentInliers.push_back(idx_3);
  
      //calculate line params
      float i = (p_2.y - p_1.y)*(p_3.z - p_1.z) - (p_2.z - p_1.z)*(p_3.y - p_1.y);  
      float j = (p_2.z - p_1.z)*(p_3.x - p_1.x) - (p_2.x - p_1.x)*(p_3.z - p_1.z);  
      float k = (p_2.x - p_1.x)*(p_3.y - p_1.y) - (p_2.y - p_1.y)*(p_3.x - p_1.x);  
   
      float a = i;
      float b = j;
      float c = k;
      float d = -(i*p_1.x + j*p_1.y + k*p_1.z);
      // Measure distance between every point and fitted line

      for(int pointIdx = 0; pointIdx < cloud->points.size() ;pointIdx++ )
       {
   
         //if(currentInliers.count(pointIdx) > 0)  //ignore points
         if(std::find(currentInliers.begin(), currentInliers.end(), pointIdx) != currentInliers.end())  //ignore points
         {
            continue;
         }
         PointT data = cloud->points[pointIdx];  
         float dist = fabs((a*data.x + b*data.y + c*data.z + d))/ sqrt(a*a + b*b + c*c); //no normalizing constance needed
   	
         // If distance is smaller than threshold count it as inlier
         if (dist <= distanceThreshold)
         {
           currentInliers.push_back(pointIdx); 
         }
       }
  
       if(currentInliers.size() > inliers.size())
       {
         inliers = currentInliers;
       }
    }



    pcl::PointIndices * id =  new pcl::PointIndices();
    id->indices = inliers;
    pcl::PointIndices::Ptr inlierIdx (id);
    auto endTime = std::chrono::steady_clock::now();
    
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inlierIdx,cloud);
    return segResult;
}


template<typename PointT>
void ProcessPointClouds<PointT>::proximityCluster(typename pcl::PointCloud<PointT>::Ptr cloud,
        int index, std::vector<int>& isMarked, std::vector<int>& cluster,KdTree* tree,float distanceTol)
{

  //TODO: Go back an make function to do this conversion
  isMarked[index] = 1;
  cluster.push_back(index);
  std::vector<float> pointOfInterest;
  pointOfInterest.push_back(cloud->points[index].x);
  pointOfInterest.push_back(cloud->points[index].y);
  pointOfInterest.push_back(cloud->points[index].z);
  std::vector<int> neighbors = tree->search(pointOfInterest,distanceTol);
  
  for(auto itr = neighbors.begin(); itr < neighbors.end(); itr++)
  {
    if(isMarked[*itr] == 0)
    {
      proximityCluster(cloud,*itr, isMarked, 
          cluster, tree, distanceTol);
   
    }
  }
      
}










template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

   

    //Generate KD Tree for 3D points
  
    std::shared_ptr<KdTree> tree = std::shared_ptr<KdTree>(new KdTree); 

    for(int pointId = 0; pointId <cloud->points.size(); pointId ++)
    {

      //TODO, make this take in intensity too? 
      auto pclPoint = cloud->points[pointId];
      std::vector<float> point;
      point.push_back(pclPoint.x);
      point.push_back(pclPoint.y);
      point.push_back(pclPoint.z);
      tree->insert(point,pointId); 


    }



    // Euclidian Clustering Logic 
    std::vector<int> isMarked(cloud->points.size(), 0); 
    std::vector<std::vector<int>> clusterIdxs;

    for(int i = 0; i < cloud->points.size(); i++)
    {
      if(isMarked[i] == 0)
      {
        std::vector<int> currentClusterIdx;
        proximityCluster(cloud,i, isMarked, currentClusterIdx,tree.get(),clusterTolerance); 
        if((currentClusterIdx.size() > minSize) && (currentClusterIdx.size() < maxSize))
        {
          clusterIdxs.push_back(currentClusterIdx);
        } 

      }
    }

    std::cout<<"Num CLUSTERSS: "<<clusterIdxs.size()<<std::endl;
    //formating data
    for(auto itr = clusterIdxs.begin() ; itr < clusterIdxs.end(); itr++)
    {


      pcl::PointIndices * pclClusterId =  new pcl::PointIndices();
      pclClusterId->indices = *itr;
      pcl::PointIndices::Ptr pclClusterIdPtr (pclClusterId);
      std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> clusterPointClouds = SeparateClouds(pclClusterIdPtr,cloud);
      clusters.push_back(clusterPointClouds.first);


    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}




//===========================
 ///PCA Utilities
template<typename PointT>
void ProcessPointClouds<PointT>::computeCentroid(typename pcl::PointCloud<PointT>::Ptr cluster, Eigen::Vector3f& pcaCentroid)
{

    Eigen::Vector3f cumSum; 
    cumSum.setZero(3);
    int numPoints = 0; //might as well fill points..sometime width not populated right  
    for(auto point: cluster->points)
    {
        cumSum(0)+=point.x;
        cumSum(1)+=point.y;
        cumSum(2)+=point.z;
        numPoints++; 
    }
    pcaCentroid = cumSum/numPoints; 
       
}

template<typename PointT>
void ProcessPointClouds<PointT>::computeNormalizedCov(typename pcl::PointCloud<PointT>::Ptr cluster,Eigen::Vector3f& pcaCentroid, Eigen::Matrix2f& pcaCovNorm)
{

   
    Eigen::Matrix2f cumSumCovariance;
    cumSumCovariance.setZero(2,2); 
    int numPoints = 0; 
    for(auto point: cluster->points)
    {
         
        //Zero-mean covariance
        Eigen::MatrixXf X(2,1);
        X(0,0) = point.x - pcaCentroid(0);
        X(1,0) = point.y - pcaCentroid(1);
        
        cumSumCovariance += X*(X.transpose());       
        numPoints++; 
    } 
    
  
    pcaCovNorm = cumSumCovariance/numPoints; 

       
}


//alternative methrod to finding boundng box

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::PcaBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

       

    
     std::cout<<"CLUSTER"<<std::endl;
    for(auto point: cluster->points)
    {


    //    std::cout<<point.x<<std::endl;

    }
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCluster(new pcl::PointCloud<pcl::PointXYZ>);

    BoxQ box;
     
    //PCA params 
    Eigen::Vector3f clusterCentroid; 
    Eigen::Matrix2f clusterCov; 
    
    computeCentroid(cluster, clusterCentroid); //find mean of clusters in 2D
    computeNormalizedCov(cluster, clusterCentroid,clusterCov); //find mean of cluster  

   
    //SVD on covariane
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(clusterCov, Eigen::ComputeEigenvectors); 
    Eigen::Matrix2f eigenvectors = eigen_solver.eigenvectors();   
   
//    std::cout<<"MEANS" <<clusterCentroid<<std::endl;
    
//    std::cout<<"EIG"<<eigenvectors<<std::endl;

 //   std::cout<<"EIG"<<eigen_solver.eigenvalues()<<std::endl;

 
 //   std::cout<<clusterCov<<std::endl; 
    
 
    //Create transformation matrix
    Eigen::Matrix3f projectionTransform(Eigen::Matrix3f::Identity());
    projectionTransform.block<2,2>(0,0) = eigenvectors.transpose(); 
    projectionTransform.block<2,1>(0,2) = -1.f* (eigenvectors.transpose()*clusterCentroid.head<2>());

    
    for(auto point: cluster->points)
    {
  
        Eigen::Vector3f eigenPoint;
        eigenPoint << point.x, point.y,1;
        auto output = projectionTransform*eigenPoint.head<3>(); 
        pcl::PointXYZ xyz;
        xyz.x = output(0,0);
        xyz.y = output(1,0);
        xyz.z = point.z; 
        projectedCluster->push_back(xyz);

        std::cout<<eigenPoint<<std::endl;
        std::cout<<output.transpose()<<std::endl;

     } 
   
  

  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*projectedCluster, minPoint, maxPoint);
  
    std::cout<<"MINIMAXX "<<minPoint.x<<" "<<maxPoint.x<<std::endl; 
    std::cout<<"MINIAXY "<<minPoint.y<<" "<<maxPoint.y<<std::endl; 
  
 const Eigen::Vector3f meanDiagonal  = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
  

 
   auto trans2d= eigenvectors*meanDiagonal.head<2>() + clusterCentroid.head<2>(); 
   Eigen::Vector3f trans3d;
   trans3d<<trans2d(0,0), trans2d(1,0),clusterCentroid(2,0);
 
    //Create transformation matrix
   Eigen::Matrix3f eigenvector3d(Eigen::Matrix3f::Identity());
   eigenvector3d.block<2,2>(0,0) = eigenvectors;
   Eigen::Quaternionf quaternionBox(eigenvector3d); 

   box.bboxQuaternion =quaternionBox;  
   box.bboxTransform = trans3d;
   box.cube_length = maxPoint.x - minPoint.x;
   box.cube_width = maxPoint.y - minPoint.y;
   box.cube_height = maxPoint.z - minPoint.z;
    
    
    return box;
}


//===========================




template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;

}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
