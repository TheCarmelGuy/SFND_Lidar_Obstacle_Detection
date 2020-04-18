/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;




	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

  std::unordered_set<int> inliersResult; srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

  for(int numIter = 0; numIter < maxIterations; numIter++)
  {

  
    std::unordered_set<int> currentInliers;
    srand(time(NULL));
    int  idx_1 = (int)(rand() % cloud->points.size()); 
    srand(time(NULL));
    int  idx_2 = (int)(rand() % cloud->points.size());
    srand(time(NULL));
    int  idx_3 = (int)(rand() % cloud->points.size());
  
 
    while(idx_2 == idx_1) // prevent bad line
    {
      srand(time(NULL));
      idx_2 = (int)(rand() % cloud->points.size());
    } 
 
    while((idx_2 == idx_3) || (idx_1 == idx_3)) // prevent bad line
    {
      srand(time(NULL));
      idx_3 = (int)(rand() % cloud->points.size());
    }

//    fprintf(stderr, "idx_1: %d  idx_2: %d idx_3 %d", idx_1,idx_2, idx_3);

   // Randomly sample subset and fit line
    pcl::PointXYZ p_1 = cloud->points[idx_1];
    pcl::PointXYZ p_2= cloud->points[idx_2];
    pcl::PointXYZ p_3= cloud->points[idx_3];


    currentInliers.insert(idx_1);
    currentInliers.insert(idx_2);
    currentInliers.insert(idx_3);

    //calculate line params
   
   double i = (p_2.y - p_1.y)*(p_3.z - p_1.z) - (p_2.z - p_1.z)*(p_3.y - p_1.y);  
   double j = (p_2.z - p_1.z)*(p_3.x - p_1.x) - (p_2.x - p_1.x)*(p_3.z - p_1.z);  
   double k = (p_2.x - p_1.x)*(p_3.y - p_1.y) - (p_2.y - p_1.y)*(p_3.x - p_1.x);  
 

   double a = i;
   double b = j;
   double c = k;
   double d = -(i*p_1.x + j*p_1.y + k*p_1.z);
        
   fprintf(stderr, "a: %.10f, b: %.10f, c: %.10f \n", a,b,c);  
   fprintf(stderr, "p1 x:%.4f , p1 y %.4f\n", p_1.x, p_1.y );  
   fprintf(stderr, "p2 x:%.4f , p2 y %.4f\n", p_2.x, p_2.y );  
   fprintf(stderr, "p3 x:%.4f , p3 y %.4f\n", p_3.x, p_3.y );  
        
    // Measure distance between every point and fitted line
    
   for(int pointIdx = 0; pointIdx < cloud->points.size() ;pointIdx++ )
   {

     if(currentInliers.count(pointIdx) > 0)  //ignore points
     {
        continue;
     }

     pcl::PointXYZ data = cloud->points[pointIdx];  


      float dist = fabs((a*data.x + b*data.y + c*data.z + d))/ sqrt(a*a + b*b + c*c); //no normalizing constance needed
	

//      fprintf(stderr, "a: %.4f, b: %.4f, c: %.4f \n", a,b,c);  



      fprintf(stderr, "p1 x:%.4f , p1 y %.4f\n", p_1.x, p_1.y );  
      fprintf(stderr, "p2 x:%.4f , p2 y %.4f\n", p_2.x, p_2.y );  
      fprintf(stderr, "p3 x:%.4f , p3 y %.4f\n", p_3.x, p_3.y );  
        



      fprintf(stderr, "Disance of %.4f \n", dist);  
      // If distance is smaller than threshold count it as inlier
      if (dist <= distanceTol)
      {
        fprintf(stderr, "Disance of %.4f \n", dist);  
        fprintf(stderr, "a: %.4f, b: %.4f, c: %.4f \n", a,b,c);  
        currentInliers.insert(pointIdx); 
      }
    }

    if(currentInliers.size() > inliersResult.size())
    {

      inliersResult = currentInliers;
      fprintf(stderr, "current %d , previous %d", currentInliers.size(), inliersResult.size() );  

    }
     
  }



  
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

























std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult; srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

  for(int i = 0; i < maxIterations; i++)
  {

  
    std::unordered_set<int> currentInliers;
    srand(time(NULL));
    int  idx_1 = (int)(rand() % cloud->points.size()); 
    srand(time(NULL));
    int  idx_2 = (int)(rand() % cloud->points.size());
  
 
    while(idx_2 == idx_1) // prevent bad line
    {
      srand(time(NULL));
      idx_2 = (int)(rand() % cloud->points.size());
    } 


    //fprintf(stderr, "idx_1: %d  idx_2: %d", idx_1, idx_2);

   // Randomly sample subset and fit line
    pcl::PointXYZ p_1 = cloud->points[idx_1];
    pcl::PointXYZ p_2= cloud->points[idx_2];


    currentInliers.insert(idx_1);
    currentInliers.insert(idx_2);

    //calculate line params
    float a = p_1.y - p_2.y;
    float b = p_2.x - p_1.x;
    float c = (p_1.x*p_2.y - p_2.x*p_1.y);
	
        
    // Measure distance between every point and fitted line
    
    for(int j = 0; j < cloud->points.size() ;j++ )
    {

      pcl::PointXYZ data = cloud->points[j];  
      if(currentInliers.count(j)>0) //ignore points
      {
        continue;
      }

      float d = fabs(a*data.x + b*data.y + c)/ sqrt(a*a + b*b); //no normalizing constance needed
        
      fprintf(stderr, "Disance of %.4f \n", d);  
	
      // If distance is smaller than threshold count it as inlier
      if (d <= distanceTol)
      {
        fprintf(stderr, "Disance of %.4f \n", d);  
        fprintf(stderr, "a: %.4f, b: %.4f, c: %.4f \n", a,b,c);  
        currentInliers.insert(j); 
      }
    }

    if(currentInliers.size() > inliersResult.size())
    {

      inliersResult = currentInliers;
      fprintf(stderr, "current %d , previous %d", currentInliers.size(), inliersResult.size() );  

       
      fprintf(stderr, "p1 x:%.4f , p1 y %.4f\n", p_1.x, p_1.y );  
      fprintf(stderr, "p2 x:%.4f , p2 y %.4f\n", p_2.x, p_2.y );  
    }
     
  }



  
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

//	// Create data
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
//	
//
//	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
//	std::unordered_set<int> inliers = Ransac(cloud, 1, 1.0);


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.20);




  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLine(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}



 // auto iter = inliers.begin(); 
 // cloudLine->points.push_back(cloud->points[*iter]); 
 // cloudLine->points.push_back(cloud->points[*(++iter)]); 

	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));

 // 		renderPointCloud(viewer,cloudLine,"linepoints",Color(1,0,1));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
