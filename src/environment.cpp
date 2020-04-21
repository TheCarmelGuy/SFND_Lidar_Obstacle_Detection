/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include  <unistd.h> 



std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}



void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, 
const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // PARAMS
    //---------
    float voxelRes = 0.5;

    Eigen::Vector4f minPoint;
    minPoint<<-10.0,-6.0, -2.0, 1; 
     
    Eigen::Vector4f maxPoint;
    maxPoint<<40.0f,6.0f, 100.0f, 1; 

    int debugBox = 0;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterInputCloud = pointProcessorI->FilterCloud(inputCloud, voxelRes, minPoint, maxPoint);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->SegmentPlane(filterInputCloud,50, 0.2);


    
    renderPointCloud(viewer,segmentedCloud.first,"Ground", Color(0,1,0));
//    renderPointCloud(viewer,segmentedCloud.second,"Obstacles", Color(1,0,0));


     std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>  clusters = pointProcessorI->Clustering(segmentedCloud.second, 0.6, 3 , 100000);     
 

    
    int clusterId = 0;
    std::vector<Color> colors = {Color(0.4,0.56,0.4),Color(1,0.56,0.4),Color(1,1,0), Color(0,1,1), Color(1,0,1), Color(0.5,0,0.23), Color(0,0.5,0), Color(0.24,0.3,0.4)
    ,Color(0.4,0.2,0.4), Color(0.56,0.1,0.0) };
   for(auto itr = clusters.begin(); itr<clusters.end(); itr++)
   {

      //render Cluster
      renderPointCloud(viewer,*itr, "Clustered Obstacle "+std::to_string(clusterId), 
colors[clusterId%colors.size()] );


      //render bounding boxa


      Box box = pointProcessorI->BoundingBox(*itr);
      renderBox(viewer,box,clusterId);
     clusterId++;

    }


}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();


    // PARAMS
    //---------
    float voxelRes = 0.5;

    Eigen::Vector4f minPoint;
    minPoint<<-10.0,-7, -2.0, 1; 
     
    Eigen::Vector4f maxPoint;
    maxPoint<<50.0f,7.0f, 500.0f, 1; 

    int debugBox = 0;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_2/0000000011.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterInputCloud = pointProcessorI->FilterCloud(inputCloud, voxelRes, minPoint, maxPoint);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->SegmentPlanePCL(filterInputCloud,100,0.2);


    
    renderPointCloud(viewer,segmentedCloud.first,"Ground", Color(0,1,0));
 //   renderPointCloud(viewer,segmentedCloud.second,"Obstacles", Color(1,0,0));


//     std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>  clusters = pointProcessorI->Clustering(segmentedCloud.second, 1.0, 5 , 100000);     
// 
//
//    
//    int clusterId = 0;
//    std::vector<Color> colors = {Color(0.4,0.56,0.4),Color(1,0.56,0.4),Color(1,1,0), Color(0,1,1), Color(1,0,1), Color(0.5,0,0.23), Color(0,0.5,0), Color(0.24,0.3,0.4)};
//   for(auto itr = clusters.begin(); itr<clusters.end(); itr++)
//   {
//   
//      renderPointCloud(viewer,*itr, "Clustered Obstacle "+std::to_string(clusterId), 
//colors[clusterId%colors.size()] );
//  
//     clusterId++;
//
//    }
//
//
 //  Debug Box 
    if(debugBox)  
    {
       Box box;
       box.x_min = minPoint(0);
       box.y_min = minPoint(1);
       box.z_min = minPoint(2);
       box.x_max = maxPoint(0);
       box.y_max = maxPoint(1);
       box.z_max = maxPoint(2);
       renderBox(viewer,box,3);

  }
}









void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    

    Lidar* lidarUnit = new Lidar(cars, 0); 
    
    ProcessPointClouds<pcl::PointXYZ> * pointProcessor = new ProcessPointClouds<pcl::PointXYZ>(); 
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidarUnit->scan();
    //renderRays(viewer, lidarUnit->position, outputPointCloud);
  
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedCloud = pointProcessor->SegmentPlane(pointCloud,100,0.4);

   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  clusters = pointProcessor->Clustering(segmentedCloud.second, 2.0, 0 , 100);     
 

   if(!renderScene)
   {
     renderPointCloud(viewer, segmentedCloud.second, "obs",Color(1,0,0));
   }



   int clusterId = 0;
   std::vector<Color> colors = {Color(1,1,0), Color(0,1,1), Color(1,0,1), Color(0.5,0,0), Color(0,0.5,0), Color(0,0,0.5)};
   for(auto itr = clusters.begin(); itr<clusters.end(); itr++)
   {

  
     renderPointCloud(viewer,*itr, "clustered Obs "+std::to_string(clusterId), colors[clusterId] );
     BoxQ box = pointProcessor->PcaBoundingBox(*itr); 
     renderBox(viewer,box, clusterId);

     clusterId++;

   }
   renderPointCloud(viewer, segmentedCloud.first , "ground",Color(0,1,0));

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer);
   // cityBlock(viewer);


    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    while (!viewer->wasStopped ())
    {
      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
     inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);
        
      streamIterator++;
      if(streamIterator == stream.end())
        streamIterator = stream.begin();
    
      viewer->spinOnce ();
    } 
}
