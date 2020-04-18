/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

//==================================================================================
// 3D Implementation


  void insertHelper3D(std::vector<float> point, int depth, int id, Node*& root)
  {

    if(root == NULL)
    {
      root = new Node(point, id);

    }
    else if((depth % 3) == 0)
    {
     
      if(root->point[0] < point[0])
      {
        insertHelper3D(point,depth+1, id, root->right);
      } 
      else 
      { 
        insertHelper3D(point,depth+1, id, root->left);
      }
    }
    else if((depth % 3) == 1)
    {
     if(root->point[1] < point[1])
      {
        insertHelper3D(point,depth+1, id, root->right);
      } 
      else 
      { 
        insertHelper3D(point,depth+1, id, root->left);
      }
    }
    else if((depth % 3) == 2)
    {
     if(root->point[2] < point[2])
      {
        insertHelper3D(point,depth+1, id, root->right);
      } 
      else 
      { 
        insertHelper3D(point,depth+1, id, root->left);
      }
    }
  }




  void insert3D(std::vector<float> point, int id)
	{
    insertHelper3D(point, 0,id, root);
  }
 


  void searchHelper3D(std::vector<int>& ids,int depth,
      std::vector<float> target, float distanceTol, Node*& root)
  {

    if(root == NULL)
    {
      return;

    }
    else if (isWithinTol3D(target,root->point,distanceTol))
    {
       if(sqrt((target[0]- root->point[0])*(target[0]- root->point[0]) +  
         (target[1]- root->point[1])*(target[1]- root->point[1]) +
         (target[2]- root->point[2])*(target[2]- root->point[2])) < distanceTol)
       {
          
     
     
         ids.push_back(root->id); 
         
         searchHelper3D(ids,depth+1, target,distanceTol, root->left);
         searchHelper3D(ids,depth+1, target,distanceTol, root->right);

      
       }
       else
       {
         if(target[depth %3] - distanceTol < root->point[depth%3])
         {
           searchHelper(ids,depth+1, target,distanceTol, root->left);
         } 
         if(target[depth %3] + distanceTol > root->point[depth%3])
         {
           searchHelper(ids,depth+1, target,distanceTol, root->right);
         }
       }
    }
   // else if((depth % 3) == 0)
   // {
   //  
   //  if(root->point[0] < target[0])
   //   {
   //   
   //     searchHelper3D(ids,depth+1, target,distanceTol, root->right);
   //   } 
   //   else 
   //   { 
   //     searchHelper3D(ids,depth+1, target,distanceTol, root->left);
   //   }
   // }
   // else if((depth % 3) == 1)
   // {
   //   if(root->point[1] < target[1])
   //   {
   //     searchHelper3D(ids,depth+1, target,distanceTol, root->right);
   //   } 
   //   else 
   //   { 
   //     searchHelper3D(ids,depth+1, target,distanceTol, root->left);
   //   }
   // }
   // else if((depth % 3) == 2)
   // {
   //   if(root->point[2] < target[2])
   //   {
   //     searchHelper3D(ids,depth+1, target,distanceTol, root->right);
   //   } 
   //   else 
   //   { 
   //     searchHelper3D(ids,depth+1, target,distanceTol, root->left);
   //   }
   // }
  }


  bool isWithinTol3D(std::vector<float> target,std::vector<float> comp, int distanceTol)
  {

    bool xTol = (comp[0] >= target[0] - distanceTol)&&(comp[0] <= target[0] + distanceTol);
    bool yTol = (comp[1] >= target[1] - distanceTol)&&(comp[1] <= target[1] + distanceTol);
    bool zTol = (comp[2] >= target[2] - distanceTol)&&(comp[2] <= target[2] + distanceTol);

    return (xTol&&yTol&&zTol);
  }





//==================================================================






  void insertHelper(std::vector<float> point, int depth, int id, Node*& root)
  {

    if(root == NULL)
    {
      fprintf(stderr, "Inserting %d \n",id);
      root = new Node(point, id);

    }
    else if((depth % 2) == 0)
    {
     
      if(root->point[0] < point[0])
      {
        insertHelper(point,depth+1, id, root->right);
      } 
      else 
      { 
        insertHelper(point,depth+1, id, root->left);
      }
    }
    else if((depth % 2) == 1)
    {
     if(root->point[1] < point[1])
      {
        insertHelper(point,depth+1, id, root->right);
      } 
      else 
      { 
        insertHelper(point,depth+1, id, root->left);
      }

    }

  }




  void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

    insertHelper(point, 0,id, root);


  }
 



  // return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

    searchHelper(ids,0,target,distanceTol,root);


		return ids;
	}

 
  void searchHelper(std::vector<int>& ids,int depth,std::vector<float> target, float distanceTol, Node*& root)
  {

    if(root == NULL)
    {
      return;

    }
    else if (isWithinTol(target,root->point,distanceTol))
    {
        
       if(sqrt((target[0]- root->point[0])*(target[0]- root->point[0]) +  
         (target[0]- root->point[0])*(target[0]- root->point[0])) < distanceTol)
       {
          
     
     
         ids.push_back(root->id); 
         
         searchHelper(ids,depth+1, target,distanceTol, root->left);
         searchHelper(ids,depth+1, target,distanceTol, root->right);

      
       }
    } 
    else if((depth % 2) == 0)
    {
     
     if(root->point[0] < target[0])
      {
      
        searchHelper(ids,depth+1, target,distanceTol, root->right);
      } 
      else 
      { 
        searchHelper(ids,depth+1, target,distanceTol, root->left);
      }
    }
    else if((depth % 2) == 1)
    {
      if(root->point[1] < target[1])
      {
        searchHelper(ids,depth+1, target,distanceTol, root->right);
      } 
      else 
      { 
        searchHelper(ids,depth+1, target,distanceTol, root->left);
      }
    }

  }


  bool isWithinTol(std::vector<float> target,std::vector<float> comp, int distanceTol)
  {

    bool xTol = (comp[0] >= target[0] - distanceTol)&&(comp[0] <= target[0] + distanceTol);
    bool yTol = (comp[1] >= target[1] - distanceTol)&&(comp[1] <= target[1] + distanceTol);

    return (xTol&&yTol);
  }

};

