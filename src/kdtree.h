/* \author Aaron Brown */
// Quiz on implementing kd tree



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
//  Implementation


  void insertHelper(std::vector<float> point, int depth, int id, Node*& root)
  {

    if(root == NULL)
    {
      root = new Node(point, id);

    }
    else
    {
 
      if(root->point[depth %3] < point[depth %3])
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
    insertHelper(point, 0,id, root);
  }
 


  void searchHelper(std::vector<int>& ids,int depth,
      std::vector<float> target, float distanceTol, Node*& root)
  {

   if (root != NULL)
   {
   if (isWithinTol(target,root->point,distanceTol))
    {
      float distance = sqrt((target[0]- root->point[0])*(target[0]- root->point[0]) +  
         (target[1]- root->point[1])*(target[1]- root->point[1]) +
         (target[2]- root->point[2])*(target[2]- root->point[2]));
       if(distance <= distanceTol)
       {
          
     
         ids.push_back(root->id); 
         
     
         
       } 
    
    } else
    {




    }
   
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
 
  // return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
    searchHelper(ids,0,target,distanceTol,root);
		
    return ids;
	}



  bool isWithinTol(std::vector<float> target,std::vector<float> comp, float distanceTol)
  {

    bool xTol = (comp[0] >= target[0] - distanceTol)&&(comp[0] <= target[0] + distanceTol);
    bool yTol = (comp[1] >= target[1] - distanceTol)&&(comp[1] <= target[1] + distanceTol);
    bool zTol = (comp[2] >= target[2] - distanceTol)&&(comp[2] <= target[2] + distanceTol);

    return (xTol&&yTol&&zTol);
  }



};

