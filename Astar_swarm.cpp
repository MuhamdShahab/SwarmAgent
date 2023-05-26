#include "Astar_swarm.h"

int** getmap(int entry, int col, int row)
{
  int** arr = new int*[row];
  for (int i = 0; i < row; ++i) {
    arr[i] = new int[col];
    for (int j = 0; j < col; ++j) {
      arr[i][j] = entry;
    }
  }
  //Serial.println("Map of Ones Created Successfully");
  return arr;
}

// functions to display array
void printmap(int** arr,int col, int row)
{
  for (int i = 0; i < row; ++i) {
    Serial.print("\t");
    for (int j = 0; j < col; ++j) {
      Serial.print(arr[i][j]);
      if(j < col-1)
        Serial.print(" ");
    }
      Serial.println();
  }
}
int** place_obstacle(int** arr1,int posi_x,int posi_y)
{ 
  //node obj(posi_x,posi_y);
  arr1[posi_y][posi_x] = 0;
  return arr1;
}

int** remove_obstacle(int** arr1,int posi_x,int posi_y)
{ 
  //node obj(posi_x,posi_y);
  arr1[posi_y][posi_x] = 1;
  return arr1;
}

/////////////////////////////////////////////
///////// Astar starts here ////////////////
///////////////////////////////////////////

//each node has some features
complete_node::complete_node(int x1, int y1,int Px1, int Py1, double gnn1, double hnn1)
{
    //tag = tag1;
    x = x1;
    y = y1;
    Px =  Px1;
    Py = Py1;
    gnn = gnn1;
    hnn = hnn1;
    fnn = gnn + hnn;
}
bool operator== (const complete_node& n1, const complete_node& n2) //operator OVerloading for Equal
{
    return (n1.x == n2.x &&
            n1.y == n2.y);
}
bool operator!= (const complete_node& n1, const complete_node& n2)//Operator Overloading for Unequal
{
    return (n1.x != n2.x ||
            n1.y != n2.y);
}
void print_complete_node(complete_node &param)
{
  Serial.print("\tDestination(x,y): ");
  Serial.print(param.getcol());
  Serial.print(" , ");
  Serial.print(param.getrow());
  Serial.print("");
  Serial.print(" Source(x,y): ");
  Serial.print(param.getPcol());
  Serial.print(" , ");
  Serial.print(param.getProw());
  Serial.print(" Gnn: ");
  Serial.print(param.getgnn());
  Serial.print(" Hnn: ");
  Serial.print(param.gethnn());
  Serial.print(" Fnn: ");
  Serial.println(param.getfnn());
}

bool isgoalsourcevalid(int**arr7, int sx, int sy, int gx, int gy, int left_lim, int right_lim)
{
  if((sy >= 0) && (sy < ROW ) && (gy >= 0) && (gy < ROW) && (arr7[sy][sx] != 0) && (arr7[gy][gx] != 0) && (sx >= left_lim) && (sx <= right_lim) && (gx >= left_lim) && (gx <= right_lim))
  {
      Serial.println("Passed: Source & Goal Valid.");
      return true;
  }
  else
  {
    Serial.println("Failed: Source or Goal InValid.");
    return false;
  }
}
bool issourcegoalsame(int sx, int sy, int gx, int gy)
{
  if((sx == gx) && (sy == gy))
  {
    Serial.println("Failed: Goal and Source same");
    return false;
  }
  else
  {
    Serial.println("Passed: Goal and Source not same");
    return true;
  }
}
int** markgoalandstart(int** arr6,int sx,int sy,int gx,int gy)
{
  arr6[sy][sx] = 7; //start point
  arr6[gy][gx] = 9; //goal point
  Serial.println("Printing Map after Goal and Start Placement : { 7 is Start, 9 is Goal & 0 is obstacle }");printmap(arr6,COL, ROW);
  return arr6;
}
double cost_calculator(double sx, double sy, double gx, double gy)
{
  return sqrt(((gx-sx)*(gx-sx)) + ((gy-sy)*(gy-sy)));
}
double heuristic(double sx, double sy, double gx, double gy)
{
  return sqrt(((gx-sx)*(gx-sx)) + ((gy-sy)*(gy-sy)));
}
bool isdestination(int sx, int sy, int gx, int gy)
{
  if ((sx == gx) && (sy == gy))
  {
    return true;
  }
  else
  {
    return false;
  }
}

priority_queue<complete_node, vector<complete_node>,comparefnn> openedlist;
queue<complete_node> closedlist;
complete_node existingputar(-1,-1,-1,-1,-1,-1);
bool notinclosed(complete_node &param) //return true if not found in closed
{
  bool out = true;
  int size = closedlist.size();
  if(size >0)
  {
    int i =0;
    while(i<size)
    {
      complete_node mynode = closedlist.front();
      closedlist.pop();
      if((param.getcol() == mynode.getcol())  && (param.getrow() == mynode.getrow()) && out)
      {
        existingputar = mynode;
        out = false;
      }
      closedlist.emplace(mynode);
      i++;
    }
    return out;
  }
  else
  {
    return out;
  }
}

void updateexistinginclosed(complete_node &param) //upadtes the existing on in closed
{
  int size = closedlist.size();
  if(size >0)
  {
    int i =0;
    while(i<size)
    {
      complete_node mynode = closedlist.front();
      closedlist.pop();
      if(mynode == existingputar)
      {
        mynode = param;
      }
      closedlist.emplace(mynode);
      i++;
    }
  }
  else
  {}
}

bool notinopened(complete_node &param) //return true if not found in open
{
  bool out = true;
  int size = openedlist.size();
  if(size >0)
  {
    queue<complete_node> openedlisttemp;
    while(!openedlist.empty())
    {
      openedlisttemp.emplace(openedlist.top());
      openedlist.pop();
    }
    int i =0;
    while(i<size)
    {
      complete_node mynode = openedlisttemp.front();
      openedlisttemp.pop();
      if((param.getcol() == mynode.getcol())  && (param.getrow() == mynode.getrow()) && out)
      {
        existingputar = mynode;
        out = false;
      }
      openedlisttemp.emplace(mynode);
      i++;
    }
    while(!openedlisttemp.empty())
    {
      openedlist.emplace(openedlisttemp.front());
      openedlisttemp.pop();
    }
    return out;
  }
  else
  {
    return out;
  }
}

void updateexistinginopened(complete_node &param)
{
  int size = openedlist.size();
  if(size >0)
  {
    queue<complete_node> openedlisttemp;
    while(!openedlist.empty())
    {
      openedlisttemp.emplace(openedlist.top());
      openedlist.pop();
    }
    int i =0;
    while(i<size)
    {
      complete_node mynode = openedlisttemp.front();
      openedlisttemp.pop();
      if(mynode == existingputar)
      {
        mynode = param;
      }
      openedlisttemp.emplace(mynode);
      i++;
    }
    while(!openedlisttemp.empty())
    {
      openedlist.emplace(openedlisttemp.front());
      openedlisttemp.pop();
    }
  }
  else
  {}
}

bool generate_successors(int** arr, complete_node& papa,int gx,int gy, int left_lim, int right_lim) //generates the child for the given node and put them back in OPEN
{
  //Serial.println("1");
  double ind_hn = 0;
  int rowi =0;
  int coli = 0;
  double pathgn =0;
  int px = papa.getcol();
  int py = papa.getrow();
  bool destip = false;
  double fnn =0;
  //Serial.println("Expand_Array: ");
  for(int i = 1;i>=-1; --i) //for row iterators in child
  {
    rowi = py+i;
    if((rowi >= 0) && (rowi < ROW  ))//if rows are in -upper and lower limits
    { 
      for(int j=1;j>=-1;--j) //for column iterators
      {
        coli = px+j;
        if((coli >= left_lim) && (coli <= right_lim )) //if columns are in left/right limits
        {
          int elem = arr[rowi][coli];
          if((papa.getcol() != coli) || (papa.getrow() != rowi))
          {
            if( elem != 0)
            {
              complete_node putar(coli,rowi,papa.getcol(), papa.getrow(),0,0);
              putar.setcol(coli);putar.setrow(rowi);
              putar.setPcol(papa.getcol());putar.setProw(papa.getrow());
              //print_complete_node(putar);
              //d-i) if successor is goal stop search
              if(isdestination(putar.getcol(),putar.getrow(),gx,gy))
              {
                Serial.println("Success: The Goal matched with successor. Hurayyyy!");
                destip = true;
              }
              //d-ii) compute both g and h for successor
              else
              {
                pathgn =  papa.getgnn() + cost_calculator(px,py,coli,rowi); //succesor current cost
                putar.setgnn(pathgn);
                ind_hn = heuristic(coli,rowi,gx,gy);
                putar.sethnn(ind_hn);
                putar.setfnn(pathgn+ind_hn);

                if(notinclosed(putar)) //putar is not in closed
                {
                  if(notinopened(putar))
                  {
                    //Serial.println("New Child: Adding to Openlist");
                    openedlist.emplace(putar);
                  }
                  else //if it is in open
                  {
                    if(putar.getgnn() < existingputar.getgnn()) //already in open has cost less than new coming
                    {
                      //Serial.println("Updating exisiting in opened.");
                      updateexistinginopened(putar);
                    }
                    else
                    {
                      //skip the child
                    }
                  }
                }
                else //putar is already in closed
                {
                  if(putar.getgnn() < existingputar.getgnn()) //already in close has cost less than new coming
                  {
                    //Serial.print("Updating exisiting in closed."<<endl;
                    updateexistinginclosed(putar);
                  }
                  else
                  {
                    //skip the child
                  }
                }
              }
            }
            else
            {
              //Serial.println("\tSkipped: Encountered node as Obstacle.");
            }
          }
          else
          {
            //Serial.println("\tSkipped: Encountered node as Parent.");
          }
        }
        else
        {
          //Serial.println("Skipped : Child in X is outside limit.");
        }
      }
    }
    else
    {
      //Serial.println("Skipped: Child in Y Axis deviated.");
    }
  }
  return destip;
}
void printopenedlist()
{
  Serial.println("Opened List: ");
  int size = openedlist.size();
  if(size >0)
  {
    queue<complete_node> openedlisttemp;
    while(!openedlist.empty())
    {
      openedlisttemp.emplace(openedlist.top());
      openedlist.pop();
    }
    int i =0;
    while(i<size)
    {
      complete_node mynode = openedlisttemp.front();
      openedlisttemp.pop();
      print_complete_node(mynode);
      openedlisttemp.emplace(mynode);
      i++;
    }
    while(!openedlisttemp.empty())
    {
      openedlist.emplace(openedlisttemp.front());
      openedlisttemp.pop();
    }
  }
  else
  {
  }
}
void printclosedlist()
{
Serial.println("Closed List: ");
  int size = closedlist.size();
  if(size >0)
  {
    int i =0;
    while(i<size)
    {
      complete_node mynode = closedlist.front();
      closedlist.pop();
      print_complete_node(mynode);
      closedlist.emplace(mynode);
      i++;
    }
  }
  else
  {
  }
}

void releaseopenedlistmemory()
{
  while(!openedlist.empty())
  {
    openedlist.pop();
  }
  //Serial.println("Opened List memory released.");
}

astar_result::astar_result(int ** param, int val, bool inp)
  {
    arr = param;
    size = val;
    success = inp;
  }

astar_result trackthepath(int sx, int sy, int gx, int gy)
{
  releaseopenedlistmemory(); //released opened memory
  complete_node goal(gx,gy,closedlist.back().getcol(),closedlist.back().getrow(),(closedlist.back().getgnn()+closedlist.back().gethnn()),0);
  int cx = goal.getPcol();
  int cy = goal.getProw();
  stack<complete_node>closedstack;
  stack<complete_node>pathstack;
  //Serial.println("-----closedlist to closed stack------");
  while(!closedlist.empty()) // released closed memory
  {
    closedstack.emplace(closedlist.front());
    //print_complete_node(closedstack.top());
    closedlist.pop();
  }
  //Serial.println("Closed List memory released.");
  pathstack.emplace(goal);
  //Serial.println("Cleaned Path stack: ");    // ---------------------uncomment the 466, 467,473 to see cleaned path with full information
  //print_complete_node(pathstack.top());
  while(!closedstack.empty()) //releasing closedstack memory & collecting the cleaned nodes
  {
    if((closedstack.top().getcol() == cx) && (closedstack.top().getrow() == cy))
    {
      pathstack.emplace(closedstack.top());
      //print_complete_node(pathstack.top());
      cx = closedstack.top().getPcol();
      cy = closedstack.top().getProw();
      closedstack.pop();
    }
    else
    {
      closedstack.pop();
    }
  }
  //Serial.println("Closed stack memory released.");
  //size of the pathclosed stack cleaned version
  int path_size = pathstack.size();
  int** pathi = getmap(0,path_size,2); //generate empty zeros path array
  int i=0;
  while(i<path_size) //removing from node stack for just coordinates in 2D array
  {
    pathi[0][i] = pathstack.top().getcol();
    pathi[1][i] = pathstack.top().getrow();
    pathstack.pop();
    i++;
  }

  astar_result obj(pathi, path_size, true);
  return obj;
}

astar_result Astar(int sx, int sy, int gx, int gy, int**arr5, int left_lim, int right_lim)
{
  int i = 0;
  bool found_dest = false;
  Serial.println("A-Star Started...");
  bool gsv = isgoalsourcevalid(arr5, sx, sy, gx, gy, left_lim, right_lim); //returns bool
  bool sgs = issourcegoalsame(sx, sy, gx, gy); //returns bool
  if(gsv && sgs)
  {

    arr5 = markgoalandstart(arr5,sx,sy,gx,gy);
    //object for start node
    complete_node start(sx,sy,sx,sy,0,heuristic(sx,sy,gx,gy));
    // 1. Initialize the
    openedlist.emplace(start); //initializing Open list with the start
    while(!found_dest)
    {
      //object for closed list
      //----- ( a ) node to be expanded with least fn
      complete_node parent = openedlist.top(); 
      //----- ( b ) pop the least fn node
      openedlist.pop();
      //----- ( c ) & ( d ) Generate successors & set their parent
      found_dest = generate_successors(arr5, parent, gx, gy, left_lim, right_lim);//child expanded
      //----- ( e ) push explored to the closed list
      closedlist.emplace(parent); //sending previously expanded node in closed list.
      // printclosedlist();
      // printopenedlist();
      i++;
    }
  }
  else
  {
    Serial.println("Unsuccessful: Something went Wrong in A-start! Recheck Goal/ Source & Limits.");
    astar_result obj(getmap(0,0,0),0, false);
    return obj;
  }
  if(!found_dest)
  {
    // printclosedlist();
    // printopenedlist();
    Serial.println("Unsucessful: Goal not found in Map.");
    astar_result obj(getmap(0,0,0),0, false);
    return obj;
  }
  else
  {
    // printclosedlist();
    // printopenedlist();
    Serial.print("Success: A-Star Finished succesfully, Returning Path after ");
    Serial.print("Iteration# ");
    Serial.println(i);
    astar_result obj = trackthepath(sx, sy, gx,gy);
    return obj;
  }
}
