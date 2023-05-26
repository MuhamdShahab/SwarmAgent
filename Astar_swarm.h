#ifndef ASTAR_SWARM_H
#define ASTAR_SWARM_H

#include <Arduino.h>
#include<iostream>
#include<queue>
#include<math.h>
#include <stack>
using namespace std;

// Author: Muhammad Shahab 
// Github: https://github.com/MuhamdShahab/
// Linkedin: https://www.linkedin.com/in/muhammad-shahab-ph923165051365/

extern const int COL; //total length/box length
extern const int ROW; //total height/box height

int** getmap(int entry,int col, int row);
void printmap(int** arr,int col, int row);
int** place_obstacle(int** arr1,int posi_x,int posi_y);
int** remove_obstacle(int** arr1, int posi_x, int posi_y);
class complete_node
{
  private:
    //int tag =0;
    int x = 0;
    int y = 0;
    int Px = 0;
    int Py = 0;
    double gnn = 0;
    double hnn = 0;
    double fnn = 0;

  public:
  complete_node(int x1, int y1,int Px1,int Py1, double gnn1, double hnn1);
  //operator Overloading
  friend bool operator== (const complete_node& n1, const complete_node& n2);
  friend bool operator!= (const complete_node& n1, const complete_node& n2);
  //getters
  //int gettag()const {return tag;}
  int getProw()const {return Py;}
  int getPcol()const {return Px;}
  int getrow()const {return y;}
  int getcol()const{return x;}
  double gethnn()const {return hnn;}
  double getgnn()const {return gnn;}
  double getfnn()const {return fnn;}
  //setters
  //void settag(int val){tag = val;}
  void setProw(int val){Py = val;}
  void setPcol(int val){Px = val;}
  void setrow(int val){ y = val;}
  void setcol(int val){ x = val;}
  void sethnn(float val){ hnn = val;}
  void setgnn(float val){ gnn = val;}
  void setfnn(float val){ fnn = val;}
  ~complete_node(){}

};
void print_complete_node(complete_node& param);
class comparefnn
{
public:
    bool operator() (const complete_node & a, const complete_node & b)
    {
        return a.getfnn() > b.getfnn();
    }
  ~comparefnn(){}
};
bool isgoalsourcevalid(int**arr7,int sx, int sy, int gx, int gy, int left_lim, int right_lim);
bool issourcegoalsame(int sx, int sy, int gx, int gy);
int** markgoalandstart(int** arr6,int sx,int sy,int gx,int gy);
double cost_calculator(double sx, double sy, double gx, double gy);
double heuristic(double sx, double sy, double gx, double gy);
bool isdestination(int sx, int sy, int gx, int gy);
bool notinclosed(complete_node &param);
void updateexistinginclosed(complete_node &param);
bool notinopened(complete_node &param);
void updateexistinginopened(complete_node &param);
bool generate_successors(int** arr7, complete_node &param,int gx,int gy, int left_lim, int right_lim);
void printopenedlist();
void printclosedlist();
void releaseopenedlistmemory();
class astar_result
{
  int** arr;
  int size = 2;
  bool success = false;
  public:
    astar_result(int ** param, int val, bool inp);
    int** getpath()
    {
      return arr;
    }
    int getsize()
    {
      return size;
    }
    bool getsuccess()
    {
      return success;
    }
    void setpath(int** param)
    {
      arr = param;
    }
    void setsize(int val)
    {
      size = val;
    }
    bool setsuccess(bool inp)
    {
      success = inp;
    }
   ~astar_result(){}
};
astar_result trackthepath(int sx, int sy, int gx,int gy);
astar_result Astar(int sx, int sy, int gx, int gy, int** arr5, int left_lim = 0, int right_lim = COL);


#endif
