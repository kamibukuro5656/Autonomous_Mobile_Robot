#ifndef LPOINT2DFILTER
#define LPOINT2DFILTER

#include "LPoint2DArrayClass.hpp"

class Filter{
protected:
  ClassLPoint2DArray::ConstPtr lpcd;

public:
  Filter(){};
  virtual ~Filter(){};
  virtual void SetPointCloud(const ClassLPoint2DArray::ConstPtr &_input){
    lpcd = _input;
  };
  ClassLPoint2DArray::ConstPtr GetPointCloud(){
    return lpcd;
  };
  virtual void Execute(ClassLPoint2DArray &_output) = 0;
};

#endif
