#include "InterpolateFilter.hpp"

bool InterpolateFilter::FindInterpolatePoint(const localization_msgs::LPoint2D &_cp,
      const localization_msgs::LPoint2D &_pp, localization_msgs::LPoint2D &_np, bool &_inserted)
{
  double dx = _cp.x - _pp.x;
  double dy = _cp.y - _pp.y;
  double l = std::sqrt(dx * dx + dy * dy);

  if(dis + l < dthre_s){
    dis += l;
    return false;
  }  
  else if(dis + l >= dthre_l){
    _np.x = _cp.x;
    _np.y = _cp.y;
  }
  else{
    double ratio = (dthre_s - dis) / l;
    double x2 = dx * ratio + _pp.x;
    double y2 = dy * ratio + _pp.y;
    _np.x = x2;
    _np.y = y2;
    _inserted = true;
  }

  return true;
}

void InterpolateFilter::Execute(ClassLPoint2DArray &_output)
{
  _output.pcd.header = lpcd->pcd.header;
  _output.pcd.atd = lpcd->pcd.atd;
  const std::vector<localization_msgs::LPoint2D> &ref_ps = lpcd->pcd.points;
  std::vector<localization_msgs::LPoint2D> &out_ps = _output.pcd.points;

  if(ref_ps.size() == 0)
    return;

  out_ps.clear();
  out_ps.reserve(ref_ps.size() * 2); //適当に確保

  dis = 0.0;
  localization_msgs::LPoint2D rp = ref_ps[0];
  localization_msgs::LPoint2D prev_rp = rp;
  localization_msgs::LPoint2D np = rp;
  out_ps.emplace_back(np);

  for(size_t i = 1; i < ref_ps.size(); i++){
    rp = ref_ps[i];
    bool inserted = false;
    
    bool exist = FindInterpolatePoint(rp, prev_rp, np, inserted);

    if(exist){
      out_ps.emplace_back(np);
      prev_rp = np;
      dis = 0.0;
      if(inserted)
        i--;
    }
    else
      prev_rp = rp;
  }
  _output.CalcNormalVector();
}
