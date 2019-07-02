# smb_global_planner

In "__smb_planner_msgs__" -> __srv/PlannerService.srv__

```$xslt
# request fields
geometry_msgs/PoseStamped goal_pose #start pose for the planner
---
# True on success, false on planning failure
bool success
float64 x
float64 y
float64 th
```

And in "__src/planner/smb_global_planner.cpp__" add the following starting from 184 line
```$xslt
  res.x=current_state_(0);
  res.y=current_state_(1);
  res.th=current_state_(2);
```
