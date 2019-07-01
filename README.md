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
