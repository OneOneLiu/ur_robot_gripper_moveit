# 如何进行精准运动规划
> 是为了让UR机器人尽量走捷径，不要走那些远路，下面是个明显对比

## 当前效果

| UR Robot                        | Panda Robot                        |
| ------------------------------- | ---------------------------------- |
| ![ur](images/ur_robot_move.gif) | ![panda](images/panda_robot_move.gif) |

同样follow这个[tutorial](https://moveit.picknik.ai/main/doc/examples/motion_planning_api/motion_planning_api_tutorial.html)来做的，但是UR明显在走远路，但是panda的轨迹就很好，非常的捷径。

我需要搞清楚这是为什么。

现在已经问了一个问题：https://robotics.stackexchange.com/questions/113562/issues-with-motion-planning-path-on-ur-robot-using-moveit-api-tutorial

有一个人回答说让我做插值，但是效果仍然不行，做了插值，每个姿态之间的距离差异很小了，还是会绕远路。

