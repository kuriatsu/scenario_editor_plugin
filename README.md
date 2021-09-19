# aidi_scenario_editor
This is rviz plugin to edit scenario for Ai Driving Instructure application using Rviz.  
Written in C++ and Qt.

## Requirement
* ROS (tested in melodic)

## Install/Build
No additional package required
```bash
mkdir -p catkin_make/src
cd catkin_make/src
catkin_init_workspace
git clone git@github.com:kuriatsu/scenario_editor_plugin.git
cd ..
catkin_make
```

## How to use
### Start
1. Start Rviz
```bash
roscore
rviz
```
2. Add panel  
  ![image](https://user-images.githubusercontent.com/38074802/133915935-7a360210-b3d0-4893-8e3a-894c19e1bd63.png)
  
3. Import File and click `OK`  
  ![image](https://user-images.githubusercontent.com/38074802/133915941-283cd278-59b8-49e4-9184-e62d1900b88a.png)
  
4. Add visualization topic  
  ![image](https://user-images.githubusercontent.com/38074802/133915944-a871ad13-6d1e-4255-86d9-99d83e786059.png)
  
5. Click `Scenario` tab  
  ![image](https://user-images.githubusercontent.com/38074802/133915945-5a94a579-377d-44e0-b12d-fb584d8e68df.png)
  
6. Focus on the map coodinate (It's better to inport pcl/lanelet map and publish static tf for /map) or you must adjust current view position manually.  
  ![image](https://user-images.githubusercontent.com/38074802/133915946-aefb4327-d6f4-44f8-af42-4542d844d75d.png)
  
7. Start Editing  

### Interface  
![image](https://user-images.githubusercontent.com/38074802/133916321-469b6abe-802c-4eb4-9ecb-a8ab1ef2dc29.png)

* `Layer` : Edit layer (layer=0 means 0-300 waypoints are editing target)
* `Layer Size` : Editor batch size of waypoint
* `Add Scenario` Add new scenario
* `Remove Scenario` : Remove selected scenario
* `Scenario` / `Start` / `End` : Selected scenario info 
* List view : 
  * Left : Not added errors in selected scenario
  * Right : Added errors
* `Speed Limit` : Speed Limit value of selected scenario
* Error Info : Description of selected error
  * `Highlight` : Highlight selected error in 3D view

### Edit existing scenario
* add/remove error id
  1. Double click error item in left box to add
  2. Double click error item in right box to remove

* move checkpoint position
  1. Move circle around point in 3Dview (Realigned on the current waypoints when released)  
    ![image](https://user-images.githubusercontent.com/38074802/133915951-4b9ed630-08e7-4a69-9688-0da153e797e8.png)
    
  2. If the direction is wrong, start and end id turn to Red. Swap start and end point.  
    ![image](https://user-images.githubusercontent.com/38074802/133915953-5e0f8283-ff45-40c5-be4f-36dff360952e.png)

### Add new scenario
1. Click `Add Scenario` button. (All waypoints appear)
2. Select Start point from 3D view
3. Select End point from 3D view

### Remove scenario
1. Select scenario
2. Click `Remove Scenario`

## Contribution
### Recommended Tool
* qt_designer
* qt_creator

1. Fork
2. Clone
3. Create Pull Request

## Change Log
* 2021/9/19 : First Release

## Todo
* Add doxygen
* Handle Japanese character
