# runtime-manager-lite
* Autoware의 runtime manager를 새로 구성하여 현재 개발하고 있는 자율주행자동차에 필요한 노드을 쉽게 구동할 수 있도록 구성하였습니다.
* 제 레포지토리 중 qt lidar detection 노드를 구동 할 수 있도록 추가하였습니다.
* 기존 runtime manager에는 없는 lane change 기능을 제 레포지토리 중 Lane Change Manager 노드를 추가하여 가능하도록 하였습니다.
* runtime manager에 없는 자율주행을 위해 필요한 노드가 제대로 켜져있는지, 죽었는지 체크하는 기능을 추가할 예정입니다.
* runtime manager를 구동하므로써 사용하지 않는 기능으로 인한 리소스 낭비를 최소화 하였습니다.

# Run
```sh
$ rosrun autoware_lite runtime_manager_lite.py
``` 

# Result
1. GUI (nodes)
<img src="autoware_lite/img/nodes.png" width="60%" height="60%">

2. GUI (param & alive check)
<img src="autoware_lite/img/alive check.png" width="60%" height="60%">

1. GUI (resource check)
<img src="autoware_lite/img/resoucre check.png" width="60%" height="60%">
