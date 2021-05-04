# Week 5 - Path Planning & the A* Algorithm

---

## Report

이번 과제는 DP(Dynamic Programming)을 기반으로 경로 탐색 알고리즘을 구현하는 것이다. 차량의 헤딩까지 고려하고, 각 action마다 다른 cost를 가지는 상황에서 최적의 경로를 탐색해야 한다.

큰 틀에서의 코드가 주어져 있고, 중간 중간 핵심적인 알고리즘은 TODO로써 과제로 구현하였다. 순차적으로 알아보면 다음과 같다.

---
* Line 72 ~ 76 in ![assignment.py](./assignment.py)
```python
if (y, x) == goal and value[(t, y, x)] > 0:
    # TODO: implement code.
    value[(t, y, x)] = 0
    policy[(t, y, x)] = -999
    change = True
```
* for 반복문 내에서 지점을 탐색하다 목적지에 도착하고 (```if (y, x) == goal```), 그 때의 value 값 (``` value[(t, y, x)]```)이 유효한 값(>0)이면 value를 0으로 설정하여 목적지의 cost는 0이 되도록 한다.
* 목적지 `policy`에는 무의미한 값인 -999를 주었다.
* `change`를 True로 설정하여 while 반복문이 계속 돌아갈 수 있도록 한다.
---
* Line 79 ~ 93 in ![assignment.py](./assignment.py)
```python
elif grid[(y, x)] == 0:
    # TODO: implement code.
    for i, a in enumerate(action):            
        direction = (t + a) % 4
        y2, x2 = y + forward[direction][0], x + forward[direction][1]
                    
        if 0 <= y2 < grid.shape[0] \
            and 0 <= x2 < grid.shape[1] \
            and grid[(y2, x2)] == 0:      
            v2 = value[(direction, y2, x2)] + cost[i]
                            
            if v2 < value[(t, y, x)]:
                value[(t, y, x)] = v2
                policy[(t, y, x)] = a
                change = True
```
* 목적지가 아닌 지점에서 모든 action을 반복한다.
* 새로운 방향 `direction`은 기존 방향 `t`에 action `a`를 더한 후 4로 나머지 연산하여 결정한다.
* 현재 지점에 `forward` 행렬의 값을 더하여 새로운 지점 (y2, x2)를 얻는다.
* `if ~ and ~ and` 문에서 새로운 지점이 맵 내의 지점이고, 장애물이 없는 지점인지 체크한다.
* 새로운 지점이 유효한 지점이면, 택한 행동의 비용 `cost`를 `value`에 더하여 `v2`를 얻는다.
* `v2`가 기존의 `value` 값보다 낮다면 지금의 행동이 최적이므로, `value` 값을 업데이트 하고 `policy`에는 현재 행동 `a`를 업데이트 한다.
* `change`를 True로 하여 while 반복문을 다시 진행한다.
---
* Line 98 ~ 128 in ![assignment.py](./assignment.py)
```python
y, x, o = init

policy_start = policy[(o, y, x)]
for i in range(len(action)):
    if policy_start == action[i]:
        policy_name_start = action_name[i]

policy2D[(y, x)] = policy_name_start
while policy[(o, y, x)] != -999:
    if policy[(o, y, x)] == action[0]:
        o2 = (o - 1) % 4  # turn left
    elif policy[(o, y, x)] == action[1]:
        o2 = o  # go straight
    elif policy[(o, y, x)] == action[2]:
        o2 = (o + 1) % 4  # turn right

    y, x = y + forward[o2][0], x + forward[o2][1]
    o = o2

    policy_temp = policy[(o,y,x)]
    if policy_temp == -999:
        policy_name = "*"
    else:
        for i in range(len(action)):
            if policy_temp == action[i]:
                policy_name = action_name[i]

    policy2D[(y,x)] = policy_name 

return policy2D
```
* 경로 탐색이 완료된 후 임의의 2차원 공간에 차량이 각 지점에서 취한 행동을 표시하는 코드이다.
* 출발지에서부터 시작하고 정책을 얻는다.
* 시작지에서의 정책에 맞는 행동을 찾아 `policy2D`에 입력한다.
* 목적지에 표시해둔 -999를 만날 때까지 아래를 반복한다.
* 해당 지점에서의 행동을 찾는다.
* 그 행동으로 다음 지점으로 이동한다.
* 이동한 지점에서의 `policy`를 찾고, 목적지라면 "*"을 입력한다.
* 목적지가 아니라면 그 지점에서의 `policy`를 찾아 `policy2D`에 입력한다.
---
* 아래는 `cost`를 달리 하였을 때의 실행 결과들이다.
* 좌회전인 'L'의 cost를 크게 주었을 때 교차로에서 좌회전하지 않고 크게 돌아가는 것을 확인할 수 있다.

* `cost = (2, 1, 20)`의 실행 결과
```
[[' ' ' ' ' ' 'R' '#' 'R']
 [' ' ' ' ' ' '#' ' ' '#']
 ['*' '#' '#' '#' '#' 'R']
 [' ' ' ' ' ' '#' ' ' ' ']
 [' ' ' ' ' ' '#' ' ' ' ']]
```

* `cost = (2, 1, 10)`의 실행 결과
```
[[' ' ' ' ' ' ' ' ' ' ' ']
 [' ' ' ' ' ' ' ' ' ' ' ']
 ['*' '#' '#' 'L' ' ' ' ']
 [' ' ' ' ' ' '#' ' ' ' ']
 [' ' ' ' ' ' '#' ' ' ' ']]
```

* `cost = (2, 1, 2)`의 실행 결과
```
[[' ' ' ' ' ' ' ' ' ' ' ']
 [' ' ' ' ' ' ' ' ' ' ' ']
 ['*' '#' '#' 'L' ' ' ' ']
 [' ' ' ' ' ' '#' ' ' ' ']
 [' ' ' ' ' ' '#' ' ' ' ']]
```

---

## Examples

We have four small working examples for demonstration of basic path planning algorithms:

* `search.py`: simple shortest path search algorithm based on BFS (breadth first search) - only calculating the cost.
* `path.py`: built on top of the above, generating an optimum (in terms of action steps) path plan.
* `astar.py`: basic implementation of the A* algorithm that employs a heuristic function in expanding the search.
* `policy.py`: computation of the shortest path based on a dynamic programming technique.

These sample source can be run as they are. Explanation and test results are given in the lecture notes.

## Assignment

You will complete the implementation of a simple path planning algorithm based on the dynamic programming technique demonstrated in `policy.py`. A template code is given by `assignment.py`.

The assignmemt extends `policy.py` in two aspects:

* State space: since we now consider not only the position of the vehicle but also its orientation, the state is now represented in 3D instead of 2D.
* Cost function: we define different cost for different actions. They are:
	- Turn right and move forward
	- Move forward
	- Turn left and move forward

This example is intended to illustrate the algorithm's capability of generating an alternative (detouring) path when left turns are penalized by a higher cost than the other two possible actions. When run with the settings defined in `assignment.py` without modification, a successful implementation shall generate the following output,

```
[[' ', ' ', ' ', 'R', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', '#'],
 ['*', '#', '#', '#', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', ' '],
 [' ', ' ', ' ', '#', ' ', ' ']]
```

because of the prohibitively high cost associated with a left turn.

You are highly encouraged to experiment with different (more complex) maps and different cost settings for each type of action.
