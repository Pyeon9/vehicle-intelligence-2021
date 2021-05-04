# Week 6 - Prediction & Behaviour Planning

---
## Report #1 - Assignment #1

* 과제 1은 `Gaussian Naive Bayes Classifier` (GNB Classifier) 를 구현하는 것이다. 
* 구현해야 할 함수는 `train(X, Y)`와 `predict(observation)`의 두가지이다. 
---
* `train(X, Y)` : Line 38~51 in ![./GNB/classifier.py](./GNB/classifier.py)
```python
def train(self, X, Y):
        self.X_arr, self.Y_arr = np.array(X), np.array(Y)
        unique, cnt = np.unique(Y, return_counts=True)
        self.priors = cnt / len(Y)
        
        self.means = np.array([self.X_arr[np.where(self.Y_arr==i)].mean(axis=0) for i in self.classes])
        self.stds = np.array([self.X_arr[np.where(self.Y_arr==i)].std(axis=0) for i in self.classes])
        
        return self
```
* 데이터 X와 라벨 Y를 인자로 받고, `np.unique()` 함수를 사용하여 Y의 고유 값과 개수를 파악한다.
* Y의 개수를 전체 Y의 개수로 나누어 사전확률 `priors`를 계산한다.
* `self.classes`에 있는 각 원소에 해당하는 Y index를 찾고, 그 index의 X 값들로 각 class의 평균과 표준편차를 계산한다.
---
* `predict(X, Y)` : Line 56~79 in ![./GNB/classifier.py](./GNB/classifier.py)
```python
def predict(self, observation):
        probas = np.zeros(len(self.classes))
        # Calculate Gaussian probability for each variable based on the
        # mean and standard deviation calculated in the training process.
        for i in range(len(self.classes)):
            proba = 1
            for k in range(self.X_arr.shape[1]):
                # Multiply all the probabilities for variables, and then
                # normalize them to get conditional probabilities.
                proba *= gaussian_prob(observation[k], self.means[i][k], self.stds[i][k])
#             proba *= self.priors[i]
            probas[i] = proba
        res = probas / probas.sum()
        
        # Return the label for the highest conditional probability.
        return self.classes[res.argmax(axis=0)]
```
* 우선 총 class의 개수에 맞춰 0 행렬 `probas`를 생성한다.
* 각 class에 대해 observation의 확률을 gaussian으로부터 계산한다.
* 앞서 계산해두었던 사전확률 `priors`를 곱할 수 있다. 
	- 성능이 약간 저하되어 주석 처리함. 비교는 아래를 참고
* 계산한 확률을 총합으로 나누어 정규화한다.
---
### Result
* 위: 사전확률 곱하지 않음 / 아래: 사전확률 곱함

![result](./GNB/result.png)

---
## Report #2 - Assignment #2

### Task 1
* `choose_next_state(predictions)` : Line 24~68 in ![./BP/vehicle.py](./BP/vehicle.py)
* 이 함수는 predicted trajectory들을 입력으로 받고 cost function에 따라 최적의 경로를 반환한다.
* 각 trajectory는 현재 타임스텝과 다음 타임스텝에서의 차량 오브젝트의 list로 구성된다.
```python
def choose_next_state(self, predictions):
        # TODO: implement state transition function based on the cost
        #          associated with each transition.
        states = self.successor_states()
        min_cost = 9999
        best_traj = None
        for i in range(len(states)):
            traj = self.generate_trajectory(states[i], predictions)
            cost = calculate_cost(self, traj, predictions)
            if cost < min_cost:
                min_cost = cost
                best_traj = traj
        
        return best_traj
```
* `self.successor_states()` 함수를 사용하여 finite state machine을 기반으로 가능한 다음 상태 `states`를 구한다.
* `min_cost = 9999`, `best_traj = None`으로 초기 세팅하고, 모든 `states`에 대해 for 문을 돌며 아래를 반복한다.
* `self.generate_trajectory(state, predictions)` 함수를 통해 경로를 생성한다.
* `calculate_cost(traj, predictions)` 함수로 해당 경로의 `cost`를 계산한다.
* `cost`가 `min_cost`보다 낮다면 `min_cost`와 `best_traj`를 갱신한다.
* 모든 `states`에 대해 반복이 끝나면 `best_traj`를 반환한다. 
---
### Task 2

* `goal_distance_cost()`: Line 28~39 in ![./BP/cost_functions.py](./BP/cost_functions.py)
* 이 함수는 차량의 lane이 `intended lane`과 `final lane`과 차이가 클 수록 큰 cost를 반환해야 한다.
* 또한, 차량이 목적지에 가까이 다가갈수록 그 차이에 대한 cost는 더 커져야 한다.
* 아래와 같이 `1 - exp(-x)`의 형태로 cost function을 설계하였다.
```python
def goal_distance_cost(vehicle, trajectory, predictions, data):
    distance = abs(data.end_distance_to_goal)
    cost = 1 - exp(-(abs(2.0*vehicle.goal_lane - data.intended_lane - data.final_lane) / distance))
    
    return cost
```
* `abs(2.0*vehicle.goal_lane - data.intended_lane - data.final_lane)`에서 차량의 lane과 `intended lane`, `final lane`의 차이를 exp의 지수로 설정한다.
* 또한, 이를 목적지까지의 거리인 `distance`로 나누어 목적지에 가까워질수록 더 큰 값을 가지도록 하였다.
* 참고로, `1 - exp(-x)`의 개형은 아래와 같다.

![1-exp(-x)](./BP/1-exp(-x).png)
---
* `inefficiency_cost()`: Line 41~51 in ![./BP/cost_functions.py](./BP/cost_functions.py)
* 이 함수는 차량의 속도에 따라 cost를 결정하는 함수로, 차량의 속도와  `intended lane`, `final lane`의 속도를 비교하여 느린 속도에 큰 cost를 부여한다.
```python
def inefficiency_cost(vehicle, trajectory, predictions, data):
    speed_intended = velocity(predictions, data.intended_lane) or vehicle.target_speed
    speed_final = velocity(predictions, data.final_lane) or vehicle.target_speed
    
    cost = (2.0*vehicle.target_speed - speed_intended - speed_final) / vehicle.target_speed
    
    return cost
```
* 차량의 `target_speed`에서 `speed_intended`와 `speed_final`를 뺀 다음 차량의 `target_speed`으로 나누어 정규화 하였다. 
---
## Result

* 위의 구현으로 `simulate_behavior.py`를 실행하면 가장 속도가 빠른 lane으로 주행하다 목적지에 가까워질 때 target lane으로 이동하여 목적지에 도달하는 것을 볼 수 있다.

![result](./BP/result.png)
---
## Assignment #1

Under the directory [./GNB](./GNB), you are given two Python modules:

* `prediction.py`: the main module you run. The `main()` function does two things: (1) read an input file ([`train.json`](./GNB/train.json)) and train the GNB (Gaussian Naive Bayes) classifier using the data stored in it, and (2) read another input file ([`test.json`](./GNB/test.json)) and make predictions for a number of data points. The accuracy measure is taken and displayed.
* `classifier.py`: main implementation of the GNB classifier. You shall implement two methods (`train()` and `precict()`), which are used to train the classifier and make predictions, respectively.

Both input files ([`train.json`](./GNB/train.json) and [`test.json`](./GNB/test.json)) have the same format, which is a JSON-encoded representation of training data set and test data set, respectively. The format is shown below:

```
{
	"states": [[s_1, d_1, s_dot_1, d_dot_1],
	           [s_2, d_2, s_dot_2, d_dot_2],
	           ...
	           [s_n, d_n, s_dot_n, d_dot_n]
	          ],
	"labels": [L_1, L_2, ..., L_n]
}
```

The array `"states"` have a total of `n` items, each of which gives a (hypothetically) measured state of a vehicle, where `s_i` and `d_i` denote its position in the Frenet coordinate system. In addition, `s_dot_i` and `d_dot_i` give their first derivates, respectively. For each measured state, a label is associated (given in the `"labels"` array) that represents the vehicle's behaviour. The label is one of `"keep"`, `"left"`, and `"right"`, which denote keeping the current lane, making a left turn, and making a right turn, respectively.

The training set has a total of 750 data points, whereas the test set contains 250 data points with the ground truth contained in `"labels"`.

The GNB classifier is trained by computing the mean and variance of each component in the state variable for each observed behaviour. Later it is used to predict the behaviour by computing the Gaussian probability of an observed state for each behaviour and taking the maximum. You are going to implement that functionality. For convcenience, a separate function `gaussian_prob()` is already given in the module `classifier.py`.


---

## Assignment #2

Under the directory [./BP](./BP), you are given four Python modules:

* `simulate_behavior.py`: the main module you run. It instantiates a simple text-based simulation environment and runs it using the configuration specified in the same module.
* `road.py`: `class Road` is implemented here. It captures the state of the simulated road with a number of vehicles (including the ego) running on it, and visualizes it using terminal output.
* `vehicle.py`: `class Vehicle` implements the states of a vehicle and its transition, along with the vehicle's dynamics based on a simple kinematic assumption. Note that a vehicle's trajectory is represented by two instances of object of this class, where the first one gives the current state and the second one predicts the state that the vehicle is going to be in after one timestep.
* `cost_functions.py`: implementation of cost functions governing the state transition of the ego vehicle. The main job required for your assignment is to provide an adequate combination of cost functions by implementing them in this module.

### Task 1

Implement the method `choose_next_state()` in `vehicle.py`. It should

* determine which state transitions are possible from the current state (`successor_states()` function in the same module will be helpful),
* calculate cost for each state transition using the trajectory generated for each behaviour, and
* select the minimum cost trajectory and return it.

Note that you must return a planned trajectory (as described above) instead of the state that the vehicle is going to be in.

### Task 2

In `cost_functions.py`, templates for two different cost functions (`goal_distance_cost()` and `inefficiency_cost()`) are given. They are intended to capture the cost of the trajectory in terms of

* the lateral distance of the vehicle's lane selection from the goal position, and
* the time expected to be taken to reach the goal (because of different lane speeds),

respectively.

Note that the range of cost functions should be carefully defined so that they can be combined by a weighted sum, which is done in the function `calculate_cost()` (to be used in `choose_next_state()` as described above). In computing the weighted sum, a set of weights are used. For example, `REACH_GOAL` and `EFFICIENCY` are already defined (but initialized to zero values). You are going to find out a good combination of weights by an empirical manner.

You are highly encouraged to experiment with your own additional cost functions. In implementing cost functions, a trajectory's summary (defined in `TrajectoryData` and given by `get_helper_data()`) can be useful.

You are also invited to experiment with a number of different simulation settings, especially in terms of

* number of lanes
* lane speed settings (all non-ego vehicles follow these)
* traffic density (governing the number of non-ego vehicles)

and so on.

Remember that our state machine should be geared towards reaching the goal in an *efficient* manner. Try to compare a behaviour that switches to the goal lane as soon as possible (note that the goal position is in the slowest lane in the given setting) and one that follows a faster lane and move to the goal lane as the remaining distance decreases. Observe different behaviour taken by the ego vehicle when different weights are given to different cost functions, and also when other cost metrics (additional cost functions) are used.
