# Reproducing "UAV Trajectory Planning for Data Collection from Time-Constrained IoT Devices"
The first unofficial implementation of a paper with the titled "UAV Trajectory Planning for Data Collection from Time-Constrained IoT Devices". In this repository, I implemented a sub-optimal algorithm based on successive convex approximation (SCA) as introduced by Samir et al. (2019). Specifically, I reproduced the Figure 3 in the paper, where the goal is to optimize the UAV trajectory and allocation of resources to maximize the total number of served IoT devices within a flight mission duration based on a given set of target time constraints.

![image](https://user-images.githubusercontent.com/5786636/210189851-7a6396bb-ba24-41b6-b8e2-802902fa4bc3.png)


**Paper**: M. Samir, S. Sharafeddine, C. M. Assi, T. M. Nguyen and A. Ghrayeb, "UAV Trajectory Planning for Data Collection from Time-Constrained IoT Devices," in IEEE Transactions on Wireless Communications, vol. 19, no. 1, pp. 34-46, Jan. 2020, doi: 10.1109/TWC.2019.2940447. > https://ieeexplore.ieee.org/document/8842600

## Summary
I summarize the paper and the implementation in this [slides](https://github.com/willyfh/uav-trajectory-planning/blob/main/doc/Summary%20-%20UAV%20Trajectory%20Planning%20for%20Data%20Collection%20from%20Time-Constrained%20IoT%20Devices%20.pdf)

![optimization-problem](https://github.com/willyfh/uav-trajectory-planning/assets/5786636/08bd5c75-7e04-4714-92d3-0b9bcae2f1bd)

## How to execute the code
1. Install CVX on MATLAB : http://cvxr.com/cvx/download/
2. Change `rician` variable in the `main.m` to `true` or `false` as needed
2. Execute the matlab code, i.e., `main.m`


If the plot is not displayed after the iteration is completed, please execute this in the command window:
```bash
hold on; scatter(X, Y, 10, 'b');scatter(device_X, device_Y,50, 'r'); xlim([0 800]);ylim([0 800]); text(device_X, device_Y, split(num2str(deadline))); hold off
```

## Reproduced Results
### Rician Channel
![rician_figure](https://user-images.githubusercontent.com/5786636/210167035-c8c00597-fa5d-4e1a-b104-a775710a6d23.PNG)

### Path-loss Channel
![path_loss_figure](https://user-images.githubusercontent.com/5786636/210167061-a63fc936-84a2-47e3-b1a2-cbae3dac0bb4.PNG)
