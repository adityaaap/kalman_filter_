# Kalman Filter in C++ using Eigen #

* To run the program:

```
g++ -o kf kalman_filter.cpp
./kf
```

* The Kalman Filter uses the u.csv and y.csv data to estimate the y_hat
* The variable named 'x_hat_store' has the estimations stored in it

# Note
* This Filter is specifically for Process Noise Covarience of 2x2 and Measurement Noise Covariance of 1x1. But this can be easily scaled to the dimensions intended by the user because of the use of Eigen library in formulating equations for the Filter.
